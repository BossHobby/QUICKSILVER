#include "control/wing/navigation.h"

#include <float.h>
#include <math.h>

#include "control/control.h"
#include "control/mission.h"
#include "control/navigation.h"
#include "core/profile.h"
#include "driver/baro/baro.h"
#include "driver/time.h"
#include "io/blackbox.h"
#include "rx/rx.h"
#include "util/util.h"

static constexpr float L1_PERIOD = 15.0f;                      // seconds, lateral guidance response period
static constexpr float L1_DAMPING = 0.75f;
static constexpr float LOITER_CENTER_MIN_SPEED = 3.0f;         // m/s, slower GPS course does not orient the circle
static constexpr float FALLBACK_BANK = 20.0f * DEGTORAD;
// degrees; a sustained coordinated turn loads 1 / cos(bank). Beyond about 40
// degrees that exceeds the IMU's 1.3 g gravity-correction limit.
static constexpr float MAX_BANK_LIMIT = 40.0f;
static constexpr float BANK_SLEW_RATE = 45.0f * DEGTORAD;      // rad/s
static constexpr float PITCH_SLEW_RATE = 20.0f * DEGTORAD;     // rad/s
static constexpr float THROTTLE_SLEW_RATE = 0.5f;              // per second
static constexpr float ALT_POSITION_KP = 0.5f;                 // m/s climb per meter of altitude error
static constexpr float ALT_MAX_CLIMB_RATE = 3.0f;              // m/s
static constexpr float ALT_MAX_SINK_RATE = 2.0f;               // m/s
static constexpr float ALT_RATE_KP = 3.0f * DEGTORAD;          // climb angle per m/s of climb-rate error
static constexpr float ALT_RATE_KI = 1.0f * DEGTORAD;          // climb angle per m of accumulated climb-rate error
static constexpr float ALT_MAX_CLIMB_ANGLE = 20.0f * DEGTORAD;
static constexpr float ALT_MAX_DIVE_ANGLE = 15.0f * DEGTORAD;
static constexpr uint32_t RTH_HEADING_TIMEOUT_MS = 10000;      // IMU-heading return after GPS loss
static constexpr float DESCEND_GLIDE_ANGLE = 5.0f * DEGTORAD;  // nose-down during an unpowered failsafe glide
static constexpr uint32_t DESCEND_LATCH_MS = 2000;             // GPS recovery before this resumes the return
static constexpr uint32_t CRUISE_LOCK_DELAY_MS = 500;          // centered roll and yaw before the course locks, as in ArduPlane
static constexpr float CRUISE_STICK_CENTER = 0.05f;            // filtered stick magnitude treated as centered
static constexpr float CRUISE_LOOKAHEAD = 1000.0f;             // meters along the locked track, as in ArduPlane
// A wing cannot stop on a waypoint: advance within the turn radius at the
// navigation bank, bounded for slow GPS speed and shallow bank settings.
static constexpr float MISSION_ARRIVAL_MIN_BANK = 10.0f * DEGTORAD;
static constexpr float MISSION_ARRIVAL_MIN_RADIUS = 10.0f;     // meters
static constexpr float MISSION_ARRIVAL_MAX_RADIUS = 100.0f;    // meters
// A leg can need more climb than a wing makes on it; circle a waypoint arrived
// at farther below its altitude than this before flying on.
static constexpr float MISSION_ALTITUDE_TOLERANCE = 10.0f;     // meters

typedef enum {
  NAV_REQUEST_NONE,
  NAV_REQUEST_LOITER,
  NAV_REQUEST_RTH,
  NAV_REQUEST_FAILSAFE,
  NAV_REQUEST_CRUISE,
  NAV_REQUEST_MISSION,
} nav_request_t;

wing_nav_command_t wing_nav_command;

static struct {
  nav_request_t request; // previous update's request; a new one latches its own altitude target
  uint32_t gps_lost_ms; // start of the IMU-heading return
  uint32_t descend_ms;  // start of the failsafe glide
  float speed;          // latest valid GPS ground speed, m/s
  bool mission_held;    // mission switch on since before navigation was available
  bool mission_requested; // the switches requested the mission in the latest update
} nav;

static struct {
  gps_coord_t origin; // GPS position the center is measured from
  float center_north; // circle center relative to origin, meters
  float center_east;
} circle;

static struct {
  bool target_valid; // RTH retains its latched target across barometer outages
  float target;      // meters above launch
  float integral;    // climb angle, radians; also trims the extra lift a bank needs
} altitude;

static struct {
  uint32_t steer_ms;  // latest pass the pilot steered or the course could not lock
  gps_coord_t origin; // GPS position where the course locked
  float dir_north;    // unit ground track of the locked course
  float dir_east;
  bool climbing;      // pitch stick was off center in the previous pass
} cruise;

// What the mission request flies. A new request resumes nav_route at the
// active waypoint unless the mission finished, and checks it anew. A ground
// station target in nav_route.guided replaces the route until a mission command
// or a new request; nav_route.returning covers the returns of both.
typedef enum {
  ROUTE_UNCHECKED,      // checked until the stored route can be flown
  ROUTE_FLYING,         // nav_route.complete circles its last waypoint
  ROUTE_CLIMBING,       // circling the active waypoint until its altitude is reached
  ROUTE_GUIDED,         // flying to the ground station target
  ROUTE_GUIDED_ARRIVED, // circling the ground station target
} route_phase_t;

static route_phase_t route;

static float loiter_direction() {
  return profile.navigation.loiter_direction == NAV_LOITER_LEFT ? -1.0f : 1.0f;
}


// Roll or yaw input releases the course, as does losing the GPS track. It
// locks again once both sticks have stayed centered, as in ArduPlane's CRUISE.
static wing_nav_state_t cruise_next_state(uint32_t now_ms) {
  const wing_nav_state_t previous = (wing_nav_state_t)state.wing_nav_state;
  const bool cruising = previous == WING_NAV_CRUISE_STEER || previous == WING_NAV_CRUISE || previous == WING_NAV_CRUISE_NO_GPS;
  const bool steering = fabsf(state.rx_filtered.roll) > CRUISE_STICK_CENTER || fabsf(state.rx_filtered.yaw) > CRUISE_STICK_CENTER;
  if (!flags.gps_valid) {
    cruise.steer_ms = now_ms;
    return WING_NAV_CRUISE_NO_GPS;
  }
  if (!cruising || steering || nav.speed < LOITER_CENTER_MIN_SPEED) {
    cruise.steer_ms = now_ms;
    return WING_NAV_CRUISE_STEER;
  }
  if (previous == WING_NAV_CRUISE || now_ms - cruise.steer_ms >= CRUISE_LOCK_DELAY_MS)
    return WING_NAV_CRUISE;
  return WING_NAV_CRUISE_STEER;
}

// What the switches and failsafe request, in priority order. A mission
// request maps onto what the aircraft flies: the route, a return after
// MISSION_END_RTH or without GPS, or a loiter in place until the route can
// be flown.
static nav_request_t nav_request() {
  // Launch owns attitude and throttle until it finishes; navigation never
  // starts on the ground. Failsafe ignores the switches, which hold stale values.
  const bool launching = flags.wing_launch_available && state.wing_launch_state < WING_LAUNCH_DONE;
  // As on a multirotor, a mission needs a switch edge in flight: a switch left
  // on through arming and launch does not fly an old route.
  const bool mission_switch = rx_aux_on(AUX_MISSION);
  nav_request_t request = NAV_REQUEST_NONE;
  if (!flags.arm_state || !flags.in_air || launching) {
    nav.mission_held = mission_switch;
  } else {
    nav.mission_held = nav.mission_held && mission_switch;
    // A mission flying through a link loss locks the switches until the pilot
    // unlocks them with the sticks.
    if (state.failsafe_owner == FAILSAFE_OWNER_MISSION)
      request = NAV_REQUEST_MISSION;
    else if (flags.failsafe)
      request = profile.navigation.rth_on_failsafe ? NAV_REQUEST_FAILSAFE : NAV_REQUEST_NONE;
    else if (rx_aux_on(AUX_RETURN_TO_HOME))
      request = NAV_REQUEST_RTH;
    else if (mission_switch && !nav.mission_held)
      request = NAV_REQUEST_MISSION;
    else if (rx_aux_on(AUX_LOITER))
      request = NAV_REQUEST_LOITER;
    else if (rx_aux_on(AUX_CRUISE))
      request = NAV_REQUEST_CRUISE;
  }

  if (!flags.arm_state)
    nav_route = {};
  const bool requested = request == NAV_REQUEST_MISSION;
  if (requested && !nav.mission_requested) {
    nav_route.index = nav_route_resume();
    nav_route.complete = false;
    nav_route.returning = false;
    route = ROUTE_UNCHECKED;
  }
  nav.mission_requested = requested;
  if (!requested) {
    // Nothing is flown, so a ground station may clear or replace the mission.
    route = ROUTE_UNCHECKED;
    return request;
  }
  if (nav_route.returning)
    return NAV_REQUEST_RTH;
  if (route == ROUTE_UNCHECKED) {
    // Checked until the route can be flown, so a brief GPS dropout at the
    // switch edge does not leave the aircraft loitering.
    if (mission.count == 0 || !flags.gps_valid || !nav_mission_in_start_range(nav_route.index))
      return NAV_REQUEST_LOITER;
    route = ROUTE_FLYING;
    nav_route_start_leg();
  }
  // Without GPS neither can be followed; return on the IMU heading, at the
  // RTH altitude, until it recovers.
  return flags.gps_valid ? NAV_REQUEST_MISSION : NAV_REQUEST_RTH;
}

static bool route_guided() {
  return route == ROUTE_GUIDED || route == ROUTE_GUIDED_ARRIVED;
}

// The ground station target, or the active waypoint of the stored route.
static const mission_waypoint_t &route_waypoint() {
  return route_guided() ? nav_route.guided : mission.waypoints[nav_route.index];
}

static bool route_holding() {
  return route == ROUTE_GUIDED_ARRIVED || route == ROUTE_CLIMBING || (route == ROUTE_FLYING && nav_route.complete);
}

// Beyond the line through the waypoint square to the leg. Turning toward a
// point inside the turn circle, or with wind stretching it, can otherwise
// orbit just outside the arrival radius.
static bool nav_mission_passed() {
  const gps_coord_t waypoint = route_waypoint().coord;
  float leg_north, leg_east, north, east;
  nav_position_delta(nav_route.leg_origin, waypoint, &leg_north, &leg_east);
  nav_position_delta(waypoint, state.gps_coord, &north, &east);
  return leg_north * north + leg_east * east > 0.0f;
}

// tan(turn / 2) for the turn onto the next leg: a turn of that radius started
// this fraction of it before the waypoint rolls out on the next leg, as in
// ArduPlane's turn_distance(). Turns of 90 degrees or more, and the last
// waypoint, use the whole radius.
static float nav_mission_turn_scale() {
  if (route_guided() || nav_route.index + 1 >= mission.count)
    return 1.0f;
  const gps_coord_t waypoint = mission.waypoints[nav_route.index].coord;
  float in_north, in_east, out_north, out_east;
  nav_position_delta(nav_route.leg_origin, waypoint, &in_north, &in_east);
  nav_position_delta(waypoint, mission.waypoints[nav_route.index + 1].coord, &out_north, &out_east);
  const float dot = in_north * out_north + in_east * out_east;
  if (dot <= 0.0f)
    return 1.0f;
  const float cross = in_north * out_east - in_east * out_north;
  return MIN(fabsf(cross) / (hypotf(in_north, in_east) * hypotf(out_north, out_east) + dot), 1.0f);
}

static bool nav_mission_arrived(float max_bank) {
  const float bank = MAX(max_bank, MISSION_ARRIVAL_MIN_BANK);
  const float turn_radius = MIN(nav.speed * nav.speed / (GRAVITY * tanf(bank)), MISSION_ARRIVAL_MAX_RADIUS);
  const float radius = MAX(turn_radius * nav_mission_turn_scale(), MISSION_ARRIVAL_MIN_RADIUS);
  return state.nav_target_distance <= radius || nav_mission_passed();
}

// The active waypoint's altitude is reached, or cannot be checked without the
// barometer.
static bool nav_mission_altitude_reached() {
  if (!altitude.target_valid || !flags.baro_valid)
    return true;
  return state.altitude >= route_waypoint().altitude - MISSION_ALTITUDE_TOLERANCE;
}

static void nav_mission_advance(float max_bank) {
  if (route == ROUTE_CLIMBING) {
    if (!nav_mission_altitude_reached())
      return;
    route = ROUTE_FLYING;
  } else {
    if (!nav_mission_arrived(max_bank))
      return;
    // Circle the waypoint rather than fly the next leg below its planned
    // altitude. Mission end actions set their own altitude.
    if (route == ROUTE_FLYING && nav_route.index + 1 < mission.count && !nav_mission_altitude_reached()) {
      route = ROUTE_CLIMBING;
      return;
    }
  }
  if (route == ROUTE_GUIDED) {
    route = ROUTE_GUIDED_ARRIVED;
  } else if (nav_route.index + 1 < mission.count) {
    nav_route_next();
    nav_update_target();
  } else {
    nav_route.complete = true;
    nav_route.returning = mission.end_action == MISSION_END_RTH;
  }
}

static wing_nav_state_t nav_next_state(nav_request_t request, uint32_t now_ms) {
  if (request == NAV_REQUEST_CRUISE)
    return cruise_next_state(now_ms);
  const wing_nav_state_t previous = (wing_nav_state_t)state.wing_nav_state;
  // A brief GPS quality dip cannot have landed the aircraft, so recovery
  // resumes the return. After a longer glide, GPS recovery alone cannot tell
  // whether it has landed; keep the motor off until the pilot recovers control.
  if (request == NAV_REQUEST_FAILSAFE && previous == WING_NAV_DESCEND && now_ms - nav.descend_ms >= DESCEND_LATCH_MS)
    return WING_NAV_DESCEND;
  const bool returning = request != NAV_REQUEST_LOITER && flags.home_valid;
  if (flags.gps_valid) {
    if (request == NAV_REQUEST_MISSION)
      return route_holding() ? WING_NAV_MISSION_HOLD : WING_NAV_MISSION;
    if (returning) {
      if (previous == WING_NAV_RTH_HOME || state.home_distance <= profile.navigation.loiter_radius)
        return WING_NAV_RTH_HOME;
      return WING_NAV_RTH_RETURN;
    }
    // Failsafe without a home descends; a manual RTH without one loiters in place.
    return request == NAV_REQUEST_FAILSAFE ? WING_NAV_DESCEND : WING_NAV_LOITER;
  }

  if (returning && state.heading_confidence >= RTH_MIN_HEADING_CONFIDENCE) {
    // GPS lost on a mission also heads home on the IMU heading.
    if (previous == WING_NAV_RTH_RETURN || previous == WING_NAV_RTH_HOME ||
        previous == WING_NAV_MISSION || previous == WING_NAV_MISSION_HOLD) {
      nav.gps_lost_ms = now_ms;
      return WING_NAV_RTH_HEADING;
    }
    if (previous == WING_NAV_RTH_HEADING && now_ms - nav.gps_lost_ms < RTH_HEADING_TIMEOUT_MS)
      return WING_NAV_RTH_HEADING;
  }
  return request == NAV_REQUEST_FAILSAFE ? WING_NAV_DESCEND : WING_NAV_LOITER_BANK;
}

static void circle_place_ahead() {
  // Offset the center to the turn side of the ground track so the aircraft
  // rolls straight onto the circle instead of first flying out from its center.
  circle.origin = state.gps_coord;
  circle.center_north = 0.0f;
  circle.center_east = 0.0f;
  const float speed = hypotf(state.gps_vel_north, state.gps_vel_east);
  if (speed < LOITER_CENTER_MIN_SPEED)
    return;
  const float scale = loiter_direction() * profile.navigation.loiter_radius / speed;
  circle.center_north = -state.gps_vel_east * scale;
  circle.center_east = state.gps_vel_north * scale;
}

static void circle_place_at(gps_coord_t center) {
  circle.origin = center;
  circle.center_north = 0.0f;
  circle.center_east = 0.0f;
}

static float l1_distance(float speed) {
  return MAX(L1_DAMPING * L1_PERIOD * speed / M_PI_F, 1.0f);
}

// L1 lateral acceleration for the angle nu from the ground track to the
// look-ahead point, clamped to +-90 degrees, as in ArduPlane's AP_L1_Control.
static float l1_lateral_accel(float sin_nu, float speed) {
  return 4.0f * L1_DAMPING * L1_DAMPING * speed * speed / l1_distance(speed) * sin_nu;
}

// L1 loiter guidance on the GPS ground track, as in ArduPlane's
// AP_L1_Control::update_loiter(). Returns lateral acceleration in m/s^2,
// positive to the right.
static float circle_lateral_accel() {
  float north, east;
  nav_position_delta(circle.origin, state.gps_coord, &north, &east);
  north -= circle.center_north;
  east -= circle.center_east;
  const float distance = hypotf(north, east);

  const float vel_north = state.gps_vel_north;
  const float vel_east = state.gps_vel_east;
  const float speed = hypotf(vel_north, vel_east);

  // Unit vector from the center to the aircraft.
  float radial_north = 1.0f;
  float radial_east = 0.0f;
  if (distance > 0.1f) {
    radial_north = north / distance;
    radial_east = east / distance;
  } else if (speed > 0.1f) {
    radial_north = vel_north / speed;
    radial_east = vel_east / speed;
  }

  const float direction = loiter_direction();
  const float radius = MAX(profile.navigation.loiter_radius, 1.0f);
  const float omega = 2.0f * M_PI_F / L1_PERIOD;

  // Capture: steer the velocity vector toward the center from outside.
  const float cross_velocity = radial_north * vel_east - radial_east * vel_north;
  const float inward_velocity = -(radial_north * vel_north + radial_east * vel_east);
  const float nu = constrain(atan2f(cross_velocity, inward_velocity), -M_PI_F / 2.0f, M_PI_F / 2.0f);
  const float capture = l1_lateral_accel(sinf(nu), speed);

  // Circle: PD on the radial error plus the centripetal demand of the turn.
  const float radial_error = distance - radius;
  const float tangent_velocity = cross_velocity * direction;
  float correction = radial_error * omega * omega - inward_velocity * 2.0f * L1_DAMPING * omega;
  if (inward_velocity < 0.0f && tangent_velocity < 0.0f)
    correction = MAX(correction, 0.0f);
  const float centripetal = tangent_velocity * tangent_velocity / MAX(0.5f * radius, radius + radial_error);
  const float circle_accel = direction * (correction + centripetal);

  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 5, (int16_t)constrain(distance, 0.0f, 32767.0f)); // meters

  if (radial_error > 0.0f && direction * capture < direction * circle_accel)
    return capture;
  return circle_accel;
}

// L1 waypoint guidance toward a bearing, as in ArduPlane's
// AP_L1_Control::update_waypoint(). Course and bearing in radians.
static float waypoint_lateral_accel(float bearing, float course, float speed) {
  // Signed bearing error in -pi..pi; normalize_rad() wraps to 0..2pi.
  const float error = bearing - course;
  const float nu = constrain(atan2f(sinf(error), cosf(error)), -M_PI_F / 2.0f, M_PI_F / 2.0f);
  return l1_lateral_accel(sinf(nu), speed);
}

// L1 toward a point lookahead meters ahead of the aircraft along the track
// from origin in the unit direction dir, ending length meters along it, so
// cross-track error is steered out as well as wind drift. Looking at least as
// far ahead as the cross-track error caps the intercept at 45 degrees, as in
// ArduPlane's AP_L1_Control::update_waypoint(). Vector form keeps libm trig
// out of the Flight loop.
static float track_lateral_accel(gps_coord_t origin, float dir_north, float dir_east, float length, float lookahead) {
  float north, east;
  nav_position_delta(origin, state.gps_coord, &north, &east);
  const float cross_track = dir_north * east - dir_east * north; // right positive
  const float along = north * dir_north + east * dir_east;
  const float ahead = MIN(along + MAX(lookahead, fabsf(cross_track)), length);
  const float los_north = ahead * dir_north - north;
  const float los_east = ahead * dir_east - east;

  // Cross and dot products of ground velocity and line of sight scale sin(nu)
  // and cos(nu) by both lengths; beyond 90 degrees nu clamps to the turn side.
  const float vel_north = state.gps_vel_north;
  const float vel_east = state.gps_vel_east;
  const float cross = vel_north * los_east - vel_east * los_north;
  const float dot = vel_north * los_north + vel_east * los_east;
  const float sin_nu = dot > 0.0f ? cross / MAX(nav.speed * hypotf(los_north, los_east), 1e-3f) : copysignf(1.0f, cross);

  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 5, (int16_t)constrain(cross_track, -32768.0f, 32767.0f)); // cross-track meters
  return l1_lateral_accel(sin_nu, nav.speed);
}

// Track the leg from its origin to the target, as ArduPlane does between
// waypoints, so crosswind does not curve the path away from the planned line.
static float mission_lateral_accel() {
  const gps_coord_t target = route_waypoint().coord;
  gps_coord_t origin = nav_route.leg_origin;
  float north, east;
  nav_position_delta(origin, target, &north, &east);
  float length = hypotf(north, east);
  if (length < 1.0f) {
    // Repeated waypoints leave the leg without a direction; fly at the target.
    origin = state.gps_coord;
    nav_position_delta(origin, target, &north, &east);
    length = MAX(hypotf(north, east), 1.0f);
  }
  return track_lateral_accel(origin, north / length, east / length, length, l1_distance(nav.speed));
}

static void cruise_lock() {
  cruise.origin = state.gps_coord;
  cruise.dir_north = state.gps_vel_north / nav.speed;
  cruise.dir_east = state.gps_vel_east / nav.speed;
}

// The locked track has no end; look CRUISE_LOOKAHEAD along it, as in ArduPlane's CRUISE.
static float cruise_lateral_accel() {
  return track_lateral_accel(cruise.origin, cruise.dir_north, cruise.dir_east, FLT_MAX, CRUISE_LOOKAHEAD);
}

// Holds the target latched when the barometer first becomes usable for this
// request. Returns the climb angle in radians, positive nose-up.
static float altitude_climb_angle(nav_request_t request, float entry_target, float dt) {
  if (!flags.baro_valid) {
    // Loiter and cruise relatch the altitude reached; a return keeps its target.
    if (request == NAV_REQUEST_LOITER || request == NAV_REQUEST_CRUISE)
      altitude.target_valid = false;
    altitude.integral = 0.0f;
    return 0.0f;
  }
  if (!altitude.target_valid) {
    altitude.target_valid = true;
    altitude.target = entry_target;
    altitude.integral = 0.0f;
  }

  const float rate = constrain(ALT_POSITION_KP * (altitude.target - state.altitude), -ALT_MAX_SINK_RATE, ALT_MAX_CLIMB_RATE);
  const float rate_error = rate - state.baro_vertical_speed;
  const float proportional = ALT_RATE_KP * rate_error;
  const float integral = altitude.integral + ALT_RATE_KI * rate_error * dt;
  const float angle = proportional + integral;
  // Stop integrating into a saturated climb or dive.
  if ((angle < ALT_MAX_CLIMB_ANGLE || rate_error < 0.0f) && (angle > -ALT_MAX_DIVE_ANGLE || rate_error > 0.0f))
    altitude.integral = constrain(integral, -ALT_MAX_DIVE_ANGLE, ALT_MAX_CLIMB_ANGLE);

  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 6, (int16_t)constrain((altitude.target - state.altitude) * 10.0f, -32768.0f, 32767.0f)); // dm
  return constrain(proportional + altitude.integral, -ALT_MAX_DIVE_ANGLE, ALT_MAX_CLIMB_ANGLE);
}

// Pitch stick commands a climb rate and releasing it holds the altitude
// reached, as in ArduPlane's FBWB. Without a barometer it commands the climb
// angle directly. Returns the climb angle in radians, positive nose-up.
static float cruise_climb_angle(float dt) {
  // The pitch stick is positive nose-down, like attitude targets.
  const float stick = -state.rx_filtered.pitch;
  const bool climbing = fabsf(stick) > CRUISE_STICK_CENTER;
  if (altitude.target_valid && climbing) {
    // Lead the altitude by the error the position loop turns into this rate.
    const float rate = stick * (stick > 0.0f ? ALT_MAX_CLIMB_RATE : ALT_MAX_SINK_RATE);
    altitude.target = state.altitude + rate / ALT_POSITION_KP;
  } else if (altitude.target_valid && cruise.climbing) {
    altitude.target = state.altitude;
  }
  cruise.climbing = climbing;

  const float hold = altitude_climb_angle(NAV_REQUEST_CRUISE, state.altitude, dt);
  if (altitude.target_valid)
    return hold;
  return stick * (stick > 0.0f ? ALT_MAX_CLIMB_ANGLE : ALT_MAX_DIVE_ANGLE);
}

// The pilot grants ground station commands with the mission switch, which
// RTH and failsafe requests override, also while a mission continues
// through a link loss.
static bool nav_guided_ready() {
  return nav.mission_requested && flags.gps_valid && !flags.failsafe;
}

nav_command_result_t nav_command_apply(nav_command_t *cmd) {
  if (!nav_guided_ready())
    return NAV_RESULT_NO_AUTHORITY;
  const nav_command_result_t result = nav_command_check(cmd);
  if (result != NAV_RESULT_OK)
    return result;

  switch (cmd->type) {
  case NAV_COMMAND_GOTO:
    nav_route.guided = cmd->waypoint;
    route = ROUTE_GUIDED;
    nav_route.returning = false;
    nav_route_start_leg();
    break;
  case NAV_COMMAND_HOLD:
    nav_route.guided = {.coord = state.gps_coord, .altitude = (int16_t)lroundf(state.altitude)};
    route = ROUTE_GUIDED_ARRIVED;
    nav_route.returning = false;
    nav_route_start_leg();
    circle_place_at(nav_route.guided.coord);
    break;
  case NAV_COMMAND_SET_ALT:
    nav_route.guided.altitude = cmd->waypoint.altitude;
    // Change to it evenly over the rest of the leg, still along its line.
    nav_route_restart_altitude();
    break;
  case NAV_COMMAND_RTH:
    // Like the return after a mission, turning the mission switch off ends it.
    route = ROUTE_UNCHECKED;
    nav_route.returning = true;
    break;
  case NAV_COMMAND_MISSION:
    // Checked again, with a new leg origin, on the next update.
    route = ROUTE_UNCHECKED;
    nav_route.index = cmd->index;
    nav_route.complete = false;
    nav_route.returning = false;
    break;
  }
  return NAV_RESULT_OK;
}

nav_command_result_t nav_mission_replace(const mission_t *updated) {
  // A checked route is flown, or resumes without a new check once a GPS
  // dropout ends; a route still to be checked is checked with the new mission.
  bool retarget;
  const bool flying = route == ROUTE_FLYING || route == ROUTE_CLIMBING;
  const nav_command_result_t result = nav_route_replace(updated, flying, false, &retarget);
  if (result != NAV_RESULT_OK || !retarget)
    return result;
  // A new leg from here. A completed mission circles its moved waypoint.
  nav_route_start_leg();
  route = ROUTE_FLYING;
  if (nav_route.complete && !nav_route.returning)
    circle_place_at(mission.waypoints[nav_route.index].coord);
  nav_update_target();
  return NAV_RESULT_OK;
}

// Placed or latched once on entering a state.
static void nav_enter_state(wing_nav_state_t next, uint32_t now_ms) {
  switch (next) {
  case WING_NAV_LOITER:
    circle_place_ahead();
    break;
  case WING_NAV_RTH_HOME:
    circle_place_at(state.gps_home);
    break;
  case WING_NAV_MISSION_HOLD:
    circle_place_at(route_waypoint().coord);
    break;
  case WING_NAV_DESCEND:
    nav.descend_ms = now_ms;
    break;
  case WING_NAV_CRUISE:
    cruise_lock();
    break;
  default:
    break;
  }
}

// Bank for the state's lateral guidance, radians positive right.
static float nav_roll(wing_nav_state_t next) {
  switch (next) {
  case WING_NAV_CRUISE_STEER:
  case WING_NAV_CRUISE_NO_GPS:
    // Wing control applies the pilot's bank; follow it so the lock takes over smoothly.
    return state.attitude.roll;
  case WING_NAV_CRUISE:
    return atanf(cruise_lateral_accel() / GRAVITY);
  case WING_NAV_LOITER:
  case WING_NAV_RTH_HOME:
  case WING_NAV_MISSION_HOLD:
    return atanf(circle_lateral_accel() / GRAVITY);
  case WING_NAV_MISSION:
    return atanf(mission_lateral_accel() / GRAVITY);
  case WING_NAV_RTH_RETURN: {
    const float course = atan2f(state.gps_vel_east, state.gps_vel_north);
    return atanf(waypoint_lateral_accel(state.home_bearing * DEGTORAD, course, nav.speed) / GRAVITY);
  }
  case WING_NAV_RTH_HEADING:
    // Last home bearing and ground speed, steered on the GPS-aided IMU heading.
    return atanf(waypoint_lateral_accel(state.home_bearing * DEGTORAD, state.heading * DEGTORAD, nav.speed) / GRAVITY);
  default:
    return loiter_direction() * FALLBACK_BANK;
  }
}

// Climb angle in radians, positive nose-up.
static float nav_climb_angle(nav_request_t request, wing_nav_state_t next, float dt) {
  if (next == WING_NAV_DESCEND) {
    // A powered descent cannot detect touchdown with GPS and barometer absent.
    // Glide with the motor stopped; do not demand a sink rate that needs power.
    return -DESCEND_GLIDE_ANGLE;
  }
  if (request == NAV_REQUEST_CRUISE)
    return cruise_climb_angle(dt);
  if (request == NAV_REQUEST_MISSION) {
    // Circling a waypoint holds its altitude; a leg changes it evenly, from
    // where the barometer first gives this request an altitude, along the
    // line it started on.
    if (!altitude.target_valid && flags.baro_valid && next == WING_NAV_MISSION)
      nav_route_restart_altitude();
    const mission_waypoint_t &target = route_waypoint();
    const float target_altitude = next == WING_NAV_MISSION
                                      ? nav_leg_altitude(nav_route.altitude_origin, nav_route.leg_altitude, target.coord, target.altitude)
                                      : target.altitude;
    if (altitude.target_valid)
      altitude.target = target_altitude;
    return altitude_climb_angle(request, target_altitude, dt);
  }
  // Loiter holds the altitude it starts at; a return climbs by rth_altitude first.
  const bool returning = request != NAV_REQUEST_LOITER && flags.home_valid;
  return altitude_climb_angle(request, state.altitude + (returning ? MAX(profile.navigation.rth_altitude, 0.0f) : 0.0f), dt);
}

void nav_update_wing(float dt) {
  const uint32_t now_ms = time_millis();
  const nav_request_t request = nav_request();
  flags.guided_ready = nav_guided_ready();
  if (request != NAV_REQUEST_MISSION)
    state.nav_target = NAV_TARGET_HOME;
  else
    state.nav_target = route_guided() ? NAV_TARGET_GUIDED : NAV_TARGET_MISSION;
  nav_update_target();
  // Each request latches its own altitude target; a mission follows its
  // waypoints. A manual RTH that continues as failsafe RTH keeps its target.
  const bool was_returning = nav.request == NAV_REQUEST_RTH || nav.request == NAV_REQUEST_FAILSAFE;
  const bool is_returning = request == NAV_REQUEST_RTH || request == NAV_REQUEST_FAILSAFE;
  if (request != nav.request && !(was_returning && is_returning))
    altitude.target_valid = false;
  nav.request = request;
  flags.wing_nav_failsafe = request == NAV_REQUEST_FAILSAFE;
  flags.mission_engaged = false;
  if (request == NAV_REQUEST_NONE) {
    state.wing_nav_state = WING_NAV_INACTIVE;
    blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 4, state.wing_nav_state);
    return;
  }

  const float max_bank = constrain(profile.navigation.max_bank_angle, 0.0f, MAX_BANK_LIMIT) * DEGTORAD;
  const wing_nav_state_t previous = (wing_nav_state_t)state.wing_nav_state;
  if (previous == WING_NAV_INACTIVE) {
    // Take over from the current attitude and throttle; the slew limits
    // below move them to the navigation targets.
    wing_nav_command.roll = constrain(state.attitude.roll, -max_bank, max_bank);
    wing_nav_command.pitch = state.attitude.pitch;
    wing_nav_command.throttle = state.throttle;
  }
  if (flags.gps_valid)
    nav.speed = hypotf(state.gps_vel_north, state.gps_vel_east);

  if (request == NAV_REQUEST_MISSION && flags.gps_valid && (!route_holding() || route == ROUTE_CLIMBING))
    nav_mission_advance(max_bank);
  const wing_nav_state_t next = nav_next_state(request, now_ms);
  state.wing_nav_state = next;
  // The route, its end action or a ground station return. Losing GPS on it
  // returns on the IMU heading for a while; circling without GPS or without a
  // home is no longer the mission.
  const bool route_flown = route == ROUTE_FLYING || route == ROUTE_CLIMBING || nav_route.returning;
  flags.mission_engaged = nav.mission_requested && route_flown && next != WING_NAV_LOITER && next != WING_NAV_LOITER_BANK;
  if (next != previous)
    nav_enter_state(next, now_ms);

  const float roll = constrain(nav_roll(next), -max_bank, max_bank);
  const float climb_angle = nav_climb_angle(request, next, dt);
  wing_nav_command.roll = slew(wing_nav_command.roll, roll, BANK_SLEW_RATE * dt);
  // Attitude pitch targets are positive nose-down.
  wing_nav_command.pitch = slew(wing_nav_command.pitch, -climb_angle, PITCH_SLEW_RATE * dt);
  const float throttle = constrain(profile.navigation.cruise_throttle + WING_THROTTLE_PER_CLIMB_DEG * climb_angle * RADTODEG, 0.0f, 1.0f);
  wing_nav_command.throttle = next == WING_NAV_DESCEND ? 0.0f : slew(wing_nav_command.throttle, throttle, THROTTLE_SLEW_RATE * dt);

  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 4, state.wing_nav_state);
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 14, (int16_t)(wing_nav_command.pitch * RADTODEG * 10.0f)); // 0.1 deg
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 15, (int16_t)(wing_nav_command.roll * RADTODEG * 10.0f));  // 0.1 deg
}
