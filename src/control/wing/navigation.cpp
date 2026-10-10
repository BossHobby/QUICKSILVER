#include "control/wing/navigation.h"

#include <math.h>

#include "control/control.h"
#include "control/navigation.h"
#include "core/profile.h"
#include "driver/baro/baro.h"
#include "driver/time.h"
#include "io/blackbox.h"
#include "rx/rx.h"
#include "util/util.h"

static constexpr float GRAVITY = 9.80665f;                     // m/s^2
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
static constexpr float ALT_THROTTLE_PER_CLIMB_ANGLE = 0.015f;  // throttle per degree, holds airspeed without a sensor
static constexpr uint32_t RTH_HEADING_TIMEOUT_MS = 10000;      // IMU-heading return after GPS loss
static constexpr float DESCEND_GLIDE_ANGLE = 5.0f * DEGTORAD;  // nose-down during an unpowered failsafe glide
static constexpr uint32_t DESCEND_LATCH_MS = 2000;             // GPS recovery before this resumes the return

typedef enum {
  NAV_REQUEST_NONE,
  NAV_REQUEST_LOITER,
  NAV_REQUEST_RTH,
  NAV_REQUEST_FAILSAFE,
} nav_request_t;

wing_nav_command_t wing_nav_command;

static struct {
  nav_request_t request;
  uint32_t updated_us;
  uint32_t gps_lost_ms; // start of the IMU-heading return
  uint32_t descend_ms;  // start of the failsafe glide
  float speed;          // latest valid GPS ground speed, m/s
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

static float nav_slew(float current, float target, float step) {
  return constrain(target, current - step, current + step);
}

static float loiter_direction() {
  return profile.navigation.loiter_direction == NAV_LOITER_LEFT ? -1.0f : 1.0f;
}

static nav_request_t nav_request() {
  // Launch owns attitude and throttle until it finishes; navigation never
  // starts on the ground. Failsafe ignores the switches, which hold stale values.
  const bool launching = state.wing_launch_available && state.wing_launch_state < WING_LAUNCH_DONE;
  if (!flags.arm_state || !flags.in_air || launching)
    return NAV_REQUEST_NONE;
  if (flags.failsafe)
    return profile.navigation.rth_on_failsafe ? NAV_REQUEST_FAILSAFE : NAV_REQUEST_NONE;
  if (rx_aux_on(AUX_RETURN_TO_HOME))
    return NAV_REQUEST_RTH;
  if (rx_aux_on(AUX_LOITER))
    return NAV_REQUEST_LOITER;
  return NAV_REQUEST_NONE;
}

static wing_nav_state_t nav_next_state(nav_request_t request, bool gps_valid, bool home_valid, uint32_t now_ms) {
  const wing_nav_state_t previous = (wing_nav_state_t)state.wing_nav_state;
  // A brief GPS quality dip cannot have landed the aircraft, so recovery
  // resumes the return. After a longer glide, GPS recovery alone cannot tell
  // whether it has landed; keep the motor off until the pilot recovers control.
  if (request == NAV_REQUEST_FAILSAFE && previous == WING_NAV_DESCEND && now_ms - nav.descend_ms >= DESCEND_LATCH_MS)
    return WING_NAV_DESCEND;
  const bool returning = request != NAV_REQUEST_LOITER && home_valid;
  if (gps_valid) {
    if (returning) {
      if (previous == WING_NAV_RTH_HOME || state.home_distance <= profile.navigation.loiter_radius)
        return WING_NAV_RTH_HOME;
      return WING_NAV_RTH_RETURN;
    }
    // Failsafe without a home descends; a manual RTH without one loiters in place.
    return request == NAV_REQUEST_FAILSAFE ? WING_NAV_DESCEND : WING_NAV_LOITER;
  }

  if (returning && state.heading_confidence >= RTH_MIN_HEADING_CONFIDENCE) {
    if (previous == WING_NAV_RTH_RETURN || previous == WING_NAV_RTH_HOME) {
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

static void circle_place_home() {
  circle.origin = state.gps_home;
  circle.center_north = 0.0f;
  circle.center_east = 0.0f;
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
  const float l1_distance = MAX(L1_DAMPING * L1_PERIOD * speed / M_PI_F, 1.0f);

  // Capture: steer the velocity vector toward the center from outside.
  const float cross_velocity = radial_north * vel_east - radial_east * vel_north;
  const float inward_velocity = -(radial_north * vel_north + radial_east * vel_east);
  const float nu = constrain(atan2f(cross_velocity, inward_velocity), -M_PI_F / 2.0f, M_PI_F / 2.0f);
  const float capture = 4.0f * L1_DAMPING * L1_DAMPING * speed * speed / l1_distance * sinf(nu);

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
  const float l1_distance = MAX(L1_DAMPING * L1_PERIOD * speed / M_PI_F, 1.0f);
  // Signed bearing error in -pi..pi; normalize_rad() wraps to 0..2pi.
  const float error = bearing - course;
  const float nu = constrain(atan2f(sinf(error), cosf(error)), -M_PI_F / 2.0f, M_PI_F / 2.0f);
  return 4.0f * L1_DAMPING * L1_DAMPING * speed * speed / l1_distance * sinf(nu);
}

// Holds the target latched when the barometer first becomes usable for this
// request. Returns the climb angle in radians, positive nose-up.
static float altitude_climb_angle(float dt, float entry_target) {
  const bool baro_ok = state.baro_valid && time_millis() - state.baro_last_update_ms <= BARO_STALE_MS;
  if (!baro_ok) {
    if (nav.request == NAV_REQUEST_LOITER)
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

void nav_update_wing(bool gps_valid, bool home_valid) {
  const uint32_t now = time_micros();
  const uint32_t now_ms = time_millis();
  const float dt = constrain((now - nav.updated_us) * 1e-6f, 0.0f, 0.1f);
  nav.updated_us = now;

  // A sustained turn's centripetal acceleration reads as level, so the gravity
  // estimate drifts in bank and pitch in every flight mode. The IMU rotates
  // this acceleration by heading, so publish only while heading is trusted.
  if (flags.arm_state && flags.in_air && gps_valid && state.heading_confidence >= RTH_MIN_HEADING_CONFIDENCE)
    nav_update_gps_accel();
  else
    nav_reset_gps_accel();

  const nav_request_t request = nav_request();
  // Loiter and return latch their own altitude targets. A manual RTH that
  // continues as failsafe RTH keeps its target.
  const bool was_returning = nav.request == NAV_REQUEST_RTH || nav.request == NAV_REQUEST_FAILSAFE;
  const bool is_returning = request == NAV_REQUEST_RTH || request == NAV_REQUEST_FAILSAFE;
  if (request != nav.request && !(was_returning && is_returning))
    altitude.target_valid = false;
  nav.request = request;
  state.wing_nav_failsafe = request == NAV_REQUEST_FAILSAFE;
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
  if (gps_valid)
    nav.speed = hypotf(state.gps_vel_north, state.gps_vel_east);

  const wing_nav_state_t next = nav_next_state(request, gps_valid, home_valid, now_ms);
  state.wing_nav_state = next;
  if (next == WING_NAV_LOITER && previous != WING_NAV_LOITER)
    circle_place_ahead();
  if (next == WING_NAV_RTH_HOME && previous != WING_NAV_RTH_HOME)
    circle_place_home();
  if (next == WING_NAV_DESCEND && previous != WING_NAV_DESCEND)
    nav.descend_ms = now_ms;

  float roll = loiter_direction() * FALLBACK_BANK;
  switch (next) {
  case WING_NAV_LOITER:
  case WING_NAV_RTH_HOME:
    roll = atanf(circle_lateral_accel() / GRAVITY);
    break;
  case WING_NAV_RTH_RETURN: {
    const float course = atan2f(state.gps_vel_east, state.gps_vel_north);
    roll = atanf(waypoint_lateral_accel(state.home_bearing * DEGTORAD, course, nav.speed) / GRAVITY);
    break;
  }
  case WING_NAV_RTH_HEADING:
    // Last home bearing and ground speed, steered on the GPS-aided IMU heading.
    roll = atanf(waypoint_lateral_accel(state.home_bearing * DEGTORAD, state.heading * DEGTORAD, nav.speed) / GRAVITY);
    break;
  default:
    break;
  }

  const bool returning = request != NAV_REQUEST_LOITER && home_valid;
  const float entry_target = state.altitude + (returning ? MAX(profile.navigation.rth_altitude, 0.0f) : 0.0f);
  const bool descend = next == WING_NAV_DESCEND;
  // A powered descent cannot detect touchdown with GPS and barometer absent.
  // Glide with the motor stopped; do not demand a sink rate that needs power.
  const float climb_angle = descend ? -DESCEND_GLIDE_ANGLE : altitude_climb_angle(dt, entry_target);

  roll = constrain(roll, -max_bank, max_bank);
  wing_nav_command.roll = nav_slew(wing_nav_command.roll, roll, BANK_SLEW_RATE * dt);
  // Attitude pitch targets are positive nose-down.
  wing_nav_command.pitch = nav_slew(wing_nav_command.pitch, -climb_angle, PITCH_SLEW_RATE * dt);
  const float throttle = constrain(profile.navigation.cruise_throttle + ALT_THROTTLE_PER_CLIMB_ANGLE * climb_angle * RADTODEG, 0.0f, 1.0f);
  wing_nav_command.throttle = descend ? 0.0f : nav_slew(wing_nav_command.throttle, throttle, THROTTLE_SLEW_RATE * dt);

  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 4, state.wing_nav_state);
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 14, (int16_t)(wing_nav_command.pitch * RADTODEG * 10.0f)); // 0.1 deg
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 15, (int16_t)(wing_nav_command.roll * RADTODEG * 10.0f));  // 0.1 deg
}
