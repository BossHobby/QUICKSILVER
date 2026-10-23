#include "navigation.h"

#include <math.h>

#include "control/control.h"
#include "control/imu.h"
#include "control/mission.h"
#include "control/navigation.h"
#include "core/profile.h"
#include "io/blackbox.h"
#include "rx/rx.h"
#include "util/util.h"
#include "util/vector.h"

// Vertical control.
static constexpr float ALT_KP = 0.15f;               // relative to the estimated level hover throttle
static constexpr float ALT_KI = 0.05f;
static constexpr float ALT_POSITION_KP = 0.5f;       // m/s per meter of altitude error
static constexpr float ALT_MAX_RATE = 2.0f;          // m/s climb or descent
static constexpr float ALT_HOLD_BAND = 1.0f;         // meters, the altitude needs no tighter hold
static constexpr float ALT_HOLD_SCALE = 0.3f;        // position gain within the band
static constexpr float ALT_ACCEL_LIMIT = 1.0f;       // m/s^2 target vertical acceleration
static constexpr float ALT_BRAKE_ACCEL_LIMIT = 5.0f; // m/s^2, stopping a climb or descent is never smoothed as gently
static constexpr float ALT_REACHED = 1.0f;           // meters
static constexpr float THROTTLE_HEADROOM = 0.15f;    // kept above the navigation throttle for attitude authority
static constexpr float THROTTLE_SLEW_RATE = 1.5f;    // per second

// Hover throttle learning.
static constexpr float HOVER_LEARN_TIME = 3.0f;         // seconds, estimate time constant
static constexpr float HOVER_LEARN_MIN_TIME = 2.0f;     // seconds of steady flight before the estimate is used
static constexpr float HOVER_LEARN_MIN_TILT = 0.94f;    // cos(20 degrees)
static constexpr float HOVER_LEARN_MAX_SPEED = 0.5f;    // m/s vertical
static constexpr float HOVER_LEARN_MIN_ALTITUDE = 1.5f; // meters above launch, clear of the ground and ground effect
static constexpr float HOVER_LEARN_MAX_ERROR = 0.1f;    // trust learned hover only this close to the takeover estimate

// Horizontal velocity control.
static constexpr float VEL_KP = 0.08f;
static constexpr float VEL_KI = 0.02f;
static constexpr float VEL_ACCEL_LIMIT = 2.0f;         // m/s^2 desired velocity change
static constexpr float VEL_AWAY_DECAY_RATE = 2.0f;     // per second, integral decay while moving away
static constexpr float VEL_AWAY_SPEED = 0.5f;          // m/s away from the target, slower is a hold's creep
static constexpr float BRAKE_ACCEL = 1.5f;             // m/s^2, conservative approach deceleration
static constexpr float BRAKE_DELAY = 4.0f;             // seconds, includes the velocity PI controller's response
static constexpr float POSITION_KP = 0.5f;             // m/s per meter near the target
static constexpr float COMMAND_TIME_CONSTANT = 0.15f;  // seconds, smooth GPS-driven tilt changes
static constexpr float ANGLE_SLEW_RATE = 2.0f;         // normalized stick/s
static constexpr float MIN_ANGLE_AUTHORITY = 0.25f;    // share of the tilt limit without heading confidence

// Along-track velocity gains at cruise speed, relative to VEL_KP and VEL_KI,
// ramped in between the two speeds. Slow flight and cross-track keep the full
// gains. An integral as fast as the reduced proportional term makes the speed
// loop resonate, pitching back and forth over seconds; it trims slowly within
// VEL_CRUISE_KI_BAND and at the full rate beyond.
static constexpr float VEL_CRUISE_KP_SCALE = 0.5f;
static constexpr float VEL_CRUISE_KI_SCALE = 0.25f;
static constexpr float VEL_CRUISE_KI_BAND = 1.0f;    // m/s along-track
static constexpr float VEL_CRUISE_SPEED_MIN = 2.0f;  // m/s
static constexpr float VEL_CRUISE_SPEED_FULL = 5.0f; // m/s

// Steady cruise moves the along-track integral into a drag trim over this
// time. Headwind learns as drag too; the integral corrects it on other legs.
static constexpr float DRAG_LEARN_TIME = 10.0f; // seconds
static constexpr float DRAG_LEARN_ERROR = 1.0f; // m/s along-track
static constexpr float DRAG_TRIM_MAX = 0.1f;    // radians per m/s

// Yaw toward the target.
static constexpr float YAW_KP = 1.0f;
static constexpr float YAW_MAX_RATE = 30.0f;       // degrees/s
static constexpr float YAW_DEADBAND = 2.0f;        // degrees
static constexpr float YAW_TIME_CONSTANT = 0.03f;  // seconds
// GPS noise swings the target bearing close in; hold yaw rather than chase it.
static constexpr float YAW_HOLD_RADIUS = 8.0f; // meters

// Phases.
static constexpr float ARRIVAL_RADIUS = 3.0f; // meters
static constexpr float TURN_ERROR = 10.0f;    // degrees of heading error that count as aligned
static constexpr float TURN_RATE = 5.0f;      // degrees/s of yaw rate that count as settled
// A half-turn at the yaw-rate limit plus ten seconds to settle.
static constexpr uint32_t TURN_TIMEOUT_MS = 10000U + (uint32_t)(180000.0f / YAW_MAX_RATE);
// A turn counts as aligned inside the yaw hold radius. Realign only well
// beyond it, so position wander near that radius keeps navigating.
static constexpr float REALIGN_ERROR = 45.0f;                  // degrees
static constexpr float REALIGN_RADIUS = 2.0f * YAW_HOLD_RADIUS; // meters
static constexpr float PROGRESS_DISTANCE = 5.0f;               // meters
static constexpr uint32_t PROGRESS_TIMEOUT_MS = 15000U;
static constexpr float CLIMB_PROGRESS = 0.5f; // meters
static constexpr uint32_t CLIMB_TIMEOUT_MS = 10000U;
// Level altitude hold before giving up, so the sensor or the link can return.
static constexpr uint32_t SENSOR_LOSS_TIMEOUT_MS = 10000U;
static constexpr uint32_t FAILURE_HOLD_MS = 10000U;

// Heading acquisition flies forward until GPS course yields a heading. GPS
// course follows the nose once drift no longer dominates the run. Wind can
// keep the drift up, so leveling is bounded.
static constexpr uint32_t ACQUIRE_TIMEOUT_MS = 15000U;
static constexpr float ACQUIRE_DISTANCE = 75.0f; // meters
static constexpr float ACQUIRE_PITCH = 35.0f;    // degrees
static constexpr float ACQUIRE_SLEW_RATE = 0.2f; // normalized stick/s
static constexpr float ACQUIRE_CONFIDENCE = 0.6f;
static constexpr float ACQUIRE_START_SPEED = 3.0f; // m/s
static constexpr uint32_t ACQUIRE_LEVEL_TIMEOUT_MS = 5000U;

// Past 90 degrees of heading error, tilt pushes the vehicle away from the
// desired velocity and the loop diverges sideways or backward.
static constexpr float DIVERGE_SPEED = 3.0f; // m/s across or against the desired velocity
static constexpr uint32_t DIVERGE_TIMEOUT_MS = 3000U;

static constexpr float MISSION_START_MIN_ALTITUDE = 2.0f; // meters above launch

// Leg lines steer toward a carrot this far ahead on the line, and never
// closer than the line itself, so an intercept is at most 45 degrees.
static constexpr float CARROT_LEAD_MIN = 6.0f;  // meters
static constexpr float CARROT_LEAD_TIME = 1.5f; // seconds at the desired speed
static constexpr float CARROT_LEAD_MAX = 30.0f; // meters

// A PASS corner sheds the same velocity change, 2*v*sin(turn/2), at any angle.
static constexpr float PASS_CORNER_DELTA_V = 4.0f;   // m/s
static constexpr float PASS_CORNER_SPEED_MIN = 2.0f; // m/s
// The next leg begins this far ahead of the waypoint at the corner speed.
static constexpr float PASS_SWITCH_TIME = 1.5f;        // seconds
static constexpr float PASS_SWITCH_RADIUS_MAX = 25.0f; // meters
// Through a corner the nose follows the new leg without stopping to turn;
// realigning resumes once it points along it.
static constexpr float PASS_TURN_DONE = 15.0f; // degrees

// TAKEOFF starts on a throttle stick raised to here, so taking control back
// later never hands over an idle throttle.
static constexpr float TAKEOFF_STICK_MIN = 0.4f;
// Roll or pitch beyond this hands control back during the takeoff and the
// heading run after it, as for a wing launch; not within the grace time after
// the throttle raise.
static constexpr float TAKEOFF_STICK_ABORT = 0.15f;
static constexpr uint32_t TAKEOFF_STICK_GRACE_MS = 300;
static constexpr float TAKEOFF_SPINUP_FRACTION = 0.8f; // of the hover throttle, reached in TAKEOFF_SPINUP_TIME
static constexpr float TAKEOFF_SPINUP_TIME = 1.0f;     // seconds
static constexpr float TAKEOFF_RAMP_RATE = 0.1f;       // throttle/s beyond that until lift-off
// Lift-off: climbing this fast for this long, or this far above the spin-up altitude.
static constexpr float TAKEOFF_LIFTOFF_SPEED = 0.3f;  // m/s up
static constexpr float TAKEOFF_LIFTOFF_TIME = 0.2f;   // seconds
static constexpr float TAKEOFF_LIFTOFF_HEIGHT = 1.0f; // meters
static constexpr float TAKEOFF_TIP_ANGLE = 30.0f * DEGTORAD;
// Prop wash can read on the barometer as a lift-off; until the climb is this
// far above the spin-up altitude, tipping over still stops the motors.
static constexpr float TAKEOFF_TIP_HEIGHT = 2.0f * TAKEOFF_LIFTOFF_HEIGHT; // meters

// Descend fast to this height above the LAND ground, then slowly.
static constexpr float LAND_SLOW_HEIGHT = 5.0f; // meters
static constexpr float LAND_FINAL_RATE = 0.5f;  // m/s
// The altitude target stays this far below the craft: touchdown, not an
// altitude, ends the descent, so a wrong ground height cannot stop it.
static constexpr float LAND_TARGET_DEPTH = 10.0f; // meters
// Half the tilt authority this close to the ground.
static constexpr float LAND_LOW_HEIGHT = 1.5f; // meters
// Throttle floor while landing, so the trim can wind down onto the ground.
static constexpr float LAND_THROTTLE_MIN = 0.05f;
// Touchdown follows ArduPilot's land detector: throttle at its floor while
// level and still. The altitude loop alone takes tens of seconds to trim down
// once the ground stops the descent, so after LAND_MAYBE_TIME of level, still
// flight the trim winds down toward the floor; at the floor, LAND_DETECT_TIME
// of the same disarms. Aloft, winding down starts a descent the checks see.
static constexpr float LAND_MAYBE_TIME = 0.2f;         // seconds
static constexpr float LAND_DETECT_TIME = 1.0f;        // seconds
static constexpr float LAND_WIND_DOWN_RATE = 0.5f;     // throttle/s
static constexpr float LAND_ACCEL_MAX = 1.0f;          // m/s^2, 1 Hz low-passed
static constexpr float LAND_ACCEL_FILTER_TIME = 0.16f; // seconds, 1 Hz
// Unlike ArduPilot, landed-maybe precedes the throttle floor, so it also
// needs the commanded descent to have stopped: a steady descent is level and
// still too.
static constexpr float LAND_STOPPED_SPEED = 0.2f;      // m/s vertical, below LAND_FINAL_RATE
static constexpr float LAND_ANGLE_REQUEST_MAX = 15.0f; // degrees commanded
static constexpr float LAND_ANGLE_ERROR_MAX = 0.5f;    // sin(30 degrees) between commanded and estimated gravity
// Ground too steep or rough for the level and still checks: a descent stopped
// for LAND_STOPPED_TIME also winds the trim down, and below this share of the
// hover throttle without vertical motion for LAND_GROUNDED_TIME the ground
// carries the craft, so it disarms. Aloft, so little thrust sinks it.
static constexpr float LAND_STOPPED_TIME = 1.0f; // seconds
static constexpr float LAND_START_TIME = 1.5f;   // seconds for the descent to pass LAND_STOPPED_SPEED
// Descending less than this for LAND_STALL_MS, the ground holds the craft. On
// a slope the estimated vertical speed takes seconds to settle; the barometer
// shows the stall at once.
static constexpr float LAND_PROGRESS = 0.2f; // meters
static constexpr uint32_t LAND_STALL_MS = 2000U;
static constexpr float LAND_GROUNDED_THROTTLE = 0.5f;
static constexpr float LAND_GROUNDED_TIME = 3.0f; // seconds

// Sensor and input history survives RTH stop/start. Published outputs live in state.
static struct {
  bool mission_switch;      // position in the previous update; turning it on starts a mission
  float hover_learned_time; // seconds of steady flight in state.hover_throttle
  float drag_trim;          // radians of tilt per m/s of desired velocity, learned in cruise
  bool guided_granted;      // mission switch turned on in flight; grants ground station commands
  bool takeoff_done;        // a mission takeoff started since arming; it runs at most once per arm
  struct {
    bool active;
    float altitude; // meters above launch, latched on engagement
  } hold;
} nav;

// Last GPS velocity sample; the derived acceleration is published in state.
static struct {
  float velocity_north;
  float velocity_east;
  bool sample_valid;
  uint32_t updated_ms;
} gps_accel;

// Vertical controller history. Shared by the failsafe hold and RTH so a
// failsafe that escalates to RTH keeps its trim.
static struct {
  float integral;
  float throttle_command;
  float desired_rate;
} alt;

typedef enum {
  NAV_HORIZONTAL_IDLE,     // level; no velocity request seeded from the ground track
  NAV_HORIZONTAL_STEERING, // flying toward the target
  NAV_HORIZONTAL_BRAKING,  // slowing the velocity request to a stop
  NAV_HORIZONTAL_HOLDING,  // holding hold_coord, where the brake came to rest
} nav_horizontal_phase_t;

// Controller history is reset on every engagement.
static struct {
  // Every phase holds this altitude above launch; TURN and NAVIGATE change
  // to it evenly along the leg.
  float target_altitude;
  // Start of the phase, or of its latest progress toward its goal.
  uint32_t progress_ms;
  float best_error; // meters: altitude error in CLIMB, distance to the target otherwise
  // A divergence relearned the heading since the last waypoint was reached;
  // diverging again before the next one fails instead of relearning forever.
  bool relearned;
  struct {
    bool leveling; // drag slows the vehicle before the run starts
    gps_coord_t origin;
  } acquisition;
  struct {
    nav_horizontal_phase_t phase;
    gps_coord_t hold_coord;
    float integral_north;
    float integral_east;
    float desired_velocity_north;
    float desired_velocity_east;
    float roll_command;
    float pitch_command;
    bool diverging;        // moving off the desired velocity
    uint32_t diverging_ms; // start of the current divergence window
    float diverging_speed; // ground speed at that start
  } horizontal;
  // How the leg to the current target is flown.
  struct {
    bool line;           // follow the line from nav_route.leg_origin rather than aim at the target
    float dir_north;     // unit vector along the line
    float dir_east;
    float end_speed;     // m/s at the arrival radius; zero stops on the target
    float arrive_radius; // meters
    bool passing;        // entered through a PASS corner: no realign until the nose follows the line
  } leg;
  // Updated once per pass toward the current target.
  struct {
    float north; // meters to the target
    float east;
    float distance;
    float dir_north; // unit direction steering flies, toward the carrot or the target
    float dir_east;
    float course; // degrees of that direction; the nose follows it
  } steer;
  struct {
    uint32_t started_ms; // throttle stick raised
    float throttle;      // spin-up command
    float altitude;      // meters above launch where spin-up began
    float rising;        // seconds climbing faster than TAKEOFF_LIFTOFF_SPEED
    bool sticks_abort;   // roll or pitch hands back, through the heading run after the climb
  } takeoff;
  struct {
    float ground;         // meters above launch
    float hover;          // trimmed hover throttle when the landing began
    vec3_t accel;         // m/s^2 body-frame acceleration, 1 Hz low-passed
    float quiet;          // seconds level and still; negative while the descent starts
    float landed;         // seconds of that at the throttle floor
    float stopped;        // seconds without vertical motion
    float grounded;       // seconds of that at little throttle
  } land;
  struct {
    bool active;
    uint32_t started_ms;
    float altitude; // meters above launch, latched when a sensor is lost
  } pause;
} rth;

static float nav_tilt() {
  return MAX(cosf(state.attitude.roll) * cosf(state.attitude.pitch), 0.5f);
}

static bool nav_hover_learned() {
  return nav.hover_learned_time >= HOVER_LEARN_MIN_TIME;
}

// Learn level-hover throttle while the pilot flies steadily. Unlike a
// configured value, this follows battery sag and payload changes.
static void nav_update_hover(float dt) {
  if (!flags.arm_state) {
    // The next flight may have a different battery or payload.
    nav.hover_learned_time = 0.0f;
    state.hover_throttle = 0.0f;
    return;
  }
  // in_air and on_ground do not prove the vehicle has lifted off; low
  // throttle on the ground would otherwise learn as a low hover.
  const bool airborne = flags.in_air && !flags.on_ground &&
                        flags.baro_valid && state.altitude > HOVER_LEARN_MIN_ALTITUDE;
  const float tilt = cosf(state.attitude.roll) * cosf(state.attitude.pitch);
  const bool steady = airborne && !flags.controls_override && !flags.failsafe &&
                      state.throttle > 0.05f && state.throttle < 0.95f &&
                      tilt > HOVER_LEARN_MIN_TILT &&
                      fabsf(state.vertical_speed) < HOVER_LEARN_MAX_SPEED;
  if (!steady)
    return;
  const float sample = state.throttle * tilt;
  if (nav.hover_learned_time <= 0.0f)
    state.hover_throttle = sample;
  else
    state.hover_throttle += dt / (HOVER_LEARN_TIME + dt) * (sample - state.hover_throttle);
  nav.hover_learned_time = MIN(nav.hover_learned_time + dt, HOVER_LEARN_MIN_TIME);
}

// Configured throttle range, below the headroom kept for attitude authority.
static float nav_throttle_min() {
  return constrain(profile.navigation.rth_throttle_min, 0.0f, 1.0f);
}

static float nav_throttle_max(float throttle_min = nav_throttle_min()) {
  return constrain(profile.navigation.rth_throttle_max, throttle_min, MAX(throttle_min, 1.0f - THROTTLE_HEADROOM));
}

static float nav_hover_throttle() {
  const float hover = nav_hover_learned() ? state.hover_throttle : profile.navigation.rth_throttle_hover;
  return constrain(hover, nav_throttle_min(), nav_throttle_max());
}

// Take over vertical control from the currently applied throttle.
static void nav_altitude_start(float throttle) {
  const float throttle_min = nav_throttle_min();
  const float throttle_max = nav_throttle_max();
  alt.throttle_command = constrain(throttle, throttle_min, throttle_max);
  // Infer hover from the applied throttle and the acceleration it produces
  // (thrust ~ throttle), so a dive does not read as a low hover.
  const float accel_ratio = constrain(1.0f + state.vertical_accel / GRAVITY, 0.5f, 1.5f);
  const float hover = constrain(throttle * nav_tilt() / accel_ratio, throttle_min, throttle_max);
  if (nav_hover_learned() && fabsf(state.hover_throttle - hover) <= HOVER_LEARN_MAX_ERROR) {
    // A learned hover that agrees with the takeover is the steadier trim.
    alt.integral = 0.0f;
  } else {
    alt.integral = hover - nav_hover_throttle();
  }
  alt.desired_rate = constrain(state.vertical_speed, -ALT_MAX_RATE, ALT_MAX_RATE);
  state.rx_override.throttle = alt.throttle_command;
}

// target_rate is how fast a moving target changes, flown ahead of the error.
// The throttle never goes below throttle_min: rth_throttle_min, or the lower
// landing floor.
static void nav_update_altitude_control_above(float target_alt, float dt, float target_rate, float max_rate, float throttle_min) {
  const float throttle_max = nav_throttle_max(throttle_min);
  const float throttle_hover = constrain(nav_hover_throttle(), throttle_min, throttle_max);

  // Within ALT_HOLD_BAND, close the error gently instead of chasing barometer
  // noise with throttle; beyond it, the full gain applies to the excess.
  const float error = target_alt - state.altitude;
  const float hold_error = error - (1.0f - ALT_HOLD_SCALE) * constrain(error, -ALT_HOLD_BAND, ALT_HOLD_BAND);
  // Without altitude, hold vertical speed at zero on the accelerometer estimate.
  const float desired_rate = flags.baro_valid ? constrain(target_rate + ALT_POSITION_KP * hold_error, -max_rate, max_rate) : 0.0f;
  // A takeover while climbing hard otherwise keeps climbing until the slew passes zero.
  const bool braking = (alt.desired_rate < 0.0f && desired_rate > alt.desired_rate) ||
                       (alt.desired_rate > 0.0f && desired_rate < alt.desired_rate);
  alt.desired_rate = slew(alt.desired_rate, desired_rate, (braking ? ALT_BRAKE_ACCEL_LIMIT : ALT_ACCEL_LIMIT) * dt);
  const float rate_error = alt.desired_rate - state.vertical_speed;

  // A low-hover-throttle craft gets more acceleration per throttle unit.
  // Scale the velocity gains with the trimmed hover estimate to compensate.
  const float tilt = nav_tilt();
  const float effective_hover = constrain(throttle_hover + alt.integral, MAX(throttle_min, 0.05f), throttle_max);
  const auto throttle_for = [&](float integral) {
    return (throttle_hover + ALT_KP * effective_hover * rate_error + integral) / tilt;
  };
  // Allow the integral to compensate a wrong hover setting across the usable
  // throttle range. A fixed +/-0.2 limit cannot trim 0.5 down to a 0.2 hover.
  const float integral = constrain(alt.integral + ALT_KI * effective_hover * rate_error * dt,
                                   throttle_min * tilt - throttle_hover,
                                   throttle_max * tilt - throttle_hover);
  // The slew limiter is an actuator limit too. Do not wind up while the
  // applied throttle is still catching up with the proportional correction.
  // Without a barometer the velocity estimate drifts, so freeze the trim.
  const float throttle_step = THROTTLE_SLEW_RATE * dt;
  const float applied_min = constrain(alt.throttle_command - throttle_step, throttle_min, throttle_max);
  const float applied_max = constrain(alt.throttle_command + throttle_step, throttle_min, throttle_max);
  const float throttle = throttle_for(integral);
  if (flags.baro_valid &&
      ((throttle >= applied_min && throttle <= applied_max) ||
       (throttle > applied_max && rate_error < 0.0f) ||
       (throttle < applied_min && rate_error > 0.0f))) {
    alt.integral = integral;
  }
  alt.throttle_command = constrain(throttle_for(alt.integral), applied_min, applied_max);
  state.rx_override.throttle = alt.throttle_command;
}

static void nav_update_altitude_control(float target_alt, float dt, float target_rate = 0.0f) {
  nav_update_altitude_control_above(target_alt, dt, target_rate, ALT_MAX_RATE, nav_throttle_min());
}

static float nav_arrive_radius() {
  return MAX(rth.leg.arrive_radius, ARRIVAL_RADIUS);
}

// m/s for a mission_waypoint_t.speed; zero uses the configured cruise speed.
static float nav_cruise_speed(uint8_t waypoint_speed) {
  return MAX(waypoint_speed ? waypoint_speed * MISSION_SPEED_SCALE : profile.navigation.rth_cruise_speed, 0.1f);
}

// The fastest approach that still slows to end_speed over distance, reserving
// response time as well as braking distance: d = v*t + (v^2 - end^2)/(2*a).
static float nav_braking_speed(float distance, float end_speed) {
  const float delay_speed = BRAKE_ACCEL * BRAKE_DELAY;
  return sqrtf(delay_speed * delay_speed + end_speed * end_speed + 2.0f * BRAKE_ACCEL * distance) - delay_speed;
}

// A PASS waypoint ends its leg at the corner speed for the turn onto the next
// one, handing over early by the corner speed.
static void nav_latch_corner() {
  const uint8_t index = nav_route.index;
  const mission_waypoint_t &waypoint = mission.waypoints[index];
  if (waypoint.type != MISSION_WAYPOINT_PASS || index + 1 >= mission.count || !rth.leg.line)
    return;
  const mission_waypoint_t &next = mission.waypoints[index + 1];
  float north, east;
  nav_position_delta(waypoint.coord, next.coord, &north, &east);
  const float length = sqrtf(north * north + east * east);
  if (length < 1.0f)
    return;
  const float cos_turn = (rth.leg.dir_north * north + rth.leg.dir_east * east) / length;
  const float shed = sqrtf(MAX(2.0f - 2.0f * cos_turn, 0.0f)); // 2 sin(turn / 2)
  float speed = MIN(nav_cruise_speed(waypoint.speed), nav_cruise_speed(next.speed));
  if (shed > 0.05f)
    speed = constrain(PASS_CORNER_DELTA_V / shed, MIN(PASS_CORNER_SPEED_MIN, speed), speed);
  // The next leg stops on its waypoint unless it passes through it too.
  if (next.type != MISSION_WAYPOINT_PASS || index + 2 >= mission.count)
    speed = MIN(speed, nav_braking_speed(length, 0.0f));
  rth.leg.end_speed = speed;
  rth.leg.arrive_radius = MAX(ARRIVAL_RADIUS, MIN(speed * PASS_SWITCH_TIME, MIN(PASS_SWITCH_RADIUS_MAX, 0.5f * length)));
}

// Latch how the leg to the current target is flown: along its line from
// nav_route.leg_origin, and how it ends. Home, and a target closer than a
// carrot lead, is aimed at directly.
static void nav_latch_leg() {
  rth.leg = {.arrive_radius = ARRIVAL_RADIUS};
  if (state.nav_target == NAV_TARGET_HOME)
    return;
  float north, east;
  nav_position_delta(nav_route.leg_origin, nav_target_coord(), &north, &east);
  const float length = sqrtf(north * north + east * east);
  rth.leg.line = length > CARROT_LEAD_MIN;
  if (rth.leg.line) {
    rth.leg.dir_north = north / length;
    rth.leg.dir_east = east / length;
  }
  if (state.nav_target == NAV_TARGET_MISSION)
    nav_latch_corner();
}

// Start the leg to the current target here.
static void nav_start_leg() {
  nav_route_start_leg();
  nav_latch_leg();
}

// Steer along the leg line toward a carrot one lead ahead of the closest
// point on it, or straight at the target within a lead of it.
static void nav_update_course() {
  float north, east;
  nav_position_delta(state.gps_coord, nav_target_coord(), &north, &east);
  rth.steer.north = north;
  rth.steer.east = east;
  rth.steer.distance = sqrtf(north * north + east * east);

  float aim_north = north, aim_east = east;
  bool carrot = false;
  if (rth.leg.line) {
    const float along = north * rth.leg.dir_north + east * rth.leg.dir_east;
    const float cross_north = north - along * rth.leg.dir_north;
    const float cross_east = east - along * rth.leg.dir_east;
    const float speed = hypotf(rth.horizontal.desired_velocity_north, rth.horizontal.desired_velocity_east);
    const float lead = MAX(constrain(speed * CARROT_LEAD_TIME, CARROT_LEAD_MIN, CARROT_LEAD_MAX),
                           hypotf(cross_north, cross_east));
    carrot = along > lead;
    if (carrot) {
      aim_north = cross_north + rth.leg.dir_north * lead;
      aim_east = cross_east + rth.leg.dir_east * lead;
    }
  }
  const float aim = hypotf(aim_north, aim_east);
  rth.steer.dir_north = aim > 0.01f ? aim_north / aim : 0.0f;
  rth.steer.dir_east = aim > 0.01f ? aim_east / aim : 0.0f;
  // Aimed at the target, the nose follows its published bearing.
  rth.steer.course = carrot ? normalize_rad(atan2f(aim_east, aim_north)) * RADTODEG : state.nav_target_bearing;
}

static float nav_heading_error() {
  return remainderf(rth.steer.course - state.heading, 360.0f);
}

static void nav_update_yaw(float distance, float dt) {
  // RTH supplies radians/s directly; rx_override.yaw must bypass pilot rates.
  state.rx_override.yaw = 0.0f;
  if (distance <= YAW_HOLD_RADIUS) {
    state.rth_yaw_rate = 0.0f;
    return;
  }
  float heading_error = nav_heading_error();
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 9, (int16_t)(heading_error * 10)); // 0.1 degrees
  if (fabsf(heading_error) < YAW_DEADBAND)
    heading_error = 0.0f;
  // Yaw must work while stationary; GPS ground speed does not limit yaw authority.
  const float max_yaw_rate = YAW_MAX_RATE * DEGTORAD;
  const float yaw_rate = constrain(heading_error * DEGTORAD * YAW_KP, -max_yaw_rate, max_yaw_rate);
  state.rth_yaw_rate += dt / (YAW_TIME_CONSTANT + dt) * (yaw_rate - state.rth_yaw_rate);
}

// Slew the desired earth-frame velocity toward the request and fly it with
// tilt. Navigation calls this every pass it steers, so the tilt and the
// wind and drag trim carry across state changes.
static void nav_update_velocity_control(float request_north, float request_east, bool moving_away, float dt) {
  auto &horizontal = rth.horizontal;
  const float velocity_step = VEL_ACCEL_LIMIT * dt;
  horizontal.desired_velocity_north = slew(horizontal.desired_velocity_north, request_north, velocity_step);
  horizontal.desired_velocity_east = slew(horizontal.desired_velocity_east, request_east, velocity_step);
  const float desired_north = horizontal.desired_velocity_north;
  const float desired_east = horizontal.desired_velocity_east;
  const float error_north = desired_north - state.gps_vel_north;
  const float error_east = desired_east - state.gps_vel_east;

  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 10, (int16_t)(desired_north * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 11, (int16_t)(desired_east * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 12, (int16_t)(state.gps_vel_north * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 13, (int16_t)(state.gps_vel_east * 10));

  if (moving_away) {
    const float decay = expf(-VEL_AWAY_DECAY_RATE * dt);
    horizontal.integral_north *= decay;
    horizontal.integral_east *= decay;
  }
  float authority = MIN_ANGLE_AUTHORITY + (1.0f - MIN_ANGLE_AUTHORITY) * state.heading_confidence;
  // Lean less close to the landing ground, so touchdown does not catch a leg.
  if (state.rth_state == RTH_STATE_LAND && state.altitude - rth.land.ground < LAND_LOW_HEIGHT)
    authority *= 0.5f;
  const float max_angle = MAX(profile.rate.level_max_angle, 0.0f) * DEGTORAD * authority;

  // At cruise speed, the exact speed matters less than a steady one. Scale the
  // along-track gains down there; cross-track and slow flight keep them.
  const float desired_speed = hypotf(desired_north, desired_east);
  const float along_north = desired_speed > 0.01f ? desired_north / desired_speed : 0.0f;
  const float along_east = desired_speed > 0.01f ? desired_east / desired_speed : 0.0f;
  const float along_error = error_north * along_north + error_east * along_east;
  const float cruise = constrain((desired_speed - VEL_CRUISE_SPEED_MIN) / (VEL_CRUISE_SPEED_FULL - VEL_CRUISE_SPEED_MIN), 0.0f, 1.0f);
  const float p_reduction = (1.0f - VEL_CRUISE_KP_SCALE) * cruise * along_error;
  // Only small speed errors trim slowly: a new leg or a lasting change of wind
  // still trims at the full rate, so the speed cannot settle far off.
  const float i_reduction = (1.0f - VEL_CRUISE_KI_SCALE) * cruise * constrain(along_error, -VEL_CRUISE_KI_BAND, VEL_CRUISE_KI_BAND);
  // Drag tilt follows the desired velocity: it turns with each leg and fades
  // while braking, leaving the earth-frame integral to trim the wind.
  const float north_p = VEL_KP * (error_north - p_reduction * along_north) + nav.drag_trim * desired_north;
  const float east_p = VEL_KP * (error_east - p_reduction * along_east) + nav.drag_trim * desired_east;
  // Cruise drag into a headwind can need most of the tilt: the trim may use
  // all of it rather than leaving the proportional term a standing error.
  float north_i = horizontal.integral_north + VEL_KI * (error_north - i_reduction * along_north) * dt;
  float east_i = horizontal.integral_east + VEL_KI * (error_east - i_reduction * along_east) * dt;
  const float integral_scale = MIN(1.0f, max_angle / MAX(hypotf(north_i, east_i), 0.001f));
  north_i *= integral_scale;
  east_i *= integral_scale;
  const float candidate_north = north_p + north_i;
  const float candidate_east = east_p + east_i;
  if (!moving_away && (hypotf(candidate_north, candidate_east) <= max_angle ||
                       candidate_north * error_north + candidate_east * error_east < 0.0f)) {
    horizontal.integral_north = north_i;
    horizontal.integral_east = east_i;
  }
  if (cruise > 0.0f && !moving_away && fabsf(along_error) < DRAG_LEARN_ERROR) {
    // Move the along-track trim into the drag trim; the total tilt is unchanged.
    const float along_integral = horizontal.integral_north * along_north + horizontal.integral_east * along_east;
    const float drag_trim = constrain(nav.drag_trim + dt / (DRAG_LEARN_TIME + dt) * along_integral / desired_speed,
                                      0.0f, DRAG_TRIM_MAX);
    const float moved = (drag_trim - nav.drag_trim) * desired_speed;
    nav.drag_trim = drag_trim;
    horizontal.integral_north -= moved * along_north;
    horizontal.integral_east -= moved * along_east;
  }
  float north_angle = north_p + horizontal.integral_north;
  float east_angle = east_p + horizontal.integral_east;
  const float angle_scale = MIN(1.0f, max_angle / MAX(hypotf(north_angle, east_angle), 0.001f));
  north_angle *= angle_scale;
  east_angle *= angle_scale;
  const float cos_heading = cosf(state.heading * DEGTORAD);
  const float sin_heading = sinf(state.heading * DEGTORAD);
  const float pitch_angle = north_angle * cos_heading + east_angle * sin_heading;
  const float roll_angle = east_angle * cos_heading - north_angle * sin_heading;

  // Angle mode interprets stick input using level_max_angle, so scale by that
  // rather than by the navigation angle limit.
  const float level_max_angle = MAX(profile.rate.level_max_angle, 1.0f) * DEGTORAD;
  const float slew_step = ANGLE_SLEW_RATE * dt;
  const float command_gain = dt / (COMMAND_TIME_CONSTANT + dt);
  horizontal.pitch_command += constrain(command_gain * (pitch_angle / level_max_angle - horizontal.pitch_command), -slew_step, slew_step);
  horizontal.roll_command += constrain(command_gain * (roll_angle / level_max_angle - horizontal.roll_command), -slew_step, slew_step);
  state.rx_override.pitch = horizontal.pitch_command;
  state.rx_override.roll = horizontal.roll_command;
}

// Fly along the unit direction toward a point (north, east) meters away at
// the current target's speed, slowing to stop on it, or to end_speed at
// end_radius from it.
static void nav_steer(float north, float east, float dir_north, float dir_east, float end_speed, float end_radius, float dt) {
  const float distance = sqrtf(north * north + east * east);

  uint8_t waypoint_speed = 0;
  if (state.nav_target == NAV_TARGET_GUIDED)
    waypoint_speed = nav_route.guided.speed;
  else if (state.nav_target == NAV_TARGET_MISSION)
    waypoint_speed = mission.waypoints[nav_route.index].speed;
  const float cruise_speed = nav_cruise_speed(waypoint_speed);
  // Slow toward a stop continuously, without a minimum approach speed.
  const float approach_speed = end_speed > 0.0f
                                   ? MAX(nav_braking_speed(MAX(distance - end_radius, 0.0f), end_speed), end_speed)
                                   : MIN(nav_braking_speed(distance, 0.0f), distance * POSITION_KP);

  if (rth.horizontal.phase == NAV_HORIZONTAL_IDLE && distance > 0.01f) {
    // Keep the speed already made along the direction, even above cruise, and
    // slew it down: braking to a stop first pitches hard against the motion.
    const float along = state.gps_vel_north * dir_north + state.gps_vel_east * dir_east;
    const float seed = constrain(along, 0.0f, approach_speed);
    rth.horizontal.desired_velocity_north = dir_north * seed;
    rth.horizontal.desired_velocity_east = dir_east * seed;
  }
  const float desired_speed = distance > 0.01f ? MIN(cruise_speed, approach_speed) : 0.0f;

  // Do not build a persistent tilt demand while heading recovery is needed.
  // Use actual progress toward the point, independent of estimated yaw.
  // Leaving a position hold still creeps toward the hold point; decaying the
  // wind integral then would start the leg without it.
  const bool moving_away = state.rth_state == RTH_STATE_NAVIGATE &&
                           state.gps_vel_north * north + state.gps_vel_east * east < -VEL_AWAY_SPEED * distance;
  nav_update_velocity_control(dir_north * desired_speed, dir_east * desired_speed, moving_away, dt);
}

// Fly toward target, slowing to stop on it.
static void nav_steer_to(gps_coord_t target, float dt) {
  float north, east;
  nav_position_delta(state.gps_coord, target, &north, &east);
  const float distance = sqrtf(north * north + east * east);
  const float dir_north = distance > 0.01f ? north / distance : 0.0f;
  const float dir_east = distance > 0.01f ? east / distance : 0.0f;
  nav_steer(north, east, dir_north, dir_east, 0.0f, 0.0f, dt);
}

// Fly the leg toward the navigation target, along its line where it has one.
static void nav_update_horizontal_control(float dt) {
  nav_steer(rth.steer.north, rth.steer.east, rth.steer.dir_north, rth.steer.dir_east,
            rth.leg.end_speed, nav_arrive_radius(), dt);
  rth.horizontal.phase = NAV_HORIZONTAL_STEERING;
  nav_update_yaw(rth.steer.distance, dt);
}

static void nav_reset_gps_accel() {
  gps_accel = {};
  state.nav_accel_north = state.nav_accel_east = 0.0f;
}

// Publish horizontal GPS acceleration for IMU gravity compensation. The IMU
// rotates it by heading, so navigation updates it only while heading is trusted.
static void nav_update_gps_accel() {
  // Differentiate measurements at GPS cadence, not at the 100 Hz control rate.
  if (gps_accel.sample_valid && state.gps_last_update_ms == gps_accel.updated_ms)
    return;

  const uint32_t elapsed_ms = state.gps_last_update_ms - gps_accel.updated_ms;
  if (gps_accel.sample_valid && elapsed_ms > 0 && elapsed_ms <= NAV_GPS_STALE_MS) {
    const float sample_dt = elapsed_ms * 0.001f;
    const float gain = sample_dt / (0.2f + sample_dt);
    state.nav_accel_north += gain * ((state.gps_vel_north - gps_accel.velocity_north) / sample_dt - state.nav_accel_north);
    state.nav_accel_east += gain * ((state.gps_vel_east - gps_accel.velocity_east) / sample_dt - state.nav_accel_east);
  } else {
    state.nav_accel_north = state.nav_accel_east = 0.0f;
  }
  gps_accel.velocity_north = state.gps_vel_north;
  gps_accel.velocity_east = state.gps_vel_east;
  gps_accel.updated_ms = state.gps_last_update_ms;
  gps_accel.sample_valid = true;
}

static void nav_reset_horizontal() {
  rth.horizontal = {};
  state.rth_yaw_rate = 0.0f;
  state.rx_override.roll = state.rx_override.pitch = state.rx_override.yaw = 0.0f;
}

// Slow to a stop over the ground without steering toward the target, then
// latch where it came to rest. Earth-frame tilt needs a trusted heading;
// level without one.
static void nav_update_brake(float dt) {
  if (state.heading_confidence < RTH_MIN_HEADING_CONFIDENCE) {
    nav_reset_horizontal();
    return;
  }
  if (rth.horizontal.phase == NAV_HORIZONTAL_IDLE) {
    // Slow down from the ground velocity instead of pitching hard against it.
    rth.horizontal.desired_velocity_north = state.gps_vel_north;
    rth.horizontal.desired_velocity_east = state.gps_vel_east;
  }
  rth.horizontal.phase = NAV_HORIZONTAL_BRAKING;
  nav_update_velocity_control(0.0f, 0.0f, false, dt);
  if (rth.horizontal.desired_velocity_north == 0.0f && rth.horizontal.desired_velocity_east == 0.0f) {
    rth.horizontal.phase = NAV_HORIZONTAL_HOLDING;
    rth.horizontal.hold_coord = state.gps_coord;
  }
}

// Brake, then hold the point where the brake came to rest. Holding zero
// velocity alone lets gusts and GPS velocity error drift the position.
static void nav_update_position_hold(float dt) {
  if (rth.horizontal.phase != NAV_HORIZONTAL_HOLDING) {
    nav_update_brake(dt);
    return;
  }
  if (state.heading_confidence < RTH_MIN_HEADING_CONFIDENCE) {
    nav_reset_horizontal();
    return;
  }
  nav_steer_to(rth.horizontal.hold_coord, dt);
}

// The velocity loop has diverged for DIVERGE_TIMEOUT_MS. Falling short along
// the desired velocity is not divergence, nor is overshooting it while braking;
// moving across or against it is. Holding, drift alone may be wind beyond the
// tilt limit or a slow stop; a wrong heading tilts with the drift, so it must
// also gain DIVERGE_SPEED within the window.
static bool nav_update_divergence(uint32_t now_ms) {
  const float desired_north = rth.horizontal.desired_velocity_north;
  const float desired_east = rth.horizontal.desired_velocity_east;
  const float desired_speed = hypotf(desired_north, desired_east);
  const float speed = hypotf(state.gps_vel_north, state.gps_vel_east);
  const bool holding = desired_speed <= 0.5f;
  bool diverging;
  if (!holding) {
    const float along = (state.gps_vel_north * desired_north + state.gps_vel_east * desired_east) / desired_speed;
    const float across = (state.gps_vel_east * desired_north - state.gps_vel_north * desired_east) / desired_speed;
    diverging = fabsf(across) > DIVERGE_SPEED || along < -DIVERGE_SPEED;
  } else {
    diverging = speed > DIVERGE_SPEED;
  }
  if (!diverging) {
    rth.horizontal.diverging = false;
    return false;
  }
  if (!rth.horizontal.diverging) {
    rth.horizontal.diverging = true;
    rth.horizontal.diverging_ms = now_ms;
    rth.horizontal.diverging_speed = speed;
  }
  if (now_ms - rth.horizontal.diverging_ms < DIVERGE_TIMEOUT_MS)
    return false;
  if (!holding || speed > rth.horizontal.diverging_speed + DIVERGE_SPEED)
    return true;
  // Steady drift: judge the next window from here.
  rth.horizontal.diverging_ms = now_ms;
  rth.horizontal.diverging_speed = speed;
  return false;
}

// What the phase's progress is measured by.
static float nav_progress_error() {
  if (state.rth_state == RTH_STATE_CLIMB || state.rth_state == RTH_STATE_TAKEOFF)
    return fabsf(rth.target_altitude - state.altitude);
  if (state.rth_state == RTH_STATE_LAND)
    return state.altitude - rth.land.ground;
  return state.nav_target_distance;
}

// Enter a phase toward the current target and supervise its progress from here.
static void nav_set_phase(rth_state_t phase, uint32_t now_ms) {
  nav_update_target();
  state.rth_state = phase;
  rth.progress_ms = now_ms;
  rth.horizontal.diverging = false;
  rth.best_error = nav_progress_error();
}

// Fly forward to learn the heading, without earth-frame steering.
static void nav_begin_heading_acquisition(uint32_t now_ms) {
  nav_reset_horizontal();
  nav_set_phase(RTH_STATE_ACQUIRE_HEADING, now_ms);
  rth.acquisition.origin = state.gps_coord;
  // Momentum across the nose would dominate the course; let drag take it out first.
  rth.acquisition.leveling = state.gps_speed > ACQUIRE_START_SPEED;
}

// Navigation cannot be trusted. Level and hold altitude, then give up.
static void nav_begin_failure(rth_state_t failure, uint32_t now_ms) {
  nav_reset_horizontal();
  rth.target_altitude = state.altitude;
  nav_set_phase(failure, now_ms);
}

static void nav_arrive(uint32_t now_ms);

// The mission flies its TAKEOFF waypoint: climbing in place to its altitude.
static bool nav_mission_takeoff() {
  return state.nav_target == NAV_TARGET_MISSION && !nav_route.complete &&
         mission.waypoints[nav_route.index].type == MISSION_WAYPOINT_TAKEOFF;
}

// Fly to the selected target at rth.target_altitude: climb in place to a
// higher one, descend evenly toward a lower one on the way. A TAKEOFF
// waypoint flown in the air is reached by climbing in place. Steering needs a
// trusted heading; learn it first if needed. The turn brakes from the
// current tilt and velocity; it does not level first.
static void nav_begin_leg(uint32_t now_ms) {
  if (rth.target_altitude > state.altitude + ALT_REACHED) {
    nav_set_phase(RTH_STATE_CLIMB, now_ms);
    return;
  }
  if (nav_mission_takeoff()) {
    nav_arrive(now_ms);
    return;
  }
  // A climb in place, or a heading acquisition run, starts the leg here.
  nav_start_leg();
  if (state.heading_confidence < RTH_MIN_HEADING_CONFIDENCE)
    nav_begin_heading_acquisition(now_ms);
  else
    nav_set_phase(RTH_STATE_TURN, now_ms);
}

// Fly to a new target. Flying or holding with a trusted heading continues
// the leg without leveling to turn first: it keeps the velocity, realigns
// past REALIGN_ERROR and changes altitude evenly on the way.
static void nav_retarget(uint32_t now_ms) {
  const bool underway = state.heading_confidence >= RTH_MIN_HEADING_CONFIDENCE &&
                        (state.rth_state == RTH_STATE_NAVIGATE || state.rth_state == RTH_STATE_HOVER_HOME ||
                         state.rth_state == RTH_STATE_BRAKE);
  if (underway)
    nav_set_phase(RTH_STATE_NAVIGATE, now_ms);
  else
    nav_begin_leg(now_ms);
}

// Leave the mission or guided target. nav_route keeps where the mission left off.
static void nav_clear_target() {
  state.nav_target = NAV_TARGET_HOME;
  nav_route.returning = false;
}

// Climb in place by rth_altitude, then fly home.
static void nav_begin_return(uint32_t now_ms) {
  nav_clear_target();
  // Home is aimed at directly.
  rth.leg = {.arrive_radius = ARRIVAL_RADIUS};
  rth.target_altitude = state.altitude + profile.navigation.rth_altitude;
  nav_set_phase(RTH_STATE_CLIMB, now_ms);
}

// The altitude to fly waypoint index at. A LAND leg flies level at
// level_altitude: the waypoint's altitude is the ground it descends to.
static float nav_mission_altitude(uint8_t index, float level_altitude) {
  const mission_waypoint_t &waypoint = mission.waypoints[index];
  return waypoint.type == MISSION_WAYPOINT_LAND ? level_altitude : waypoint.altitude;
}

// Fly the mission leg to index from here; the caller enters a phase for it.
static void nav_mission_select(uint8_t index) {
  state.nav_target = NAV_TARGET_MISSION;
  nav_route.index = index;
  rth.target_altitude = nav_mission_altitude(index, state.altitude);
  nav_update_target();
  nav_start_leg();
}

// Fly the stored mission from index; the caller owns the controls.
static void nav_mission_fly(uint8_t index, uint32_t now_ms) {
  nav_route.complete = false;
  nav_mission_select(index);
  nav_begin_leg(now_ms);
}

static void nav_begin_land(uint32_t now_ms) {
  rth.land = {
      .ground = (float)mission.waypoints[nav_route.index].altitude,
      // The configured hover may be far off; the trim flown so far is not.
      .hover = constrain(nav_hover_throttle() + alt.integral, LAND_THROTTLE_MIN, nav_throttle_max()),
      // The landing begins stopped over the waypoint. Until the descent gets
      // going, the checks would take that for touchdown.
      .quiet = -LAND_START_TIME,
      .stopped = -LAND_START_TIME,
  };
  nav_set_phase(RTH_STATE_LAND, now_ms);
}

// Within the arrival radius of the target: continue the mission, or hold.
static void nav_arrive(uint32_t now_ms) {
  const mission_waypoint_t &reached = mission.waypoints[nav_route.index];
  rth.relearned = false;
  if (state.nav_target != NAV_TARGET_MISSION || nav_route.complete) {
    nav_set_phase(RTH_STATE_HOVER_HOME, now_ms);
  } else if (reached.type == MISSION_WAYPOINT_LAND) {
    nav_begin_land(now_ms);
  } else if (nav_route.index + 1 < mission.count) {
    // The next leg runs from this waypoint and its altitude, also while still
    // catching up with it; a takeoff climbed where it lifted off.
    nav_route_next();
    if (reached.type == MISSION_WAYPOINT_TAKEOFF)
      nav_route.leg_origin = nav_route.altitude_origin = state.gps_coord;
    rth.target_altitude = nav_mission_altitude(nav_route.index, rth.target_altitude);
    nav_update_target();
    nav_latch_leg();
    if (reached.type == MISSION_WAYPOINT_PASS) {
      // Carry the corner speed onto the next leg; the nose follows it round.
      rth.leg.passing = true;
      nav_set_phase(RTH_STATE_NAVIGATE, now_ms);
    } else {
      // Stop on the waypoint, then turn toward the next.
      nav_set_phase(RTH_STATE_TURN, now_ms);
    }
  } else if (mission.end_action == MISSION_END_RTH) {
    nav_route.complete = true;
    nav_begin_return(now_ms);
    nav_route.returning = true;
  } else {
    nav_route.complete = true;
    nav_set_phase(RTH_STATE_HOVER_HOME, now_ms);
  }
}

// Inside the arrival radius, or for a PASS corner flown fast, past the
// waypoint along the line and close to it.
static bool nav_leg_arrived() {
  if (state.nav_target_distance < nav_arrive_radius())
    return true;
  if (rth.leg.end_speed <= 0.0f || !rth.leg.line)
    return false;
  const float along = rth.steer.north * rth.leg.dir_north + rth.steer.east * rth.leg.dir_east;
  const float cross = hypotf(rth.steer.north - along * rth.leg.dir_north, rth.steer.east - along * rth.leg.dir_east);
  return along < 0.0f && cross < 2.0f * nav_arrive_radius();
}

// Progress toward the phase's goal within its timeout, measured by error.
// progress is the improvement that counts.
static bool nav_update_progress(float error, float progress, uint32_t timeout_ms, uint32_t now_ms) {
  if (error + progress < rth.best_error) {
    rth.best_error = error;
    rth.progress_ms = now_ms;
  }
  return now_ms - rth.progress_ms <= timeout_ms;
}

// Hand the controls back to the pilot.
static void nav_rth_stop() {
  if (state.rth_state >= RTH_STATE_CLIMB) {
    flags.controls_override = 0;
    state.rx_override = (vec4_t){0};
  }
  rth = {};
  state.rth_state = RTH_STATE_INACTIVE;
  nav_clear_target();
  nav_update_target();
  state.rth_yaw_rate = 0.0f;
  nav_reset_gps_accel();
}

static void nav_rth_abort() {
  nav_rth_stop();
  // Do not automatically retry a failed rescue during the same signal loss.
  state.rth_state = RTH_STATE_ABORTED;
  // Nor let the ground station restart it: the mission switch grants that again.
  nav.guided_granted = false;
}

// Ready or spinning up: the motors have not lifted the craft yet.
static bool nav_takeoff_on_ground() {
  return state.rth_state == RTH_STATE_TAKEOFF_READY ||
         state.rth_state == RTH_STATE_TAKEOFF_SPINUP;
}

// End a takeoff still on the ground. Spinning motors stop instead of jumping
// to the throttle stick, which the pilot raised to start the spin-up.
static void nav_takeoff_cancel() {
  const bool spinning = state.rth_state == RTH_STATE_TAKEOFF_SPINUP;
  nav_rth_abort();
  if (spinning)
    control_request_disarm();
}

// Roll or pitch hands control back from the takeoff and the heading run
// after it. Spin-up and the throttle raise may jostle the sticks at first,
// and a lost link holds stale ones.
static bool nav_takeoff_sticks_moved(uint32_t now_ms) {
  if (flags.failsafe_signal_lost)
    return false;
  if (state.rth_state != RTH_STATE_TAKEOFF_READY && now_ms - rth.takeoff.started_ms < TAKEOFF_STICK_GRACE_MS)
    return false;
  return fabsf(state.rx.roll) >= TAKEOFF_STICK_ABORT || fabsf(state.rx.pitch) >= TAKEOFF_STICK_ABORT;
}

// The stick abort lasts from takeoff until the route is flown with a trusted
// heading: through the climb, the heading run and the turn onto the first leg.
static bool nav_takeoff_abort_window() {
  switch (state.rth_state) {
  case RTH_STATE_TAKEOFF_READY:
  case RTH_STATE_TAKEOFF_SPINUP:
  case RTH_STATE_TAKEOFF:
  case RTH_STATE_ACQUIRE_HEADING:
  case RTH_STATE_TURN:
    return true;
  default:
    return false;
  }
}

// Tipped over on its legs: stop the motors.
static bool nav_takeoff_tipped() {
  const bool low = state.rth_state == RTH_STATE_TAKEOFF_SPINUP || state.altitude - rth.takeoff.altitude < TAKEOFF_TIP_HEIGHT;
  if (!low || (fabsf(state.attitude.roll) <= TAKEOFF_TIP_ANGLE && fabsf(state.attitude.pitch) <= TAKEOFF_TIP_ANGLE))
    return false;
  nav_rth_stop();
  control_request_disarm();
  return true;
}

// Spin up from the ground until lift-off, then climb in place.
static void nav_update_spinup(float dt, uint32_t now_ms) {
  state.rx_override.roll = state.rx_override.pitch = 0.0f;
  state.rx_override.yaw = state.rth_yaw_rate = 0.0f;
  if (nav_takeoff_tipped())
    return;

  // No lift-off at full throttle: stop the motors.
  const float throttle_max = nav_throttle_max();
  if (rth.takeoff.throttle >= throttle_max) {
    nav_rth_stop();
    control_request_disarm();
    return;
  }

  const float spinup = TAKEOFF_SPINUP_FRACTION * nav_hover_throttle();
  const float rate = rth.takeoff.throttle < spinup ? spinup / TAKEOFF_SPINUP_TIME : TAKEOFF_RAMP_RATE;
  rth.takeoff.throttle = MIN(rth.takeoff.throttle + rate * dt, throttle_max);
  state.rx_override.throttle = rth.takeoff.throttle;

  rth.takeoff.rising = state.vertical_speed > TAKEOFF_LIFTOFF_SPEED ? rth.takeoff.rising + dt : 0.0f;
  if (rth.takeoff.rising >= TAKEOFF_LIFTOFF_TIME || state.altitude - rth.takeoff.altitude > TAKEOFF_LIFTOFF_HEIGHT) {
    // The lift-off throttle and the climb it makes seed the hover trim.
    nav_altitude_start(rth.takeoff.throttle);
    nav_set_phase(RTH_STATE_TAKEOFF, now_ms);
  }
}

// Climb in place to the TAKEOFF altitude and continue the route. Returns
// false once the climb gave up.
static bool nav_update_takeoff(float dt, uint32_t now_ms) {
  if (nav_takeoff_tipped())
    return true;
  // Level until the heading is learned; the heading run follows the climb.
  nav_update_position_hold(dt);
  state.rx_override.yaw = state.rth_yaw_rate = 0.0f;
  const float altitude_error = fabsf(state.altitude - rth.target_altitude);
  if (!nav_update_progress(altitude_error, CLIMB_PROGRESS, CLIMB_TIMEOUT_MS, now_ms))
    return false;
  if (altitude_error < ALT_REACHED)
    nav_arrive(now_ms);
  return true;
}

// Possibly on the ground: stop leaning against the position error and let
// the wind trim decay, so the craft cannot tip over on its legs.
static void nav_update_soften(float dt) {
  const float decay = expf(-2.0f * dt);
  rth.horizontal.integral_north *= decay;
  rth.horizontal.integral_east *= decay;
  rth.horizontal.desired_velocity_north = rth.horizontal.desired_velocity_east = 0.0f;
  rth.horizontal.phase = NAV_HORIZONTAL_IDLE;
  const float slew_step = ANGLE_SLEW_RATE * dt;
  rth.horizontal.pitch_command = slew(rth.horizontal.pitch_command, 0.0f, slew_step);
  rth.horizontal.roll_command = slew(rth.horizontal.roll_command, 0.0f, slew_step);
  state.rx_override.pitch = rth.horizontal.pitch_command;
  state.rx_override.roll = rth.horizontal.roll_command;
  state.rx_override.yaw = state.rth_yaw_rate = 0.0f;
}

// Descend on the LAND waypoint until the land detector sees touchdown, then
// disarm.
static void nav_update_land(float dt, uint32_t now_ms) {
  const float throttle_floor = MIN(LAND_THROTTLE_MIN, nav_throttle_min());
  const float rate = state.altitude - rth.land.ground > LAND_SLOW_HEIGHT ? ALT_MAX_RATE : LAND_FINAL_RATE;
  nav_update_altitude_control_above(state.altitude - LAND_TARGET_DEPTH, dt, 0.0f, rate, throttle_floor);

  // Earth-frame acceleration has the same magnitude as the body-frame
  // specific force less gravity.
  const float accel_gain = dt / (LAND_ACCEL_FILTER_TIME + dt);
  for (uint32_t i = 0; i < 3; i++) {
    const float accel = (state.accel_raw.axis[i] - state.GEstG.axis[i]) / ACC_1G * GRAVITY;
    rth.land.accel.axis[i] += accel_gain * (accel - rth.land.accel.axis[i]);
  }

  const bool stalled = !nav_update_progress(state.altitude - rth.land.ground, LAND_PROGRESS, LAND_STALL_MS, now_ms);
  const bool maybe_landed = rth.land.quiet >= LAND_MAYBE_TIME || rth.land.stopped >= LAND_STOPPED_TIME || stalled;
  // Low, on the waypoint and no longer descending at the final rate: the
  // ground may hold the craft. Creeping onto the waypoint would lean against
  // the ground, which disturbs the checks below. Without GPS, land level
  // where it is: the descent and the checks need only the barometer.
  const bool contact = state.altitude - rth.land.ground < LAND_LOW_HEIGHT && fabsf(state.vertical_speed) < LAND_FINAL_RATE &&
                       state.nav_target_distance < nav_arrive_radius();
  if (maybe_landed || contact || !flags.gps_valid)
    nav_update_soften(dt);
  else
    nav_update_horizontal_control(dt);

  const float request = hypotf(state.rx_override.roll, state.rx_override.pitch) * MAX(profile.rate.level_max_angle, 0.0f);
  const bool level_request = request < LAND_ANGLE_REQUEST_MAX;
  const bool level = vec3_magnitude(state.angle_error) < LAND_ANGLE_ERROR_MAX;
  const bool still = vec3_magnitude(rth.land.accel) < LAND_ACCEL_MAX;
  const bool stopped = fabsf(state.vertical_speed) < LAND_STOPPED_SPEED;
  const bool at_floor = alt.throttle_command <= throttle_floor + 0.01f;
  const bool quiet = level_request && level && still && stopped;
  rth.land.quiet = quiet ? rth.land.quiet + dt : 0.0f;
  rth.land.landed = quiet && at_floor ? rth.land.landed + dt : 0.0f;
  rth.land.stopped = stopped ? rth.land.stopped + dt : 0.0f;
  const bool carried = alt.throttle_command < LAND_GROUNDED_THROTTLE * rth.land.hover;
  rth.land.grounded = (stopped || stalled) && carried ? rth.land.grounded + dt : 0.0f;
  // Yaw is held while landing, so its heading error channel carries the checks.
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 9, (int16_t)(level_request | level << 1 | still << 2 | stopped << 3 | at_floor << 4 | maybe_landed << 5));

  if (maybe_landed) {
    // Wind the trim down toward the floor. On the ground the throttle reaches
    // it; aloft the craft starts to sink, which ends the checks.
    const float trim_min = throttle_floor * nav_tilt() - nav_hover_throttle();
    alt.integral = MAX(alt.integral - LAND_WIND_DOWN_RATE * dt, trim_min);
  }
  if (rth.land.landed >= LAND_DETECT_TIME || rth.land.grounded >= LAND_GROUNDED_TIME) {
    nav_route.complete = true;
    control_request_disarm();
  }
}

// Run the active phase. Returns false once it gave up.
static bool nav_update_phase(float dt, uint32_t now_ms) {
  switch (state.rth_state) {
  case RTH_STATE_CLIMB: {
    // Stop and hold position while climbing; navigation starts at the target altitude.
    nav_update_position_hold(dt);
    state.rx_override.yaw = state.rth_yaw_rate = 0.0f;
    const float altitude_error = fabsf(state.altitude - rth.target_altitude);
    if (!nav_update_progress(altitude_error, CLIMB_PROGRESS, CLIMB_TIMEOUT_MS, now_ms))
      return false;
    if (altitude_error < ALT_REACHED)
      nav_begin_leg(now_ms);
    return true;
  }

  case RTH_STATE_TAKEOFF_READY:
    // Motors stopped until the pilot raises the throttle stick.
    state.rx_override = (vec4_t){0};
    state.rth_yaw_rate = 0.0f;
    if (state.rx.throttle >= TAKEOFF_STICK_MIN) {
      nav_set_phase(RTH_STATE_TAKEOFF_SPINUP, now_ms);
      rth.takeoff.started_ms = now_ms;
      rth.takeoff.altitude = state.altitude;
    }
    return true;

  case RTH_STATE_TAKEOFF_SPINUP:
    nav_update_spinup(dt, now_ms);
    return true;

  case RTH_STATE_TAKEOFF:
    return nav_update_takeoff(dt, now_ms);

  case RTH_STATE_LAND:
    nav_update_land(dt, now_ms);
    return true;

  case RTH_STATE_ACQUIRE_HEADING: {
    // No earth-frame steering until heading is observable. Zero yaw rate holds
    // the physical orientation without chasing GPS corrections to the estimate.
    state.rx_override.roll = state.rx_override.yaw = state.rth_yaw_rate = 0.0f;
    if (rth.acquisition.leveling) {
      if (state.gps_speed <= ACQUIRE_START_SPEED || now_ms - rth.progress_ms >= ACQUIRE_LEVEL_TIMEOUT_MS) {
        // The run starts here.
        rth.acquisition.leveling = false;
        rth.acquisition.origin = state.gps_coord;
        rth.progress_ms = now_ms;
      }
      return true;
    }
    float north, east;
    nav_position_delta(rth.acquisition.origin, state.gps_coord, &north, &east);
    if (now_ms - rth.progress_ms >= ACQUIRE_TIMEOUT_MS || hypotf(north, east) >= ACQUIRE_DISTANCE) {
      nav_begin_failure(RTH_STATE_HEADING_FAILED, now_ms);
    } else if (state.heading_confidence >= ACQUIRE_CONFIDENCE && !state.heading_correction_flags) {
      // A climb that diverged first finishes in place.
      nav_begin_leg(now_ms);
    } else {
      const float pitch = constrain(profile.rate.level_max_angle, 0.0f, ACQUIRE_PITCH) / MAX(profile.rate.level_max_angle, 1.0f);
      rth.horizontal.pitch_command = slew(rth.horizontal.pitch_command, pitch, ACQUIRE_SLEW_RATE * dt);
      state.rx_override.pitch = rth.horizontal.pitch_command;
    }
    return true;
  }

  case RTH_STATE_HEADING_FAILED:
  case RTH_STATE_NAV_FAILED:
    nav_reset_horizontal();
    return now_ms - rth.progress_ms < FAILURE_HOLD_MS;

  case RTH_STATE_TURN: {
    if (state.heading_confidence < RTH_MIN_HEADING_CONFIDENCE) {
      nav_begin_heading_acquisition(now_ms);
      return true;
    }
    // Stop and hold position until the nose points at the target and the turn has slowed down.
    nav_update_position_hold(dt);
    nav_update_yaw(state.nav_target_distance, dt);
    // Inside the yaw hold radius the nose stays put; steering is earth-frame.
    const bool aligned = state.nav_target_distance <= YAW_HOLD_RADIUS ||
                         (fabsf(nav_heading_error()) < TURN_ERROR && fabsf(state.gyro.yaw) < TURN_RATE * DEGTORAD);
    if (nav_leg_arrived())
      nav_arrive(now_ms);
    else if (aligned)
      nav_set_phase(RTH_STATE_NAVIGATE, now_ms);
    else if (now_ms - rth.progress_ms > TURN_TIMEOUT_MS)
      nav_begin_failure(RTH_STATE_NAV_FAILED, now_ms);
    return true;
  }

  case RTH_STATE_NAVIGATE: {
    // Allow acceleration at reduced heading confidence before declaring no progress.
    // Close in, a full progress step can exceed the remaining approach.
    const float progress = MIN(PROGRESS_DISTANCE, 0.5f * rth.best_error);
    // Out of a PASS corner the nose comes round with the velocity; stopping
    // to turn would throw the corner speed away.
    if (rth.leg.passing && fabsf(nav_heading_error()) < PASS_TURN_DONE)
      rth.leg.passing = false;
    if (state.nav_target_distance >= ARRIVAL_RADIUS &&
        !nav_update_progress(state.nav_target_distance, progress, PROGRESS_TIMEOUT_MS, now_ms)) {
      nav_begin_failure(RTH_STATE_NAV_FAILED, now_ms);
    } else if (!rth.leg.passing && state.nav_target_distance > REALIGN_RADIUS &&
               fabsf(nav_heading_error()) > REALIGN_ERROR) {
      nav_set_phase(RTH_STATE_TURN, now_ms);
    } else {
      nav_update_horizontal_control(dt);
      if (nav_leg_arrived())
        nav_arrive(now_ms);
    }
    return true;
  }

  case RTH_STATE_BRAKE:
    // HOLD keeps the point where the brake comes to rest, not where it began.
    nav_update_brake(dt);
    state.rx_override.yaw = state.rth_yaw_rate = 0.0f;
    nav_route.guided.coord = state.gps_coord;
    if (rth.horizontal.desired_velocity_north == 0.0f && rth.horizontal.desired_velocity_east == 0.0f)
      nav_set_phase(RTH_STATE_HOVER_HOME, now_ms);
    return true;

  case RTH_STATE_HOVER_HOME:
    if (state.nav_target_distance > 2.0f * ARRIVAL_RADIUS) {
      // Stationary hover is success. Resume progress supervision if blown out.
      nav_set_phase(RTH_STATE_TURN, now_ms);
    } else {
      nav_update_horizontal_control(dt);
    }
    return true;

  default:
    return true;
  }
}

static void nav_update_rth_control(float dt) {
  const uint32_t now_ms = time_millis();

  if (nav_takeoff_on_ground() && (!flags.baro_valid || !flags.gps_valid)) {
    // Not airborne yet: end the takeoff instead of holding an altitude.
    nav_takeoff_cancel();
    return;
  }
  if (rth.takeoff.sticks_abort) {
    if (!nav_takeoff_abort_window()) {
      rth.takeoff.sticks_abort = false;
    } else if (nav_takeoff_sticks_moved(now_ms)) {
      if (nav_takeoff_on_ground())
        nav_takeoff_cancel();
      else
        nav_rth_stop();
      return;
    }
  }

  // A landing needs no GPS to finish: it descends level where it is.
  const bool landing = state.rth_state == RTH_STATE_LAND && flags.baro_valid;
  if (!landing && (!flags.baro_valid || !flags.gps_valid)) {
    if (!rth.pause.active) {
      rth.pause = {.active = true, .started_ms = now_ms, .altitude = state.altitude};
      nav_reset_horizontal();
      // Restart differentiation from a fresh sample once the pause ends.
      nav_reset_gps_accel();
    }
    if (now_ms - rth.pause.started_ms >= SENSOR_LOSS_TIMEOUT_MS) {
      nav_rth_abort();
      return;
    }
    // Level and hold a latched altitude while the sensor may return. Without
    // a barometer the controller holds zero vertical speed instead.
    nav_update_altitude_control(rth.pause.altitude, dt);
    return;
  }
  if (rth.pause.active) {
    rth.pause.active = false;
    // The progress clock keeps running, so a flapping sensor cannot extend
    // the attempt; only the baseline restarts from the current position.
    rth.best_error = nav_progress_error();
  }

  // The IMU rotates published acceleration by heading. Publish only while the
  // heading is trusted for steering; leveling during CLIMB is a drag brake.
  if (flags.gps_valid && state.heading_confidence >= RTH_MIN_HEADING_CONFIDENCE &&
      state.rth_state != RTH_STATE_ACQUIRE_HEADING && state.rth_state != RTH_STATE_HEADING_FAILED)
    nav_update_gps_accel();
  else
    nav_reset_gps_accel();
  nav_update_course();

  if (!nav_update_phase(dt, now_ms)) {
    nav_rth_abort();
    return;
  }
  // A takeoff that tipped over handed the controls back.
  if (state.rth_state < RTH_STATE_CLIMB)
    return;

  // GPS course cannot correct a heading this wrong: the correcting roll makes
  // the motion read as not forward. Learn it anew from forward flight. Each
  // phase is timed from its entry. Taking off and landing stay too close to
  // the ground for an acquisition run; they keep the heading they have.
  const bool near_ground = state.rth_state == RTH_STATE_TAKEOFF_SPINUP || state.rth_state == RTH_STATE_TAKEOFF ||
                           state.rth_state == RTH_STATE_LAND;
  if (!near_ground && rth.horizontal.phase != NAV_HORIZONTAL_IDLE && nav_update_divergence(now_ms)) {
    if (rth.relearned) {
      // The learned heading diverged too: wind beyond the tilt limit, or a
      // heading that cannot be learned here.
      nav_begin_failure(RTH_STATE_NAV_FAILED, now_ms);
    } else {
      rth.relearned = true;
      imu_reject_heading();
      nav_begin_heading_acquisition(now_ms);
    }
  }

  switch (state.rth_state) {
  case RTH_STATE_TAKEOFF_READY:
  case RTH_STATE_TAKEOFF_SPINUP:
  case RTH_STATE_LAND:
    // Motors stopped, spinning up, or the landing flies its own descent.
    break;
  case RTH_STATE_TURN:
  case RTH_STATE_NAVIGATE: {
    float rate;
    const float target = nav_leg_altitude(nav_route.altitude_origin, nav_route.leg_altitude, nav_target_coord(), rth.target_altitude, &rate);
    nav_update_altitude_control(target, dt, rate);
    break;
  }
  default:
    nav_update_altitude_control(rth.target_altitude, dt);
    break;
  }
}

static bool nav_rth_can_start() {
  // Any distance is acceptable: close to home, RTH climbs and holds position.
  return state.rth_state != RTH_STATE_ABORTED && flags.home_valid && flags.gps_valid && flags.baro_valid &&
         profile.navigation.rth_throttle_min < profile.navigation.rth_throttle_max;
}

static bool nav_airborne() {
  return flags.in_air && !flags.on_ground && state.altitude >= MISSION_START_MIN_ALTITUDE;
}

// Take over the controls for navigation, or keep them and leave the current
// target. The caller enters the first phase for the next one.
static void nav_take_control() {
  if (state.rth_state >= RTH_STATE_CLIMB) {
    nav_clear_target();
    return;
  }
  rth = {};
  nav_reset_horizontal();
  if (nav.hold.active) {
    // The failsafe hold already flies the vertical axis; keep its trim.
    nav.hold.active = false;
  } else {
    nav_altitude_start(state.throttle);
  }
  flags.controls_override = 1;
}

static void nav_rth_start() {
  if (nav_takeoff_on_ground()) {
    // Still on the ground: cancel instead of lifting off, and do not lift off
    // for the request either while it lasts.
    nav_takeoff_cancel();
    return;
  }
  // An RTH request owns a return the mission started.
  nav_route.returning = false;
  // Already guided: retarget home without a second takeover.
  if (state.nav_target == NAV_TARGET_HOME) {
    if (state.rth_state >= RTH_STATE_CLIMB || !nav_rth_can_start())
      return;
    nav_take_control();
  }
  nav_begin_return(time_millis());
}

static void nav_mission_start() {
  // A mission is not a takeoff command: start only clear of the ground.
  const uint8_t index = nav_route_resume();
  if (state.rth_state >= RTH_STATE_CLIMB || mission.count == 0 || !nav_airborne() || !nav_rth_can_start() ||
      !nav_mission_in_start_range(index))
    return;
  nav_take_control();
  nav_mission_fly(index, time_millis());
}

// Armed on the ground at the TAKEOFF waypoint and never throttled up since:
// the mission switch readies a mission starting with TAKEOFF, the throttle
// stick starts it.
static bool nav_takeoff_can_start() {
  const uint8_t index = nav_route_resume();
  return mission.count > 0 && mission.waypoints[index].type == MISSION_WAYPOINT_TAKEOFF &&
         !flags.in_air && !nav.takeoff_done && nav_rth_can_start() && nav_mission_in_takeoff_range(index);
}

static void nav_takeoff_start() {
  const uint8_t index = nav_route_resume();
  nav_take_control();
  // Like a launched wing, a takeoff runs once per arm; an abort hands back for good.
  nav.takeoff_done = true;
  // The switch was turned on after arming: grant ground station commands once airborne.
  nav.guided_granted = true;
  nav_route.complete = false;
  nav_mission_select(index);
  nav_set_phase(RTH_STATE_TAKEOFF_READY, time_millis());
  rth.takeoff.sticks_abort = true;
  state.rx_override = (vec4_t){0};
}

// Control applied stage 1 this pass. Keep its override and let the failsafe
// hold, updated next, fly the vertical axis with the mission's trim.
static void nav_hand_to_failsafe_hold() {
  nav_rth_stop();
  flags.controls_override = 1;
  nav.hold = {.active = true, .altitude = state.altitude};
}

// The RTH switch and an airborne failsafe request a return for as long as
// they last, and RTH starts once it can. The mission switch starts a
// mission, and grants guided flight, only when turned on in flight; turning
// it off ends what it started. Receivers may report preset switch positions
// during a link loss, so nothing stops then.
static void nav_update_request() {
  // A mission flying through a link loss locks the switches until the pilot
  // unlocks them with the sticks. The mission switch's previous position is
  // kept, so moving it meanwhile acts as an edge once unlocked.
  if (state.failsafe_owner == FAILSAFE_OWNER_MISSION)
    return;

  const bool link_lost = flags.failsafe_signal_lost;
  const bool rth_switch = rx_aux_on(AUX_RETURN_TO_HOME);
  // Only an airborne failsafe flies home; a landed vehicle idles and disarms.
  // Failsafe stays set until the pilot has taken over.
  const bool failsafe_rth = profile.navigation.rth_on_failsafe && flags.failsafe &&
                            (flags.failsafe_hold || state.failsafe_owner == FAILSAFE_OWNER_RTH);
  if (rth_switch || failsafe_rth) {
    // RTH takes over a mission, its return and a ground station target.
    if (state.rth_state < RTH_STATE_CLIMB || state.nav_target != NAV_TARGET_HOME || nav_route.returning)
      nav_rth_start();
  } else if (!link_lost && state.rth_state >= RTH_STATE_CLIMB && state.nav_target == NAV_TARGET_HOME &&
             !nav_route.returning) {
    nav_rth_stop();
  }

  // RTH takes priority: a mission switched on during RTH is ignored.
  const bool mission_switch = rx_aux_on(AUX_MISSION);
  if (mission_switch != nav.mission_switch) {
    // As for a mission, a switch left on through arming and takeoff grants nothing.
    nav.guided_granted = mission_switch && nav_airborne();
    if (mission_switch && nav_takeoff_can_start())
      nav_takeoff_start();
    else if (mission_switch)
      nav_mission_start();
    else if (!link_lost && (state.nav_target != NAV_TARGET_HOME || nav_route.returning))
      nav_rth_stop();
    nav.mission_switch = mission_switch;
  }

  // A mission or ground station target rides out the hold-last window. Unless
  // the mission continues through the loss, it never continues away from a
  // pilot once failsafe engages: failsafe RTH returned above, otherwise the
  // failsafe hold takes over.
  if (state.nav_target != NAV_TARGET_HOME && flags.failsafe) {
    // A takeoff that has not lifted off ends; it never holds an altitude on the ground.
    if (nav_takeoff_on_ground())
      nav_takeoff_cancel();
    else if (flags.failsafe_hold)
      nav_hand_to_failsafe_hold();
    else
      nav_rth_stop();
  }

  // A rescue that gave up is not retried while an RTH request lasts. The
  // mission switch acts only on its edges, so a position left on does not
  // hold the abort, which would keep RTH and failsafe RTH from starting.
  if (state.rth_state == RTH_STATE_ABORTED && !rth_switch && !flags.failsafe)
    nav_rth_stop();
}

// The pilot grants ground station commands with the mission switch, unless
// an RTH request or failsafe takes priority. Once guided flight owns the
// controls the grant holds below the start altitude, so the ground station
// can still stop or recall the vehicle. A navigation failure holds until it
// gives up; commands cannot restart it. A takeoff finishes its climb first.
static bool nav_guided_ready() {
  const bool failed = state.rth_state == RTH_STATE_NAV_FAILED || state.rth_state == RTH_STATE_HEADING_FAILED;
  const bool taking_off = state.rth_state == RTH_STATE_TAKEOFF_READY || state.rth_state == RTH_STATE_TAKEOFF_SPINUP ||
                          state.rth_state == RTH_STATE_TAKEOFF;
  return nav.guided_granted && !rx_aux_on(AUX_RETURN_TO_HOME) && !flags.failsafe && !failed && !taking_off &&
         (state.rth_state >= RTH_STATE_CLIMB || (nav_airborne() && nav_rth_can_start()));
}

static void nav_guided_fly(const mission_waypoint_t &target) {
  nav_take_control();
  state.nav_target = NAV_TARGET_GUIDED;
  nav_route.guided = target;
  rth.target_altitude = target.altitude;
  nav_update_target();
  nav_start_leg();
}

nav_command_result_t nav_command_apply(nav_command_t *cmd) {
  if (profile.serial.gps == SERIAL_PORT_INVALID || !nav_guided_ready())
    return NAV_RESULT_NO_AUTHORITY;
  const nav_command_result_t result = nav_command_check(cmd);
  if (result != NAV_RESULT_OK)
    return result;
  const uint32_t now_ms = time_millis();

  switch (cmd->type) {
  case NAV_COMMAND_GOTO:
    nav_guided_fly(cmd->waypoint);
    nav_retarget(now_ms);
    break;
  case NAV_COMMAND_HOLD:
    nav_guided_fly({.coord = state.gps_coord, .altitude = (int16_t)lroundf(state.altitude)});
    // Hold the altitude it is at, not the whole meter a waypoint stores.
    rth.target_altitude = state.altitude;
    // Position hold steers in the earth frame: learn an untrusted heading
    // first, then return to the hold point, as on any leg.
    if (state.heading_confidence < RTH_MIN_HEADING_CONFIDENCE) {
      nav_begin_leg(now_ms);
      break;
    }
    // Brake and hover in place without turning toward the target first.
    nav_set_phase(RTH_STATE_BRAKE, now_ms);
    break;
  case NAV_COMMAND_SET_ALT:
    nav_route.guided.altitude = rth.target_altitude = cmd->waypoint.altitude;
    // Change to it evenly over the rest of the leg, still along its line.
    nav_route_restart_altitude();
    // A climb supervises progress toward its target altitude: measure it anew.
    if (state.rth_state == RTH_STATE_CLIMB)
      nav_set_phase(RTH_STATE_CLIMB, now_ms);
    break;
  case NAV_COMMAND_RTH:
    nav_take_control();
    nav_begin_return(now_ms);
    // Like the return after a mission, turning the mission switch off ends it.
    nav_route.returning = true;
    break;
  case NAV_COMMAND_MISSION:
    nav_take_control();
    nav_mission_fly(cmd->index, now_ms);
    break;
  }
  return NAV_RESULT_OK;
}

nav_command_result_t nav_mission_replace(const mission_t *updated) {
  bool retarget;
  // A takeoff or a landing under way keeps its waypoint.
  const bool locked = state.rth_state == RTH_STATE_TAKEOFF_READY || state.rth_state == RTH_STATE_TAKEOFF_SPINUP ||
                      state.rth_state == RTH_STATE_TAKEOFF || state.rth_state == RTH_STATE_LAND;
  const nav_command_result_t result = nav_route_replace(updated, state.nav_target == NAV_TARGET_MISSION, locked, &retarget);
  if (result != NAV_RESULT_OK)
    return result;
  if (!retarget) {
    // The waypoints after the target set how a PASS leg ends.
    if (state.nav_target == NAV_TARGET_MISSION && !nav_route.complete) {
      rth.leg.end_speed = 0.0f;
      rth.leg.arrive_radius = ARRIVAL_RADIUS;
      nav_latch_corner();
    }
    return NAV_RESULT_OK;
  }
  // A navigation failure keeps its latched hold until it gives up.
  if (state.rth_state == RTH_STATE_NAV_FAILED || state.rth_state == RTH_STATE_HEADING_FAILED) {
    nav_update_target();
    return NAV_RESULT_OK;
  }
  // Retarget like a guided GOTO.
  nav_mission_select(nav_route.index);
  nav_retarget(time_millis());
  return NAV_RESULT_OK;
}

static void nav_update_rth(float dt) {
  if (!flags.arm_state) {
    nav_rth_stop();
    nav_route = {};
    // The next flight may have a different airframe drag or payload.
    nav.drag_trim = 0.0f;
    nav.guided_granted = false;
    nav.takeoff_done = false;
    // A mission needs a switch edge after arming.
    nav.mission_switch = rx_aux_on(AUX_MISSION);
  } else {
    nav_update_request();
  }
  nav_update_target();
  if (state.rth_state >= RTH_STATE_CLIMB)
    nav_update_rth_control(dt);
  // After control, which can fail or abort guided flight.
  flags.guided_ready = nav_guided_ready();
  // A takeoff waiting on the ground is not flying a mission through a link loss.
  flags.mission_engaged = state.rth_state >= RTH_STATE_CLIMB && state.rth_state != RTH_STATE_TAKEOFF_READY &&
                          (state.nav_target == NAV_TARGET_MISSION || nav_route.returning);
  // Navigation debug: 0 heading, 1 GPS course, 2 altitude (dm), 3 confidence.
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 4, (int16_t)(state.rth_state));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 5, (int16_t)(state.nav_target_distance));     // meters
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 6, (int16_t)(state.nav_target_bearing * 10)); // 0.1 deg
}

// Airborne stage 1 failsafe without RTH: level, and hold the altitude at loss.
static void nav_update_failsafe_hold(float dt) {
  if (!flags.failsafe_hold || state.rth_state >= RTH_STATE_CLIMB || !flags.arm_state) {
    nav.hold.active = false;
    return;
  }
  if (!nav.hold.active) {
    nav.hold = {.active = true, .altitude = state.altitude};
    // Control applies the pilot's last throttle until this first update.
    nav_altitude_start(state.throttle);
  }
  nav_update_altitude_control(nav.hold.altitude, dt);
}

void nav_update_multi(float dt) {
  nav_update_hover(dt);
  if (profile.serial.gps != SERIAL_PORT_INVALID) {
    nav_update_rth(dt);
  } else {
    flags.guided_ready = false;
    flags.mission_engaged = false;
  }
  nav_update_failsafe_hold(dt);
  // Publish applied requests in every phase, including acquisition and failure.
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 14, (int16_t)(state.rx_override.pitch * 1000));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 15, (int16_t)(state.rx_override.roll * 1000));
}

#ifdef PIO_UNIT_TESTING
void nav_test_reset() {
  nav_rth_stop();
  nav = {};
  alt = {};
  nav_route = {};
}

void nav_test_update_horizontal_control(float dt) {
  nav_update_target();
  nav_update_course();
  nav_update_horizontal_control(dt);
}

void nav_test_rth_start() {
  nav_rth_start();
}

void nav_test_rth_stop() {
  nav_rth_stop();
}

void nav_test_mission_start() {
  nav_mission_start();
}

void nav_test_update_rth(float dt) {
  nav_update_target();
  nav_update_rth_control(dt);
}

void nav_test_set_hover_learned(float throttle) {
  state.hover_throttle = throttle;
  nav.hover_learned_time = throttle > 0.0f ? HOVER_LEARN_MIN_TIME : 0.0f;
}

void nav_test_update_hover(float dt) {
  nav_update_hover(dt);
}

void nav_test_altitude_control(float target, float dt) {
  nav_update_altitude_control(target, dt);
}
#endif
