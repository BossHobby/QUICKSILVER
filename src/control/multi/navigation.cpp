#include "navigation.h"

#include <math.h>

#include "control/control.h"
#include "control/imu.h"
#include "control/mission.h"
#include "control/multi/nav_control.h"
#include "control/navigation.h"
#include "core/profile.h"
#include "io/blackbox.h"
#include "rx/rx.h"
#include "util/util.h"
#include "util/vector.h"

// Rotor wash near the ground raises the pressure at the barometer, which
// reads meters low from spin-up until clear of the ground, and on touchdown.
static constexpr float GROUND_EFFECT_HEIGHT = 0.5f;     // meters above launch
static constexpr float GROUND_EFFECT_CLEAR_TIME = 2.0f; // seconds above it end the takeoff
// Seconds of raised throttle that end the takeoff at any altitude, as
// ArduPilot's AP_GROUNDEFFECT_TAKEOFF_MAX_MS: a launch from a hill may never
// climb above it. Idling on the ground does not count.
static constexpr float GROUND_EFFECT_LAUNCH_TIME = 5.0f;

// Yaw toward the target.
static constexpr float YAW_KP = 1.0f;
static constexpr float YAW_MAX_RATE = 30.0f;       // degrees/s
static constexpr float YAW_DEADBAND = 2.0f;        // degrees
static constexpr float YAW_TIME_CONSTANT = 0.03f;  // seconds
// GPS noise swings the target bearing close in; hold yaw rather than chase it.
static constexpr float YAW_HOLD_RADIUS = 8.0f; // meters

static constexpr float ALT_REACHED = 1.0f; // meters

static constexpr float VEL_AWAY_SPEED = 0.5f; // m/s away from the target, slower is a hold's creep
static constexpr float BRAKE_ACCEL = 1.5f;    // m/s^2, conservative approach deceleration
// Seconds reserved for the velocity shaping and control to respond. Near the
// target the approach closes the distance at 1 / BRAKE_DELAY per second.
static constexpr float BRAKE_DELAY = 2.0f;

// Phases.
static constexpr float ARRIVAL_RADIUS = 3.0f; // meters
// A PASS corner flown fast arrives past the waypoint within this many arrival
// radii of the line.
static constexpr float PASS_ARRIVAL_CROSS = 2.0f;
// Blown this far off the hold point, HOLD turns and flies back under supervision.
static constexpr float HOLD_DRIFT_RADIUS = 2.0f * ARRIVAL_RADIUS; // meters
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
static constexpr float DIVERGE_HOLD_SPEED = 0.5f; // m/s desired; slower judges the loop as holding
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
// The next leg begins at most this far ahead of the waypoint.
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
// Less tilt authority this close to the ground.
static constexpr float LAND_LOW_HEIGHT = 1.5f; // meters
static constexpr float LAND_LOW_AUTHORITY = 0.5f;
// Throttle floor while landing, so the trim can wind down onto the ground.
static constexpr float LAND_THROTTLE_MIN = 0.05f;
static constexpr float LAND_FLOOR_MARGIN = 0.01f; // throttle above the floor that counts as at it
// Touchdown follows ArduPilot's land detector: throttle at its floor while
// level and still. The altitude loop alone takes tens of seconds to trim down
// once the ground stops the descent, so after LAND_MAYBE_TIME of level, still
// flight the trim winds down toward the floor; at the floor, LAND_DETECT_TIME
// of the same disarms.
static constexpr float LAND_MAYBE_TIME = 0.2f;         // seconds
static constexpr float LAND_DETECT_TIME = 1.0f;        // seconds
static constexpr float LAND_WIND_DOWN_RATE = 1.0f;     // hover throttles/s
static constexpr float LAND_ACCEL_MAX = 1.0f;          // m/s^2, 1 Hz low-passed
static constexpr float LAND_ACCEL_FILTER_TIME = 0.16f; // seconds, 1 Hz
// Unlike ArduPilot, landed-maybe precedes the throttle floor, so it also
// needs the commanded descent to have stopped: a steady descent is level and
// still too.
static constexpr float LAND_STOPPED_SPEED = 0.2f;      // m/s vertical, below LAND_FINAL_RATE
static constexpr float LAND_ANGLE_REQUEST_MAX = 15.0f; // degrees commanded
static constexpr float LAND_ANGLE_ERROR_MAX = 0.5f;    // sin(30 degrees) between commanded and estimated gravity
// Winding down asks the accelerometer whether the ground carries the craft.
// Aloft only the thrust does: below this share of the hover thrust the
// specific force falls with it, and the trim comes straight back. On the
// ground it stays at 1 g however the craft lies, so the trim winds on to the
// floor, where LAND_DETECT_TIME of that disarms too. That needs neither the
// barometer, which the wash moves on touchdown, nor a level attitude.
static constexpr float LAND_CARRIED_THRUST = 0.9f;
// Falling shows as a specific force between the thrust share and 1 g, but
// never above this: as ArduPilot's detector allows 1 m/s^2, an accelerometer
// reading a few percent low on the ground is not a fall.
static constexpr float LAND_CARRIED_FORCE = 0.9f; // g
// The hover throttle is what the craft flies on at 1 g. It drifts with the
// battery and the descent, so it follows the throttle over this time while
// the trim is not wound down.
static constexpr float LAND_HOVER_TIME = 2.0f; // seconds
// Ground too steep or rough for the level and still checks: a descent stopped
// for LAND_STOPPED_TIME also winds the trim down.
static constexpr float LAND_STOPPED_TIME = 1.0f; // seconds
static constexpr float LAND_START_TIME = 1.5f;   // seconds for the descent to pass LAND_STOPPED_SPEED
// Descending less than this for LAND_STALL_MS, the ground holds the craft. On
// a slope the estimated vertical speed takes seconds to settle; the altitude
// shows the stall at once.
static constexpr float LAND_PROGRESS = 0.2f; // meters
static constexpr uint32_t LAND_STALL_MS = 2000U;
// Roll and pitch beyond this move the landing point, as ArduPilot's
// LAND_REPOSITION does, at up to this speed in the stick's direction.
static constexpr float LAND_REPOSITION_DEADBAND = 0.15f;
static constexpr float LAND_REPOSITION_SPEED = 2.0f; // m/s at full stick
// Possibly on the ground, the wind trim decays at this rate.
static constexpr float SOFTEN_DECAY_RATE = 2.0f; // per second

// Position hold: roll and pitch beyond this fly over the ground, up to the
// travel speed at full stick.
static constexpr float POSHOLD_STICK_DEADBAND = 0.1f;

// Sensor and input history survives RTH stop/start. Published outputs live in state.
static struct {
  bool mission_switch;      // position in the previous update; turning it on starts a mission
  float clear_of_ground;    // seconds above GROUND_EFFECT_HEIGHT since arming, up to GROUND_EFFECT_CLEAR_TIME
  float launch_time;        // seconds of throttle above THROTTLE_SAFETY since arming, up to GROUND_EFFECT_LAUNCH_TIME
  bool guided_granted;      // mission switch turned on in flight; grants ground station commands
  bool takeoff_done;        // a mission takeoff started since arming; it runs at most once per arm
  struct {
    bool active;
    float altitude; // meters above launch, latched on engagement
  } hold;
} nav;

// Where a landing comes down.
typedef enum {
  NAV_LAND_POINT_WAYPOINT, // the LAND waypoint
  NAV_LAND_POINT_MOVED,    // the sticks move it; released, the brake places it
  NAV_LAND_POINT_PLACED,   // land.coord, where that brake came to rest
} nav_land_point_t;

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
    bool diverging;        // moving off the desired velocity
    uint32_t diverging_ms; // start of the current divergence window
    float diverging_speed; // ground speed at that start
  } horizontal;
  // How the leg to the current target is flown.
  struct {
    bool line;           // follow the line from nav_route.leg_origin rather than aim at the target
    vec2_t dir;          // unit vector along the line
    float end_speed;     // m/s at the arrival radius; zero stops on the target
    float arrive_radius; // meters
    bool passing;        // entered through a PASS corner: no realign until the nose follows the line
  } leg;
  // A STOP waypoint's hold time runs from the first arrival on it.
  struct {
    bool started;
    uint32_t started_ms;
  } waypoint_hold;
  // Updated once per pass toward the current target.
  struct {
    vec2_t delta;        // meters to the target
    vec2_t dir;          // unit direction steering flies, toward the carrot or the target
    float course;        // degrees of that direction; the nose follows it
    float heading_error; // degrees from the heading to course, -180..180
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
    float hover;          // throttle the craft hovers on, followed until the trim winds down
    vec3_t accel;         // m/s^2 body-frame acceleration, 1 Hz low-passed
    float quiet;          // seconds level and still; negative while the descent starts
    float landed;         // seconds of that at the throttle floor
    float stopped;        // seconds without vertical motion
    float grounded;       // seconds carried by the ground at the throttle floor
    float thrust;         // throttle, low-passed like accel
    float wound;          // throttle trim wound down so far
    nav_land_point_t point;
    gps_coord_t coord;    // the placed landing point
  } land;
  struct {
    bool active;
    uint32_t started_ms;
    float altitude; // meters above launch, latched when a sensor is lost
  } pause;
} guide;

static float nav_arrive_radius() {
  return MAX(guide.leg.arrive_radius, ARRIVAL_RADIUS);
}

// m/s for a mission_waypoint_t.speed; zero uses the configured cruise speed,
// no slower than a waypoint may set.
static float nav_cruise_speed(uint8_t waypoint_speed) {
  return MAX(waypoint_speed ? waypoint_speed * MISSION_SPEED_SCALE : profile.navigation.travel_speed, MISSION_SPEED_MIN);
}

// The fastest approach that still stops within distance, reserving response
// time as well as braking distance: d = v*t + v^2/(2*a), with one smooth
// slope from distance / t near the end. A leg ending at end_speed eases onto
// it there instead of meeting it with a kink.
static float nav_approach_speed(float distance, float end_speed) {
  const float delay_speed = BRAKE_ACCEL * BRAKE_DELAY;
  const float stop_speed = sqrtf(delay_speed * delay_speed + 2.0f * BRAKE_ACCEL * distance) - delay_speed;
  return sqrtf(stop_speed * stop_speed + end_speed * end_speed);
}

// A PASS waypoint ends its leg at the corner speed for the turn onto the next
// one. The velocity turns within the acceleration limits: handing over half
// that turn's travel early centres the curve on the corner.
static void nav_latch_corner() {
  const uint8_t index = nav_route.index;
  const mission_waypoint_t &waypoint = mission.waypoints[index];
  if (waypoint.type != MISSION_WAYPOINT_PASS || index + 1 >= mission.count || !guide.leg.line)
    return;
  const mission_waypoint_t &next = mission.waypoints[index + 1];
  const vec2_t next_leg = nav_position_delta(waypoint.coord, next.coord);
  const float length = vec2_magnitude(next_leg);
  if (length < 1.0f)
    return;
  const float cos_turn = vec2_dot(guide.leg.dir, next_leg) / length;
  const float shed = sqrtf(MAX(2.0f - 2.0f * cos_turn, 0.0f)); // 2 sin(turn / 2)
  float speed = MIN(nav_cruise_speed(waypoint.speed), nav_cruise_speed(next.speed));
  if (shed > 0.05f)
    speed = constrain(PASS_CORNER_DELTA_V / shed, MIN(PASS_CORNER_SPEED_MIN, speed), speed);
  // The next leg stops on its waypoint unless it passes through it too.
  if (next.type != MISSION_WAYPOINT_PASS || index + 2 >= mission.count)
    speed = MIN(speed, nav_approach_speed(length, 0.0f));
  guide.leg.end_speed = speed;
  const float turn_time = speed * shed / VEL_ACCEL_LIMIT + VEL_ACCEL_LIMIT / VEL_JERK_LIMIT;
  guide.leg.arrive_radius = MAX(ARRIVAL_RADIUS, MIN(0.5f * speed * turn_time, MIN(PASS_SWITCH_RADIUS_MAX, 0.5f * length)));
}

// Latch how the leg to the current target is flown: along its line from
// nav_route.leg_origin, and how it ends. Home, and a target closer than a
// carrot lead, is aimed at directly.
static void nav_latch_leg() {
  guide.leg = {.arrive_radius = ARRIVAL_RADIUS};
  if (state.nav_target == NAV_TARGET_HOME)
    return;
  const vec2_t leg = nav_position_delta(nav_route.leg_origin, nav_target_coord());
  const float length = vec2_magnitude(leg);
  guide.leg.line = length > CARROT_LEAD_MIN;
  if (guide.leg.line)
    guide.leg.dir = leg * (1.0f / length);
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
  const vec2_t delta = nav_position_delta(state.position, state.nav_target_position);
  guide.steer.delta = delta;

  vec2_t aim = delta;
  bool carrot = false;
  if (guide.leg.line) {
    const float along = vec2_dot(delta, guide.leg.dir);
    const vec2_t cross = delta - guide.leg.dir * along;
    const float speed = vec2_magnitude(nav_velocity.desired_velocity);
    const float lead = MAX(constrain(speed * CARROT_LEAD_TIME, CARROT_LEAD_MIN, CARROT_LEAD_MAX), vec2_magnitude(cross));
    carrot = along > lead;
    if (carrot)
      aim = cross + guide.leg.dir * lead;
  }
  const float aim_length = vec2_magnitude(aim);
  guide.steer.dir = aim_length > 0.01f ? aim * (1.0f / aim_length) : vec2_t{};
  // Aimed at the target, the nose follows its published bearing.
  guide.steer.course = carrot ? normalize_rad(atan2approx_rad(aim.east, aim.north)) * RADTODEG : state.nav_target_bearing;
  // Course and heading both lie in 0..360.
  float heading_error = guide.steer.course - state.heading;
  if (heading_error > 180.0f)
    heading_error -= 360.0f;
  else if (heading_error < -180.0f)
    heading_error += 360.0f;
  guide.steer.heading_error = heading_error;
}

static float nav_heading_error() {
  return guide.steer.heading_error;
}

// Hold the nose where it points.
static void nav_hold_yaw() {
  state.rx_override.yaw = state.nav_yaw_rate = 0.0f;
}

static void nav_update_yaw(float distance, float dt) {
  // Navigation supplies radians/s directly; rx_override.yaw must bypass pilot rates.
  state.rx_override.yaw = 0.0f;
  if (distance <= YAW_HOLD_RADIUS) {
    state.nav_yaw_rate = 0.0f;
    return;
  }
  float heading_error = nav_heading_error();
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 9, (int16_t)(heading_error * 10)); // 0.1 degrees
  if (fabsf(heading_error) < YAW_DEADBAND)
    heading_error = 0.0f;
  // Yaw must work while stationary; GPS ground speed does not limit yaw authority.
  const float max_yaw_rate = YAW_MAX_RATE * DEGTORAD;
  const float yaw_rate = constrain(heading_error * DEGTORAD * YAW_KP, -max_yaw_rate, max_yaw_rate);
  state.nav_yaw_rate += dt / (YAW_TIME_CONSTANT + dt) * (yaw_rate - state.nav_yaw_rate);
}

// Fly the velocity request. Lean less close to the landing ground, so
// touchdown does not catch a leg.
static void nav_fly_velocity(vec2_t request, bool moving_away, float dt) {
  const bool low = state.nav_state == NAV_STATE_LAND && state.altitude - guide.land.ground < LAND_LOW_HEIGHT;
  nav_update_velocity_control(request, moving_away, low ? LAND_LOW_AUTHORITY : 1.0f, dt);
}

// Fly along the unit direction dir toward a point delta meters away at the
// current target's speed, slowing to stop on it, or to end_speed at
// end_radius from it.
static void nav_steer(vec2_t delta, vec2_t dir, float end_speed, float end_radius, float dt) {
  const float distance = vec2_magnitude(delta);

  uint8_t waypoint_speed = 0;
  if (state.nav_target == NAV_TARGET_GUIDED)
    waypoint_speed = nav_route.guided.speed;
  else if (state.nav_target == NAV_TARGET_MISSION)
    waypoint_speed = mission.waypoints[nav_route.index].speed;
  const float cruise_speed = nav_cruise_speed(waypoint_speed);
  // Slow toward a stop continuously, without a minimum approach speed.
  const float approach_speed = nav_approach_speed(end_speed > 0.0f ? MAX(distance - end_radius, 0.0f) : distance, end_speed);

  if (guide.horizontal.phase == NAV_HORIZONTAL_IDLE && distance > 0.01f) {
    // Keep the speed already made along the direction, even above cruise, and
    // slew it down: braking to a stop first pitches hard against the motion.
    const float along = vec2_dot(vec3_horizontal(state.velocity), dir);
    nav_velocity.desired_velocity = dir * constrain(along, 0.0f, approach_speed);
  }
  const float desired_speed = distance > 0.01f ? MIN(cruise_speed, approach_speed) : 0.0f;

  // Do not build a persistent tilt demand while heading recovery is needed.
  // Use actual progress toward the point, independent of estimated yaw.
  // Leaving a position hold still creeps toward the hold point; decaying the
  // wind integral then would start the leg without it.
  const bool moving_away = state.nav_state == NAV_STATE_NAVIGATE &&
                           vec2_dot(vec3_horizontal(state.gps_velocity), delta) < -VEL_AWAY_SPEED * distance;
  nav_fly_velocity(dir * desired_speed, moving_away, dt);
}

// Fly toward target, slowing to stop on it.
static void nav_steer_to(gps_coord_t target, float dt) {
  const vec2_t delta = nav_position_delta(state.position, target);
  const float distance = vec2_magnitude(delta);
  nav_steer(delta, distance > 0.01f ? delta * (1.0f / distance) : vec2_t{}, 0.0f, 0.0f, dt);
}

// Fly the leg toward the navigation target, along its line where it has one.
static void nav_update_horizontal_control(float dt) {
  nav_steer(guide.steer.delta, guide.steer.dir, guide.leg.end_speed, nav_arrive_radius(), dt);
  guide.horizontal.phase = NAV_HORIZONTAL_STEERING;
  nav_update_yaw(state.nav_target_distance, dt);
}

static void nav_reset_horizontal() {
  guide.horizontal = {};
  nav_velocity = {};
  state.nav_yaw_rate = 0.0f;
  state.rx_override.roll = state.rx_override.pitch = state.rx_override.yaw = 0.0f;
}

// Slow to a stop over the ground without steering toward the target, then
// latch where it came to rest. Earth-frame tilt needs a trusted heading;
// level without one.
static void nav_update_brake(float dt) {
  if (state.heading_confidence < NAV_MIN_HEADING_CONFIDENCE) {
    nav_reset_horizontal();
    return;
  }
  if (guide.horizontal.phase == NAV_HORIZONTAL_IDLE) {
    // Slow down from the ground velocity instead of pitching hard against it.
    nav_velocity.desired_velocity = vec3_horizontal(state.velocity);
  }
  guide.horizontal.phase = NAV_HORIZONTAL_BRAKING;
  nav_fly_velocity({}, false, dt);
  if (nav_velocity.desired_velocity.north == 0.0f && nav_velocity.desired_velocity.east == 0.0f) {
    guide.horizontal.phase = NAV_HORIZONTAL_HOLDING;
    guide.horizontal.hold_coord = state.position;
  }
}

// Brake, then hold the point where the brake came to rest. Holding zero
// velocity alone lets gusts and GPS velocity error drift the position.
static void nav_update_position_hold(float dt) {
  if (guide.horizontal.phase != NAV_HORIZONTAL_HOLDING) {
    nav_update_brake(dt);
    return;
  }
  if (state.heading_confidence < NAV_MIN_HEADING_CONFIDENCE) {
    nav_reset_horizontal();
    return;
  }
  nav_steer_to(guide.horizontal.hold_coord, dt);
}

// The velocity loop has diverged for DIVERGE_TIMEOUT_MS, judged on GPS
// velocity: the fused estimate turns with the heading it checks. Falling short along
// the desired velocity is not divergence, nor is overshooting it while braking;
// moving across or against it is. Holding, drift alone may be wind beyond the
// tilt limit or a slow stop; a wrong heading tilts with the drift, so it must
// also gain DIVERGE_SPEED within the window.
static bool nav_update_divergence(uint32_t now_ms) {
  const vec2_t desired = nav_velocity.desired_velocity;
  const vec2_t gps_vel = vec3_horizontal(state.gps_velocity);
  const float desired_speed = vec2_magnitude(desired);
  const float speed = vec2_magnitude(gps_vel);
  const bool holding = desired_speed <= DIVERGE_HOLD_SPEED;
  bool diverging;
  if (!holding) {
    const float along = vec2_dot(gps_vel, desired) / desired_speed;
    const float across = vec2_cross(desired, gps_vel) / desired_speed;
    diverging = fabsf(across) > DIVERGE_SPEED || along < -DIVERGE_SPEED;
  } else {
    diverging = speed > DIVERGE_SPEED;
  }
  if (!diverging) {
    guide.horizontal.diverging = false;
    return false;
  }
  if (!guide.horizontal.diverging) {
    guide.horizontal.diverging = true;
    guide.horizontal.diverging_ms = now_ms;
    guide.horizontal.diverging_speed = speed;
  }
  if (now_ms - guide.horizontal.diverging_ms < DIVERGE_TIMEOUT_MS)
    return false;
  if (!holding || speed > guide.horizontal.diverging_speed + DIVERGE_SPEED)
    return true;
  // Steady drift: judge the next window from here.
  guide.horizontal.diverging_ms = now_ms;
  guide.horizontal.diverging_speed = speed;
  return false;
}

// What the phase's progress is measured by.
static float nav_progress_error() {
  if (state.nav_state == NAV_STATE_CLIMB || state.nav_state == NAV_STATE_TAKEOFF)
    return fabsf(guide.target_altitude - state.altitude);
  if (state.nav_state == NAV_STATE_LAND)
    return state.altitude - guide.land.ground;
  return state.nav_target_distance;
}

// Enter a phase toward the current target and supervise its progress from here.
static void nav_set_phase(nav_state_t phase, uint32_t now_ms) {
  nav_update_target();
  state.nav_state = phase;
  guide.progress_ms = now_ms;
  guide.horizontal.diverging = false;
  guide.best_error = nav_progress_error();
}

// Fly forward to learn the heading, without earth-frame steering.
static void nav_begin_heading_acquisition(uint32_t now_ms) {
  nav_reset_horizontal();
  nav_set_phase(NAV_STATE_ACQUIRE_HEADING, now_ms);
  guide.acquisition.origin = state.position;
  // Momentum across the nose would dominate the course; let drag take it out first.
  guide.acquisition.leveling = state.gps_speed > ACQUIRE_START_SPEED;
}

// Navigation cannot be trusted. Level and hold altitude, then give up.
static void nav_begin_failure(nav_state_t failure, uint32_t now_ms) {
  nav_reset_horizontal();
  guide.target_altitude = state.altitude;
  nav_set_phase(failure, now_ms);
}

// Leave the mission or guided target. nav_route keeps where the mission left off.
static void nav_clear_target() {
  state.nav_target = NAV_TARGET_HOME;
  nav_route.returning = false;
  guide.waypoint_hold = {};
}

// Climb in place by rth_altitude, then fly home.
static void nav_begin_return(uint32_t now_ms) {
  nav_clear_target();
  // Home is aimed at directly.
  guide.leg = {.arrive_radius = ARRIVAL_RADIUS};
  guide.target_altitude = state.altitude + profile.navigation.rth_altitude;
  nav_set_phase(NAV_STATE_CLIMB, now_ms);
}

// The altitude to fly waypoint index at. A LAND leg flies level at
// level_altitude: the waypoint's altitude is the ground it descends to.
static float nav_mission_altitude(uint8_t index, float level_altitude) {
  const mission_waypoint_t &waypoint = mission.waypoints[index];
  return waypoint.type == MISSION_WAYPOINT_LAND ? level_altitude : waypoint.altitude;
}

static void nav_begin_land(uint32_t now_ms) {
  guide.land = {
      .ground = (float)mission.waypoints[nav_route.index].altitude,
      // The configured hover may be far off; the trim flown so far is not.
      .hover = constrain(nav_hover_throttle() + nav_alt.integral, LAND_THROTTLE_MIN, nav_throttle_max()),
      // The landing begins stopped over the waypoint. Until the descent gets
      // going, the checks would take that for touchdown.
      .quiet = -LAND_START_TIME,
      .stopped = -LAND_START_TIME,
  };
  guide.land.thrust = guide.land.hover;
  nav_set_phase(NAV_STATE_LAND, now_ms);
}

// Still within the reached waypoint's hold time, which starts now on the
// first arrival.
static bool nav_waypoint_holding(uint32_t now_ms) {
  const uint8_t hold = mission.waypoints[nav_route.index].hold;
  if (!hold)
    return false;
  if (!guide.waypoint_hold.started)
    guide.waypoint_hold = {.started = true, .started_ms = now_ms};
  return now_ms - guide.waypoint_hold.started_ms < hold * 1000U;
}

// Within the arrival radius of the target: continue the mission, or hold.
static void nav_arrive(uint32_t now_ms) {
  const mission_waypoint_t &reached = mission.waypoints[nav_route.index];
  guide.relearned = false;
  if (state.nav_target != NAV_TARGET_MISSION || nav_route.complete) {
    nav_set_phase(NAV_STATE_HOLD, now_ms);
  } else if (reached.type == MISSION_WAYPOINT_LAND) {
    nav_begin_land(now_ms);
  } else if (nav_waypoint_holding(now_ms)) {
    // Hover on the waypoint until its hold time has passed, then arrive again.
    nav_set_phase(NAV_STATE_HOLD, now_ms);
  } else if (nav_route.index + 1 < mission.count) {
    guide.waypoint_hold = {};
    // The next leg runs from this waypoint and its altitude, also while still
    // catching up with it; a takeoff climbed where it lifted off.
    nav_route_next();
    if (reached.type == MISSION_WAYPOINT_TAKEOFF)
      nav_route.leg_origin = nav_route.altitude_origin = state.position;
    guide.target_altitude = nav_mission_altitude(nav_route.index, guide.target_altitude);
    nav_latch_leg();
    if (reached.type == MISSION_WAYPOINT_PASS) {
      // Carry the corner speed onto the next leg; the nose follows it round.
      guide.leg.passing = true;
      nav_set_phase(NAV_STATE_NAVIGATE, now_ms);
    } else {
      // Stop on the waypoint, then turn toward the next.
      nav_set_phase(NAV_STATE_TURN, now_ms);
    }
  } else if (mission.end_action == MISSION_END_RTH) {
    nav_route.complete = true;
    nav_begin_return(now_ms);
    nav_route.returning = true;
  } else {
    nav_route.complete = true;
    guide.waypoint_hold = {};
    nav_set_phase(NAV_STATE_HOLD, now_ms);
  }
}

// The mission flies its TAKEOFF waypoint: climbing in place to its altitude.
static bool nav_mission_takeoff() {
  return state.nav_target == NAV_TARGET_MISSION && !nav_route.complete &&
         mission.waypoints[nav_route.index].type == MISSION_WAYPOINT_TAKEOFF;
}

// Fly to the selected target at guide.target_altitude: climb in place to a
// higher one, descend evenly toward a lower one on the way. A TAKEOFF
// waypoint flown in the air is reached by climbing in place. Steering needs a
// trusted heading; learn it first if needed. The turn brakes from the
// current tilt and velocity; it does not level first.
static void nav_begin_leg(uint32_t now_ms) {
  if (guide.target_altitude > state.altitude + ALT_REACHED) {
    nav_set_phase(NAV_STATE_CLIMB, now_ms);
    return;
  }
  if (nav_mission_takeoff()) {
    nav_arrive(now_ms);
    return;
  }
  // A climb in place, or a heading acquisition run, starts the leg here.
  nav_start_leg();
  if (state.heading_confidence < NAV_MIN_HEADING_CONFIDENCE)
    nav_begin_heading_acquisition(now_ms);
  else
    nav_set_phase(NAV_STATE_TURN, now_ms);
}

// Fly to a new target. Flying or holding with a trusted heading continues
// the leg without leveling to turn first: it keeps the velocity, realigns
// past REALIGN_ERROR and changes altitude evenly on the way.
static void nav_retarget(uint32_t now_ms) {
  const bool underway = state.heading_confidence >= NAV_MIN_HEADING_CONFIDENCE &&
                        (state.nav_state == NAV_STATE_NAVIGATE || state.nav_state == NAV_STATE_HOLD ||
                         state.nav_state == NAV_STATE_BRAKE);
  if (underway)
    nav_set_phase(NAV_STATE_NAVIGATE, now_ms);
  else
    nav_begin_leg(now_ms);
}

// Fly the mission leg to index from here; the caller enters a phase for it.
static void nav_mission_select(uint8_t index) {
  state.nav_target = NAV_TARGET_MISSION;
  nav_route.index = index;
  guide.waypoint_hold = {};
  guide.target_altitude = nav_mission_altitude(index, state.altitude);
  nav_start_leg();
}

// Fly the stored mission from index; the caller owns the controls.
static void nav_mission_fly(uint8_t index, uint32_t now_ms) {
  nav_route.complete = false;
  nav_mission_select(index);
  nav_begin_leg(now_ms);
}

// Inside the arrival radius, or for a PASS corner flown fast, past the
// waypoint along the line and close to it.
static bool nav_leg_arrived() {
  if (state.nav_target_distance < nav_arrive_radius())
    return true;
  if (guide.leg.end_speed <= 0.0f || !guide.leg.line)
    return false;
  const float along = vec2_dot(guide.steer.delta, guide.leg.dir);
  const float cross = vec2_magnitude(guide.steer.delta - guide.leg.dir * along);
  return along < 0.0f && cross < PASS_ARRIVAL_CROSS * nav_arrive_radius();
}

// Progress toward the phase's goal within its timeout, measured by error.
// progress is the improvement that counts.
static bool nav_update_progress(float error, float progress, uint32_t timeout_ms, uint32_t now_ms) {
  if (error + progress < guide.best_error) {
    guide.best_error = error;
    guide.progress_ms = now_ms;
  }
  return now_ms - guide.progress_ms <= timeout_ms;
}

// Hand the controls back to the pilot. Motors spinning up on the ground stop
// instead of jumping to the throttle stick, which the pilot raised to start
// the spin-up.
static void nav_stop() {
  if (state.nav_state == NAV_STATE_TAKEOFF_SPINUP)
    control_request_disarm();
  if (state.nav_state >= NAV_STATE_CLIMB) {
    flags.controls_override = 0;
    state.rx_override = (vec4_t){0};
  }
  guide = {};
  nav_velocity = {};
  state.nav_state = NAV_STATE_INACTIVE;
  nav_clear_target();
  nav_update_target();
  state.nav_yaw_rate = 0.0f;
}

static void nav_abort() {
  nav_stop();
  // Do not automatically retry a failed rescue during the same signal loss.
  state.nav_state = NAV_STATE_ABORTED;
  // Nor let the ground station restart it: the mission switch grants that again.
  nav.guided_granted = false;
}

// Ready or spinning up: the motors have not lifted the craft yet.
static bool nav_takeoff_on_ground() {
  return state.nav_state == NAV_STATE_TAKEOFF_READY ||
         state.nav_state == NAV_STATE_TAKEOFF_SPINUP;
}

// A mission takeoff, from waiting on the ground through the climb in place.
static bool nav_taking_off() {
  return nav_takeoff_on_ground() || state.nav_state == NAV_STATE_TAKEOFF;
}

// Roll or pitch hands control back from the takeoff and the heading run
// after it. Spin-up and the throttle raise may jostle the sticks at first,
// and a lost link holds stale ones.
static bool nav_takeoff_sticks_moved(uint32_t now_ms) {
  if (flags.failsafe_signal_lost)
    return false;
  if (state.nav_state != NAV_STATE_TAKEOFF_READY && now_ms - guide.takeoff.started_ms < TAKEOFF_STICK_GRACE_MS)
    return false;
  return fabsf(state.rx.roll) >= TAKEOFF_STICK_ABORT || fabsf(state.rx.pitch) >= TAKEOFF_STICK_ABORT;
}

// The stick abort lasts from takeoff until the route is flown with a trusted
// heading: through the climb, the heading run and the turn onto the first leg.
static bool nav_takeoff_abort_window() {
  return nav_taking_off() || state.nav_state == NAV_STATE_ACQUIRE_HEADING || state.nav_state == NAV_STATE_TURN;
}

// Tipped over on its legs: stop the motors.
static bool nav_takeoff_tipped() {
  const bool low = state.nav_state == NAV_STATE_TAKEOFF_SPINUP || state.altitude - guide.takeoff.altitude < TAKEOFF_TIP_HEIGHT;
  if (!low || (fabsf(state.attitude.roll) <= TAKEOFF_TIP_ANGLE && fabsf(state.attitude.pitch) <= TAKEOFF_TIP_ANGLE))
    return false;
  nav_stop();
  control_request_disarm();
  return true;
}

// Spin up from the ground until lift-off, then climb in place.
static void nav_update_spinup(float dt, uint32_t now_ms) {
  state.rx_override.roll = state.rx_override.pitch = 0.0f;
  nav_hold_yaw();
  if (nav_takeoff_tipped())
    return;

  // No lift-off at full throttle: stop the motors.
  const float throttle_max = nav_throttle_max();
  if (guide.takeoff.throttle >= throttle_max) {
    nav_stop();
    return;
  }

  const float spinup = TAKEOFF_SPINUP_FRACTION * nav_hover_throttle();
  const float rate = guide.takeoff.throttle < spinup ? spinup / TAKEOFF_SPINUP_TIME : TAKEOFF_RAMP_RATE;
  guide.takeoff.throttle = MIN(guide.takeoff.throttle + rate * dt, throttle_max);
  state.rx_override.throttle = guide.takeoff.throttle;

  guide.takeoff.rising = state.velocity.up > TAKEOFF_LIFTOFF_SPEED ? guide.takeoff.rising + dt : 0.0f;
  if (guide.takeoff.rising >= TAKEOFF_LIFTOFF_TIME || state.altitude - guide.takeoff.altitude > TAKEOFF_LIFTOFF_HEIGHT) {
    // The lift-off throttle and the climb it makes seed the hover trim.
    nav_altitude_start(guide.takeoff.throttle);
    nav_set_phase(NAV_STATE_TAKEOFF, now_ms);
  }
}

// Stop and hold position while climbing to guide.target_altitude; reached
// within ALT_REACHED of it. Returns false once the climb gave up.
static bool nav_update_climb(float dt, uint32_t now_ms, bool *reached) {
  nav_update_position_hold(dt);
  nav_hold_yaw();
  const float altitude_error = fabsf(state.altitude - guide.target_altitude);
  *reached = altitude_error < ALT_REACHED;
  return nav_update_progress(altitude_error, CLIMB_PROGRESS, CLIMB_TIMEOUT_MS, now_ms);
}

// Climb in place to the TAKEOFF altitude and continue the route. Level until
// the heading is learned; the heading run follows the climb. Returns false
// once the climb gave up.
static bool nav_update_takeoff(float dt, uint32_t now_ms) {
  if (nav_takeoff_tipped())
    return true;
  bool reached;
  if (!nav_update_climb(dt, now_ms, &reached))
    return false;
  if (reached)
    nav_arrive(now_ms);
  return true;
}

// Possibly on the ground: stop leaning against the position error and let
// the wind trim decay, so the craft cannot tip over on its legs.
static void nav_update_soften(float dt) {
  nav_velocity.integral *= expf(-SOFTEN_DECAY_RATE * dt);
  nav_velocity.desired_velocity = {};
  nav_velocity.desired_accel = {};
  guide.horizontal.phase = NAV_HORIZONTAL_IDLE;
  const float slew_step = ANGLE_SLEW_RATE * dt;
  nav_velocity.pitch_command = slew(nav_velocity.pitch_command, 0.0f, slew_step);
  nav_velocity.roll_command = slew(nav_velocity.roll_command, 0.0f, slew_step);
  state.rx_override.pitch = nav_velocity.pitch_command;
  state.rx_override.roll = nav_velocity.roll_command;
  nav_hold_yaw();
}

// The pilot's roll and pitch turned onto the ground by the heading, north and
// east. Positive pitch tilts toward the nose, positive roll toward its right.
static vec2_t nav_sticks_earth() {
  const float cos_heading = fastcos(state.heading * DEGTORAD);
  const float sin_heading = fastsin(state.heading * DEGTORAD);
  return vec2_t{{state.rx.pitch * cos_heading - state.rx.roll * sin_heading,
                 state.rx.pitch * sin_heading + state.rx.roll * cos_heading}};
}

// The pilot's roll and pitch as an earth-frame velocity request, while the
// link is trusted and the heading can turn the sticks onto the ground.
static bool nav_land_reposition(vec2_t *request) {
  if (flags.failsafe || flags.failsafe_signal_lost || state.failsafe_owner == FAILSAFE_OWNER_MISSION ||
      state.heading_confidence < NAV_MIN_HEADING_CONFIDENCE)
    return false;
  if (fabsf(state.rx.roll) < LAND_REPOSITION_DEADBAND && fabsf(state.rx.pitch) < LAND_REPOSITION_DEADBAND)
    return false;
  *request = nav_sticks_earth() * LAND_REPOSITION_SPEED;
  return true;
}

// On the landing point: the waypoint, or where the brake after the sticks
// came to rest.
static bool nav_land_on_point() {
  if (guide.land.point == NAV_LAND_POINT_WAYPOINT)
    return state.nav_target_distance < nav_arrive_radius();
  if (guide.land.point == NAV_LAND_POINT_MOVED)
    return false;
  return vec2_magnitude(nav_position_delta(state.position, guide.land.coord)) < nav_arrive_radius();
}

// The final descent: within LAND_SLOW_HEIGHT of the LAND ground.
static bool nav_land_final() {
  return state.nav_state == NAV_STATE_LAND && state.altitude - guide.land.ground <= LAND_SLOW_HEIGHT;
}

// Descend on the LAND waypoint until the land detector sees touchdown, then
// disarm. The sticks move the landing point.
static void nav_update_land(float dt, uint32_t now_ms) {
  const float throttle_floor = MIN(LAND_THROTTLE_MIN, nav_throttle_min());
  const float rate = nav_land_final() ? LAND_FINAL_RATE : ALT_MAX_RATE;
  nav_update_altitude_control_above(state.altitude - LAND_TARGET_DEPTH, dt, 0.0f, rate, throttle_floor);

  // Earth-frame acceleration has the same magnitude as the body-frame
  // specific force less gravity.
  const float accel_gain = dt / (LAND_ACCEL_FILTER_TIME + dt);
  for (uint32_t i = 0; i < 3; i++) {
    const float accel = (state.accel_raw.axis[i] - state.GEstG.axis[i]) / ACC_1G * GRAVITY;
    guide.land.accel.axis[i] += accel_gain * (accel - guide.land.accel.axis[i]);
  }

  const bool stalled = !nav_update_progress(state.altitude - guide.land.ground, LAND_PROGRESS, LAND_STALL_MS, now_ms);
  const bool maybe_landed = guide.land.quiet >= LAND_MAYBE_TIME || guide.land.stopped >= LAND_STOPPED_TIME || stalled;
  // Low, on the landing point and no longer descending at the final rate: the
  // ground may hold the craft. Creeping onto the point would lean against
  // the ground, which disturbs the checks below. Without GPS, land level
  // where it is: the descent and the checks need only the barometer.
  const bool contact = state.altitude - guide.land.ground < LAND_LOW_HEIGHT && fabsf(state.velocity.up) < LAND_FINAL_RATE &&
                       nav_land_on_point();
  vec2_t reposition;
  if (maybe_landed || !flags.gps_valid) {
    nav_update_soften(dt);
  } else if (nav_land_reposition(&reposition)) {
    guide.land.point = NAV_LAND_POINT_MOVED;
    guide.horizontal.phase = NAV_HORIZONTAL_STEERING;
    nav_fly_velocity(reposition, false, dt);
    nav_hold_yaw();
  } else if (contact) {
    nav_update_soften(dt);
  } else if (guide.land.point == NAV_LAND_POINT_PLACED) {
    nav_steer_to(guide.land.coord, dt);
    guide.horizontal.phase = NAV_HORIZONTAL_STEERING;
    nav_hold_yaw();
  } else if (guide.land.point == NAV_LAND_POINT_MOVED) {
    // Released: brake, and land where it comes to rest.
    nav_update_brake(dt);
    nav_hold_yaw();
    if (guide.horizontal.phase == NAV_HORIZONTAL_HOLDING) {
      guide.land.point = NAV_LAND_POINT_PLACED;
      guide.land.coord = guide.horizontal.hold_coord;
    }
  } else {
    nav_update_horizontal_control(dt);
  }

  const float stick = sqrtf(state.rx_override.roll * state.rx_override.roll + state.rx_override.pitch * state.rx_override.pitch);
  const float request = stick * MAX(profile.rate.level_max_angle, 0.0f);
  const bool level_request = request < LAND_ANGLE_REQUEST_MAX;
  const bool level = vec3_magnitude(state.angle_error) < LAND_ANGLE_ERROR_MAX;
  const bool still = vec3_magnitude(guide.land.accel) < LAND_ACCEL_MAX;
  const bool stopped = fabsf(state.velocity.up) < LAND_STOPPED_SPEED;
  const bool at_floor = nav_alt.throttle_command <= throttle_floor + LAND_FLOOR_MARGIN;
  const bool quiet = level_request && level && still && stopped;
  guide.land.quiet = quiet ? guide.land.quiet + dt : 0.0f;
  guide.land.landed = quiet && at_floor ? guide.land.landed + dt : 0.0f;
  guide.land.stopped = stopped ? guide.land.stopped + dt : 0.0f;
  // Specific force in g: the low-passed acceleration with gravity added back,
  // so a gravity estimate that lags on a slope cancels out.
  const vec3_t force = guide.land.accel * (1.0f / GRAVITY) + state.GEstG * (1.0f / ACC_1G);
  const float weight = vec3_magnitude(force);
  guide.land.thrust += accel_gain * (nav_alt.throttle_command - guide.land.thrust);
  if (guide.land.wound <= 0.0f && !maybe_landed) {
    const float hover = MAX(guide.land.thrust / MAX(weight, 0.5f), LAND_THROTTLE_MIN);
    guide.land.hover += dt / (LAND_HOVER_TIME + dt) * (hover - guide.land.hover);
  }
  const float thrust = guide.land.thrust / guide.land.hover;
  const bool reduced = thrust < LAND_CARRIED_THRUST;
  const bool carried = reduced && weight > MIN(0.5f * (1.0f + thrust), LAND_CARRIED_FORCE);
  const bool falling = reduced && !carried;
  guide.land.grounded = carried && at_floor ? guide.land.grounded + dt : 0.0f;
  // Yaw is held while landing, so its heading error channel carries the checks.
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 9, (int16_t)(level_request | level << 1 | still << 2 | stopped << 3 | at_floor << 4 | maybe_landed << 5 | carried << 6 | falling << 7));

  if (falling && guide.land.wound > 0.0f) {
    // Aloft after all: give the trim back and let the descent get going
    // again before the checks count.
    nav_alt.integral += guide.land.wound;
    guide.land.wound = 0.0f;
    guide.land.quiet = -LAND_START_TIME;
    guide.land.stopped = -LAND_START_TIME;
    guide.progress_ms = now_ms;
  } else if (maybe_landed || guide.land.wound > 0.0f) {
    // Wind the trim down toward the floor, until the accelerometer answers.
    const float trim_min = throttle_floor * nav_tilt() - nav_hover_throttle();
    const float trim = MAX(nav_alt.integral - LAND_WIND_DOWN_RATE * guide.land.hover * dt, trim_min);
    guide.land.wound += nav_alt.integral - trim;
    nav_alt.integral = trim;
  }
  if (guide.land.landed >= LAND_DETECT_TIME || guide.land.grounded >= LAND_DETECT_TIME) {
    nav_route.complete = true;
    control_request_disarm();
  }
}

// The pilot's roll and pitch beyond the deadband as a velocity over the
// ground, up to the travel speed at full stick.
static bool nav_poshold_reposition(vec2_t *request) {
  const float deflection = sqrtf(state.rx.roll * state.rx.roll + state.rx.pitch * state.rx.pitch);
  if (deflection < POSHOLD_STICK_DEADBAND)
    return false;
  const float share = (MIN(deflection, 1.0f) - POSHOLD_STICK_DEADBAND) / (1.0f - POSHOLD_STICK_DEADBAND);
  *request = nav_sticks_earth() * (nav_cruise_speed(0) * share / deflection);
  return true;
}

// Hold the altitude latched on engagement and the point the brake comes to
// rest at. Roll and pitch fly over the ground, and the hold point follows
// where they let go; yaw turns at the pilot's rates; throttle does nothing.
// Without GPS or a trusted heading the sticks fly level as in angle mode
// while the altitude holds; a heading that diverges is relearned that way,
// from the accelerations, instead of by a heading run. Without altitude the
// pilot takes over.
static void nav_update_poshold(float dt, uint32_t now_ms) {
  if (!flags.altitude_valid) {
    nav_abort();
    return;
  }
  // A lost link holds stale sticks: hold until failsafe takes over.
  const bool sticks = !flags.failsafe_signal_lost;
  vec2_t request;
  if (!flags.gps_valid || state.heading_confidence < NAV_MIN_HEADING_CONFIDENCE) {
    nav_reset_horizontal();
    // Steering starts from the stick tilt once it can.
    nav_velocity.roll_command = state.rx_override.roll = sticks ? state.rx.roll : 0.0f;
    nav_velocity.pitch_command = state.rx_override.pitch = sticks ? state.rx.pitch : 0.0f;
  } else if (sticks && nav_poshold_reposition(&request)) {
    guide.horizontal.phase = NAV_HORIZONTAL_STEERING;
    nav_fly_velocity(request, false, dt);
  } else {
    nav_update_position_hold(dt);
  }
  if (guide.horizontal.phase != NAV_HORIZONTAL_IDLE && nav_update_divergence(now_ms)) {
    imu_reject_heading();
    nav_reset_horizontal();
  }
  state.nav_yaw_rate = 0.0f;
  state.rx_override.yaw = sticks ? state.rx.yaw : 0.0f;
  nav_update_altitude_control(guide.target_altitude, dt);
}

// Run the active phase. Returns false once it gave up.
static bool nav_update_phase(float dt, uint32_t now_ms) {
  switch (state.nav_state) {
  case NAV_STATE_CLIMB: {
    // Navigation starts at the target altitude.
    bool reached;
    if (!nav_update_climb(dt, now_ms, &reached))
      return false;
    if (reached)
      nav_begin_leg(now_ms);
    return true;
  }

  case NAV_STATE_TAKEOFF_READY:
    // Motors stopped until the pilot raises the throttle stick.
    state.rx_override = (vec4_t){0};
    state.nav_yaw_rate = 0.0f;
    if (state.rx.throttle >= TAKEOFF_STICK_MIN) {
      nav_set_phase(NAV_STATE_TAKEOFF_SPINUP, now_ms);
      guide.takeoff.started_ms = now_ms;
      guide.takeoff.altitude = state.altitude;
    }
    return true;

  case NAV_STATE_TAKEOFF_SPINUP:
    nav_update_spinup(dt, now_ms);
    return true;

  case NAV_STATE_TAKEOFF:
    return nav_update_takeoff(dt, now_ms);

  case NAV_STATE_LAND:
    nav_update_land(dt, now_ms);
    return true;

  case NAV_STATE_ACQUIRE_HEADING: {
    // No earth-frame steering until heading is observable. Zero yaw rate holds
    // the physical orientation without chasing GPS corrections to the estimate.
    state.rx_override.roll = 0.0f;
    nav_hold_yaw();
    if (guide.acquisition.leveling) {
      if (state.gps_speed <= ACQUIRE_START_SPEED || now_ms - guide.progress_ms >= ACQUIRE_LEVEL_TIMEOUT_MS) {
        // The run starts here.
        guide.acquisition.leveling = false;
        guide.acquisition.origin = state.position;
        guide.progress_ms = now_ms;
      }
      return true;
    }
    const vec2_t moved = nav_position_delta(guide.acquisition.origin, state.position);
    if (now_ms - guide.progress_ms >= ACQUIRE_TIMEOUT_MS || vec2_magnitude(moved) >= ACQUIRE_DISTANCE) {
      nav_begin_failure(NAV_STATE_HEADING_FAILED, now_ms);
    } else if (state.heading_confidence >= ACQUIRE_CONFIDENCE && !state.heading_correction_flags) {
      // A climb that diverged first finishes in place.
      nav_begin_leg(now_ms);
    } else {
      const float pitch = constrain(profile.rate.level_max_angle, 0.0f, ACQUIRE_PITCH) / MAX(profile.rate.level_max_angle, 1.0f);
      nav_velocity.pitch_command = slew(nav_velocity.pitch_command, pitch, ACQUIRE_SLEW_RATE * dt);
      state.rx_override.pitch = nav_velocity.pitch_command;
    }
    return true;
  }

  case NAV_STATE_HEADING_FAILED:
  case NAV_STATE_NAV_FAILED:
    nav_reset_horizontal();
    return now_ms - guide.progress_ms < FAILURE_HOLD_MS;

  case NAV_STATE_TURN: {
    if (state.heading_confidence < NAV_MIN_HEADING_CONFIDENCE) {
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
      nav_set_phase(NAV_STATE_NAVIGATE, now_ms);
    else if (now_ms - guide.progress_ms > TURN_TIMEOUT_MS)
      nav_begin_failure(NAV_STATE_NAV_FAILED, now_ms);
    return true;
  }

  case NAV_STATE_NAVIGATE: {
    // Allow acceleration at reduced heading confidence before declaring no progress.
    // Close in, a full progress step can exceed the remaining approach.
    const float progress = MIN(PROGRESS_DISTANCE, 0.5f * guide.best_error);
    // Out of a PASS corner the nose comes round with the velocity; stopping
    // to turn would throw the corner speed away.
    if (guide.leg.passing && fabsf(nav_heading_error()) < PASS_TURN_DONE)
      guide.leg.passing = false;
    if (state.nav_target_distance >= ARRIVAL_RADIUS &&
        !nav_update_progress(state.nav_target_distance, progress, PROGRESS_TIMEOUT_MS, now_ms)) {
      nav_begin_failure(NAV_STATE_NAV_FAILED, now_ms);
    } else if (!guide.leg.passing && state.nav_target_distance > REALIGN_RADIUS &&
               fabsf(nav_heading_error()) > REALIGN_ERROR) {
      nav_set_phase(NAV_STATE_TURN, now_ms);
    } else {
      nav_update_horizontal_control(dt);
      if (nav_leg_arrived())
        nav_arrive(now_ms);
    }
    return true;
  }

  case NAV_STATE_BRAKE:
    // Braking tilts in the earth frame. Without a trusted heading learn it
    // first, then return to the hold point, as a HOLD command does.
    if (state.heading_confidence < NAV_MIN_HEADING_CONFIDENCE) {
      nav_begin_heading_acquisition(now_ms);
      return true;
    }
    // HOLD keeps the point where the brake comes to rest, not where it began.
    nav_update_brake(dt);
    nav_hold_yaw();
    nav_route.guided.coord = state.position;
    if (guide.horizontal.phase == NAV_HORIZONTAL_HOLDING)
      nav_set_phase(NAV_STATE_HOLD, now_ms);
    return true;

  case NAV_STATE_HOLD:
    if (state.nav_target_distance > HOLD_DRIFT_RADIUS) {
      // Stationary hover is success. Resume progress supervision if blown out.
      nav_set_phase(NAV_STATE_TURN, now_ms);
    } else {
      nav_update_horizontal_control(dt);
      // A held waypoint flies on once its hold time has passed.
      if (guide.waypoint_hold.started && !nav_waypoint_holding(now_ms))
        nav_arrive(now_ms);
    }
    return true;

  default:
    return true;
  }
}

static void nav_update_control(float dt) {
  const uint32_t now_ms = time_millis();

  if (state.nav_state == NAV_STATE_POSHOLD) {
    // The pilot flies it: no target, progress supervision or heading run.
    nav_update_poshold(dt, now_ms);
    return;
  }

  if (nav_takeoff_on_ground() && (!flags.altitude_valid || !flags.gps_valid)) {
    // Not airborne yet: end the takeoff instead of holding an altitude.
    nav_abort();
    return;
  }
  if (guide.takeoff.sticks_abort) {
    if (!nav_takeoff_abort_window()) {
      guide.takeoff.sticks_abort = false;
    } else if (nav_takeoff_sticks_moved(now_ms)) {
      if (nav_takeoff_on_ground())
        nav_abort();
      else
        nav_stop();
      return;
    }
  }

  // A landing needs no GPS to finish: it descends level where it is.
  const bool landing = state.nav_state == NAV_STATE_LAND && flags.altitude_valid;
  if (!landing && (!flags.altitude_valid || !flags.gps_valid)) {
    if (!guide.pause.active) {
      guide.pause = {.active = true, .started_ms = now_ms, .altitude = state.altitude};
      nav_reset_horizontal();
    }
    if (now_ms - guide.pause.started_ms >= SENSOR_LOSS_TIMEOUT_MS) {
      nav_abort();
      return;
    }
    // Level and hold a latched altitude while the sensor may return. Without
    // a barometer the controller holds zero vertical speed instead.
    nav_update_altitude_control(guide.pause.altitude, dt);
    return;
  }
  if (guide.pause.active) {
    guide.pause.active = false;
    // The progress clock keeps running, so a flapping sensor cannot extend
    // the attempt; only the baseline restarts from the current position.
    guide.best_error = nav_progress_error();
  }

  nav_update_course();

  if (!nav_update_phase(dt, now_ms)) {
    nav_abort();
    return;
  }
  // A takeoff that tipped over handed the controls back.
  if (state.nav_state < NAV_STATE_CLIMB)
    return;

  // GPS course cannot correct a heading this wrong: the correcting roll makes
  // the motion read as not forward. Learn it anew from forward flight. Each
  // phase is timed from its entry. Taking off and landing stay too close to
  // the ground for an acquisition run; they keep the heading they have.
  const bool near_ground = state.nav_state == NAV_STATE_TAKEOFF_SPINUP || state.nav_state == NAV_STATE_TAKEOFF ||
                           state.nav_state == NAV_STATE_LAND;
  if (!near_ground && guide.horizontal.phase != NAV_HORIZONTAL_IDLE && nav_update_divergence(now_ms)) {
    if (guide.relearned) {
      // The learned heading diverged too: wind beyond the tilt limit, or a
      // heading that cannot be learned here.
      nav_begin_failure(NAV_STATE_NAV_FAILED, now_ms);
    } else {
      guide.relearned = true;
      imu_reject_heading();
      nav_begin_heading_acquisition(now_ms);
    }
  }

  switch (state.nav_state) {
  case NAV_STATE_TAKEOFF_READY:
  case NAV_STATE_TAKEOFF_SPINUP:
  case NAV_STATE_LAND:
    // Motors stopped, spinning up, or the landing flies its own descent.
    break;
  case NAV_STATE_TURN:
  case NAV_STATE_NAVIGATE: {
    float rate;
    const float target = nav_leg_altitude(nav_route.altitude_origin, nav_route.leg_altitude, nav_target_coord(), guide.target_altitude, &rate);
    nav_update_altitude_control(target, dt, rate);
    break;
  }
  default:
    nav_update_altitude_control(guide.target_altitude, dt);
    break;
  }
}

static bool nav_can_start() {
  // Any distance is acceptable: close to home, RTH climbs and holds position.
  return state.nav_state != NAV_STATE_ABORTED && flags.home_valid && flags.gps_valid && flags.altitude_valid &&
         profile.navigation.throttle_min < profile.navigation.throttle_max;
}

static bool nav_airborne() {
  return flags.in_air && !flags.on_ground && state.altitude >= MISSION_START_MIN_ALTITUDE;
}

// Take over the controls for navigation, or keep them and leave the current
// target. The caller enters the first phase for the next one.
static void nav_take_control() {
  if (state.nav_state >= NAV_STATE_CLIMB) {
    nav_clear_target();
    return;
  }
  guide = {};
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
    nav_abort();
    return;
  }
  // An RTH request owns a return the mission started.
  nav_route.returning = false;
  // Already guided: retarget home without a second takeover. A return is not
  // a takeoff command: the pilot must have flown since arming, or a switch
  // left on would lift off once GPS and home become valid. Position hold
  // needs no home; without one it keeps holding.
  if (state.nav_target == NAV_TARGET_HOME) {
    const bool returning = state.nav_state >= NAV_STATE_CLIMB && state.nav_state != NAV_STATE_POSHOLD;
    if (returning || !flags.in_air || !nav_can_start())
      return;
    nav_take_control();
  }
  nav_begin_return(time_millis());
}

static void nav_mission_start() {
  // A mission is not a takeoff command: start only clear of the ground. It
  // takes over position hold.
  const uint8_t index = nav_route_resume();
  const bool navigating = state.nav_state >= NAV_STATE_CLIMB && state.nav_state != NAV_STATE_POSHOLD;
  if (navigating || mission.count == 0 || !nav_airborne() || !nav_can_start() ||
      !nav_within(mission.waypoints[index].coord, MISSION_START_MAX_DISTANCE))
    return;
  nav_take_control();
  nav_mission_fly(index, time_millis());
}

// Position hold engages in flight, clear of the ground, while nothing else
// navigates. It needs no home.
static bool nav_poshold_can_start() {
  return state.nav_state == NAV_STATE_INACTIVE && !flags.failsafe && nav_airborne() && flags.gps_valid &&
         flags.altitude_valid && profile.navigation.throttle_min < profile.navigation.throttle_max;
}

static void nav_poshold_start() {
  nav_take_control();
  guide.target_altitude = state.altitude;
  nav_set_phase(NAV_STATE_POSHOLD, time_millis());
}

// Armed on the ground at the TAKEOFF waypoint and never throttled up since:
// the mission switch readies a mission starting with TAKEOFF, the throttle
// stick starts it.
static bool nav_takeoff_can_start() {
  const uint8_t index = nav_route_resume();
  return mission.count > 0 && mission.waypoints[index].type == MISSION_WAYPOINT_TAKEOFF &&
         !flags.in_air && !nav.takeoff_done && nav_can_start() && nav_within(mission.waypoints[index].coord, MISSION_TAKEOFF_MAX_DISTANCE);
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
  nav_set_phase(NAV_STATE_TAKEOFF_READY, time_millis());
  guide.takeoff.sticks_abort = true;
  state.rx_override = (vec4_t){0};
}

// Control applied stage 1 this pass. Keep its override and let the failsafe
// hold, updated next, fly the vertical axis with the mission's trim.
static void nav_hand_to_failsafe_hold() {
  nav_stop();
  flags.controls_override = 1;
  nav.hold = {.active = true, .altitude = state.altitude};
}

// The RTH switch and an airborne failsafe request a return for as long as
// they last, and RTH starts once it can. The mission switch starts a
// mission, and grants guided flight, only when turned on in flight; turning
// it off ends what it started. Receivers may report preset switch positions
// during a link loss, so nothing stops then and the mission switch starts
// nothing.
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
  const bool poshold = state.nav_state == NAV_STATE_POSHOLD;
  if (rth_switch || failsafe_rth) {
    // RTH takes over a mission, its return, a ground station target and position hold.
    if (state.nav_state < NAV_STATE_CLIMB || poshold || state.nav_target != NAV_TARGET_HOME || nav_route.returning)
      nav_rth_start();
  } else if (!link_lost && state.nav_state >= NAV_STATE_CLIMB && !poshold && state.nav_target == NAV_TARGET_HOME &&
             !nav_route.returning) {
    nav_stop();
  }

  // RTH takes priority: a mission switched on during RTH is ignored.
  // A lost link keeps the switch's previous position, so a real change acts
  // as an edge once the link is back and a preset position starts nothing.
  const bool mission_switch = rx_aux_on(AUX_MISSION);
  if (!link_lost && mission_switch != nav.mission_switch) {
    // As for a mission, a switch left on through arming and takeoff grants nothing.
    nav.guided_granted = mission_switch && nav_airborne();
    if (mission_switch && nav_takeoff_can_start())
      nav_takeoff_start();
    else if (mission_switch)
      nav_mission_start();
    else if (state.nav_target != NAV_TARGET_HOME || nav_route.returning)
      nav_stop();
    nav.mission_switch = mission_switch;
  }

  // Position hold flies while its switch is on and nothing else navigates:
  // RTH and a mission take it over, and it resumes once they end.
  const bool poshold_switch = rx_aux_on(AUX_POSHOLD);
  if (!link_lost) {
    if (!poshold_switch && state.nav_state == NAV_STATE_POSHOLD)
      nav_stop();
    else if (poshold_switch && nav_poshold_can_start())
      nav_poshold_start();
  }

  // A mission, ground station target or position hold rides out the
  // hold-last window. Unless the mission continues through the loss, it
  // never continues away from a pilot once failsafe engages: failsafe RTH
  // returned above, otherwise the failsafe hold takes over.
  if ((state.nav_target != NAV_TARGET_HOME || state.nav_state == NAV_STATE_POSHOLD) && flags.failsafe) {
    // A takeoff that has not lifted off ends; it never holds an altitude on the ground.
    if (nav_takeoff_on_ground())
      nav_abort();
    else if (flags.failsafe_hold)
      nav_hand_to_failsafe_hold();
    else
      nav_stop();
  }

  // A rescue that gave up is not retried while an RTH request lasts. The
  // mission and position hold switches do not hold the abort, which would
  // keep RTH and failsafe RTH from starting; position hold restarts only once
  // its sensors are back.
  if (state.nav_state == NAV_STATE_ABORTED && !rth_switch && !flags.failsafe)
    nav_stop();
}

// The pilot grants ground station commands with the mission switch, unless
// an RTH request or failsafe takes priority. Once guided flight owns the
// controls the grant holds below the start altitude, so the ground station
// can still stop or recall the vehicle. A navigation failure holds until it
// gives up; commands cannot restart it. A takeoff finishes its climb first.
static bool nav_guided_ready() {
  const bool failed = state.nav_state == NAV_STATE_NAV_FAILED || state.nav_state == NAV_STATE_HEADING_FAILED;
  return nav.guided_granted && !rx_aux_on(AUX_RETURN_TO_HOME) && !flags.failsafe && !failed && !nav_taking_off() &&
         (state.nav_state >= NAV_STATE_CLIMB || (nav_airborne() && nav_can_start()));
}

static void nav_guided_fly(const mission_waypoint_t &target) {
  nav_take_control();
  state.nav_target = NAV_TARGET_GUIDED;
  nav_route.guided = target;
  guide.target_altitude = target.altitude;
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
    nav_guided_fly({.coord = state.position, .altitude = (int16_t)lroundf(state.altitude)});
    // Hold the altitude it is at, not the whole meter a waypoint stores.
    guide.target_altitude = state.altitude;
    // Position hold steers in the earth frame: learn an untrusted heading
    // first, then return to the hold point, as on any leg.
    if (state.heading_confidence < NAV_MIN_HEADING_CONFIDENCE) {
      nav_begin_leg(now_ms);
      break;
    }
    // Brake and hover in place without turning toward the target first.
    nav_set_phase(NAV_STATE_BRAKE, now_ms);
    break;
  case NAV_COMMAND_SET_ALT:
    nav_route.guided.altitude = guide.target_altitude = cmd->waypoint.altitude;
    // Change to it evenly over the rest of the leg, still along its line.
    nav_route_restart_altitude();
    // A climb supervises progress toward its target altitude: measure it anew.
    if (state.nav_state == NAV_STATE_CLIMB)
      nav_set_phase(NAV_STATE_CLIMB, now_ms);
    break;
  case NAV_COMMAND_RTH:
    // Already returning or holding at home: a repeat does not climb again.
    if (state.nav_state >= NAV_STATE_CLIMB && state.nav_target == NAV_TARGET_HOME)
      break;
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
  const bool locked = nav_taking_off() || state.nav_state == NAV_STATE_LAND;
  const nav_command_result_t result = nav_route_replace(updated, state.nav_target == NAV_TARGET_MISSION, locked, &retarget);
  if (result != NAV_RESULT_OK)
    return result;
  if (!retarget) {
    // The waypoints after the target set how a PASS leg ends.
    if (state.nav_target == NAV_TARGET_MISSION && !nav_route.complete) {
      guide.leg.end_speed = 0.0f;
      guide.leg.arrive_radius = ARRIVAL_RADIUS;
      nav_latch_corner();
    }
    return NAV_RESULT_OK;
  }
  // A navigation failure keeps its latched hold until it gives up.
  if (state.nav_state == NAV_STATE_NAV_FAILED || state.nav_state == NAV_STATE_HEADING_FAILED) {
    nav_update_target();
    return NAV_RESULT_OK;
  }
  // Retarget like a guided GOTO.
  nav_mission_select(nav_route.index);
  nav_retarget(time_millis());
  return NAV_RESULT_OK;
}

static void nav_update_guidance(float dt) {
  if (!flags.arm_state) {
    nav_stop();
    nav_route = {};
    nav.guided_granted = false;
    nav.takeoff_done = false;
    // A mission needs a switch edge after arming.
    nav.mission_switch = rx_aux_on(AUX_MISSION);
  } else {
    nav_update_request();
  }
  nav_update_target();
  if (state.nav_state >= NAV_STATE_CLIMB)
    nav_update_control(dt);
  // After control, which can fail or abort guided flight.
  flags.guided_ready = nav_guided_ready();
  // A takeoff still on the ground is not flying a mission through a link loss.
  flags.mission_engaged = state.nav_state >= NAV_STATE_CLIMB && !nav_takeoff_on_ground() &&
                          (state.nav_target == NAV_TARGET_MISSION || nav_route.returning);
  // Navigation debug: 0 heading, 1 GPS course, 2 altitude (dm), 3 confidence.
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 4, (int16_t)(state.nav_state));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 5, (int16_t)(state.nav_target_distance));     // meters
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 6, (int16_t)(state.nav_target_bearing * 10)); // 0.1 deg
}

// Airborne stage 1 failsafe without RTH: level, and hold the altitude at loss.
static void nav_update_failsafe_hold(float dt) {
  if (!flags.failsafe_hold || state.nav_state >= NAV_STATE_CLIMB || !flags.arm_state) {
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

// Ground effect is expected from arming until clear of the launch ground or
// after GROUND_EFFECT_LAUNCH_TIME of raised throttle, once per arm, and on a
// landing's final descent: higher up, a barometer wandering in the craft's own
// wash while the fast descent brakes would read as touchdown. The timer only
// pauses, so the takeoff cannot start again in flight.
static void nav_update_ground_effect(float dt) {
  if (!flags.arm_state) {
    nav.clear_of_ground = 0.0f;
    nav.launch_time = 0.0f;
    state.ground_effect = GROUND_EFFECT_NONE;
    return;
  }
  if (state.altitude > GROUND_EFFECT_HEIGHT)
    nav.clear_of_ground = MIN(nav.clear_of_ground + dt, GROUND_EFFECT_CLEAR_TIME);
  if (state.throttle > THROTTLE_SAFETY)
    nav.launch_time = MIN(nav.launch_time + dt, GROUND_EFFECT_LAUNCH_TIME);
  if (nav.clear_of_ground < GROUND_EFFECT_CLEAR_TIME && nav.launch_time < GROUND_EFFECT_LAUNCH_TIME)
    state.ground_effect = GROUND_EFFECT_LAUNCH;
  else if (nav_land_final())
    state.ground_effect = GROUND_EFFECT_LANDING;
  else
    state.ground_effect = GROUND_EFFECT_NONE;
}

void nav_update_multi(float dt) {
  nav_update_trims(dt);
  if (profile.serial.gps != SERIAL_PORT_INVALID) {
    nav_update_guidance(dt);
  } else {
    flags.guided_ready = false;
    flags.mission_engaged = false;
  }
  nav_update_failsafe_hold(dt);
  nav_update_ground_effect(dt);
  // Publish applied requests in every phase, including acquisition and failure.
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 14, (int16_t)(state.rx_override.pitch * 1000));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 15, (int16_t)(state.rx_override.roll * 1000));
}

#ifdef PIO_UNIT_TESTING
void nav_test_reset() {
  nav_stop();
  nav = {};
  nav_test_reset_control();
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

void nav_test_stop() {
  nav_stop();
}

void nav_test_mission_start() {
  nav_mission_start();
}

void nav_test_update_control(float dt) {
  nav_update_target();
  nav_update_control(dt);
}

#endif
