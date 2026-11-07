#include "navigation.h"

#include <math.h>
#include <string.h>

#include "control/control.h"
#include "control/imu.h"
#include "control/mission.h"
#ifdef VEHICLE_MULTI
#include "control/multi/navigation.h"
#endif
#ifdef VEHICLE_WING
#include "control/wing/navigation.h"
#endif
#include "core/profile.h"
#include "driver/baro/baro.h"
#include "driver/time.h"
#include "io/blackbox.h"
#include "io/gps.h"
#include "io/quic_rt.h"
#include "util/util.h"
#include "util/vector.h"

// Navigation estimate: one earth-frame state predicted from the accelerometer
// and corrected by the barometer and GPS position and velocity.
// The barometer corrects height, up velocity and up bias as a third-order
// complementary filter with this time constant.
static constexpr float BARO_FUSION_TIME = 1.0f; // seconds
// GPS velocity corrects velocity and bias per axis, compared with the
// estimate GPS_LAG_MS before its epoch. The barometer leads up.
static constexpr float GPS_HORIZONTAL_FUSION_TIME = 0.35f; // seconds
static constexpr float GPS_VERTICAL_FUSION_TIME = 2.0f;    // seconds
static constexpr float GPS_BIAS_TIME = 10.0f;              // seconds, mostly tilt error horizontally
static constexpr uint32_t GPS_LAG_MS = 120U; // u-blox M8 to M10, measurement to epoch stamp, as ArduPilot
static constexpr float GPS_VELOCITY_GATE = 3.0f;      // multiples of the receiver speed accuracy
static constexpr float GPS_VELOCITY_GATE_MIN = 1.0f;  // m/s
// GPS velocity no more accurate than this corrects at the full rate; less
// accurate, by the square of the ratio. The fix limits bound the gate.
static constexpr float GPS_SPEED_ACCURACY_FULL = 0.5f; // m/s
// The horizontal accelerometer turns with the heading: the estimate predicts
// north and east only once it is well learned; otherwise they are GPS
// velocity. A rejected horizontal sample means GPS or the heading is wrong.
// Changing clearly more or less than the accelerometer's change, beyond
// GPS_SPEED_GATE, it is GPS, and the accelerometer carries north and east.
// Otherwise it may be the heading: north and east are GPS velocity for
// GPS_VELOCITY_SUSPEND_MS, and heading checks may relearn it.
static constexpr float HORIZONTAL_MIN_HEADING_CONFIDENCE = 0.5f;
static constexpr uint32_t GPS_VELOCITY_SUSPEND_MS = 5000U;
// Following GPS, north and east are its velocity carried from the
// measurement to the present at the rate it changes (state.gps_accel), so
// they describe the same moment as the position. The rate is low-passed over
// this time against receiver noise, and carried at most
// GPS_EXTRAPOLATE_MAX_MS.
static constexpr float GPS_ACCEL_FILTER_TIME = 0.2f; // seconds
static constexpr uint32_t GPS_EXTRAPOLATE_MAX_MS = 500U;
// GPS position corrects north and east position, compared with
// the estimate GPS_LAG_MS before its epoch: at the full rate up to
// GPS_HORIZONTAL_ACCURACY_FULL of reported accuracy, less accurate by the
// square of the ratio. Velocity carries the position between fixes.
static constexpr float GPS_POSITION_FUSION_TIME = 0.5f;     // seconds
static constexpr float GPS_HORIZONTAL_ACCURACY_FULL = 2.0f; // meters
// GPS consistency, from two tests in multiples of what the
// receiver reports as its accuracy, which receivers state optimistically, so
// no less than the minimums:
// - Position innovations test GPS position against the velocity carrying the
//   estimate: GPS velocity, or the accelerometer where it predicts velocity.
// - How much GPS velocity changed over GPS_SPEED_CHECK_TIME tests against the
//   accelerometer's change. A wrong heading turns that change, but keeps its
//   size; a receiver that stalls or lags shrinks it.
// The mean square of the larger over GPS_CONSISTENCY_TIME grades the trust in
// GPS: full up to GPS_CONSISTENT, none from GPS_INCONSISTENT. Trust scales
// every GPS correction, no lower than GPS_TRUST_MIN so a consistent receiver
// can earn it back. Below GPS_TRUST_VALID, held to GPS_TRUST_HOLD once valid,
// navigation stops using GPS.
static constexpr float GPS_POSITION_ACCURACY_MIN = 1.0f; // meters
static constexpr float GPS_SPEED_CHECK_TIME = 2.0f;      // seconds
static constexpr float GPS_SPEED_CHECK_MIN = 1.0f;       // m/s
static constexpr float GPS_SPEED_GATE = 3.0f;
static constexpr float GPS_CONSISTENCY_TIME = 2.0f;      // seconds
static constexpr float GPS_CONSISTENT = 1.0f;
static constexpr float GPS_INCONSISTENT = 4.0f;
static constexpr float GPS_INNOVATION_MAX = 4.0f * GPS_INCONSISTENT; // counted at most, in the mean square
static constexpr float GPS_TRUST_MIN = 0.1f;
static constexpr float GPS_TRUST_VALID = 0.5f;
static constexpr float GPS_TRUST_HOLD = 0.2f;
// A position innovation beyond GPS_POSITION_GATE is a glitch: not fused and
// not graded. One lasting GPS_POSITION_RESET_MS while GPS velocity is
// consistent is where GPS now is, as ArduPilot's GPS glitch reset: the
// position restarts from it.
static constexpr float GPS_POSITION_GATE = 3.0f;
static constexpr uint32_t GPS_POSITION_RESET_MS = 5000U;
// Armed, GPS altitude above home slowly corrects barometer drift from
// temperature and weather, as PX4's barometer bias estimate: at the full rate
// up to GPS_VERTICAL_ACCURACY_FULL of reported accuracy, less accurate by the
// square of the ratio. Not near the ground, where rotor wash moves the barometer.
static constexpr float GPS_ALTITUDE_FUSION_TIME = 60.0f;  // seconds
static constexpr float GPS_VERTICAL_ACCURACY_FULL = 3.0f; // meters
static constexpr float BARO_DRIFT_LIMIT = 10.0f;          // meters
// Disarmed, GPS altitude of the height reference averages over this time, so
// home altitude, and the drift correction measured from it, do not start
// from the error of a single GPS sample.
static constexpr float GPS_LAUNCH_ALTITUDE_TIME = 20.0f; // seconds
static constexpr float ESTIMATE_BIAS_LIMIT = 1.0f; // m/s^2 per axis
// Without a barometer, up velocity is an accelerometer integral; leak it
// toward zero so bias cannot build into a sustained climb or descent demand.
static constexpr float NO_BARO_DECAY_TIME = 5.0f;     // seconds
static constexpr uint32_t ESTIMATE_HISTORY_SIZE = 48U; // navigation steps, beyond GPS_LAG_MS and a 4 Hz GPS interval
// Heading from accelerations: GPS velocity changes are the
// accelerometer's turned by the heading error, plus a steady offset from tilt
// error. Removing their means over HEADING_OBSERVE_TIME removes the offset;
// changing accelerations that still correlate then measure the rotation.
static constexpr float HEADING_OBSERVE_TIME = 2.0f;      // seconds
static constexpr float HEADING_OBSERVE_MIN = 1.5f;       // (m/s^2)^2, mean product of varying accelerations on both sides
static constexpr float HEADING_OBSERVE_COHERENCE = 0.7f; // normalized correlation
static constexpr float NAV_MAX_DT = 0.1f; // seconds
// Touchdown in rotor wash: below the estimate by GROUND_EFFECT_SUSPECT, the
// barometer may read the wash; the accelerometer alone then shows whether a
// descent stopped, while the barometer falls GROUND_EFFECT_DEVIATION below.
static constexpr float GROUND_EFFECT_SUSPECT = 0.25f;    // meters
static constexpr float GROUND_EFFECT_DEVIATION = 0.5f;   // meters
static constexpr float GROUND_EFFECT_STOPPED = 0.2f;     // m/s
static constexpr float GROUND_EFFECT_LOW_WEIGHT = 0.25f; // of a reading between the held ground and the estimate
// Below a held ground the barometer may read the wash this far low, just
// under the 5.5 m logged on Orange_3in lift-offs; lower readings count by the
// excess, as ArduPilot's EK3_GND_EFF_DZ.
static constexpr float GROUND_EFFECT_DEAD_ZONE = 5.0f; // meters

// A fix navigation accepts, as ArduPilot's GPS pre-arm limits. An accepted
// fix keeps until it fails looser limits, so a marginal one does not flap.
static constexpr uint8_t NAV_GPS_MIN_SATS = 6;
static constexpr float NAV_GPS_MAX_HORIZONTAL_ACCURACY = 5.0f; // meters
static constexpr float NAV_GPS_MAX_SPEED_ACCURACY = 1.0f;      // m/s
static constexpr uint8_t NAV_GPS_HOLD_MIN_SATS = 5;
static constexpr float NAV_GPS_HOLD_ACCURACY_SCALE = 1.5f;
static constexpr float NAV_GPS_MAX_POSITION_JUMP = 50.0f;       // meters per fix
// A jump is where GPS now is once the fixes after it agree for this long.
static constexpr uint32_t NAV_GPS_JUMP_SETTLE_MS = 1000U;

static constexpr int64_t GPS_LON_TURN = 3600000000LL; // degrees * 1e7 around the earth

static struct {
  uint32_t last_update_us;
  bool armed; // Arming edge for the launch altitude and home.
} nav;

static struct {
  float launch;            // absolute altitude of the height reference, meters
  bool referenced;         // a finite sample set launch; until then no altitude is published
  uint32_t last_update_ms; // time of the latest finite sample
} baro;

// Latest fix, kept across arm cycles to detect jumps.
static struct {
  gps_coord_t position;
  bool position_valid;
  uint32_t checked_ms;
  bool jumped;      // a fix jumped beyond NAV_GPS_MAX_POSITION_JUMP from the previous one
  uint32_t jump_ms; // epoch of the latest such fix
} gps;

// The navigation estimate, earth frame north, east, up. Heights are relative
// to baro.launch, positions to origin.
static struct {
  float height;
  float position[2]; // meters north and east of origin
  vec3_t velocity;
  vec3_t bias;           // m/s^2
  bool horizontal;       // north and east follow the accelerometer; otherwise GPS velocity
  uint32_t suspended_ms; // a GPS velocity last contradicted north and east
  uint32_t gps_ms;       // epoch of the latest GPS sample taken
  float heading_correction_seen; // state.heading_correction_total at that sample
  gps_coord_t origin;    // GPS position the latest position restart began at
  float lon_scale;       // meters per longitude unit at origin
  bool positioned;       // position follows GPS since the latest restart; restarts at the next fix otherwise
  float inconsistency;   // mean square of the larger normalized GPS innovation per sample
  uint32_t glitch_ms;    // epoch of the first innovation of flags.gps_glitch
  float baro_drift;      // meters the barometer reads above GPS altitude since arming
  float gps_launch;      // GPS altitude of the height reference, averaged while disarmed
  bool gps_launch_valid;
  struct {
    uint32_t ms;
    vec3_t velocity;
    float position[2];
    float inertial[2];
  } history[ESTIMATE_HISTORY_SIZE];
  uint32_t history_index;
  // Accelerometer-only north and east velocity change, turned by the heading
  // estimate; at the latest GPS measurement with GPS velocity.
  float inertial[2];
  struct {
    float inertial[2];
    float gps[2];
  } measured;
  // Decaying sums of the accelerometer-only and GPS velocity changes over
  // GPS_SPEED_CHECK_TIME.
  struct {
    float inertial[2];
    float gps[2];
  } speed_check;
  // Decaying sums of accelerometer (a) and GPS (g) mean accelerations over
  // each GPS interval, their squares and their cross and dot products, for
  // the heading observation.
  struct {
    float weight;
    float a[2];
    float g[2];
    float aa;
    float gg;
    float cross;
    float dot;
  } heading;
  float baro_height; // latest sample
  float ground;      // launch ground, or where touchdown was detected; held through the ground effect
  bool ground_held;
  // Up motion from the accelerometer alone since the barometer began reading
  // below the estimate while landing, and the velocity just before.
  struct {
    float height;
    float velocity;
    float descent;
  } contact;
} estimate;

// Specific force summed over the Flight loops of a navigation step, in g in
// the accel_raw frame.
static struct {
  vec3_t sum;
  uint32_t count;
} accel;

nav_route_t nav_route;

// Move the height reference up by shift meters, keeping the estimate continuous.
static void nav_shift_reference(float shift) {
  baro.launch += shift;
  estimate.height -= shift;
  estimate.baro_height -= shift;
  estimate.ground -= shift;
  estimate.contact.height -= shift;
  estimate.gps_launch += shift;
  baro.referenced = true;
}

// A non-finite sample is dropped like a missing one; altitude lapses after BARO_STALE_MS.
static void nav_take_altitude(float altitude, uint32_t timestamp_ms) {
  if (!float_is_finite(altitude))
    return;
  baro.last_update_ms = timestamp_ms;
  estimate.baro_height = altitude - baro.launch - estimate.baro_drift;
  // Disarmed altitude is relative to the first sample (or the last launch),
  // so the bench shows baro movement.
  if (!baro.referenced)
    nav_shift_reference(estimate.baro_height);
}

static void nav_update_altitude(bool arming) {
  baro_sample_t sample;
  if (baro_take_sample(sample))
    nav_take_altitude(sample.altitude, sample.timestamp_ms);
  if (arming) {
    // GPS altitude corrects the barometer from the arming reference on: the
    // raw barometer, which the estimate and its GPS altitude move onto too.
    estimate.baro_height += estimate.baro_drift;
    estimate.height += estimate.baro_drift;
    estimate.gps_launch -= estimate.baro_drift;
    estimate.baro_drift = 0.0f;
    if (baro.referenced)
      nav_shift_reference(estimate.height);
  }
}

static void nav_accumulate_accel() {
  if (!float_is_finite(vec3_dot(state.accel_raw, state.accel_raw)))
    return;
  accel.sum += state.accel_raw;
  accel.count++;
}

// The mean specific force since the last navigation step; at rest without samples.
static vec3_t nav_take_accel() {
  const vec3_t force = accel.count ? accel.sum * (1.0f / accel.count) : state.GEstG;
  accel = {};
  return force;
}

// On touchdown the accelerometer shows a descent stop while rotor wash makes
// the barometer fall below the estimate. Then hold the ground there, and undo
// what the wash pulled from the estimate meanwhile.
static void nav_detect_touchdown(float error, float accel_up, float dt) {
  if (error > -GROUND_EFFECT_SUSPECT) {
    estimate.contact = {estimate.height, estimate.velocity.up, estimate.velocity.up};
    return;
  }
  estimate.contact.height += estimate.contact.velocity * dt;
  estimate.contact.velocity += accel_up * dt;
  if (estimate.contact.descent < -GROUND_EFFECT_STOPPED && estimate.contact.velocity > -GROUND_EFFECT_STOPPED &&
      error < -GROUND_EFFECT_DEVIATION) {
    estimate.ground = estimate.height = estimate.contact.height;
    estimate.velocity.up = estimate.contact.velocity;
    estimate.ground_held = true;
  }
}

// Correct height, up velocity and up bias with the barometer. Without a fresh
// sample up velocity leaks toward zero.
static void nav_correct_baro(float accel_up, float dt) {
  const bool baro_fresh = baro.referenced && time_millis() - baro.last_update_ms <= BARO_STALE_MS;
  if (baro_fresh && !flags.altitude_valid) {
    // (Re)acquisition: start from the measurement instead of a drifted integral.
    estimate.height = estimate.baro_height;
  }
  if (state.ground_effect == GROUND_EFFECT_NONE) {
    estimate.ground_held = false;
  } else if (state.ground_effect == GROUND_EFFECT_LAUNCH) {
    estimate.ground = 0.0f;
    estimate.ground_held = true;
  }
  flags.altitude_valid = baro_fresh;
  float &velocity = estimate.velocity.up;
  if (!baro_fresh) {
    velocity -= dt / (NO_BARO_DECAY_TIME + dt) * velocity;
    return;
  }
  if (state.ground_effect == GROUND_EFFECT_LANDING && !estimate.ground_held)
    nav_detect_touchdown(estimate.baro_height - estimate.height, accel_up, dt);
  const float k1 = 3.0f / BARO_FUSION_TIME;
  const float k2 = 3.0f / (BARO_FUSION_TIME * BARO_FUSION_TIME);
  const float k3 = 1.0f / (BARO_FUSION_TIME * BARO_FUSION_TIME * BARO_FUSION_TIME);
  float error = estimate.baro_height - estimate.height;
  if (estimate.ground_held) {
    // The wash reads low: within the dead zone the reading counts no lower
    // than the ground, beyond it by the excess. Any lower than the estimate
    // counts only a little, and nothing teaches the bias.
    const float reading = estimate.ground - estimate.baro_height > GROUND_EFFECT_DEAD_ZONE
                              ? estimate.baro_height + GROUND_EFFECT_DEAD_ZONE
                              : MAX(estimate.baro_height, estimate.ground);
    error = reading - estimate.height;
    if (error < 0.0f)
      error *= GROUND_EFFECT_LOW_WEIGHT;
  } else {
    estimate.bias.axis[2] = constrain(estimate.bias.axis[2] - k3 * error * dt, -ESTIMATE_BIAS_LIMIT, ESTIMATE_BIAS_LIMIT);
  }
  estimate.height += k1 * error * dt;
  velocity += k2 * error * dt;
}

// Longitude units east from start to end, the short way across the antimeridian.
static int32_t nav_lon_delta(int32_t start, int32_t end) {
  int64_t delta = (int64_t)end - start;
  if (delta > GPS_LON_TURN / 2)
    delta -= GPS_LON_TURN;
  else if (delta < -GPS_LON_TURN / 2)
    delta += GPS_LON_TURN;
  return (int32_t)delta;
}

// Equirectangular at the mean latitude: within about 0.1% of the great-circle
// distance below 100 km at mid latitudes, beyond any range flown on a battery.
vec2_t nav_position_delta(gps_coord_t start, gps_coord_t end) {
  const float scale = 1.0f / 10000000.0f;
  const float lat_diff = (end.lat - start.lat) * scale;
  const float lon_diff = nav_lon_delta(start.lon, end.lon) * scale;
  const float lat_rad = 0.5f * ((float)start.lat + (float)end.lat) * scale * DEGTORAD;
  return {{lat_diff * METERS_PER_DEGREE_LAT, lon_diff * (METERS_PER_DEGREE_LAT * fastcos(lat_rad))}};
}

static float nav_distance_between(gps_coord_t start, gps_coord_t end) {
  return vec2_magnitude(nav_position_delta(start, end));
}

bool nav_within(gps_coord_t target, float radius) {
  return nav_distance_between(state.position, target) <= radius;
}

// Distance in meters and bearing in degrees true from start to end.
static void nav_vector(gps_coord_t start, gps_coord_t end, float *distance, float *bearing) {
  const vec2_t delta = nav_position_delta(start, end);
  *distance = vec2_magnitude(delta);
  *bearing = normalize_rad(atan2approx_rad(delta.east, delta.north)) * RADTODEG;
}

float nav_leg_altitude(gps_coord_t origin, float origin_altitude, gps_coord_t target, float target_altitude, float *rate) {
  *rate = 0.0f;
  const vec2_t leg = nav_position_delta(origin, target);
  const float length_squared = vec2_dot(leg, leg);
  if (length_squared < 1.0f)
    return target_altitude;
  // Along-track progress: wandering across the leg does not change the target.
  const float progress = vec2_dot(nav_position_delta(origin, state.position), leg) / length_squared;
  const float change = target_altitude - origin_altitude;
  if (progress > 0.0f && progress < 1.0f)
    *rate = change * vec2_dot(vec3_horizontal(state.velocity), leg) / length_squared;
  return origin_altitude + constrain(progress, 0.0f, 1.0f) * change;
}

uint8_t nav_route_resume() {
  return nav_route.complete ? 0 : nav_route.index;
}

void nav_route_start_leg() {
  nav_route.leg_origin = state.position;
  nav_route_restart_altitude();
}

void nav_route_restart_altitude() {
  nav_route.altitude_origin = state.position;
  nav_route.leg_altitude = state.altitude;
}

void nav_route_next() {
  const mission_waypoint_t &reached = mission.waypoints[nav_route.index];
  nav_route.leg_origin = nav_route.altitude_origin = reached.coord;
  nav_route.leg_altitude = reached.altitude;
  nav_route.index++;
}

gps_coord_t nav_target_coord() {
  if (state.nav_target == NAV_TARGET_GUIDED)
    return nav_route.guided.coord;
  if (state.nav_target == NAV_TARGET_MISSION)
    return mission.waypoints[nav_route.index].coord;
  return state.home;
}

void nav_update_target() {
  // nav_route.index survives interruptions, but is published only while flown.
  state.mission_index = state.nav_target == NAV_TARGET_MISSION ? nav_route.index : 0;
  state.nav_target_position = nav_target_coord();
  if (state.nav_target == NAV_TARGET_HOME) {
    state.nav_target_distance = state.home_distance;
    state.nav_target_bearing = state.home_bearing;
    return;
  }
  nav_vector(state.position, state.nav_target_position, &state.nav_target_distance, &state.nav_target_bearing);
}

// The receiver reports a usable fix.
static bool nav_gps_quality_ok(void) {
  const bool held = gps.position_valid;
  const float scale = held ? NAV_GPS_HOLD_ACCURACY_SCALE : 1.0f;
  return state.gps_health.lock &&
         state.gps_sats >= (held ? NAV_GPS_HOLD_MIN_SATS : NAV_GPS_MIN_SATS) &&
         state.gps_horizontal_accuracy <= NAV_GPS_MAX_HORIZONTAL_ACCURACY * scale &&
         state.gps_speed_accuracy <= NAV_GPS_MAX_SPEED_ACCURACY * scale &&
         (time_millis() - state.gps_last_update_ms) <= NAV_GPS_STALE_MS;
}

static void nav_update_gps_sanity(void) {
  if (!nav_gps_quality_ok()) {
    flags.gps_valid = false;
    gps.position_valid = false;
    return;
  }

  if (state.gps_last_update_ms == gps.checked_ms) {
    return;
  }
  gps.checked_ms = state.gps_last_update_ms;

  // A jump is a glitch until the fixes after it agree, then where GPS is.
  if (gps.position_valid && nav_distance_between(gps.position, state.gps_coord) > NAV_GPS_MAX_POSITION_JUMP) {
    gps.jumped = true;
    gps.jump_ms = state.gps_last_update_ms;
  }
  gps.position = state.gps_coord;
  gps.position_valid = true;
  if (gps.jumped) {
    if (state.gps_last_update_ms - gps.jump_ms < NAV_GPS_JUMP_SETTLE_MS) {
      flags.gps_valid = false;
      return;
    }
    gps.jumped = false;
  }
  // The estimate grades how GPS agrees with itself and the accelerometer.
  flags.gps_valid = state.gps_trust >= (flags.gps_valid ? GPS_TRUST_HOLD : GPS_TRUST_VALID);
}

// Home latches on arming, or at the first valid fix after it while still on
// the launch ground, so arming before GPS settles still leaves a home.
static void nav_update_home(bool arming) {
  if (!flags.arm_state) {
    state.home = state.position;
    flags.home_valid = false;
    return;
  }
  if (!flags.home_valid && flags.gps_valid && (arming || !flags.in_air)) {
    flags.home_valid = true;
    state.home = state.position;
    state.home_altitude = estimate.gps_launch_valid ? estimate.gps_launch : state.gps_altitude - estimate.height;
  }
  if (flags.gps_valid && flags.home_valid)
    nav_vector(state.position, state.home, &state.home_distance, &state.home_bearing);
}

// Earth-frame acceleration in m/s^2, north, east, up: the specific force
// along the level forward, left and up axes, the first two turned by heading.
static vec3_t nav_earth_accel(vec3_t force) {
  // Legacy body axes are {left, back, up}. Project the nose onto the plane
  // normal to the gravity estimate, as the IMU gravity reference does.
  const vec3_t up = state.GEstG * (1.0f / ACC_1G);
  vec3_t accel = {{0.0f, 0.0f, (vec3_dot(force, up) / ACC_1G - 1.0f) * GRAVITY}};
  const vec3_t nose = {{0.0f, -1.0f, 0.0f}};
  const vec3_t level_forward = nose - up * vec3_dot(nose, up);
  const float level_forward_length = vec3_magnitude(level_forward);
  if (level_forward_length < 0.1f)
    return accel;
  const vec3_t forward_axis = level_forward * (1.0f / level_forward_length);
  const vec3_t left_axis = vec3_cross(up, forward_axis);
  const float forward = vec3_dot(force, forward_axis) * (GRAVITY / ACC_1G);
  const float left = vec3_dot(force, left_axis) * (GRAVITY / ACC_1G);
  const float heading = state.heading * DEGTORAD;
  const float cos_heading = fastcos(heading);
  const float sin_heading = fastsin(heading);
  accel.north = forward * cos_heading + left * sin_heading;
  accel.east = forward * sin_heading - left * cos_heading;
  return accel;
}

// Meters north and east of the estimate's origin.
static void nav_local_position(gps_coord_t coord, float *north, float *east) {
  *north = (coord.lat - estimate.origin.lat) * (METERS_PER_DEGREE_LAT * 1e-7f);
  *east = nav_lon_delta(estimate.origin.lon, coord.lon) * estimate.lon_scale;
}

// The estimated position as GPS coordinates.
static gps_coord_t nav_estimated_coord() {
  // Wrapped back into range past the antimeridian.
  const int32_t lon = estimate.origin.lon + (int32_t)lroundf(estimate.position[1] / estimate.lon_scale);
  return {
      .lon = nav_lon_delta(0, lon),
      .lat = estimate.origin.lat + (int32_t)lroundf(estimate.position[0] / (METERS_PER_DEGREE_LAT * 1e-7f)),
  };
}

// Restart the position at the latest GPS position, here and in the stored
// estimates. Trust keeps its grade, so a receiver that jumped earns it back.
static void nav_restart_position() {
  estimate.origin = state.gps_coord;
  estimate.lon_scale = METERS_PER_DEGREE_LAT * 1e-7f * cosf(state.gps_coord.lat * 1e-7f * DEGTORAD);
  estimate.position[0] = estimate.position[1] = 0.0f;
  for (auto &entry : estimate.history)
    entry.position[0] = entry.position[1] = 0.0f;
  estimate.positioned = true;
  flags.gps_glitch = false;
}

// North and east follow GPS velocity, carried from the measurement to the present.
static void nav_follow_gps_horizontally(uint32_t now_ms) {
  const uint32_t ahead_ms = MIN(now_ms - state.gps_last_update_ms + GPS_LAG_MS, GPS_EXTRAPOLATE_MAX_MS);
  const vec2_t present = vec3_horizontal(state.gps_velocity) + state.gps_accel * (ahead_ms * 0.001f);
  estimate.velocity.north = present.north;
  estimate.velocity.east = present.east;
}

// The accelerometer takes north and east over from GPS velocity. Only the
// horizontal innovation reads the stored north and east, and only from here
// on: they take the measurement.
static void nav_start_horizontal(uint32_t now_ms) {
  nav_follow_gps_horizontally(now_ms);
  for (auto &entry : estimate.history) {
    entry.velocity.north = state.gps_velocity.north;
    entry.velocity.east = state.gps_velocity.east;
  }
}

// The weight of a GPS measurement: one up to full accuracy, then falling
// with the square of the reported accuracy, and with the trust in GPS.
static float nav_gps_weight(float accuracy, float full) {
  const float ratio = full / MAX(accuracy, full);
  return ratio * ratio * MAX(state.gps_trust, GPS_TRUST_MIN);
}

// Corrections a GPS sample made to the present estimate, applied to the
// stored estimates too in one pass, so the next comparison does not count
// them again.
typedef struct {
  vec3_t velocity;
  float position[2];
} nav_history_shift_t;

// Correct one axis by a share of the GPS velocity innovation, weighted by
// the receiver's speed accuracy.
static void nav_correct_velocity(uint32_t axis, float innovation, float sample_dt, float fusion_time, nav_history_shift_t *shift) {
  const float weight = nav_gps_weight(state.gps_speed_accuracy, GPS_SPEED_ACCURACY_FULL);
  const float correction = MIN(sample_dt / fusion_time, 1.0f) * weight * innovation;
  estimate.velocity.axis[axis] += correction;
  shift->velocity.axis[axis] += correction;
  const float bias = estimate.bias.axis[axis] - weight * innovation * sample_dt / (fusion_time * GPS_BIAS_TIME);
  estimate.bias.axis[axis] = constrain(bias, -ESTIMATE_BIAS_LIMIT, ESTIMATE_BIAS_LIMIT);
}

// Observe the heading error from the velocity changes since the previous GPS
// measurement, turned between the accelerometer and GPS. The heading turned
// by corrected since then.
static void nav_observe_heading(const float inertial[2], const float gps[2], float corrected, float sample_dt, bool consecutive) {
  // As mean accelerations over the interval, so the sums mean the same at any GPS rate.
  const float rate = 1.0f / sample_dt;
  const float a[2] = {inertial[0] * rate, inertial[1] * rate};
  const float g[2] = {gps[0] * rate, gps[1] * rate};
  auto &heading = estimate.heading;
  // The accelerometer side turns with every correction of the heading, from
  // any source: so do its sums.
  const float c = fastcos(corrected), s = fastsin(corrected);
  const float a_north = heading.a[0] * c - heading.a[1] * s;
  heading.a[1] = heading.a[0] * s + heading.a[1] * c;
  heading.a[0] = a_north;
  const float dot = heading.dot * c + heading.cross * s;
  heading.cross = heading.cross * c - heading.dot * s;
  heading.dot = dot;
  if (!consecutive)
    return;

  const float decay = expf(-sample_dt / HEADING_OBSERVE_TIME);
  heading.weight = heading.weight * decay + 1.0f;
  for (uint32_t axis = 0; axis < 2; axis++) {
    heading.a[axis] = heading.a[axis] * decay + a[axis];
    heading.g[axis] = heading.g[axis] * decay + g[axis];
  }
  heading.aa = heading.aa * decay + a[0] * a[0] + a[1] * a[1];
  heading.gg = heading.gg * decay + g[0] * g[0] + g[1] * g[1];
  heading.cross = heading.cross * decay + a[0] * g[1] - a[1] * g[0];
  heading.dot = heading.dot * decay + a[0] * g[0] + a[1] * g[1];

  // About the means, which carry the tilt offset.
  const float cross = heading.cross - (heading.a[0] * heading.g[1] - heading.a[1] * heading.g[0]) / heading.weight;
  const float product = heading.dot - (heading.a[0] * heading.g[0] + heading.a[1] * heading.g[1]) / heading.weight;
  const float variance_a = heading.aa - (heading.a[0] * heading.a[0] + heading.a[1] * heading.a[1]) / heading.weight;
  const float variance_g = heading.gg - (heading.g[0] * heading.g[0] + heading.g[1] * heading.g[1]) / heading.weight;
  const float spread = sqrtf(MAX(variance_a * variance_g, 0.0f));
  if (spread < HEADING_OBSERVE_MIN * heading.weight || sqrtf(cross * cross + product * product) < HEADING_OBSERVE_COHERENCE * spread)
    return;
  imu_observe_heading(atan2approx_rad(cross, product), sample_dt);
}

// How far the size of the GPS velocity change over GPS_SPEED_CHECK_TIME
// differs from the accelerometer's, in multiples of the GPS speed accuracy.
// The heading turned by corrected since the previous sample.
static float nav_check_speed(const float inertial[2], const float gps[2], float corrected, float sample_dt, bool checked) {
  auto &check = estimate.speed_check;
  if (!checked) {
    check = {};
    return 0.0f;
  }
  const float c = fastcos(corrected), s = fastsin(corrected);
  const float decay = expf(-sample_dt / GPS_SPEED_CHECK_TIME);
  const float north = check.inertial[0] * c - check.inertial[1] * s;
  check.inertial[1] = (check.inertial[0] * s + check.inertial[1] * c) * decay + inertial[1];
  check.inertial[0] = north * decay + inertial[0];
  for (uint32_t axis = 0; axis < 2; axis++)
    check.gps[axis] = check.gps[axis] * decay + gps[axis];
  const float inertial_change = sqrtf(check.inertial[0] * check.inertial[0] + check.inertial[1] * check.inertial[1]);
  const float gps_change = sqrtf(check.gps[0] * check.gps[0] + check.gps[1] * check.gps[1]);
  const float mismatch = fabsf(inertial_change - gps_change);
  return mismatch / MAX(2.0f * state.gps_speed_accuracy, GPS_SPEED_CHECK_MIN);
}

// Grade GPS by the larger of the normalized innovations of a sample.
static void nav_grade_gps(float ratio, float sample_dt) {
  const float square = MIN(ratio * ratio, GPS_INNOVATION_MAX);
  estimate.inconsistency += MIN(sample_dt / GPS_CONSISTENCY_TIME, 1.0f) * (square - estimate.inconsistency);
  state.gps_trust = constrain((GPS_INCONSISTENT - estimate.inconsistency) / (GPS_INCONSISTENT - GPS_CONSISTENT), 0.0f, 1.0f);
}

// Correct position by a share of the GPS position innovation, weighted by
// the receiver's accuracy.
static void nav_correct_position(const float innovation[2], float sample_dt, nav_history_shift_t *shift) {
  const float weight = nav_gps_weight(state.gps_horizontal_accuracy, GPS_HORIZONTAL_ACCURACY_FULL);
  const float gain = MIN(sample_dt / GPS_POSITION_FUSION_TIME, 1.0f) * weight;
  for (uint32_t axis = 0; axis < 2; axis++) {
    const float correction = gain * innovation[axis];
    estimate.position[axis] += correction;
    shift->position[axis] += correction;
  }
}

// Correct position with a GPS sample, compared with the estimate at its
// measurement, and grade GPS by how it agrees, and by speed_ratio from
// nav_check_speed(). The first fix, or the first after GPS lapsed, restarts
// the position.
static void nav_correct_gps_position(const float past[2], float sample_dt, uint32_t sample_ms, float speed_ratio, nav_history_shift_t *shift) {
  if (!estimate.positioned) {
    nav_restart_position();
    return;
  }
  float north, east;
  nav_local_position(state.gps_coord, &north, &east);
  const float innovation[2] = {north - past[0], east - past[1]};
  const float ratio = sqrtf(innovation[0] * innovation[0] + innovation[1] * innovation[1]) / MAX(state.gps_horizontal_accuracy, GPS_POSITION_ACCURACY_MIN);
  // A glitch is not fused and does not grade GPS: velocity carries the
  // position meanwhile.
  const bool fused = ratio <= GPS_POSITION_GATE;
  nav_grade_gps(MAX(fused ? ratio : 0.0f, speed_ratio), sample_dt);
  if (fused) {
    flags.gps_glitch = false;
    nav_correct_position(innovation, sample_dt, shift);
  } else if (!flags.gps_glitch) {
    flags.gps_glitch = true;
    estimate.glitch_ms = sample_ms;
  } else if (speed_ratio <= GPS_CONSISTENT && sample_ms - estimate.glitch_ms >= GPS_POSITION_RESET_MS) {
    nav_restart_position();
  }
}

// Disarmed, average the GPS altitude of the height reference. Armed and clear
// of the ground, slowly correct barometer drift toward GPS altitude above
// home. Its lag is negligible at these rates.
static void nav_correct_baro_drift(float sample_dt) {
  if (!flags.altitude_valid || !float_is_finite(state.gps_altitude) || !float_is_finite(state.gps_vertical_accuracy))
    return;
  const float weight = nav_gps_weight(state.gps_vertical_accuracy, GPS_VERTICAL_ACCURACY_FULL);
  if (!flags.arm_state) {
    const float launch = state.gps_altitude - estimate.height;
    if (!estimate.gps_launch_valid)
      estimate.gps_launch = launch;
    else
      estimate.gps_launch += MIN(sample_dt / GPS_LAUNCH_ALTITUDE_TIME, 1.0f) * weight * (launch - estimate.gps_launch);
    estimate.gps_launch_valid = true;
    return;
  }
  if (!flags.home_valid || state.ground_effect != GROUND_EFFECT_NONE)
    return;
  const float innovation = state.gps_altitude - state.home_altitude - estimate.height;
  const float drift = estimate.baro_drift - weight * innovation * sample_dt / GPS_ALTITUDE_FUSION_TIME;
  estimate.baro_drift = constrain(drift, -BARO_DRIFT_LIMIT, BARO_DRIFT_LIMIT);
}

// Correct position and velocity with a new GPS sample, compared with the
// estimate GPS_LAG_MS before its epoch, and grade GPS by how they agree.
static void nav_correct_gps(uint32_t now_ms) {
  if (!gps.position_valid) {
    estimate.positioned = false;
    flags.gps_glitch = false;
    state.gps_accel = {};
    return;
  }
  const uint32_t sample_ms = state.gps_last_update_ms;
  if (sample_ms == estimate.gps_ms)
    return;
  const uint32_t interval_ms = sample_ms - estimate.gps_ms;
  const float sample_dt = constrain(interval_ms * 0.001f, 0.0f, 0.5f);
  estimate.gps_ms = sample_ms;
  const uint32_t measured_ms = sample_ms - GPS_LAG_MS;
  uint32_t index = estimate.history_index;
  for (uint32_t i = 0; i < ESTIMATE_HISTORY_SIZE - 1; i++) {
    if (int32_t(estimate.history[index].ms - measured_ms) <= 0)
      break;
    index = (index + ESTIMATE_HISTORY_SIZE - 1) % ESTIMATE_HISTORY_SIZE;
  }
  const auto &past = estimate.history[index];

  // Velocity changes since the previous sample, from the accelerometer alone
  // and from GPS.
  const bool consecutive = interval_ms <= 500U;
  // Radians the heading turned since the previous sample, from any source.
  float corrected = state.heading_correction_total - estimate.heading_correction_seen;
  if (corrected > M_PI_F)
    corrected -= 2.0f * M_PI_F;
  else if (corrected < -M_PI_F)
    corrected += 2.0f * M_PI_F;
  estimate.heading_correction_seen = state.heading_correction_total;
  const float change_inertial[2] = {past.inertial[0] - estimate.measured.inertial[0], past.inertial[1] - estimate.measured.inertial[1]};
  const vec3_t gps_vel = state.gps_velocity;
  const float change_gps[2] = {gps_vel.north - estimate.measured.gps[0], gps_vel.east - estimate.measured.gps[1]};
  estimate.measured = {{past.inertial[0], past.inertial[1]}, {gps_vel.north, gps_vel.east}};
  if (consecutive && sample_dt > 0.0f) {
    const vec2_t rate = vec2_t{{change_gps[0], change_gps[1]}} * (1.0f / sample_dt);
    state.gps_accel += (rate - state.gps_accel) * (sample_dt / (GPS_ACCEL_FILTER_TIME + sample_dt));
  } else {
    state.gps_accel = {};
  }
  nav_observe_heading(change_inertial, change_gps, corrected, sample_dt, consecutive);
  // Only once GPS velocity has taught the accelerometer's tilt bias, and
  // clear of the ground, which can hold the craft tilted against the
  // attitude estimate.
  const bool speed_checked = consecutive && state.heading_confidence >= HORIZONTAL_MIN_HEADING_CONFIDENCE &&
                             state.ground_effect == GROUND_EFFECT_NONE;
  const float speed_ratio = nav_check_speed(change_inertial, change_gps, corrected, sample_dt, speed_checked);
  nav_history_shift_t shift = {};
  nav_correct_gps_position(past.position, sample_dt, sample_ms, speed_ratio, &shift);
  nav_correct_baro_drift(sample_dt);

  const vec3_t innovation = gps_vel - past.velocity;
  const float gate = MAX(GPS_VELOCITY_GATE * state.gps_speed_accuracy, GPS_VELOCITY_GATE_MIN);
  // GPS velocity changing clearly less or more than the accelerometer's says
  // is GPS at fault: the accelerometer carries north and east meanwhile.
  // Otherwise a rejected velocity may be a wrong heading.
  if (estimate.horizontal && speed_ratio <= GPS_SPEED_GATE) {
    if (vec2_magnitude(vec3_horizontal(innovation)) > gate) {
      estimate.horizontal = false;
      estimate.suspended_ms = now_ms;
    } else {
      nav_correct_velocity(0, innovation.north, sample_dt, GPS_HORIZONTAL_FUSION_TIME, &shift);
      nav_correct_velocity(1, innovation.east, sample_dt, GPS_HORIZONTAL_FUSION_TIME, &shift);
    }
  }
  if (fabsf(innovation.up) <= gate)
    nav_correct_velocity(2, innovation.up, sample_dt, GPS_VERTICAL_FUSION_TIME, &shift);
  for (auto &entry : estimate.history) {
    entry.velocity += shift.velocity;
    entry.position[0] += shift.position[0];
    entry.position[1] += shift.position[1];
  }
}

// Predict the estimate with the accelerometer, correct it with the
// barometer and GPS velocity, and publish it.
static void nav_update_estimate(float dt, vec3_t force) {
  const uint32_t now_ms = time_millis();
  vec3_t accel = nav_earth_accel(force) - estimate.bias;
  if (!float_is_finite(vec3_dot(accel, accel)))
    accel = (vec3_t){};

  const bool horizontal = gps.position_valid && state.heading_confidence >= HORIZONTAL_MIN_HEADING_CONFIDENCE &&
                          now_ms - estimate.suspended_ms >= GPS_VELOCITY_SUSPEND_MS;
  if (horizontal && !estimate.horizontal)
    nav_start_horizontal(now_ms);
  estimate.horizontal = horizontal;

  estimate.height += estimate.velocity.up * dt;
  for (uint32_t axis = 0; axis < 2; axis++)
    estimate.position[axis] += estimate.velocity.axis[axis] * dt;
  for (uint32_t axis = estimate.horizontal ? 0 : 2; axis < 3; axis++)
    estimate.velocity.axis[axis] += accel.axis[axis] * dt;
  for (uint32_t axis = 0; axis < 2; axis++)
    estimate.inertial[axis] += accel.axis[axis] * dt;
  estimate.history_index = (estimate.history_index + 1) % ESTIMATE_HISTORY_SIZE;
  estimate.history[estimate.history_index] = {
      .ms = now_ms,
      .velocity = estimate.velocity,
      .position = {estimate.position[0], estimate.position[1]},
      .inertial = {estimate.inertial[0], estimate.inertial[1]},
  };

  nav_correct_baro(accel.up, dt);
  nav_correct_gps(now_ms);
  if (!estimate.horizontal)
    nav_follow_gps_horizontally(now_ms);

  if (!float_is_finite(vec3_dot(estimate.velocity, estimate.velocity)) || !float_is_finite(estimate.height)) {
    estimate.velocity = (vec3_t){};
    estimate.height = estimate.baro_height;
  }
  if (flags.altitude_valid)
    state.altitude = estimate.height;
  state.velocity = estimate.velocity;
  if (!float_is_finite(estimate.position[0]) || !float_is_finite(estimate.position[1]))
    estimate.positioned = false;
  state.position = estimate.positioned ? nav_estimated_coord() : state.gps_coord;
  const float baro_raw = estimate.baro_height + estimate.baro_drift;
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 2, (int16_t)constrain(baro_raw * 10.0f, -32768.0f, 32767.0f));
}

nav_command_result_t nav_command_check(nav_command_t *cmd) {
  if (cmd->type == NAV_COMMAND_MISSION && cmd->index == NAV_COMMAND_MISSION_RESUME)
    cmd->index = nav_route_resume();
  switch (cmd->type) {
  case NAV_COMMAND_GOTO:
    if (!mission_waypoint_valid(&cmd->waypoint))
      return NAV_RESULT_INVALID;
    return nav_within(cmd->waypoint.coord, MISSION_START_MAX_DISTANCE) ? NAV_RESULT_OK : NAV_RESULT_TOO_FAR;
  case NAV_COMMAND_SET_ALT: {
    if (state.nav_target != NAV_TARGET_GUIDED)
      return NAV_RESULT_INVALID;
    mission_waypoint_t target = nav_route.guided;
    target.altitude = cmd->waypoint.altitude;
    return mission_waypoint_valid(&target) ? NAV_RESULT_OK : NAV_RESULT_INVALID;
  }
  case NAV_COMMAND_MISSION:
    if (cmd->index >= mission.count)
      return NAV_RESULT_NO_MISSION;
    return nav_within(mission.waypoints[cmd->index].coord, MISSION_START_MAX_DISTANCE) ? NAV_RESULT_OK : NAV_RESULT_TOO_FAR;
  case NAV_COMMAND_HOLD:
  case NAV_COMMAND_RTH:
    return NAV_RESULT_OK;
  default:
    return NAV_RESULT_INVALID;
  }
}

nav_command_result_t nav_route_replace(const mission_t *updated, bool flying, bool locked, bool *retarget) {
  *retarget = false;
  if (!flying) {
    // A paused mission resumes where it left off only on the waypoint it was
    // flying; a changed route starts over.
    const uint8_t index = nav_route.index;
    if (index >= updated->count || memcmp(&updated->waypoints[index], &mission.waypoints[index], sizeof(mission_waypoint_t)) != 0) {
      nav_route.index = 0;
      nav_route.complete = false;
    }
    mission = *updated;
    return NAV_RESULT_OK;
  }
  if (updated->count == 0)
    return NAV_RESULT_NO_MISSION;

  // A completed mission stays complete at its possibly moved waypoint.
  const uint8_t index = MIN(nav_route.index, updated->count - 1);
  const mission_waypoint_t &target = updated->waypoints[index];
  *retarget = index != nav_route.index || memcmp(&target, &mission.waypoints[nav_route.index], sizeof(target)) != 0;
  if (*retarget && !nav_within(target.coord, MISSION_START_MAX_DISTANCE))
    return NAV_RESULT_TOO_FAR;
  if (*retarget && locked)
    return NAV_RESULT_NO_AUTHORITY;
  mission = *updated;
  nav_route.index = index;
  return NAV_RESULT_OK;
}

#ifdef VEHICLE_ROVER
nav_command_result_t nav_command_apply(nav_command_t *cmd) {
  return NAV_RESULT_UNSUPPORTED;
}

nav_command_result_t nav_mission_replace(const mission_t *updated) {
  bool retarget;
  return nav_route_replace(updated, false, false, &retarget);
}
#endif

// Apply the latest ground station command before the vehicle update. IO has
// already staged and validated a mission commit; Flight only swaps it in.
static void nav_update_command() {
  nav_command_t cmd;
  if (!quic_rt_take(&cmd))
    return;
  nav_command_result_t result;
  if (cmd.type == NAV_COMMAND_MISSION_COMMIT) {
    result = nav_mission_replace(quic_rt_staged());
    if (result == NAV_RESULT_OK)
      mission_save_later();
  } else {
    result = nav_command_apply(&cmd);
  }
  quic_rt_complete(cmd.id, result);
}

void nav_init() {
  nav = {.last_update_us = time_micros()};
  baro = {};
  gps = {};
  nav_route = {};
  estimate = {.suspended_ms = time_millis() - GPS_VELOCITY_SUSPEND_MS};
  accel = {};
  state.altitude = 0;
  state.velocity = {};
  state.position = {};
  state.gps_accel = {};
  state.gps_trust = 1.0f;
  flags.altitude_valid = false;
  flags.gps_glitch = false;
  flags.gps_valid = false;
  flags.home_valid = false;
}

// The 10 ms step; kept out of line so the per-pass gate stays small.
__attribute__((noinline)) static void nav_step(uint32_t now) {
  // Coalesce delayed updates. Altitude uses sensor timestamps; control uses elapsed time.
  const float dt = MIN((now - nav.last_update_us) * 1e-6f, NAV_MAX_DT);
  nav.last_update_us = now;
  const bool arming = flags.arm_state && !nav.armed;
  nav.armed = flags.arm_state;

  nav_update_altitude(arming);
  const bool gps_configured = profile.serial.gps != SERIAL_PORT_INVALID;
  if (gps_configured) {
    nav_update_gps_sanity();
  } else {
    flags.gps_valid = false;
    gps.position_valid = false;
  }
  nav_update_estimate(dt, nav_take_accel());
  if (gps_configured)
    nav_update_home(arming);
  else
    flags.home_valid = false;
  nav_update_command();
#ifdef VEHICLE_MULTI
  nav_update_multi(dt);
#endif
#ifdef VEHICLE_WING
  // Wing navigation also runs without GPS, as a constant-bank turn.
  nav_update_wing(dt);
#endif
}

void nav_update() {
  // Average the accelerometer over every Flight loop between navigation steps.
  nav_accumulate_accel();

  const uint32_t now = time_micros();
  if (now - nav.last_update_us >= 10000)
    nav_step(now);
}

#ifdef PIO_UNIT_TESTING
// A barometer sample, relative to a zero reference, then the estimate
// published without advancing it.
void nav_test_altitude_sample(float altitude, uint32_t timestamp_ms) {
  baro.referenced = true;
  nav_take_altitude(altitude, timestamp_ms);
  nav_update_estimate(0.0f, nav_take_accel());
}

// Set the vertical estimate, as from a fresh barometer, for guidance tests.
void nav_test_set_vertical(float altitude, float climb) {
  baro.referenced = true;
  baro.last_update_ms = time_millis();
  estimate.height = estimate.baro_height = altitude;
  estimate.velocity.up = climb;
  nav_update_estimate(0.0f, nav_take_accel());
}

void nav_test_update_gps_sanity(void) {
  nav_update_gps_sanity();
}

void nav_test_vertical_update(float dt) {
  nav_accumulate_accel();
  nav_update_estimate(dt, nav_take_accel());
}
#endif
