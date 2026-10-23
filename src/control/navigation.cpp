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

// Barometer correction time constant of the vertical complementary filter.
#define VERTICAL_FUSION_TIME 1.0f
#define VERTICAL_BIAS_LIMIT 1.0f // m/s^2 accelerometer bias the filter may absorb
// Without a barometer, velocity is an accelerometer integral; leak it toward
// zero so bias cannot build into a sustained climb or descent demand.
#define VERTICAL_NO_BARO_DECAY_TIME 5.0f
#define NAV_MAX_DT 0.1f

static constexpr float NAV_GPS_MAX_HORIZONTAL_ACCURACY = 30.0f; // meters
static constexpr float NAV_GPS_MAX_POSITION_JUMP = 50.0f;       // meters per fix

static struct {
  uint32_t last_update_us;
  bool armed; // Arming edge for the launch altitude and home.
} nav;

static struct {
  float filtered;  // Absolute altitude, meters.
  float launch;    // Filtered absolute altitude latched on the first sample and on arming, meters.
  bool referenced; // launch holds a reference; until then altitude stays zero.
} baro;

// Last sane position, kept across arm cycles to reject jumps.
static struct {
  gps_coord_t position;
  bool position_valid;
  uint32_t checked_ms;
} gps;

// Complementary filter state. Heights are relative to baro.launch.
static struct {
  float height;
  float velocity;
  float bias;
  float baro_height; // latest unfiltered sample
  bool baro_fresh;
  float accel_sum;
  uint32_t accel_count;
} vertical;

nav_route_t nav_route;

static void nav_filter_altitude(float altitude, uint32_t timestamp_ms) {
  if (!float_is_finite(altitude)) {
    flags.baro_valid = false;
    return;
  }
  const uint32_t elapsed_ms = timestamp_ms - state.baro_last_update_ms;
  if (!flags.baro_valid || elapsed_ms > BARO_STALE_MS) {
    baro.filtered = altitude;
    state.baro_vertical_speed = 0;
  } else if (elapsed_ms > 0) {
    const float dt = elapsed_ms * 0.001f;
    const float previous = baro.filtered;
    baro.filtered += dt / (0.1f + dt) * (altitude - baro.filtered);
    const float velocity = (baro.filtered - previous) / dt;
    state.baro_vertical_speed += dt / (0.2f + dt) * (velocity - state.baro_vertical_speed);
  }
  flags.baro_valid = true;
  state.baro_last_update_ms = timestamp_ms;
  vertical.baro_height = altitude - baro.launch;
}

static void nav_update_altitude(bool arming) {
  baro_sample_t sample;
  const bool updated = baro_take_sample(sample);
  if (updated)
    nav_filter_altitude(sample.altitude, sample.timestamp_ms);

  // Disarmed altitude is relative to the first sample (or the last launch),
  // so the bench shows baro movement; arming re-zeroes it.
  const bool first = updated && flags.baro_valid && !baro.referenced;
  if (arming || first) {
    // Keep the fused estimate continuous across the new reference.
    const float shift = baro.filtered - baro.launch;
    vertical.height -= shift;
    vertical.baro_height -= shift;
    baro.launch = baro.filtered;
    baro.referenced = true;
  }
  if (baro.referenced && (arming || (updated && flags.baro_valid)))
    state.altitude = baro.filtered - baro.launch;
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 2, (int16_t)constrain(state.altitude * 10.0f, -32768.0f, 32767.0f));
}

static void nav_accumulate_accel() {
  // accel_raw and GEstG share the body frame in g, and GEstG is normalized to
  // ACC_1G: their dot product is the specific force along earth vertical.
  const float specific_force = vec3_dot(state.accel_raw, state.GEstG) / (ACC_1G * ACC_1G);
  if (!float_is_finite(specific_force))
    return;
  vertical.accel_sum += (specific_force - 1.0f) * GRAVITY;
  vertical.accel_count++;
}

static void nav_update_vertical(float dt) {
  float accel = 0.0f;
  if (vertical.accel_count) {
    accel = vertical.accel_sum / vertical.accel_count;
  }
  vertical.accel_sum = 0.0f;
  vertical.accel_count = 0;
  accel -= vertical.bias;
  state.vertical_accel += dt / (0.1f + dt) * (accel - state.vertical_accel);

  const bool baro_fresh = flags.baro_valid && time_millis() - state.baro_last_update_ms <= BARO_STALE_MS;
  if (!baro_fresh)
    flags.baro_valid = false;
  if (baro_fresh && !vertical.baro_fresh) {
    // (Re)acquisition: start from the measurement instead of a drifted integral.
    vertical.height = vertical.baro_height;
  }
  vertical.baro_fresh = baro_fresh;

  if (baro_fresh) {
    // Third-order complementary filter: accelerometer for the fast response,
    // barometer for height, velocity and accelerometer bias.
    const float k1 = 3.0f / VERTICAL_FUSION_TIME;
    const float k2 = 3.0f / (VERTICAL_FUSION_TIME * VERTICAL_FUSION_TIME);
    const float k3 = 1.0f / (VERTICAL_FUSION_TIME * VERTICAL_FUSION_TIME * VERTICAL_FUSION_TIME);
    const float error = vertical.baro_height - vertical.height;
    vertical.height += (vertical.velocity + k1 * error) * dt;
    vertical.velocity += (accel + k2 * error) * dt;
    vertical.bias = constrain(vertical.bias - k3 * error * dt, -VERTICAL_BIAS_LIMIT, VERTICAL_BIAS_LIMIT);
  } else {
    vertical.height += vertical.velocity * dt;
    vertical.velocity += accel * dt;
    vertical.velocity -= dt / (VERTICAL_NO_BARO_DECAY_TIME + dt) * vertical.velocity;
  }

  if (!float_is_finite(vertical.velocity) || !float_is_finite(vertical.height)) {
    vertical.velocity = 0.0f;
    vertical.height = vertical.baro_height;
  }
  state.vertical_speed = vertical.velocity;
}

// Equirectangular at the mean latitude: within about 0.1% of the great-circle
// distance below 100 km at mid latitudes, beyond any range flown on a battery.
void nav_position_delta(gps_coord_t start, gps_coord_t end, float *north, float *east) {
  const float scale = 1.0f / 10000000.0f;
  const float lat_diff = (end.lat - start.lat) * scale;
  const float lon_diff = (end.lon - start.lon) * scale;
  const float lat_rad = 0.5f * ((float)start.lat + (float)end.lat) * scale * DEGTORAD;
  *north = lat_diff * METERS_PER_DEGREE_LAT;
  *east = lon_diff * (METERS_PER_DEGREE_LAT * cosf(lat_rad));
}

static float nav_distance_between(gps_coord_t start, gps_coord_t end) {
  float north, east;
  nav_position_delta(start, end, &north, &east);
  return hypotf(north, east);
}

// Distance in meters and bearing in degrees true from start to end.
static void nav_vector(gps_coord_t start, gps_coord_t end, float *distance, float *bearing) {
  float north, east;
  nav_position_delta(start, end, &north, &east);
  *distance = hypotf(north, east);
  *bearing = normalize_rad(atan2f(east, north)) * RADTODEG;
}

bool nav_in_start_range(gps_coord_t target) {
  return nav_distance_between(state.gps_coord, target) <= MISSION_START_MAX_DISTANCE;
}

bool nav_mission_in_start_range(uint8_t index) {
  return nav_in_start_range(mission.waypoints[index].coord);
}

float nav_leg_altitude(gps_coord_t origin, float origin_altitude, gps_coord_t target, float target_altitude, float *rate) {
  if (rate)
    *rate = 0.0f;
  float leg_north, leg_east, north, east;
  nav_position_delta(origin, target, &leg_north, &leg_east);
  const float length_squared = leg_north * leg_north + leg_east * leg_east;
  if (length_squared < 1.0f)
    return target_altitude;
  // Along-track progress: wandering across the leg does not change the target.
  nav_position_delta(origin, state.gps_coord, &north, &east);
  const float progress = (north * leg_north + east * leg_east) / length_squared;
  const float change = target_altitude - origin_altitude;
  if (rate && progress > 0.0f && progress < 1.0f)
    *rate = change * (state.gps_vel_north * leg_north + state.gps_vel_east * leg_east) / length_squared;
  return origin_altitude + constrain(progress, 0.0f, 1.0f) * change;
}

uint8_t nav_route_resume() {
  return nav_route.complete ? 0 : nav_route.index;
}

void nav_route_start_leg() {
  nav_route.leg_origin = state.gps_coord;
  nav_route_restart_altitude();
}

void nav_route_restart_altitude() {
  nav_route.altitude_origin = state.gps_coord;
  nav_route.leg_altitude = state.altitude;
}

void nav_route_next() {
  const mission_waypoint_t &reached = mission.waypoints[nav_route.index];
  nav_route.leg_origin = nav_route.altitude_origin = reached.coord;
  nav_route.leg_altitude = reached.altitude;
  nav_route.index++;
}

bool nav_mission_in_takeoff_range(uint8_t index) {
  return nav_distance_between(state.gps_coord, mission.waypoints[index].coord) <= MISSION_TAKEOFF_MAX_DISTANCE;
}

gps_coord_t nav_target_coord() {
  if (state.nav_target == NAV_TARGET_GUIDED)
    return nav_route.guided.coord;
  if (state.nav_target == NAV_TARGET_MISSION)
    return mission.waypoints[nav_route.index].coord;
  return state.gps_home;
}

void nav_update_target() {
  // nav_route.index survives interruptions, but is published only while flown.
  state.mission_index = state.nav_target == NAV_TARGET_MISSION ? nav_route.index : 0;
  if (state.nav_target == NAV_TARGET_HOME) {
    state.nav_target_distance = state.home_distance;
    state.nav_target_bearing = state.home_bearing;
    return;
  }
  nav_vector(state.gps_coord, nav_target_coord(), &state.nav_target_distance, &state.nav_target_bearing);
}

static bool nav_gps_quality_ok(void) {
  return state.gps_lock &&
         state.gps_sats >= GPS_MIN_SATS_FOR_LOCK &&
         state.gps_horizontal_accuracy <= NAV_GPS_MAX_HORIZONTAL_ACCURACY &&
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

  if (gps.position_valid) {
    const float jump = nav_distance_between(gps.position, state.gps_coord);
    if (jump > NAV_GPS_MAX_POSITION_JUMP) {
      flags.gps_valid = false;
      return;
    }
  }

  gps.position = state.gps_coord;
  gps.position_valid = true;
  flags.gps_valid = true;
}

static void nav_update_home(bool arming) {
  nav_update_gps_sanity();

  if (arming) {
    flags.home_valid = flags.gps_valid;
    if (flags.home_valid) {
      state.gps_home = state.gps_coord;
      state.home_altitude = state.gps_altitude;
    }
  }

  if (!flags.arm_state) {
    state.gps_home = state.gps_coord;
    flags.home_valid = false;
  } else if (flags.gps_valid && flags.home_valid) {
    nav_vector(state.gps_coord, state.gps_home, &state.home_distance, &state.home_bearing);
  }
}

nav_command_result_t nav_command_check(nav_command_t *cmd) {
  if (cmd->type == NAV_COMMAND_MISSION && cmd->index == NAV_COMMAND_MISSION_RESUME)
    cmd->index = nav_route_resume();
  switch (cmd->type) {
  case NAV_COMMAND_GOTO:
    if (!mission_waypoint_valid(&cmd->waypoint))
      return NAV_RESULT_INVALID;
    return nav_in_start_range(cmd->waypoint.coord) ? NAV_RESULT_OK : NAV_RESULT_TOO_FAR;
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
    return nav_mission_in_start_range(cmd->index) ? NAV_RESULT_OK : NAV_RESULT_TOO_FAR;
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
    mission = *updated;
    nav_route.index = MIN(nav_route.index, MAX(updated->count, 1) - 1);
    return NAV_RESULT_OK;
  }
  if (updated->count == 0)
    return NAV_RESULT_NO_MISSION;

  // A completed mission stays complete at its possibly moved waypoint.
  const uint8_t index = MIN(nav_route.index, updated->count - 1);
  const mission_waypoint_t &target = updated->waypoints[index];
  *retarget = index != nav_route.index || memcmp(&target, &mission.waypoints[nav_route.index], sizeof(target)) != 0;
  if (*retarget && !nav_in_start_range(target.coord))
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
    if (result == NAV_RESULT_OK) {
      mission_changed();
      mission_save_later();
    }
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
  vertical = {};
  state.altitude = 0;
  state.baro_vertical_speed = 0;
  state.vertical_speed = 0;
  state.baro_last_update_ms = 0;
  flags.baro_valid = false;
  state.vertical_accel = 0;
  flags.gps_valid = false;
  flags.home_valid = false;
}

void nav_update() {
  // Average the accelerometer over every Flight loop between navigation steps.
  nav_accumulate_accel();

  const uint32_t now = time_micros();
  if (now - nav.last_update_us < 10000)
    return;
  // Coalesce delayed updates. Altitude uses sensor timestamps; control uses elapsed time.
  const float dt = MIN((now - nav.last_update_us) * 1e-6f, NAV_MAX_DT);
  nav.last_update_us = now;
  const bool arming = flags.arm_state && !nav.armed;
  nav.armed = flags.arm_state;

  nav_update_altitude(arming);
  nav_update_vertical(dt);
  if (profile.serial.gps != SERIAL_PORT_INVALID) {
    nav_update_home(arming);
  } else {
    flags.gps_valid = false;
    flags.home_valid = false;
  }
  nav_update_command();
#ifdef VEHICLE_MULTI
  nav_update_multi(dt);
#endif
#ifdef VEHICLE_WING
  // Wing navigation also runs without GPS, as a constant-bank turn.
  nav_update_wing(dt);
#endif
}

#ifdef PIO_UNIT_TESTING
void nav_test_altitude_sample(float altitude, uint32_t timestamp_ms) {
  nav_filter_altitude(altitude, timestamp_ms);
  state.altitude = baro.filtered - baro.launch;
}

void nav_test_update_gps_sanity(void) {
  nav_update_gps_sanity();
}

void nav_test_vertical_update(float dt) {
  nav_accumulate_accel();
  nav_update_vertical(dt);
}
#endif
