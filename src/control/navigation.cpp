#include "navigation.h"

#include <math.h>

#include "control/control.h"
#include "control/imu.h"
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
#include "util/util.h"
#include "util/vector.h"

#define GRAVITY_MS2 9.80665f
// Barometer correction time constant of the vertical complementary filter.
#define VERTICAL_FUSION_TIME 1.0f
#define VERTICAL_BIAS_LIMIT 1.0f // m/s^2 accelerometer bias the filter may absorb
// Without a barometer, velocity is an accelerometer integral; leak it toward
// zero so bias cannot build into a sustained climb or descent demand.
#define VERTICAL_NO_BARO_DECAY_TIME 5.0f
#define NAV_MAX_DT 0.1f

static constexpr float NAV_GPS_MAX_HORIZONTAL_ACCURACY = 30.0f; // meters
static constexpr float NAV_GPS_MAX_POSITION_JUMP = 50.0f;       // meters per fix
static constexpr int64_t GPS_LON_HALF_TURN = 1800000000;        // 180 degrees * 1e7

static struct {
  uint32_t last_update_us;
  bool armed; // Arming edge for the launch altitude and home.
} nav;

static struct {
  float filtered;  // Absolute altitude, meters.
  float launch;    // Filtered absolute altitude latched on the first sample and on arming, meters.
  bool referenced; // launch holds a reference; until then altitude stays zero.
} baro;

// Position sanity survives arm cycles. Home follows the aircraft while
// disarmed and is latched on arming when the position is sane.
static struct {
  bool valid;
  bool home_valid;
  float home_altitude; // GPS altitude at the home capture, meters MSL.
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
  float accel; // bias-corrected, smoothed earth-vertical acceleration, m/s^2 up
} vertical;

// Last GPS velocity sample; the derived acceleration is published in state.
static struct {
  float velocity_north;
  float velocity_east;
  bool sample_valid;
  uint32_t updated_ms;
} gps_accel;

static void nav_filter_altitude(float altitude, uint32_t timestamp_ms) {
  if (!float_is_finite(altitude)) {
    state.baro_valid = false;
    return;
  }
  const uint32_t elapsed_ms = timestamp_ms - state.baro_last_update_ms;
  if (!state.baro_valid || elapsed_ms > BARO_STALE_MS) {
    baro.filtered = altitude;
    state.baro_vertical_speed = 0;
  } else if (elapsed_ms > 0) {
    const float dt = elapsed_ms * 0.001f;
    const float previous = baro.filtered;
    baro.filtered += dt / (0.1f + dt) * (altitude - baro.filtered);
    const float velocity = (baro.filtered - previous) / dt;
    state.baro_vertical_speed += dt / (0.2f + dt) * (velocity - state.baro_vertical_speed);
  }
  state.baro_valid = true;
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
  const bool first = updated && state.baro_valid && !baro.referenced;
  if (arming || first) {
    // Keep the fused estimate continuous across the new reference.
    const float shift = baro.filtered - baro.launch;
    vertical.height -= shift;
    vertical.baro_height -= shift;
    baro.launch = baro.filtered;
    baro.referenced = true;
  }
  if (baro.referenced && (arming || (updated && state.baro_valid)))
    state.altitude = baro.filtered - baro.launch;
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 2, (int16_t)constrain(state.altitude * 10.0f, -32768.0f, 32767.0f));
}

static void nav_accumulate_accel() {
  // accel_raw and GEstG share the body frame in g, and GEstG is normalized to
  // ACC_1G: their dot product is the specific force along earth vertical.
  const float specific_force = vec3_dot(state.accel_raw, state.GEstG) / (ACC_1G * ACC_1G);
  if (!float_is_finite(specific_force))
    return;
  vertical.accel_sum += (specific_force - 1.0f) * GRAVITY_MS2;
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
  vertical.accel += dt / (0.1f + dt) * (accel - vertical.accel);

  const bool baro_fresh = state.baro_valid && time_millis() - state.baro_last_update_ms <= BARO_STALE_MS;
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

float nav_vertical_accel() {
  return vertical.accel;
}

void nav_reset_gps_accel() {
  gps_accel = {};
  state.nav_accel_north = state.nav_accel_east = 0.0f;
}

void nav_update_gps_accel() {
  // Differentiate measurements at GPS cadence, not at the 100 Hz control rate.
  if (gps_accel.sample_valid && state.gps_last_update_ms == gps_accel.updated_ms) {
    return;
  }

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

vec2_t nav_position_delta(gps_coord_t start, gps_coord_t end) {
  const float scale = 1.0f / 10000000.0f;
  const float lat_diff = (end.lat - start.lat) * scale;
  // Subtract in 64 bits and take the short way across the antimeridian.
  int64_t lon_delta = int64_t(end.lon) - start.lon;
  if (lon_delta > GPS_LON_HALF_TURN)
    lon_delta -= 2 * GPS_LON_HALF_TURN;
  else if (lon_delta < -GPS_LON_HALF_TURN)
    lon_delta += 2 * GPS_LON_HALF_TURN;
  const float lon_diff = lon_delta * scale;
  const float lat_rad = start.lat * scale * DEGTORAD;
  return {{lat_diff * METERS_PER_DEGREE_LAT, lon_diff * (METERS_PER_DEGREE_LAT * fastcos(lat_rad))}};
}

static bool nav_gps_quality_ok(void) {
  return state.gps_lock &&
         state.gps_sats >= GPS_MIN_SATS_FOR_LOCK &&
         state.gps_horizontal_accuracy <= NAV_GPS_MAX_HORIZONTAL_ACCURACY &&
         (time_millis() - state.gps_last_update_ms) <= NAV_GPS_STALE_MS;
}

static void nav_update_gps_sanity(void) {
  if (!nav_gps_quality_ok()) {
    gps.valid = false;
    gps.position_valid = false;
    return;
  }

  if (state.gps_last_update_ms == gps.checked_ms) {
    return;
  }
  gps.checked_ms = state.gps_last_update_ms;

  if (gps.position_valid) {
    const float jump = vec2_magnitude(nav_position_delta(gps.position, state.gps_coord));
    if (jump > NAV_GPS_MAX_POSITION_JUMP) {
      gps.valid = false;
      return;
    }
  }

  gps.position = state.gps_coord;
  gps.position_valid = true;
  gps.valid = true;
}

// Flat-earth vector from integer coordinate differences. Float haversine on
// absolute coordinates loses ~0.5 m and several degrees of bearing near home.
static void nav_update_home_vector(const gps_coord_t start, const gps_coord_t end) {
  const vec2_t delta = nav_position_delta(start, end);
  state.home_distance = vec2_magnitude(delta);
  state.home_bearing = normalize_rad(atan2approx_rad(delta.east, delta.north)) * RADTODEG;
}

static void nav_update_home(bool arming) {
  nav_update_gps_sanity();

  if (arming) {
    gps.home_valid = gps.valid;
    if (gps.home_valid) {
      state.gps_home = state.gps_coord;
      gps.home_altitude = state.gps_altitude;
    }
  }

  if (!flags.arm_state) {
    state.gps_home = state.gps_coord;
    gps.home_valid = false;
  } else if (gps.valid && gps.home_valid) {
    nav_update_home_vector(state.gps_coord, state.gps_home);
  }
}

void nav_init() {
  nav = {.last_update_us = time_micros()};
  baro = {};
  gps = {};
  vertical = {};
  nav_reset_gps_accel();
  state.altitude = 0;
  state.baro_vertical_speed = 0;
  state.vertical_speed = 0;
  state.baro_last_update_ms = 0;
  state.baro_valid = false;
}

// The 10 ms step; kept out of line so the per-pass gate stays small.
__attribute__((noinline)) static void nav_step(uint32_t now) {
  // Coalesce delayed updates. Altitude uses sensor timestamps; control uses elapsed time.
  const float dt = MIN((now - nav.last_update_us) * 1e-6f, NAV_MAX_DT);
  nav.last_update_us = now;
  const bool arming = flags.arm_state && !nav.armed;
  nav.armed = flags.arm_state;

  nav_update_altitude(arming);
  nav_update_vertical(dt);
  const bool gps_configured = profile.serial.gps != SERIAL_PORT_INVALID;
  if (gps_configured)
    nav_update_home(arming);
#ifdef VEHICLE_MULTI
  nav_update_multi(dt, gps.valid, gps.home_valid);
#endif
#ifdef VEHICLE_WING
  // Wing navigation also runs without GPS, as a constant-bank turn.
  nav_update_wing(gps_configured && gps.valid, gps_configured && gps.home_valid);
#endif
}

void nav_update() {
  // Average the accelerometer over every Flight loop between navigation steps.
  nav_accumulate_accel();

  const uint32_t now = time_micros();
  if (now - nav.last_update_us >= 10000)
    nav_step(now);
}

bool nav_home_valid() {
  return gps.home_valid;
}

float nav_home_altitude() {
  return gps.home_altitude;
}

#ifdef PIO_UNIT_TESTING
void nav_test_altitude_sample(float altitude, uint32_t timestamp_ms) {
  nav_filter_altitude(altitude, timestamp_ms);
  state.altitude = baro.filtered - baro.launch;
}

void nav_test_update_gps_sanity(void) {
  nav_update_gps_sanity();
}

void nav_test_set_home(bool valid, float altitude) {
  gps.home_valid = valid;
  gps.home_altitude = altitude;
}

bool nav_test_gps_sane(void) {
  return gps.valid;
}

void nav_test_vertical_update(float dt) {
  nav_accumulate_accel();
  nav_update_vertical(dt);
}
#endif
