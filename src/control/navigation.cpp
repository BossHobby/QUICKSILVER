#include "navigation.h"

#include <math.h>

#include "control/control.h"
#ifdef VEHICLE_MULTI
#include "control/multi/navigation.h"
#endif
#include "core/profile.h"
#include "driver/baro/baro.h"
#include "driver/time.h"
#include "io/blackbox.h"
#include "io/gps.h"
#include "util/util.h"

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

// Position sanity survives arm cycles. Home follows the aircraft while
// disarmed and is latched on arming when the position is sane.
static struct {
  bool valid;
  bool home_valid;
  gps_coord_t position;
  bool position_valid;
  uint32_t checked_ms;
} gps;

static void nav_filter_altitude(float altitude, uint32_t timestamp_ms) {
  // Hardware uses -Ofast, which can remove ordinary isfinite() checks.
  const union { float value; uint32_t bits; } sample = {.value = altitude};
  if ((sample.bits & 0x7f800000U) == 0x7f800000U) {
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
    baro.launch = baro.filtered;
    baro.referenced = true;
  }
  if (baro.referenced && (arming || (updated && state.baro_valid)))
    state.altitude = baro.filtered - baro.launch;
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 2, (int16_t)constrain(state.altitude * 10.0f, -32768.0f, 32767.0f));
}

void nav_position_delta(gps_coord_t start, gps_coord_t end, float *north, float *east) {
  const float scale = 1.0f / 10000000.0f;
  const float lat_diff = (end.lat - start.lat) * scale;
  const float lon_diff = (end.lon - start.lon) * scale;
  const float lat_rad = start.lat * scale * DEGTORAD;
  *north = lat_diff * METERS_PER_DEGREE_LAT;
  *east = lon_diff * (METERS_PER_DEGREE_LAT * cosf(lat_rad));
}

static float nav_distance_between(gps_coord_t start, gps_coord_t end) {
  float north, east;
  nav_position_delta(start, end, &north, &east);
  return sqrtf(north * north + east * east);
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
    const float jump = nav_distance_between(gps.position, state.gps_coord);
    if (jump > NAV_GPS_MAX_POSITION_JUMP) {
      gps.valid = false;
      return;
    }
  }

  gps.position = state.gps_coord;
  gps.position_valid = true;
  gps.valid = true;
}

static void nav_update_home_vector(const gps_coord_t start, const gps_coord_t end) {
  const float scale = M_PI_F / (180.0f * 10000000.0f);

  const float lat_start_rad = (float)start.lat * scale;
  const float lon_start_rad = (float)start.lon * scale;
  const float lat_end_rad = (float)end.lat * scale;
  const float lon_end_rad = (float)end.lon * scale;

  const float cos_lat_start = cosf(lat_start_rad);
  const float sin_lat_start = sinf(lat_start_rad);
  const float cos_lat_end = cosf(lat_end_rad);
  const float sin_lat_end = sinf(lat_end_rad);

  const float delta_lon = lon_end_rad - lon_start_rad;
  const float cos_delta_lon = cosf(delta_lon);
  const float sin_delta_lon = sinf(delta_lon);

  const float sin_half_delta_lat = sinf((lat_end_rad - lat_start_rad) * 0.5f);
  const float sin_half_delta_lon = sinf(delta_lon * 0.5f);
  const float a = sin_half_delta_lat * sin_half_delta_lat +
                  cos_lat_start * cos_lat_end * sin_half_delta_lon * sin_half_delta_lon;

  state.home_distance = EARTH_RADIUS * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

  const float y = sin_delta_lon * cos_lat_end;
  const float x = cos_lat_start * sin_lat_end - sin_lat_start * cos_lat_end * cos_delta_lon;

  state.home_bearing = normalize_rad(atan2f(y, x)) * RADTODEG;
}

static void nav_update_home(bool arming) {
  nav_update_gps_sanity();

  if (arming) {
    gps.home_valid = gps.valid;
    if (gps.home_valid)
      state.gps_home = state.gps_coord;
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
  state.altitude = 0;
  state.baro_vertical_speed = 0;
  state.baro_last_update_ms = 0;
  state.baro_valid = false;
}

void nav_update() {
  const uint32_t now = time_micros();
  if (now - nav.last_update_us < 10000)
    return;
  // Coalesce delayed updates. Altitude uses sensor timestamps; RTH uses elapsed time.
  nav.last_update_us = now;
  const bool arming = flags.arm_state && !nav.armed;
  nav.armed = flags.arm_state;

  nav_update_altitude(arming);
  if (profile.serial.gps == SERIAL_PORT_INVALID)
    return;
  nav_update_home(arming);
#ifdef VEHICLE_MULTI
  nav_update_rth(gps.valid, gps.home_valid);
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

bool nav_test_gps_sane(void) {
  return gps.valid;
}
#endif
