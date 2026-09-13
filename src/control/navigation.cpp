#include "navigation.h"

#include "control/control.h"
#ifdef VEHICLE_MULTI
#include "control/multi/navigation.h"
#endif
#include "driver/baro/baro.h"
#include "driver/time.h"
#include "io/blackbox.h"
#include "util/util.h"

static uint32_t last_update_us;
static float filtered_altitude;
static float launch_altitude; // Filtered absolute altitude latched on arming, meters.
static bool was_armed;

static void nav_filter_altitude(float altitude, uint32_t timestamp_ms) {
  // Hardware uses -Ofast, which can remove ordinary isfinite() checks.
  const union { float value; uint32_t bits; } sample = {.value = altitude};
  if ((sample.bits & 0x7f800000U) == 0x7f800000U) {
    state.baro_valid = false;
    return;
  }
  const uint32_t elapsed_ms = timestamp_ms - state.baro_last_update_ms;
  if (!state.baro_valid || elapsed_ms > BARO_STALE_MS) {
    filtered_altitude = altitude;
    state.baro_vertical_speed = 0;
  } else if (elapsed_ms > 0) {
    const float dt = elapsed_ms * 0.001f;
    const float previous = filtered_altitude;
    filtered_altitude += dt / (0.1f + dt) * (altitude - filtered_altitude);
    const float velocity = (filtered_altitude - previous) / dt;
    state.baro_vertical_speed += dt / (0.2f + dt) * (velocity - state.baro_vertical_speed);
  }
  state.baro_valid = true;
  state.baro_last_update_ms = timestamp_ms;
}

static void nav_update_altitude() {
  baro_sample_t sample;
  const bool updated = baro_take_sample(sample);
  if (updated)
    nav_filter_altitude(sample.altitude, sample.timestamp_ms);

  const bool armed = flags.arm_state;
  if (armed && !was_armed)
    launch_altitude = filtered_altitude;
  was_armed = armed;
  if (!armed) {
    state.altitude = 0;
    launch_altitude = 0;
  } else if (updated && state.baro_valid) {
    state.altitude = filtered_altitude - launch_altitude;
  }
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 2, (int16_t)constrain(state.altitude * 10.0f, -32768.0f, 32767.0f));
}

void nav_init() {
  last_update_us = time_micros();
  filtered_altitude = 0;
  launch_altitude = 0;
  was_armed = false;
  state.altitude = 0;
  state.baro_vertical_speed = 0;
  state.baro_last_update_ms = 0;
  state.baro_valid = false;
}

void nav_update() {
  const uint32_t now = time_micros();
  if (now - last_update_us < 10000)
    return;
  // Coalesce delayed updates. Altitude uses sensor timestamps; RTH uses elapsed time.
  last_update_us = now;
  nav_update_altitude();
#ifdef VEHICLE_MULTI
  if (profile.serial.gps != SERIAL_PORT_INVALID)
    nav_update_rth();
#endif
}

#ifdef PIO_UNIT_TESTING
void nav_test_altitude_sample(float altitude, uint32_t timestamp_ms) {
  nav_filter_altitude(altitude, timestamp_ms);
  state.altitude = filtered_altitude - launch_altitude;
}
#endif
