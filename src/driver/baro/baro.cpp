#include "baro.h"

#include <math.h>

#include "control/control.h"
#include "core/tasks.h"
#include "driver/i2c.h"
#include "driver/time.h"
#include "io/blackbox.h"
#include "util/util.h"

#include "driver/baro/bmp280.h"
#include "driver/baro/bmp388.h"
#include "driver/baro/dps310.h"

struct baro_sample_t {
  float altitude;
  float vertical_speed;
  uint32_t last_update_ms;
  bool valid;
};

static float filtered_altitude;
static float launch_altitude; // Filtered absolute altitude latched on arming, meters.
static bool was_armed;

#ifdef USE_BARO

#define P0 101325.0f // Standard pressure at sea level in pascals (Pa)

uint8_t baro_buf[6];
i2c_bus_device_t baro_bus;

static baro_interface_t *baro = NULL;
static baro_types_t baro_type = BARO_TYPE_INVALID;
static baro_interface_t *const baro_interfaces[BARO_TYPE_MAX] = {
    [BARO_TYPE_INVALID] = NULL,
    [BARO_TYPE_BMP280] = &bmp280_interface,
    [BARO_TYPE_BMP388] = &bmp388_interface,
    [BARO_TYPE_DPS310] = &dps310_interface,
};

static baro_types_t baro_detect() {
  baro_type = BARO_TYPE_INVALID;
  if (target.baro.port == I2C_PORT_INVALID)
    return baro_type;

  baro_bus.port = target.baro.port;
  if (!i2c_bus_device_init(&baro_bus))
    return baro_type;

  for (unsigned i = BARO_TYPE_INVALID + 1; i < BARO_TYPE_MAX; i++) {
    baro = baro_interfaces[i];
    baro_type = baro->init();
    if (baro_type != BARO_TYPE_INVALID)
      break;
  }

  return baro_type;
}

static float baro_pressure_to_altitude(const float pressure) {
  return (1.0f - powf(pressure / P0, 1.0f / 5.25588f)) / 2.25577e-5f;
}

static bool baro_read(float &altitude) {
  if (baro_type == BARO_TYPE_INVALID)
    return false;

  if (!i2c_is_idle(&baro_bus))
    return false;

  float pressure;
  if (!baro->get_pressure(&pressure))
    return false;

  altitude = baro_pressure_to_altitude(pressure);
  return true;
}
#else
static baro_types_t baro_detect() { return BARO_TYPE_INVALID; }
static bool baro_read(float &altitude) { return false; }
#endif

baro_types_t baro_init() {
  filtered_altitude = 0;
  was_armed = false;
  state.baro_valid = false;
  state.baro_last_update_ms = 0;
  state.baro_vertical_speed = 0;
  launch_altitude = 0;
  state.altitude = 0;
  const baro_types_t detected = baro_detect();
  state.baro_detected = detected != BARO_TYPE_INVALID;
  return detected;
}

// TODO: Move filtering, vertical speed and launch-relative altitude to a shared altitude estimator.
static void baro_filter_sample(float altitude, uint32_t now_ms, baro_sample_t &next) {
  // Hardware uses -Ofast, which can remove ordinary isfinite() checks.
  const union { float value; uint32_t bits; } sample = {.value = altitude};
  if ((sample.bits & 0x7f800000U) == 0x7f800000U) {
    next.valid = false;
    return;
  }
  const uint32_t elapsed_ms = now_ms - next.last_update_ms;
  if (!next.valid || elapsed_ms > BARO_STALE_MS) {
    filtered_altitude = altitude;
    next.vertical_speed = 0;
  } else if (elapsed_ms > 0) {
    const float dt = elapsed_ms * 0.001f;
    const float previous = filtered_altitude;
    filtered_altitude += dt / (0.1f + dt) * (altitude - filtered_altitude);
    const float velocity = (filtered_altitude - previous) / dt;
    next.vertical_speed += dt / (0.2f + dt) * (velocity - next.vertical_speed);
  }
  next.valid = true;
  next.last_update_ms = now_ms;
}

static void baro_publish_sample(const baro_sample_t &next) {
  // Only the completed sample publication excludes Flight preemption.
  taskENTER_CRITICAL();
  state.altitude = next.altitude;
  state.baro_vertical_speed = next.vertical_speed;
  state.baro_last_update_ms = next.last_update_ms;
  state.baro_valid = next.valid;
  taskEXIT_CRITICAL();
}

void baro_update() {
  float altitude;
  const bool updated = baro_read(altitude);
  baro_sample_t next = {state.altitude, state.baro_vertical_speed, state.baro_last_update_ms, state.baro_valid};
  if (updated)
    baro_filter_sample(altitude, time_millis(), next);

  const bool armed = flags.arm_state;
  if (armed && !was_armed)
    launch_altitude = filtered_altitude;
  was_armed = armed;

  if (!armed) {
    next.altitude = 0;
    launch_altitude = 0;
  } else if (updated && next.valid) {
    next.altitude = filtered_altitude - launch_altitude;
  }
  baro_publish_sample(next);
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 2, (int16_t)constrain(state.altitude * 10.0f, -32768.0f, 32767.0f));
}

#ifdef PIO_UNIT_TESTING
void baro_test_sample(float altitude, uint32_t now_ms) {
  baro_sample_t next = {state.altitude, state.baro_vertical_speed, state.baro_last_update_ms, state.baro_valid};
  baro_filter_sample(altitude, now_ms, next);
  next.altitude = filtered_altitude - launch_altitude;
  baro_publish_sample(next);
}
#endif
