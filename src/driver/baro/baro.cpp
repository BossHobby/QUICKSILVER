#include "baro.h"

#include <math.h>

#include "control/control.h"
#include "core/tasks.h"
#include "driver/i2c.h"
#include "driver/time.h"

#include "driver/baro/bmp280.h"
#include "driver/baro/bmp388.h"
#include "driver/baro/dps310.h"

static baro_sample_t latest_sample;
static bool sample_pending;
static uint32_t last_update_us;

#ifdef USE_BARO

#define P0 101325.0f // Standard pressure at sea level in pascals (Pa)
#define BARO_UPDATE_PERIOD_US 10000

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
  sample_pending = false;
  last_update_us = 0; // first sample runs immediately
  const baro_types_t detected = baro_detect();
  state.baro_detected = detected != BARO_TYPE_INVALID;
  return detected;
}

static void baro_publish_sample(float altitude, uint32_t timestamp_ms) {
  taskENTER_CRITICAL();
  latest_sample = {altitude, timestamp_ms};
  sample_pending = true;
  taskEXIT_CRITICAL();
}

bool baro_take_sample(baro_sample_t &sample) {
  taskENTER_CRITICAL();
  const bool pending = sample_pending;
  if (pending)
    sample = latest_sample;
  sample_pending = false;
  taskEXIT_CRITICAL();
  return pending;
}

TickType_t baro_update() {
#ifdef USE_BARO
  if (baro_type == BARO_TYPE_INVALID)
    return portMAX_DELAY;

  // A status or pressure read completes in the background; its completion
  // interrupt wakes the worker, so no periodic retry is needed here.
  if (!i2c_is_idle(&baro_bus))
    return portMAX_DELAY;

  const uint32_t now = time_micros();
  if (now - last_update_us < BARO_UPDATE_PERIOD_US) {
    return pdMS_TO_TICKS((BARO_UPDATE_PERIOD_US - (now - last_update_us)) / 1000);
  }
  last_update_us = now;

  float altitude;
  if (baro_read(altitude)) {
    baro_publish_sample(altitude, time_millis());
    return pdMS_TO_TICKS(BARO_UPDATE_PERIOD_US / 1000);
  }
  // Device-side conversion still in flight; unlike the transfer there is no
  // interrupt for it, so poll the status register shortly.
  return pdMS_TO_TICKS(2);
#else
  return portMAX_DELAY;
#endif
}

#ifdef PIO_UNIT_TESTING
void baro_test_sample(float altitude, uint32_t timestamp_ms) {
  baro_publish_sample(altitude, timestamp_ms);
}
#endif
