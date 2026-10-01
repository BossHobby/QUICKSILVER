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

#ifdef USE_BARO

#define P0 101325.0f // Standard pressure at sea level in pascals (Pa)
#define BARO_UPDATE_PERIOD_US 10000
static constexpr uint32_t BARO_CONVERSION_POLL_US = 2000;

typedef enum {
  BARO_PHASE_IDLE,
  BARO_PHASE_STATUS, // Status read in flight.
  BARO_PHASE_DATA,   // Data read in flight.
} baro_phase_t;

i2c_bus_device_t baro_bus;

static baro_interface_t *baro = NULL;
static baro_types_t baro_type = BARO_TYPE_INVALID;
static baro_interface_t *const baro_interfaces[BARO_TYPE_MAX] = {
    [BARO_TYPE_INVALID] = NULL,
    [BARO_TYPE_BMP280] = &bmp280_interface,
    [BARO_TYPE_BMP388] = &bmp388_interface,
    [BARO_TYPE_DPS310] = &dps310_interface,
};

// IO owns the read cycle; the I2C ISR fills these buffers.
static baro_phase_t baro_phase;
static uint32_t next_read_us;
static uint8_t baro_status;
static uint8_t baro_data[6];

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
#else
static baro_types_t baro_detect() { return BARO_TYPE_INVALID; }
#endif

baro_types_t baro_init() {
  sample_pending = false;
#ifdef USE_BARO
  baro_phase = BARO_PHASE_IDLE;
  next_read_us = time_micros(); // first sample runs immediately
#endif
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

  // The transfer's completion interrupt wakes the worker to continue.
  if (!i2c_is_idle(&baro_bus))
    return portMAX_DELAY;

  const uint32_t now = time_micros();
  switch (baro_phase) {
  case BARO_PHASE_STATUS:
    if (baro->data_ready(baro_status)) {
      i2c_read_reg_bytes(&baro_bus, baro->data_reg, baro_data, sizeof(baro_data));
      baro_phase = BARO_PHASE_DATA;
      return portMAX_DELAY;
    }
    // The device-side conversion has no interrupt; poll its status shortly.
    next_read_us = now + BARO_CONVERSION_POLL_US;
    break;

  case BARO_PHASE_DATA:
    baro_publish_sample(baro_pressure_to_altitude(baro->compensate(baro_data)), time_millis());
    break;

  case BARO_PHASE_IDLE:
    break;
  }
  baro_phase = BARO_PHASE_IDLE;

  const int32_t wait_us = (int32_t)(next_read_us - now);
  if (wait_us > 0)
    return pdMS_TO_TICKS((wait_us + 999) / 1000);

  next_read_us = now + BARO_UPDATE_PERIOD_US;
  i2c_read_reg_bytes(&baro_bus, baro->status_reg, &baro_status, 1);
  baro_phase = BARO_PHASE_STATUS;
  return portMAX_DELAY;
#else
  return portMAX_DELAY;
#endif
}

#ifdef PIO_UNIT_TESTING
void baro_test_sample(float altitude, uint32_t timestamp_ms) {
  baro_publish_sample(altitude, timestamp_ms);
}
#endif
