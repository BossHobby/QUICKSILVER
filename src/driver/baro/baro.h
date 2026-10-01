#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>

// Freshness limit for navigation's filtered altitude/velocity.
#define BARO_STALE_MS 500U

typedef enum {
  BARO_TYPE_INVALID,
  BARO_TYPE_BMP280,
  BARO_TYPE_BMP388,
  BARO_TYPE_DPS310,
  BARO_TYPE_MAX,
} baro_types_t;

typedef struct {
  baro_types_t (*init)(void);
  uint8_t status_reg;
  uint8_t data_reg; // First of six pressure and temperature bytes.
  bool (*data_ready)(uint8_t status);
  float (*compensate)(const uint8_t data[6]); // Pressure in pascals.
} baro_interface_t;

struct baro_sample_t {
  float altitude; // Unfiltered pressure altitude in meters above standard sea level.
  uint32_t timestamp_ms;
};

baro_types_t baro_init(void);
// Samples on a fixed cadence; reports the next service deadline in ticks.
// Each sample reads the status, then the data register block; every transfer
// returns portMAX_DELAY and resumes when i2c_notify_from_isr wakes the worker.
// A device-side conversion has no interrupt and uses a short status poll.
// portMAX_DELAY when no barometer is present.
TickType_t baro_update(void);
// Flight consumes the latest complete IO sample, coalescing unread samples.
bool baro_take_sample(baro_sample_t &sample);
