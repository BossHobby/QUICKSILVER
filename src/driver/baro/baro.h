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
  bool (*get_pressure)(float *);
} baro_interface_t;

struct baro_sample_t {
  float altitude; // Unfiltered pressure altitude in meters above standard sea level.
  uint32_t timestamp_ms;
};

extern uint8_t baro_buf[6];

baro_types_t baro_init(void);
// Samples on a fixed cadence; reports the next service deadline in ticks.
// In-flight I2C transfers wake the worker via i2c_notify_from_isr; a
// device-side conversion has no interrupt and uses a short status poll.
// portMAX_DELAY when no barometer is present.
TickType_t baro_update(void);
// Flight consumes the latest complete IO sample, coalescing unread samples.
bool baro_take_sample(baro_sample_t &sample);
