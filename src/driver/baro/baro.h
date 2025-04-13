#pragma once

#include <stdbool.h>
#include <stdint.h>

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

extern uint8_t baro_buf[6];

baro_types_t baro_init();
bool baro_update();