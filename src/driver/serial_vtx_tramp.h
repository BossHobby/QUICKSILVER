#pragma once

#include <stdint.h>

#include "driver/serial_vtx.h"

typedef struct {
  uint16_t freq_min;
  uint16_t freq_max;

  uint16_t power_max;
  uint16_t current_power;

  uint16_t temp;

  uint16_t frequency;
  uint16_t power;
  uint8_t pit_mode;
  uint8_t control_mode;
} tramp_settings_t;

void serial_tramp_init();
vtx_update_result_t serial_tramp_update();
void serial_tramp_send_payload(uint8_t cmd, const uint16_t payload);