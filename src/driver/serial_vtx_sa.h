#pragma once

#include <stdint.h>

#include "driver/serial_vtx.h"

typedef enum {
  SA_CMD_GET_SETTINGS = 0x01,
  SA_CMD_SET_POWER = 0x02,
  SA_CMD_SET_CHANNEL = 0x03,
  SA_CMD_SET_FREQUENCY = 0x04,
  SA_CMD_SET_MODE = 0x05,
  SA_CMD_GET_SETTINGS_V2 = 0x9,
  SA_CMD_GET_SETTINGS_V21 = 0x11,
} smart_audio_cmd_t;

typedef enum {
  SA_MODE_FREQUENCY = 1 << 0,
  SA_MODE_PIT = 1 << 1,
  SA_MODE_IN_RANGE_PIT = 1 << 2,
  SA_MODE_OUT_RANGE_PIT = 1 << 3,
  SA_MODE_UNLOCKED = 1 << 4,
} smart_mode_t;

typedef struct {
  uint8_t version;
  uint8_t channel;
  uint8_t power;
  uint8_t mode;
  uint16_t frequency;
  uint16_t dac_power_levels[5];
} smart_audio_settings_t;

void serial_smart_audio_init();
vtx_update_result_t serial_smart_audio_update();
void serial_smart_audio_send_payload(uint8_t cmd, const uint8_t *payload, const uint32_t size);