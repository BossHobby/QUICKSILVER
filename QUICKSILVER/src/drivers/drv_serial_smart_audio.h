#pragma once

#include <stdint.h>

typedef enum {
  SA_CMD_GET_SETTINGS = 0x01,
  SA_CMD_SET_POWER = 0x02,
  SA_CMD_SET_CHANNEL = 0x03,
  SA_CMD_SET_FREQUENCY = 0x04,
  SA_CMD_SET_MODE = 0x05,
} smart_audio_cmd_t;

typedef struct {
  uint8_t version;
  uint8_t channel;
  uint8_t power;
  uint8_t mode;
  uint16_t frequency;
} smart_audio_settings_t;

void serial_smart_audio_init(void);
void serial_smart_audio_send_payload(uint8_t cmd, uint8_t *payload, uint32_t size);