#include "vtx.h"

#include <string.h>

#include "core/profile.h"
#include "driver/serial.h"
#include "driver/vtx.h"

#ifdef USE_VTX

typedef enum {
  PARSER_IDLE,
  PARSER_READ_MAGIC,
  PARSER_READ_PAYLOAD,
  PARSER_READ_CRC,
  PRASER_WAIT_FOR_READY
} tramp_parser_state_t;

#define TRAMP_DETECT_TRIES 5

extern uint8_t vtx_connect_tries;
extern const uint16_t frequency_table[VTX_BAND_MAX][VTX_CHANNEL_MAX];

static tramp_parser_state_t parser_state = PARSER_IDLE;
static const uint16_t tramp_power_level_values[VTX_POWER_LEVEL_MAX] = {
    25,
    100,
    200,
    300,
    400,
    0,
    0,
    0,
};
static const char tramp_power_level_labels[VTX_POWER_LEVEL_MAX][VTX_POWER_LABEL_LEN] = {
    "25   ",
    "100  ",
    "200  ",
    "300  ",
    "400  ",
    "     ",
    "     ",
    "     ",
};

static uint8_t crc8_data(const uint8_t *data) {
  uint8_t crc = 0;
  for (int i = 0; i < 13; i++) {
    crc += data[i];
  }
  return crc;
}

static void tramp_init() {
  serial_vtx_wait_for_ready();

  const target_serial_port_t *dev = serial_get_dev(profile.serial.smart_audio);
  if (!target_serial_port_valid(dev)) {
    return;
  }

  serial_port_config_t config;
  config.port = profile.serial.smart_audio;
  config.baudrate = 9600;
  config.direction = SERIAL_DIR_TX_RX;
  config.stop_bits = SERIAL_STOP_BITS_1;
  config.invert = false;
  config.half_duplex = true;
  config.half_duplex_pp = false;

  serial_init(&serial_vtx, config);
}

static bool tramp_is_query(uint8_t cmd) {
  switch (cmd) {
  case 'r':
  case 'v':
  case 's':
    return true;
  }
  return false;
}

static void tramp_parse_packet(vtx_settings_t *actual, uint8_t *payload) {
  switch (payload[0]) {
  case 'r':
    // freq_min = payload[1] | (payload[2] << 8);
    // freq_max = payload[3] | (payload[4] << 8);
    // power_max = payload[5] | (payload[6] << 8);
    break;

  case 'v': {
    const uint16_t frequency = payload[1] | (payload[2] << 8);
    const uint16_t power = payload[3] | (payload[4] << 8);
    // const uint8_t control_mode = payload[5];
    const uint8_t pit_mode = payload[6];
    // const uint16_t current_power = payload[7] | (payload[8] << 8);

    int8_t channel_index = vtx_find_frequency_index(frequency);
    if (channel_index >= 0) {
      actual->band = channel_index / VTX_CHANNEL_MAX;
      actual->channel = channel_index % VTX_CHANNEL_MAX;
    }

    actual->power_table.levels = VTX_POWER_LEVEL_MAX;
    memcpy(actual->power_table.values, tramp_power_level_values, sizeof(tramp_power_level_values));
    memcpy(actual->power_table.labels, tramp_power_level_labels, sizeof(tramp_power_level_labels));

    actual->power_level = vtx_power_level_index(&actual->power_table, power);
    actual->pit_mode = pit_mode;

    if (vtx_settings.detected != VTX_PROTOCOL_TRAMP) {
      if (vtx_settings.magic != VTX_SETTINGS_MAGIC) {
        vtx_set(actual);
      }

      vtx_settings.detected = VTX_PROTOCOL_TRAMP;
      vtx_connect_tries = 0;
    }
    break;
  }
  case 's':
    // temp = payload[5] | (payload[6] << 8);
    break;
  }
}

static void tramp_send_payload(uint8_t cmd, const uint16_t payload) {
  if (!serial_vtx_is_ready()) {
    return;
  }

  uint8_t vtx_frame[16];
  memset(vtx_frame, 0, 16);

  vtx_frame[0] = 0x0F;
  vtx_frame[1] = cmd;
  vtx_frame[2] = payload & 0xff;
  vtx_frame[3] = (payload >> 8) & 0xff;
  vtx_frame[14] = crc8_data(vtx_frame + 1);

  serial_vtx_send_data(vtx_frame, 16);

  if (tramp_is_query(vtx_frame[1])) {
    parser_state = PARSER_READ_MAGIC;
  } else {
    parser_state = PRASER_WAIT_FOR_READY;
  }
}

static vtx_detect_status_t tramp_update(vtx_settings_t *actual) {
  if (!serial_vtx_is_ready()) {
    return VTX_DETECT_WAIT;
  }
  if (vtx_connect_tries > TRAMP_DETECT_TRIES) {
    return VTX_DETECT_ERROR;
  }

  while (parser_state > PARSER_IDLE) {
    if ((time_millis() - vtx_last_valid_read) > 500) {
      parser_state = PARSER_IDLE;
      vtx_connect_tries++;
      return VTX_DETECT_WAIT;
    }

    switch (parser_state) {
    case PARSER_IDLE:
      break;

    case PARSER_READ_MAGIC: {
      if (serial_vtx_read_byte(&vtx_payload[0]) == 0) {
        return VTX_DETECT_WAIT;
      }

      if (vtx_payload[0] == 0x0F) {
        parser_state = PARSER_READ_PAYLOAD;
        vtx_payload_offset = 1;
      }
      break;
    }
    case PARSER_READ_PAYLOAD: {
      if (serial_vtx_read_byte(&vtx_payload[vtx_payload_offset]) == 0) {
        return VTX_DETECT_WAIT;
      }

      vtx_payload_offset++;
      if (vtx_payload_offset >= 16) {
        parser_state = PARSER_READ_CRC;
      }
      break;
    }
    case PARSER_READ_CRC: {
      parser_state = PARSER_IDLE;

      const uint8_t crc = crc8_data(vtx_payload + 1);
      if (vtx_payload[14] == crc && vtx_payload[15] == 0) {
        tramp_parse_packet(actual, vtx_payload + 1);
      }
      break;
    }
    case PRASER_WAIT_FOR_READY: {
      // dummy state for non querry packets
      parser_state = PARSER_IDLE;
      break;
    }
    }
  }

  if (vtx_settings.detected != VTX_PROTOCOL_TRAMP && vtx_connect_tries <= TRAMP_DETECT_TRIES) {
    // no tramp detected, try again
    tramp_send_payload('v', 0);
    vtx_connect_tries++;
    return VTX_DETECT_WAIT;
  }

  return VTX_DETECT_SUCCESS;
}

static void tramp_set_frequency(vtx_band_t band, vtx_channel_t channel) {
  const uint16_t frequency = frequency_table[band][channel];
  tramp_send_payload('F', frequency);
}

static void tramp_set_power_level(vtx_power_level_t power) {
  tramp_send_payload('P', tramp_power_level_values[power]);
}

static void tramp_set_pit_mode(vtx_pit_mode_t pit_mode) {
  tramp_send_payload('I', pit_mode == VTX_PIT_MODE_ON ? 0 : 1);
}

const vtx_device_t tramp_vtx_device = {
    .init = tramp_init,
    .update = tramp_update,
    .set_frequency = tramp_set_frequency,
    .set_power_level = tramp_set_power_level,
    .set_pit_mode = tramp_set_pit_mode,
};

#endif