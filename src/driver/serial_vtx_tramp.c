#include "driver/serial_vtx_tramp.h"

#include <string.h>

#include "core/debug.h"
#include "core/profile.h"
#include "driver/serial.h"
#include "driver/serial_vtx.h"
#include "driver/time.h"
#include "util/ring_buffer.h"

typedef enum {
  PARSER_IDLE,
  PARSER_READ_MAGIC,
  PARSER_READ_PAYLOAD,
  PARSER_READ_CRC,
  PRASER_WAIT_FOR_READY
} tramp_parser_state_t;

tramp_settings_t tramp_settings;

static tramp_parser_state_t parser_state = PARSER_IDLE;

extern uint32_t vtx_last_valid_read;
extern uint32_t vtx_last_request;

static uint8_t crc8_data(const uint8_t *data) {
  uint8_t crc = 0;
  for (int i = 0; i < 13; i++) {
    crc += data[i];
  }
  return crc;
}

void serial_tramp_init() {
  serial_vtx_wait_for_ready();

  const target_serial_port_t *dev = &target.serial_ports[profile.serial.smart_audio];
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

static uint8_t tramp_parse_packet(uint8_t *payload) {
  switch (payload[0]) {
  case 'r':
    tramp_settings.freq_min = payload[1] | (payload[2] << 8);
    tramp_settings.freq_max = payload[3] | (payload[4] << 8);
    tramp_settings.power_max = payload[5] | (payload[6] << 8);
    break;

  case 'v':
    tramp_settings.frequency = payload[1] | (payload[2] << 8);
    tramp_settings.power = payload[3] | (payload[4] << 8);
    tramp_settings.control_mode = payload[5];
    tramp_settings.pit_mode = payload[6];
    tramp_settings.current_power = payload[7] | (payload[8] << 8);
    break;

  case 's':
    tramp_settings.temp = payload[5] | (payload[6] << 8);
    break;
  }

  return 1;
}

vtx_update_result_t serial_tramp_update() {
  if (!serial_vtx_is_ready()) {
    return VTX_WAIT;
  }
  if (parser_state > PARSER_IDLE && (time_millis() - vtx_last_valid_read) > 500) {
    return VTX_ERROR;
  }

  static uint8_t payload[32];
  static uint8_t payload_offset = 0;

tramp_do_more:
  switch (parser_state) {
  case PARSER_IDLE:
    return VTX_IDLE;

  case PARSER_READ_MAGIC: {
    if (serial_vtx_read_byte(&payload[0]) == 0) {
      return VTX_WAIT;
    }

    if (payload[0] == 0x0F) {
      parser_state = PARSER_READ_PAYLOAD;
      payload_offset = 1;
    }
    goto tramp_do_more;
  }
  case PARSER_READ_PAYLOAD: {
    if (serial_vtx_read_byte(&payload[payload_offset]) == 0) {
      return VTX_WAIT;
    }

    payload_offset++;
    if (payload_offset >= 16) {
      parser_state = PARSER_READ_CRC;
    }
    goto tramp_do_more;
  }
  case PARSER_READ_CRC: {
    parser_state = PARSER_IDLE;

    const uint8_t crc = crc8_data(payload + 1);
    if (payload[14] != crc || payload[15] != 0) {
      return VTX_ERROR;
    }

    if (!tramp_parse_packet(payload + 1)) {
      return VTX_ERROR;
    }

    return VTX_SUCCESS;
  }
  case PRASER_WAIT_FOR_READY: {
    // dummy state for non querry packets
    parser_state = PARSER_IDLE;
    return VTX_SUCCESS;
  }
  }

  return VTX_ERROR;
}

void serial_tramp_send_payload(uint8_t cmd, const uint16_t payload) {
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