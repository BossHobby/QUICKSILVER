#include "driver/serial_vtx_sa.h"

#include <stddef.h>

#include "core/debug.h"
#include "core/profile.h"
#include "driver/serial.h"
#include "driver/serial_vtx.h"
#include "driver/time.h"
#include "io/usb_configurator.h"
#include "io/vtx.h"
#include "util/crc.h"
#include "util/ring_buffer.h"
#include "util/util.h"

#define SMART_AUDIO_BAUDRATE_MIN 4650
#define SMART_AUDIO_BAUDRATE_MAX 5050

#define SA_HEADER_SIZE 5
#define SA_MAGIC_1 0xaa
#define SA_MAGIC_2 0x55

#define USART usart_port_defs[serial_smart_audio_port]

typedef enum {
  PARSER_IDLE,
  PARSER_READ_MAGIC_1,
  PARSER_READ_MAGIC_2,
  PARSER_READ_CMD,
  PARSER_READ_PAYLOAD,
  PARSER_READ_LENGTH,
  PARSER_READ_CRC,
} smart_audio_parser_state_t;

smart_audio_settings_t smart_audio_settings;

static uint32_t baud_rate = 4800;
static uint32_t packets_sent = 0;
static uint32_t packets_recv = 0;

static smart_audio_parser_state_t parser_state = PARSER_IDLE;

extern uint32_t vtx_last_valid_read;
extern uint32_t vtx_last_request;

const uint8_t default_dac_power_levels[VTX_POWER_LEVEL_MAX] = {
    7,
    16,
    25,
    40,
    40,
};

static void serial_smart_audio_reconfigure() {
  serial_vtx_wait_for_ready();

  const target_serial_port_t *dev = &target.serial_ports[profile.serial.smart_audio];
  if (!target_serial_port_valid(dev)) {
    return;
  }

  serial_port_config_t config;
  config.port = profile.serial.smart_audio;
  config.baudrate = baud_rate;
  config.direction = SERIAL_DIR_TX_RX;
  config.stop_bits = SERIAL_STOP_BITS_2;
  config.invert = false;
  config.half_duplex = true;

  serial_init(&serial_vtx, config);
}

static void smart_audio_auto_baud() {
  static uint8_t last_percent = 0;

  // move quickly while we have not yet found a working baud rate
  const uint8_t current_percent = ((packets_recv * 100) / packets_sent);
  if (packets_sent < (last_percent == 0 ? 3 : 10)) {
    last_percent = current_percent;
    return;
  }

  if (current_percent < 70) {
    static int8_t direction = 1;

    // if the percentage degraded, switch it up
    if (last_percent > current_percent) {
      direction = direction == 1 ? -1 : 1;
    }

    if ((direction == 1) && (baud_rate == SMART_AUDIO_BAUDRATE_MAX)) {
      direction = -1;
    } else if ((direction == -1 && baud_rate == SMART_AUDIO_BAUDRATE_MIN)) {
      direction = 1;
    }

    baud_rate += direction * 50;
    quic_debugf("SMART_AUDIO: auto baud %d (%d) change %d vs %d", baud_rate, direction * 50, last_percent, current_percent);
    serial_smart_audio_reconfigure();
  }

  last_percent = current_percent;
  packets_sent = 0;
  packets_recv = 0;
}

static uint8_t serial_smart_audio_read_byte_crc(uint8_t *crc, uint8_t *data) {
  if (serial_vtx_read_byte(data) == 0) {
    return 0;
  }
  *crc = crc8_dvb_s2_calc(*crc, *data);
  return 1;
}

static uint8_t serial_smart_audio_parse_packet(uint8_t cmd, uint8_t *payload, uint32_t length) {
  switch (cmd) {
  case SA_CMD_GET_SETTINGS:
  case SA_CMD_GET_SETTINGS_V2:
  case SA_CMD_GET_SETTINGS_V21:
    smart_audio_settings.version = (cmd == SA_CMD_GET_SETTINGS ? 1 : (cmd == SA_CMD_GET_SETTINGS_V2 ? 2 : 3));
    smart_audio_settings.channel = payload[0];
    smart_audio_settings.power = payload[1];
    smart_audio_settings.mode = payload[2];
    smart_audio_settings.frequency = (uint16_t)(((uint16_t)payload[3] << 8) | payload[4]);

    if (cmd == SA_CMD_GET_SETTINGS_V21) {
      smart_audio_settings.power = payload[5];

      const uint8_t count = max(payload[6], VTX_POWER_LEVEL_MAX);

      for (uint8_t i = 0; i < count; i++) {
        smart_audio_settings.dac_power_levels[i] = payload[7 + i];
      }
    } else {
      for (uint8_t i = 0; i < VTX_POWER_LEVEL_MAX; i++) {
        smart_audio_settings.dac_power_levels[i] = default_dac_power_levels[i];
      }
    }
    break;

  case SA_CMD_SET_FREQUENCY:
    smart_audio_settings.frequency = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
    break;

  case SA_CMD_SET_CHANNEL:
    smart_audio_settings.channel = payload[0];
    break;

  case SA_CMD_SET_POWER: {
    smart_audio_settings.power = payload[0];
    break;
  }
  case SA_CMD_SET_MODE: {
    const uint8_t mode = payload[0];

    // in-range pitmode
    smart_audio_settings.mode |= ((mode >> 0) & 0x1) << 2;

    // out-range pitmode
    smart_audio_settings.mode |= ((mode >> 1) & 0x1) << 3;

    // pit mode runnig
    smart_audio_settings.mode |= ((mode >> 2) & 0x1) << 1;

    // locked bit
    smart_audio_settings.mode |= ((mode >> 3) & 0x1) << 4;
    break;
  }
  default:
    quic_debugf("SMART_AUDIO: invalid cmd %d (%d)", cmd, length);
    return 0;
  }

  packets_recv++;
  return 1;
}

void serial_smart_audio_init() {
  serial_smart_audio_reconfigure();
}

vtx_update_result_t serial_smart_audio_update() {
  if (!serial_vtx_is_ready()) {
    return VTX_WAIT;
  }
  if (parser_state > PARSER_IDLE && (time_millis() - vtx_last_valid_read) > 500) {
    return VTX_ERROR;
  }

  static uint8_t payload_offset = 0;

  static uint8_t crc = 0;
  static uint8_t cmd = 0;
  static uint8_t length = 0;
  static uint8_t payload[32];

sa_do_more:
  switch (parser_state) {
  case PARSER_IDLE:
    return VTX_IDLE;

  case PARSER_READ_MAGIC_1: {
    uint8_t data = 0;
    if (serial_vtx_read_byte(&data) == 0) {
      return VTX_WAIT;
    }

    if (data == 0x0) {
      // skip leading zero
      goto sa_do_more;
    }

    if (data == SA_MAGIC_1) {
      crc = 0;
      cmd = 0;
      length = 0;
      payload_offset = 0;
      parser_state = PARSER_READ_MAGIC_2;
    }
    goto sa_do_more;
  }
  case PARSER_READ_MAGIC_2: {
    uint8_t data = 0;
    if (serial_vtx_read_byte(&data) == 0) {
      return VTX_WAIT;
    }

    if (data != SA_MAGIC_2) {
      parser_state = PARSER_READ_MAGIC_1;
    } else {
      parser_state = PARSER_READ_CMD;
    }
    goto sa_do_more;
  }
  case PARSER_READ_CMD: {
    if (serial_smart_audio_read_byte_crc(&crc, &cmd) == 0) {
      return VTX_WAIT;
    }
    parser_state = PARSER_READ_LENGTH;
    goto sa_do_more;
  }
  case PARSER_READ_LENGTH: {
    if (serial_smart_audio_read_byte_crc(&crc, &length) == 0) {
      return VTX_WAIT;
    }
    parser_state = length ? PARSER_READ_PAYLOAD : PARSER_READ_CRC;
    goto sa_do_more;
  }
  case PARSER_READ_PAYLOAD: {
    uint8_t data = 0;
    if (serial_smart_audio_read_byte_crc(&crc, &data) == 0) {
      return VTX_WAIT;
    }

    payload[payload_offset++] = data;
    if (payload_offset >= length) {
      parser_state = PARSER_READ_CRC;
    }
    goto sa_do_more;
  }
  case PARSER_READ_CRC: {
    uint8_t data = 0;
    if (serial_vtx_read_byte(&data) == 0) {
      return VTX_WAIT;
    }

    parser_state = PARSER_IDLE;
    if (data == crc && serial_smart_audio_parse_packet(cmd, payload, length)) {
      return VTX_SUCCESS;
    }
    return VTX_ERROR;
  }
  }

  return VTX_ERROR;
}

void serial_smart_audio_send_payload(uint8_t cmd, const uint8_t *payload, const uint32_t size) {
  if (!serial_vtx_is_ready()) {
    return;
  }

  const uint32_t len = size + 2 + SA_HEADER_SIZE;
  uint8_t vtx_frame[len];

  vtx_frame[0] = 0x00;
  vtx_frame[1] = 0xAA;
  vtx_frame[2] = 0x55;
  vtx_frame[3] = (cmd << 1) | 0x1;
  vtx_frame[4] = size;
  for (uint8_t i = 0; i < size; i++) {
    vtx_frame[i + SA_HEADER_SIZE] = payload[i];
  }
  vtx_frame[size + SA_HEADER_SIZE] = crc8_dvb_s2_data(0, vtx_frame + 1, len - 3);
  vtx_frame[size + 1 + SA_HEADER_SIZE] = 0x00;

  smart_audio_auto_baud();

  serial_vtx_send_data(vtx_frame, len);
  parser_state = PARSER_READ_MAGIC_1;
  packets_sent++;
}
