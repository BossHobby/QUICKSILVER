#include "driver/vtx/sa.h"

#include <stddef.h>

#include "core/debug.h"
#include "core/profile.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "driver/vtx/vtx.h"
#include "io/usb_configurator.h"
#include "io/vtx.h"
#include "util/crc.h"
#include "util/ring_buffer.h"
#include "util/util.h"

#ifdef USE_VTX

#define SMART_AUDIO_BAUDRATE_MIN 4650
#define SMART_AUDIO_BAUDRATE_MAX 5050

#define SA_HEADER_SIZE 5
#define SA_MAGIC_1 0xaa
#define SA_MAGIC_2 0x55

typedef enum {
  PARSER_IDLE,
  PARSER_READ_MAGIC_1,
  PARSER_READ_MAGIC_2,
  PARSER_READ_CMD,
  PARSER_READ_LENGTH,
  PARSER_READ_PAYLOAD,
  PARSER_READ_CRC,
} smart_audio_parser_state_t;

smart_audio_settings_t smart_audio_settings;

static uint32_t baud_rate = 4800;
static uint32_t packets_sent = 0;
static uint32_t packets_recv = 0;

static smart_audio_parser_state_t parser_state = PARSER_IDLE;

extern uint32_t vtx_last_valid_read;
extern uint32_t vtx_last_request;

extern uint8_t vtx_payload[32];
extern uint8_t vtx_payload_offset;

const uint8_t default_dac_power_levels[VTX_POWER_LEVEL_MAX] = {
    7,
    16,
    25,
    40,
    40,
};

static void serial_smart_audio_reconfigure() {
  serial_vtx_wait_for_ready();

  const target_serial_port_t *dev = serial_get_dev(profile.serial.smart_audio);
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
  config.half_duplex_pp = true;

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
  if (parser_state == PARSER_IDLE) {
    return VTX_IDLE;
  }
  if ((time_millis() - vtx_last_valid_read) > 500) {
    return VTX_ERROR;
  }

  static uint8_t length = 0;

  while (true) {
    uint8_t data = 0;
    if (serial_vtx_read_byte(&data) == 0) {
      return VTX_WAIT;
    }
    quic_debugf("VTX parser_state: %d 0x%x", parser_state, data);
    switch (parser_state) {
    case PARSER_IDLE:
      return VTX_IDLE;

    case PARSER_READ_MAGIC_1:
      if (data == SA_MAGIC_1) {
        parser_state = PARSER_READ_MAGIC_2;
      }
      break;

    case PARSER_READ_MAGIC_2:
      if (data != SA_MAGIC_2) {
        parser_state = PARSER_READ_MAGIC_1;
      } else {
        parser_state = PARSER_READ_CMD;
      }
      break;

    case PARSER_READ_CMD:
      vtx_payload[0] = data;
      parser_state = PARSER_READ_LENGTH;
      break;

    case PARSER_READ_LENGTH:
      length = vtx_payload[1] = data;
      vtx_payload_offset = 0;
      parser_state = length ? PARSER_READ_PAYLOAD : PARSER_READ_CRC;
      break;

    case PARSER_READ_PAYLOAD:
      vtx_payload[vtx_payload_offset + 2] = data;
      if (++vtx_payload_offset == (length - 1)) {
        parser_state = PARSER_READ_CRC;
      }
      break;

    case PARSER_READ_CRC:
      parser_state = PARSER_IDLE;
      if (data != crc8_dvb_s2_data(0, vtx_payload, length + 1)) {
        return VTX_ERROR;
      }
      if (!serial_smart_audio_parse_packet(vtx_payload[0], vtx_payload + 2, length - 1)) {
        return VTX_ERROR;
      }
      return VTX_SUCCESS;
    }
  }

  return VTX_ERROR;
}

void serial_smart_audio_send_payload(uint8_t cmd, const uint8_t *payload, const uint32_t size) {
  if (!serial_vtx_is_ready()) {
    return;
  }

#ifdef USE_AKK_SA_WORKAROUND
#define EXTRA_DUMMY_BYTES 1
#else
#define EXTRA_DUMMY_BYTES 0
#endif

  const uint32_t len = size + 1 + SA_HEADER_SIZE + EXTRA_DUMMY_BYTES;
  uint8_t vtx_frame[len];

  vtx_frame[0] = 0x00;
  vtx_frame[1] = 0xAA;
  vtx_frame[2] = 0x55;
  vtx_frame[3] = (cmd << 1) | 0x1;
  vtx_frame[4] = size;
  for (uint8_t i = 0; i < size; i++) {
    vtx_frame[i + SA_HEADER_SIZE] = payload[i];
  }
  vtx_frame[size + SA_HEADER_SIZE] = crc8_dvb_s2_data(0, vtx_frame + 1, len - 2 - EXTRA_DUMMY_BYTES);
#ifdef USE_AKK_SA_WORKAROUND
  vtx_frame[size + 1 + SA_HEADER_SIZE] = 0x00;
#endif

  smart_audio_auto_baud();

  serial_vtx_send_data(vtx_frame, len);
  parser_state = PARSER_READ_MAGIC_1;
  packets_sent++;
}

#endif