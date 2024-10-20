#include "vtx.h"

#include <math.h>
#include <string.h>

#include "core/debug.h"
#include "core/profile.h"
#include "driver/serial.h"
#include "driver/vtx.h"
#include "util/crc.h"
#include "util/util.h"

#ifdef USE_VTX

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

typedef enum {
  PARSER_IDLE,
  PARSER_READ_MAGIC_1,
  PARSER_READ_MAGIC_2,
  PARSER_READ_CMD,
  PARSER_READ_LENGTH,
  PARSER_READ_PAYLOAD,
  PARSER_READ_CRC,
} smart_audio_parser_state_t;

#define SMART_AUDIO_DETECT_TRIES 30

#define SMART_AUDIO_BAUDRATE_MIN 4650
#define SMART_AUDIO_BAUDRATE_MAX 5050

#define SA_HEADER_SIZE 5
#define SA_MAGIC_1 0xaa
#define SA_MAGIC_2 0x55

static uint8_t smart_audio_version = 0;
static uint32_t baud_rate = 4800;
static uint32_t packets_sent = 0;
static uint32_t packets_recv = 0;

extern vtx_settings_t vtx_actual;
extern uint8_t vtx_connect_tries;

static bool smart_audio_needs_update = false;
static smart_audio_parser_state_t parser_state = PARSER_IDLE;
static const char default_power_level_labels[4][VTX_POWER_LABEL_LEN] = {
    "25   ",
    "200  ",
    "500  ",
    "800  ",
};
static const uint8_t default_dac_power_levels[4] = {
    7,
    16,
    25,
    40,
};

static uint32_t smart_audio_dbi_to_mw(uint16_t dbi) {
  uint16_t mw = (uint16_t)powf(10.0f, dbi / 10.0f);
  if (dbi > 14) {
    // For powers greater than 25mW round up to a multiple of 50 to match expectations
    mw = 50 * ((mw + 25) / 50);
  }
  return mw;
}

static char *i2a(char *ptr, uint32_t val) {
  const uint32_t div = val / 10;
  if (div > 0)
    ptr = i2a(ptr, div);
  *ptr = '0' + (val % 10);
  return ptr + 1;
}

static void smart_audio_write_mw(char *buf, uint32_t val) {
  char *ptr = i2a(buf, val);
  while (ptr != (buf + VTX_POWER_LABEL_LEN))
    *ptr++ = ' ';
}

static void smart_audio_init() {
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
    smart_audio_init();
  }

  last_percent = current_percent;
  packets_sent = 0;
  packets_recv = 0;
}

static void smart_audio_send_payload(uint8_t cmd, const uint8_t *payload, const uint32_t size) {
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
  if (parser_state == PARSER_IDLE) {
    parser_state = PARSER_READ_MAGIC_1;
  }
  packets_sent++;
}

static void smart_audio_parse_packet(vtx_settings_t *actual, uint8_t cmd, uint8_t *payload, uint32_t length) {
  switch (cmd) {
  case SA_CMD_GET_SETTINGS:
  case SA_CMD_GET_SETTINGS_V2:
  case SA_CMD_GET_SETTINGS_V21: {
    smart_audio_version = (cmd == SA_CMD_GET_SETTINGS ? 1 : (cmd == SA_CMD_GET_SETTINGS_V2 ? 2 : 3));

    const uint8_t channel = payload[0];
    uint8_t power = payload[1];
    const uint8_t mode = payload[2];
    const uint16_t frequency = (uint16_t)(((uint16_t)payload[3] << 8) | payload[4]);

    const uint8_t channel_index = mode & SA_MODE_FREQUENCY ? vtx_find_frequency_index(frequency) : channel;
    actual->band = channel_index / VTX_CHANNEL_MAX;
    actual->channel = channel_index % VTX_CHANNEL_MAX;

    if (cmd == SA_CMD_GET_SETTINGS_V21) {
      power = payload[5];

      // SmartAudio seems to report buf[8] + 1 power levels, but one of them is zero.
      // zero is indeed a valid power level to set the vtx to, but it activates pit mode.
      // crucially, after sending 0 dbm, the vtx does NOT report its power level to be 0 dbm.
      // instead, it reports whatever value was set previously and it reports to be in pit mode.
      actual->power_table.levels = min(payload[6], VTX_POWER_LEVEL_MAX);
      for (uint8_t i = 0; i < actual->power_table.levels; i++) {
        actual->power_table.values[i] = payload[7 + i + 1]; //+ 1 to skip the first power level, as mentioned above
      }
    } else {
      actual->power_table.levels = 4;
      for (uint8_t i = 0; i < actual->power_table.levels; i++) {
        actual->power_table.values[i] = default_dac_power_levels[i];
      }
    }

    if (smart_audio_version >= 3) {
      for (uint32_t i = 0; i < actual->power_table.levels; i++) {
        smart_audio_write_mw(actual->power_table.labels[i], smart_audio_dbi_to_mw(actual->power_table.values[i]));
      }
      actual->pit_mode = (mode & SA_MODE_PIT) ? VTX_PIT_MODE_ON : VTX_PIT_MODE_OFF;
    } else {
      memcpy(actual->power_table.labels, default_power_level_labels, sizeof(default_power_level_labels));
      actual->pit_mode = VTX_PIT_MODE_NO_SUPPORT;
    }

    if (smart_audio_version == 2) {
      actual->power_level = min(power, VTX_POWER_LEVEL_MAX - 1);
    } else {
      actual->power_level = vtx_power_level_index(&actual->power_table, power);
    }

    if (vtx_settings.detected != VTX_PROTOCOL_SMART_AUDIO) {
      if (vtx_settings.magic != VTX_SETTINGS_MAGIC) {
        vtx_set(actual);
      }

      vtx_settings.detected = VTX_PROTOCOL_SMART_AUDIO;
      vtx_connect_tries = 0;
    }
    break;
  }
  case SA_CMD_SET_FREQUENCY: {
    const uint8_t channel_index = vtx_find_frequency_index((uint16_t)(((uint16_t)payload[0] << 8) | payload[1]));
    actual->band = channel_index / VTX_CHANNEL_MAX;
    actual->channel = channel_index % VTX_CHANNEL_MAX;
    break;
  }

  case SA_CMD_SET_CHANNEL: {
    const uint8_t channel_index = payload[0];
    actual->band = channel_index / VTX_CHANNEL_MAX;
    actual->channel = channel_index % VTX_CHANNEL_MAX;
    break;
  }

  case SA_CMD_SET_POWER: {
    const uint8_t power = payload[0];
    if (smart_audio_version == 2) {
      actual->power_level = min(power, VTX_POWER_LEVEL_MAX - 1);
    } else {
      actual->power_level = vtx_power_level_index(&actual->power_table, power);
    }
    break;
  }
  case SA_CMD_SET_MODE: {
    //  const uint8_t mode = payload[0];
    break;
  }
  default:
    break;
  }

  packets_recv++;
}

static vtx_detect_status_t smart_audio_update(vtx_settings_t *actual) {
  if (!serial_vtx_is_ready()) {
    return VTX_DETECT_WAIT;
  }
  if (vtx_connect_tries > SMART_AUDIO_DETECT_TRIES) {
    return VTX_DETECT_ERROR;
  }

  static uint8_t length = 0;

  while (parser_state > PARSER_IDLE) {
    if ((time_millis() - vtx_last_valid_read) > 500) {
      parser_state = PARSER_IDLE;
      vtx_connect_tries++;
      return VTX_DETECT_WAIT;
    }

    uint8_t data = 0;
    if (serial_vtx_read_byte(&data) == 0) {
      return VTX_DETECT_WAIT;
    }

    switch (parser_state) {
    case PARSER_IDLE:
      break;

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
      if (++vtx_payload_offset == length) {
        parser_state = PARSER_READ_CRC;
      }
      break;

    case PARSER_READ_CRC: {
      parser_state = PARSER_IDLE;
      const uint8_t theirs = crc8_dvb_s2_data(0, vtx_payload, length + 2);
      if (data == theirs) {
        smart_audio_parse_packet(actual, vtx_payload[0], vtx_payload + 2, length - 2);
      }
      break;
    }
    }
  }

  if (vtx_settings.detected != VTX_PROTOCOL_SMART_AUDIO && vtx_connect_tries <= SMART_AUDIO_DETECT_TRIES) {
    // no smart audio detected, try again
    smart_audio_send_payload(SA_CMD_GET_SETTINGS, NULL, 0);
    vtx_connect_tries++;
    return VTX_DETECT_WAIT;
  }

  if (smart_audio_needs_update) {
    smart_audio_send_payload(SA_CMD_GET_SETTINGS, NULL, 0);
    smart_audio_needs_update = false;
    return VTX_DETECT_WAIT;
  }

  return VTX_DETECT_SUCCESS;
}

static void smart_audio_set_frequency(vtx_band_t band, vtx_channel_t channel) {
#ifdef SA_USE_SET_FREQUENCY
  if (smart_audio_settings.mode & SA_MODE_FREQUENCY) {
    const uint16_t frequency = frequency_table[band][channel];
    const uint8_t payload[2] = {
        (frequency >> 8) & 0xFF,
        frequency & 0xFF,
    };
    smart_audio_send_payload(SA_CMD_SET_FREQUENCY, payload, 2);
  } else
#endif
  {
    // smart_audio_settings.mode &= ~SA_MODE_FREQUENCY;

    const uint8_t index = band * VTX_CHANNEL_MAX + channel;
    const uint8_t payload[1] = {index};
    smart_audio_send_payload(SA_CMD_SET_CHANNEL, payload, 1);
    smart_audio_needs_update = true;
  }
}

static void smart_audio_set_power_level(vtx_power_level_t power) {
  uint8_t level = power;
  if (smart_audio_version != 2) {
    level = vtx_actual.power_table.values[power];
  }
  if (smart_audio_version == 3) {
    level |= 0x80;
  }
  smart_audio_send_payload(SA_CMD_SET_POWER, &level, 1);
  smart_audio_needs_update = true;
}

static void smart_audio_set_pit_mode(vtx_pit_mode_t pit_mode) {
  if (smart_audio_version >= 3) {
    uint8_t mode = (1 << 3); // unlock

    if (pit_mode == VTX_PIT_MODE_OFF) {
      mode |= (1 << 2);
    } else if (pit_mode == VTX_PIT_MODE_ON) {
      // out-range was dropped for VTXes with SA >= v2.1
      mode |= (1 << 0);
    }

    smart_audio_send_payload(SA_CMD_SET_MODE, &mode, 1);
    smart_audio_needs_update = true;
  } else {
    vtx_settings.pit_mode = VTX_PIT_MODE_NO_SUPPORT;
  }
}

const vtx_device_t smart_audio_vtx_device = {
    .init = smart_audio_init,
    .update = smart_audio_update,
    .set_frequency = smart_audio_set_frequency,
    .set_power_level = smart_audio_set_power_level,
    .set_pit_mode = smart_audio_set_pit_mode,
};

#endif