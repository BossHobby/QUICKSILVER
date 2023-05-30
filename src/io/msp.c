#include "msp.h"

#include <stdbool.h>
#include <string.h>

#include "core/flash.h"
#include "core/reset.h"
#include "driver/serial.h"
#include "driver/serial_4way.h"
#include "flight/control.h"
#include "io/usb_configurator.h"
#include "quic.h"
#include "util/crc.h"
#include "util/util.h"
#include "vtx.h"

enum {
  MSP_REBOOT_FIRMWARE = 0,
  MSP_REBOOT_BOOTLOADER_ROM,
  MSP_REBOOT_MSC,
  MSP_REBOOT_MSC_UTC,
  MSP_REBOOT_BOOTLOADER_FLASH,
  MSP_REBOOT_COUNT,
};

extern uint8_t msp_vtx_detected;
extern vtx_settings_t msp_vtx_settings;
extern const uint16_t frequency_table[VTX_BAND_MAX][VTX_CHANNEL_MAX];
extern char msp_vtx_band_letters[VTX_BAND_MAX];
extern char msp_vtx_band_labels[VTX_BAND_MAX][8];
extern void msp_vtx_send_config_reply(msp_t *msp, msp_magic_t magic);

void msp_send_reply(msp_t *msp, msp_magic_t magic, uint16_t cmd, uint8_t *data, uint32_t len) {
  if (msp->send) {
    msp->send(magic, '>', cmd, data, len);
  }
}

static void msp_send_error(msp_t *msp, msp_magic_t magic, uint16_t cmd) {
  if (msp->send) {
    msp->send(magic, '!', cmd, NULL, 0);
  }
}

static void msp_quic_send(uint8_t *data, uint32_t len, void *priv) {
  msp_t *msp = (msp_t *)priv;
  msp_send_reply(msp, MSP1_MAGIC, MSP_RESERVE_1, data, len);
}

static void msp_write_uint16(uint8_t *data, uint16_t val) {
  data[0] = val >> 0;
  data[1] = val >> 8;
}

static void msp_write_uint32(uint8_t *data, uint32_t val) {
  data[0] = val >> 0;
  data[1] = val >> 8;
  data[2] = val >> 16;
  data[3] = val >> 24;
}

static void msp_process_serial_cmd(msp_t *msp, msp_magic_t magic, uint16_t cmd, uint8_t *payload, uint16_t size) {
  switch (cmd) {
  case MSP_API_VERSION: {
    uint8_t data[3] = {
        0,  // MSP_PROTOCOL_VERSION
        1,  // API_VERSION_MAJOR
        42, // API_VERSION_MINOR
    };
    msp_send_reply(msp, magic, cmd, data, 3);
    break;
  }
  case MSP_FC_VARIANT: {
    uint8_t data[4] = {'Q', 'U', 'I', 'C'};
    msp_send_reply(msp, magic, cmd, data, 4);
    break;
  }
  case MSP_FC_VERSION: {
    uint8_t data[3] = {
        0, // FC_VERSION_MAJOR
        1, // FC_VERSION_MINOR
        0, // FC_VERSION_PATCH_LEVEL
    };
    msp_send_reply(msp, magic, cmd, data, 3);
    break;
  }
  case MSP_BUILD_INFO: {
    uint8_t data[19] = MSP_BUILD_DATE_TIME;
    msp_send_reply(msp, magic, cmd, data, 19);
    break;
  }
  case MSP_BOARD_INFO: {
    uint8_t data[6] = {'Q', 'U', 'I', 'C', 0, 0};
    msp_send_reply(msp, magic, cmd, data, 6);
    break;
  }
  case MSP_UID: {
    uint8_t data[12] = {
        0x0,
        0x0,
        0x0,
        0x0,

        0xd,
        0xe,
        0xa,
        0xd,

        0xb,
        0xe,
        0xe,
        0xf,
    };
    msp_send_reply(msp, magic, cmd, data, 12);
    break;
  }
  case MSP_BATTERY_STATE: {
    const uint16_t current = state.ibat / 1000;
    uint8_t data[9] = {
        state.lipo_cell_count,                                  // battery detected
        0x0, 0x0,                                               // battery capacity
        (uint8_t)constrainf(state.vbat_filtered / 0.1, 0, 255), // battery voltage
        0x0, 0x0,                                               // battery drawn in mAh
        (current >> 8) & 0xFF, current & 0xFF,                  // battery current draw in A
        0x0                                                     // battery status
    };
    msp_send_reply(msp, magic, cmd, data, 9);
    break;
  }
  case MSP_FEATURE_CONFIG: {
    uint8_t data[4] = {
        0x0,
        0x0,
        0x0,
        0x0,
    };
    msp_send_reply(msp, magic, cmd, data, 4);
    break;
  }
  case MSP_MOTOR_CONFIG: {
    uint16_t data[3] = {
        1070, // min throttle
        2000, // max throttle
        1000, // min command
    };
    msp_send_reply(msp, magic, cmd, (uint8_t *)data, 3 * sizeof(uint16_t));
    break;
  }
  case MSP_MOTOR: {
    // we always have 4 motors, but blheli expects 8
    // these are pwm values
    uint16_t data[8] = {
        (uint16_t)mapf(motor_test.value[0], 0.0f, 1.0f, 1000.f, 2000.f),
        (uint16_t)mapf(motor_test.value[1], 0.0f, 1.0f, 1000.f, 2000.f),
        (uint16_t)mapf(motor_test.value[2], 0.0f, 1.0f, 1000.f, 2000.f),
        (uint16_t)mapf(motor_test.value[3], 0.0f, 1.0f, 1000.f, 2000.f),
        0,
        0,
        0,
        0,
    };
    msp_send_reply(msp, magic, cmd, (uint8_t *)data, 8 * sizeof(uint16_t));
    break;
  }
  case MSP_STATUS: {
    uint8_t data[22];
    memset(data, 0, 22);

    msp_write_uint16(data, state.cpu_load);
    msp_write_uint16(data + 2, 0); // i2c errors
    msp_write_uint16(data + 4, 0); // sensors

    // flight mode, only arm for now
    uint32_t flight_mode = 0;
    if (flags.arm_state) {
      flight_mode |= 0x1;
    }
    msp_write_uint32(data + 6, flight_mode);

    msp_send_reply(msp, magic, cmd, data, 22);
    break;
  }
  case MSP_RC: {
    uint16_t data[16];

    data[0] = mapf(state.rx_filtered.roll, -1.0f, 1.0f, 1000.f, 2000.f);
    data[1] = mapf(state.rx_filtered.pitch, -1.0f, 1.0f, 1000.f, 2000.f);
    data[2] = mapf(state.rx_filtered.yaw, -1.0f, 1.0f, 1000.f, 2000.f);
    data[3] = mapf(state.rx_filtered.throttle, 0.0f, 1.0f, 1000.f, 2000.f);

    for (uint32_t i = 0; i < AUX_CHANNEL_OFF; i++) {
      if (state.aux[i]) {
        data[i + 4] = 2000;
      } else {
        data[i + 4] = 1000;
      }
    }

    msp_send_reply(msp, magic, cmd, (uint8_t *)data, 32);
    break;
  }
  case MSP_SET_MOTOR: {
    uint16_t *values = (uint16_t *)(payload);
    for (uint8_t i = 0; i < 4; i++) {
      motor_test.value[i] = mapf(values[i], 1000.f, 2000.f, 0.0f, 1.0f);
    }
    motor_test.active = 1;

    msp_send_reply(msp, magic, cmd, NULL, 0);
    break;
  }
  case MSP_SET_PASSTHROUGH: {
    msp_passthrough_mode_t mode = MSP_PASSTHROUGH_ESC_4WAY;
    uint8_t arg = 0;

    if (size != 0) {
      mode = payload[0];
      arg = payload[1];
    }

    switch (mode) {
    case MSP_PASSTHROUGH_SERIAL_ID: {
      uint8_t data[1] = {1};
      msp_send_reply(msp, magic, cmd, data, 1);

      if (arg == serial_vtx.config.port) {
        if (vtx_settings.protocol == VTX_PROTOCOL_TRAMP) {
          usb_serial_passthrough(arg, 9600, 1, true);
        } else {
          usb_serial_passthrough(arg, 4800, 2, true);
        }
      }

      break;
    }

    default:
    case MSP_PASSTHROUGH_ESC_4WAY: {
      uint8_t data[1] = {4};
      msp_send_reply(msp, magic, cmd, data, 1);

      motor_test.active = 0;

      serial_4way_init();
      serial_4way_process();
      break;
    }
    }

    break;
  }

  case MSP_RESERVE_1: {
    quic_t quic = {
        .priv_data = msp,
        .send = msp_quic_send,
    };
    quic_process(&quic, payload, size);
    break;
  }

  case MSP2_COMMON_SERIAL_CONFIG: {
    const uint8_t uart_count = SERIAL_PORT_MAX - 1;
    uint8_t data[1 + uart_count * 5];

    data[0] = uart_count;

    for (uint32_t i = 0; i < uart_count; i++) {
      data[1 + i * 5] = i;

      uint32_t function = 0;
      if (i == serial_rx.config.port) {
        function = MSP_SERIAL_FUNCTION_RX;
      }
      if (i == serial_vtx.config.port) {
        if (vtx_settings.protocol == VTX_PROTOCOL_TRAMP) {
          function = MSP_SERIAL_FUNCTION_TRAMP;
        } else {
          function = MSP_SERIAL_FUNCTION_SA;
        }
      }
      if (i == serial_hdzero.config.port) {
        function = MSP_SERIAL_FUNCTION_HDZERO;
      }

      data[1 + i * 5 + 1] = (function >> 0) & 0xFF;
      data[1 + i * 5 + 2] = (function >> 8) & 0xFF;
      data[1 + i * 5 + 3] = (function >> 16) & 0xFF;
      data[1 + i * 5 + 4] = (function >> 24) & 0xFF;
    }

    msp_send_reply(msp, magic, cmd, data, 1 + uart_count * 5);
    break;
  }

  case MSP_VTX_CONFIG: {
    if (msp->device == MSP_DEVICE_VTX && vtx_settings.power_table.levels != 0) {
      msp_vtx_detected = 1;
    }
    msp_vtx_send_config_reply(msp, magic);
    break;
  }

  case MSP_SET_VTX_CONFIG: {
    vtx_settings_t *settings = msp->device == MSP_DEVICE_VTX ? &msp_vtx_settings : &vtx_settings;
    if (msp->device != MSP_DEVICE_VTX) {
      settings->magic = VTX_SETTINGS_MAGIC;
    }

    uint16_t remaining = size;

    uint16_t freq = (payload[1] << 8) | payload[0];
    remaining -= 2;
    if (freq < VTX_BAND_MAX * VTX_CHANNEL_MAX) {
      settings->band = freq / VTX_CHANNEL_MAX;
      settings->channel = freq % VTX_CHANNEL_MAX;
    } else {
      int8_t channel_index = vtx_find_frequency_index(freq);
      settings->band = channel_index / VTX_CHANNEL_MAX;
      settings->channel = channel_index % VTX_CHANNEL_MAX;
    }

    if (remaining >= 2) {
      settings->power_level = max(payload[2], 1) - 1;
      settings->pit_mode = payload[3];
      remaining -= 2;
    }

    if (remaining) {
      // payload[4] lowpower disarm, unused
      remaining -= 1;
    }

    if (remaining >= 2) {
      // payload[5], payload[6] pit mode freq, unused
      remaining -= 2;
    }

    if (remaining >= 4) {
      settings->band = payload[7] - 1;
      settings->channel = payload[8] - 1;
      //  payload[9], payload[10]  freq, unused
      remaining -= 4;
    }

    if (remaining >= 4) {
      // payload[11], band count, unused
      // payload[12], channel count, unused
      settings->power_table.levels = payload[13];

      if (payload[14]) {
        // clear tables
        for (uint32_t i = 0; i < VTX_POWER_LEVEL_MAX; i++) {
          settings->power_table.values[i] = 0;
          memset(settings->power_table.labels[i], 0, 3);
        }
      }

      remaining -= 4;
    }

    msp_send_reply(msp, magic, cmd, NULL, 0);
    break;
  }

  case MSP_VTXTABLE_BAND: {
    const uint8_t band = payload[0];
    if (band <= 0 || band > VTX_BAND_MAX) {
      msp_send_error(msp, magic, cmd);
      break;
    }

    uint8_t offset = 0;
    uint8_t buf[4 + 8 + VTX_CHANNEL_MAX * sizeof(uint16_t)];

    buf[offset++] = band;
    buf[offset++] = 8;

    for (uint32_t i = 0; i < 8; i++) {
      buf[offset++] = msp_vtx_band_labels[band - 1][i];
    }

    buf[offset++] = msp_vtx_band_letters[band - 1];
    buf[offset++] = 0; // is factory

    buf[offset++] = VTX_CHANNEL_MAX;
    for (uint32_t i = 0; i < VTX_CHANNEL_MAX; i++) {
      buf[offset++] = frequency_table[band - 1][i] & 0xFF;
      buf[offset++] = frequency_table[band - 1][i] >> 8;
    }

    msp_send_reply(msp, magic, cmd, buf, offset);
    break;
  }

  case MSP_SET_VTXTABLE_BAND: {
    uint8_t offset = 0;
    const uint8_t band = payload[offset++];
    if (band <= 0 || band > VTX_BAND_MAX) {
      msp_send_error(msp, magic, cmd);
      break;
    }

    const uint8_t label_len = payload[offset++];
    for (uint8_t i = 0; i < 8; i++) {
      msp_vtx_band_labels[band - 1][i] = i >= label_len ? 0 : payload[offset++];
    }

    msp_vtx_band_letters[band - 1] = payload[offset++];

    // ignore the rest
    msp_send_reply(msp, magic, cmd, NULL, 0);
    break;
  }

  case MSP_VTXTABLE_POWERLEVEL: {
    vtx_settings_t *settings = msp->device == MSP_DEVICE_VTX ? &msp_vtx_settings : &vtx_settings;

    const uint8_t level = payload[0];
    if (level <= 0 || level > VTX_POWER_LEVEL_MAX) {
      msp_send_error(msp, magic, cmd);
      break;
    }

    const uint16_t power = settings->power_table.values[level - 1];

    uint8_t buf[7];
    buf[0] = level;
    buf[1] = power & 0xFF;
    buf[2] = power >> 8;
    buf[3] = 3;
    memcpy(buf + 4, settings->power_table.labels[level - 1], 3);

    msp_send_reply(msp, magic, cmd, buf, 7);
    break;
  }

  case MSP_SET_VTXTABLE_POWERLEVEL: {
    vtx_settings_t *settings = msp->device == MSP_DEVICE_VTX ? &msp_vtx_settings : &vtx_settings;

    const uint8_t level = payload[0];
    if (level <= 0 || level > VTX_POWER_LEVEL_MAX) {
      msp_send_error(msp, magic, cmd);
      break;
    }

    settings->power_table.levels = max(level, settings->power_table.levels);
    settings->power_table.values[level - 1] = payload[2] << 8 | payload[1];

    const uint8_t label_len = payload[3];
    for (uint8_t i = 0; i < 3; i++) {
      settings->power_table.labels[level - 1][i] = i >= label_len ? 0 : payload[4 + i];
    }
    msp_send_reply(msp, magic, cmd, NULL, 0);
    break;
  }

  case MSP_EEPROM_WRITE: {
    if (msp->device == MSP_DEVICE_VTX) {
      msp_vtx_detected = 1;
    }
    if (!flags.arm_state && msp->device != MSP_DEVICE_SPI_RX) {
      flash_save();
    }
    msp_send_reply(msp, magic, cmd, NULL, 0);
    break;
  }

  case MSP_REBOOT: {
    if (flags.arm_state) {
      break;
    }

    switch (payload[0]) {
    case MSP_REBOOT_FIRMWARE:
      system_reset();
      break;

    case MSP_REBOOT_BOOTLOADER_ROM:
      system_reset_to_bootloader();
      break;

    default:
      break;
    }
    break;
  }

  default:
    msp_send_error(msp, magic, cmd);
    break;
  }
}

msp_status_t msp_process_serial(msp_t *msp, uint8_t data) {
  if (msp->buffer_offset >= msp->buffer_size) {
    msp->buffer_offset = 0;
    return MSP_ERROR;
  }

  msp->buffer[msp->buffer_offset] = data;
  msp->buffer_offset++;

  if (msp->buffer[0] != '$') {
    msp->buffer_offset = 0;
    return MSP_ERROR;
  }

  if (msp->buffer_offset < 3) {
    return MSP_EOF;
  }

  switch (msp->buffer[1]) {
  case 'M': {
    if (msp->buffer_offset < MSP_HEADER_LEN) {
      return MSP_EOF;
    }

    const uint8_t size = msp->buffer[3];
    const uint8_t cmd = msp->buffer[4];

    if (msp->buffer_offset < (MSP_HEADER_LEN + size + 1)) {
      return MSP_EOF;
    }

    uint8_t chksum = size ^ cmd;
    for (uint8_t i = 0; i < size; i++) {
      chksum ^= msp->buffer[MSP_HEADER_LEN + i];
    }

    if (msp->buffer[MSP_HEADER_LEN + size] != chksum) {
      msp->buffer_offset = 0;
      return MSP_ERROR;
    }

    msp_process_serial_cmd(msp, MSP1_MAGIC, cmd, msp->buffer + MSP_HEADER_LEN, size);
    msp->buffer_offset = 0;
    return MSP_SUCCESS;
  }

  case 'X': {
    if (msp->buffer_offset < MSP2_HEADER_LEN) {
      return MSP_EOF;
    }

    //  msp->buffer[3] flag
    const uint16_t cmd = (msp->buffer[5] << 8) | msp->buffer[4];
    const uint16_t size = (msp->buffer[7] << 8) | msp->buffer[6];

    if (msp->buffer_offset < (MSP2_HEADER_LEN + size + 1)) {
      return MSP_EOF;
    }

    const uint8_t chksum = crc8_dvb_s2_data(0, msp->buffer + 3, size + 5);
    if (msp->buffer[MSP2_HEADER_LEN + size] != chksum) {
      msp->buffer_offset = 0;
      return MSP_ERROR;
    }

    msp_process_serial_cmd(msp, MSP2_MAGIC, cmd, msp->buffer + MSP2_HEADER_LEN, size);
    msp->buffer_offset = 0;
    return MSP_SUCCESS;
  }

  default:
    msp->buffer_offset = 0;
    return MSP_ERROR;
  }
}

msp_status_t msp_process_telemetry(msp_t *msp, uint8_t *data, uint32_t len) {
  if (len < 1) {
    return MSP_EOF;
  }

  uint8_t offset = 0;

  const uint8_t status = data[offset++];
  const uint8_t version = (status & MSP_STATUS_VERSION_MASK) >> MSP_STATUS_VERSION_SHIFT;
  const uint8_t sequence = status & MSP_STATUS_SEQUENCE_MASK;
  if (version != 1) {
    return MSP_ERROR;
  }

  static bool packet_started = false;

  static uint16_t last_size = 0;
  static uint8_t last_cmd = 0;
  static uint8_t last_seq = 0;

  if (status & MSP_STATUS_START_MASK) { // first chunk
    if (len < MSP_TLM_HEADER_LEN) {
      return MSP_EOF;
    }
    last_size = data[offset++];
    last_cmd = data[offset++];

    if (last_size == 0xFF) {
      last_size = (data[offset + 1] << 8) | data[offset];
      offset += 2;
    }

    packet_started = true;
    msp->buffer_offset = 0;
  } else { // second chunk
    if (!packet_started) {
      return MSP_ERROR;
    }
    if (((last_seq + 1) & MSP_STATUS_SEQUENCE_MASK) != sequence) {
      packet_started = false;
      return MSP_ERROR;
    }
  }

  last_seq = sequence;

  memcpy(msp->buffer + msp->buffer_offset, data + offset, len - offset);
  msp->buffer_offset += len - offset;

  if (msp->buffer_offset < last_size) {
    return MSP_EOF;
  }

  msp_process_serial_cmd(msp, MSP1_MAGIC, last_cmd, msp->buffer, last_size);
  return MSP_SUCCESS;
}