#include "msp.h"

#include <stdbool.h>
#include <string.h>

#include "drv_serial.h"
#include "drv_serial_4way.h"
#include "flight/control.h"
#include "io/usb_configurator.h"
#include "quic.h"
#include "util/crc.h"
#include "util/util.h"
#include "vtx.h"

static void msp_send_reply(msp_t *msp, msp_magic_t magic, uint16_t cmd, uint8_t *data, uint32_t len) {
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

      if (arg == serial_smart_audio_port) {
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
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
      uint8_t data[1] = {4};
      msp_send_reply(msp, magic, cmd, data, 1);

      motor_test.active = 0;

      serial_4way_init();
      serial_4way_process();
#endif
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
    const uint8_t uart_count = USART_PORTS_MAX - 1;
    uint8_t data[1 + uart_count * 5];

    data[0] = uart_count;

    for (uint32_t i = 0; i < uart_count; i++) {
      const usart_ports_t port = i + 1;
      data[1 + i * 5] = port;

      uint32_t function = 0;
      if (port == serial_rx_port) {
        function = MSP_SERIAL_FUNCTION_RX;
      }
      if (port == serial_smart_audio_port) {
        if (vtx_settings.protocol == VTX_PROTOCOL_TRAMP) {
          function = MSP_SERIAL_FUNCTION_TRAMP;
        } else {
          function = MSP_SERIAL_FUNCTION_SA;
        }
      }
      if (port == serial_hdzero_port) {
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

  default:
    msp_send_error(msp, magic, cmd);
    break;
  }
}

msp_status_t msp_process_serial(msp_t *msp, uint8_t *data, uint32_t len) {
  if (len < MSP_HEADER_LEN) {
    return MSP_EOF;
  }

  if (data[0] != '$' || data[2] != '<') {
    return MSP_ERROR;
  }

  if (data[1] == 'M') {
    const uint8_t size = data[3];
    const uint8_t cmd = data[4];

    if (len < (MSP_HEADER_LEN + size + 1)) {
      return MSP_EOF;
    }

    uint8_t chksum = size ^ cmd;
    for (uint8_t i = 0; i < size; i++) {
      chksum ^= data[MSP_HEADER_LEN + i];
    }

    if (data[MSP_HEADER_LEN + size] != chksum) {
      return MSP_ERROR;
    }

    msp_process_serial_cmd(msp, MSP1_MAGIC, cmd, data + MSP_HEADER_LEN, size);
    return MSP_SUCCESS;
  }

  if (data[1] == 'X') {
    if (len < MSP2_HEADER_LEN) {
      return MSP_EOF;
    }

    //  data[3] flag
    const uint16_t cmd = (data[5] << 8) | data[4];
    const uint16_t size = (data[7] << 8) | data[6];

    if (len < (MSP2_HEADER_LEN + size + 1)) {
      return MSP_EOF;
    }

    const uint8_t chksum = crc8_dvb_s2_data(0, data + 3, size + 5);
    if (data[MSP2_HEADER_LEN + size] != chksum) {
      return MSP_ERROR;
    }

    msp_process_serial_cmd(msp, MSP2_MAGIC, cmd, data + MSP2_HEADER_LEN, size);
    return MSP_SUCCESS;
  }

  return MSP_ERROR;
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