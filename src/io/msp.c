#include "msp.h"

#include <stdbool.h>
#include <string.h>

#include "drv_serial_4way.h"
#include "flight/control.h"
#include "util/util.h"

static uint32_t msp_available(msp_t *msp) {
  if (msp->write_offset >= msp->read_offset) {
    return msp->write_offset - msp->read_offset;
  }
  return msp->buffer_size + msp->write_offset - msp->read_offset;
}

static bool msp_get(msp_t *msp, uint32_t offset, uint8_t *data) {
  if (offset >= msp_available(msp)) {
    return false;
  }
  *data = msp->buffer[msp->read_offset + offset];
  return true;
}

static bool msp_expect(msp_t *msp, uint32_t offset, uint8_t val) {
  uint8_t data = 0;
  if (!msp_get(msp, offset, &data)) {
    return false;
  }

  return data == val;
}

static void msp_send_reply(msp_t *msp, uint8_t code, uint8_t *data, uint8_t len) {
  if (msp->send) {
    msp->send('>', code, data, len);
  }
}

static void msp_send_error(msp_t *msp, uint8_t code) {
  if (msp->send) {
    msp->send('!', code, NULL, 0);
  }
}

void msp_push_byte(msp_t *msp, uint8_t val) {
  msp->buffer[msp->write_offset] = val;
  msp->write_offset++;
}

void msp_push(msp_t *msp, uint8_t *data, uint32_t size) {
  memcpy(msp->buffer, data, size);
  msp->write_offset += size;
}

static void msp_process_serial_cmd(msp_t *msp, uint8_t cmd, uint8_t *payload, uint8_t size) {
  switch (cmd) {
  case MSP_API_VERSION: {
    uint8_t data[3] = {
        0,  // MSP_PROTOCOL_VERSION
        1,  // API_VERSION_MAJOR
        41, // API_VERSION_MINOR
    };
    msp_send_reply(msp, cmd, data, 3);
    break;
  }
  case MSP_FC_VARIANT: {
    uint8_t data[4] = {'Q', 'U', 'I', 'C'};
    msp_send_reply(msp, cmd, data, 4);
    break;
  }
  case MSP_FC_VERSION: {
    uint8_t data[3] = {
        0, // FC_VERSION_MAJOR
        1, // FC_VERSION_MINOR
        0, // FC_VERSION_PATCH_LEVEL
    };
    msp_send_reply(msp, cmd, data, 3);
    break;
  }
  case MSP_BUILD_INFO: {
    uint8_t data[19] = MSP_BUILD_DATE_TIME;
    msp_send_reply(msp, cmd, data, 19);
    break;
  }
  case MSP_BOARD_INFO: {
    uint8_t data[6] = {'Q', 'U', 'I', 'C', 0, 0};
    msp_send_reply(msp, cmd, data, 6);
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
    msp_send_reply(msp, cmd, data, 12);
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
    msp_send_reply(msp, cmd, data, 9);
    break;
  }
  case MSP_FEATURE_CONFIG: {
    uint8_t data[4] = {
        0x0,
        0x0,
        0x0,
        0x0,
    };
    msp_send_reply(msp, cmd, data, 4);
    break;
  }
  case MSP_MOTOR_CONFIG: {
    uint16_t data[3] = {
        1070, // min throttle
        2000, // max throttle
        1000, // min command
    };
    msp_send_reply(msp, cmd, (uint8_t *)data, 3 * sizeof(uint16_t));
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
    msp_send_reply(msp, cmd, (uint8_t *)data, 8 * sizeof(uint16_t));
    break;
  }
  case MSP_SET_MOTOR: {
    uint16_t *values = (uint16_t *)(payload);
    for (uint8_t i = 0; i < 4; i++) {
      motor_test.value[i] = mapf(values[i], 1000.f, 2000.f, 0.0f, 1.0f);
    }
    motor_test.active = 1;

    msp_send_reply(msp, cmd, NULL, 0);
    break;
  }
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
  case MSP_SET_4WAY_IF: {
    uint8_t data[1] = {4};
    msp_send_reply(msp, cmd, data, 1);

    motor_test.active = 0;

    serial_4way_init();
    serial_4way_process();

    break;
  }
#endif
  default:
    msp_send_error(msp, cmd);
    break;
  }
}

msp_status_t msp_process_serial(msp_t *msp) {
  if (msp_available(msp) < MSP_HEADER_LEN) {
    return MSP_EOF;
  }

  if (!msp_expect(msp, 0, '$') || !msp_expect(msp, 1, 'M') || !msp_expect(msp, 2, '<')) {
    msp->read_offset++;
    return MSP_ERROR;
  }

  uint8_t size = 0;
  uint8_t cmd = 0;
  if (!msp_get(msp, 3, &size) || !msp_get(msp, 4, &cmd)) {
    return MSP_ERROR;
  }

  if (msp_available(msp) < (MSP_HEADER_LEN + size + 1)) {
    return MSP_EOF;
  }

  uint8_t chksum = size ^ cmd;
  for (uint8_t i = 0; i < size; i++) {
    chksum ^= msp->buffer[MSP_HEADER_LEN + i];
  }

  if (!msp_expect(msp, MSP_HEADER_LEN + size, chksum)) {
    msp->read_offset += (MSP_HEADER_LEN + size + 1);
    return MSP_ERROR;
  }

  msp_process_serial_cmd(msp, cmd, msp->buffer + MSP_HEADER_LEN, size);
  msp->read_offset += (MSP_HEADER_LEN + size + 1);
  return MSP_SUCCESS;
}

msp_status_t msp_process_telemetry(msp_t *msp, uint8_t *data, uint8_t len) {
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
    packet_started = true;
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

  while (msp_available(msp) < last_size) {
    if (offset == len) {
      return MSP_EOF;
    }
    msp_push_byte(msp, data[offset++]);
  }

  msp_process_serial_cmd(msp, last_cmd, msp->buffer, last_size);
  msp->read_offset += last_size;
  return MSP_SUCCESS;
}