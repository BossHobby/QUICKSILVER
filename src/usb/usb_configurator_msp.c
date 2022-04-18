#include "usb_configurator.h"

#include "debug.h"
#include "drv_serial_4way.h"
#include "drv_time.h"
#include "drv_usb.h"
#include "flight/control.h"
#include "io/msp.h"
#include "project.h"
#include "util/util.h"

extern usb_motor_test_t usb_motor_test;

void send_msp(uint8_t direction, uint8_t code, uint8_t *data, uint8_t len) {
  const uint8_t size = len + MSP_HEADER_LEN + 1;

  static uint8_t frame[64];

  frame[0] = '$';
  frame[1] = 'M';
  frame[2] = '>';
  frame[3] = len;
  frame[4] = code;

  for (uint8_t i = 0; i < len; i++) {
    frame[i + MSP_HEADER_LEN] = data[i];
  }

  uint8_t chksum = len;
  for (uint8_t i = 4; i < (size - 1); i++) {
    chksum ^= frame[i];
  }
  frame[len + MSP_HEADER_LEN] = chksum;

  usb_serial_write(frame, size);
}

void send_msp_reply(uint8_t code, uint8_t *data, uint8_t len) {
  send_msp('>', code, data, len);
}

void send_msp_error(uint8_t code) {
  send_msp('!', code, NULL, 0);
}

extern uint8_t encode_buffer[USB_BUFFER_SIZE];
extern uint8_t decode_buffer[USB_BUFFER_SIZE];

void usb_process_msp() {
  if (usb_serial_read_byte() != 'M' || usb_serial_read_byte() != '<') {
    return;
  }

  const uint8_t size = usb_serial_read_byte();
  const uint8_t code = usb_serial_read_byte();
  if (code == 0) {
    usb_serial_printf("ERROR invalid code (%d)\r\n", code);
    return;
  }

  uint8_t chksum = size ^ code;
  if (size > 0) {
    uint32_t len = usb_serial_read(decode_buffer, size);
    for (uint32_t timeout = 1000; len < size && timeout; --timeout) {
      len += usb_serial_read(decode_buffer + len, size - len);
      time_delay_us(10);
      __WFI();
    }
    if (len != size) {
      usb_serial_printf("ERROR invalid size (%d vs %d)\r\n", len, size);
      return;
    }
    for (uint8_t i = 0; i < size; i++) {
      chksum ^= decode_buffer[i];
    }
  }

  const uint8_t expected_chksum = usb_serial_read_byte();
  if (chksum != expected_chksum) {
    usb_serial_printf("ERROR invalid chksum (%d vs %d)\r\n", chksum, expected_chksum);
    return;
  }

  switch (code) {
  case MSP_API_VERSION: {
    uint8_t data[3] = {
        0,  // MSP_PROTOCOL_VERSION
        1,  // API_VERSION_MAJOR
        41, // API_VERSION_MINOR
    };
    send_msp_reply(code, data, 3);
    break;
  }
  case MSP_FC_VARIANT: {
    uint8_t data[4] = {'Q', 'U', 'I', 'C'};
    send_msp_reply(code, data, 4);
    break;
  }
  case MSP_FC_VERSION: {
    uint8_t data[3] = {
        0, // FC_VERSION_MAJOR
        1, // FC_VERSION_MINOR
        0, // FC_VERSION_PATCH_LEVEL
    };
    send_msp_reply(code, data, 3);
    break;
  }
  case MSP_BUILD_INFO: {
    uint8_t data[19] = MSP_BUILD_DATE_TIME;
    send_msp_reply(code, data, 19);
    break;
  }
  case MSP_BOARD_INFO: {
    uint8_t data[6] = {'Q', 'U', 'I', 'C', 0, 0};
    send_msp_reply(code, data, 6);
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
    send_msp_reply(code, data, 12);
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
    send_msp_reply(code, data, 9);
    break;
  }
  case MSP_FEATURE_CONFIG: {
    uint8_t data[4] = {
        0x0,
        0x0,
        0x0,
        0x0,
    };
    send_msp_reply(code, data, 4);
    break;
  }
  case MSP_MOTOR_CONFIG: {
    uint16_t data[3] = {
        1070, // min throttle
        2000, // max throttle
        1000, // min command
    };
    send_msp_reply(code, (uint8_t *)data, 3 * sizeof(uint16_t));
    break;
  }
  case MSP_MOTOR: {
    // we always have 4 motors, but blheli expects 8
    // these are pwm values
    uint16_t data[8] = {
        (uint16_t)mapf(usb_motor_test.value[0], 0.0f, 1.0f, 1000.f, 2000.f),
        (uint16_t)mapf(usb_motor_test.value[1], 0.0f, 1.0f, 1000.f, 2000.f),
        (uint16_t)mapf(usb_motor_test.value[2], 0.0f, 1.0f, 1000.f, 2000.f),
        (uint16_t)mapf(usb_motor_test.value[3], 0.0f, 1.0f, 1000.f, 2000.f),
        0,
        0,
        0,
        0,
    };
    send_msp_reply(code, (uint8_t *)data, 8 * sizeof(uint16_t));
    break;
  }
  case MSP_SET_MOTOR: {
    uint16_t *values = (uint16_t *)(decode_buffer);
    for (uint8_t i = 0; i < 4; i++) {
      usb_motor_test.value[i] = mapf(values[i], 1000.f, 2000.f, 0.0f, 1.0f);
    }
    usb_motor_test.active = 1;

    send_msp_reply(code, NULL, 0);
    break;
  }
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
  case MSP_SET_4WAY_IF: {
    uint8_t data[1] = {4};
    send_msp_reply(code, data, 1);

    usb_motor_test.active = 0;

    serial_4way_init();
    serial_4way_process();

    break;
  }
#endif
  default:
    send_msp_error(code);
    break;
  }
}