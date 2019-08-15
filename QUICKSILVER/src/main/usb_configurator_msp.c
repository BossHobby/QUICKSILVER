#include "usb_configurator.h"

#include "debug.h"
#include "drv_serial_4way.h"
#include "drv_time.h"
#include "drv_usb.h"
#include "project.h"

#if defined(F405) && defined(USE_SERIAL_4WAY_BLHELI_INTERFACE)

#define MSP_API_VERSION 1 //out message
#define MSP_FC_VARIANT 2  //out message
#define MSP_FC_VERSION 3  //out message
#define MSP_BOARD_INFO 4  //out message
#define MSP_BUILD_INFO 5  //out message

#define MSP_UID 160         //out message         Unique device ID
#define MSP_SET_4WAY_IF 245 //in message          Sets 4way interface

#define MSP_HEADER_LEN 5

#define MSP_BUILD_DATE_TIME __DATE__ __TIME__

void send_msp(uint8_t code, uint8_t *data, uint8_t len) {
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

uint8_t usb_process_msp(uint8_t *data, uint32_t len) {
  const uint8_t size = data[3];
  const uint8_t code = data[4];
  // const uint8_t chksum = data[size + MSP_HEADER_LEN];
  const uint8_t full_size = size + MSP_HEADER_LEN + 1;

  switch (code) {
  case MSP_API_VERSION: {
    uint8_t data[3] = {
        0,  // MSP_PROTOCOL_VERSION
        1,  // API_VERSION_MAJOR
        41, // API_VERSION_MINOR
    };
    send_msp(code, data, 3);
    break;
  }
  case MSP_FC_VARIANT: {
    uint8_t data[4] = {'Q', 'U', 'I', 'C'};
    send_msp(code, data, 4);
    break;
  }
  case MSP_FC_VERSION: {
    uint8_t data[3] = {
        0, // FC_VERSION_MAJOR
        1, // FC_VERSION_MINOR
        0, // FC_VERSION_PATCH_LEVEL
    };
    send_msp(code, data, 3);
    break;
  }
  case MSP_BUILD_INFO: {
    uint8_t data[19] = MSP_BUILD_DATE_TIME;
    send_msp(code, data, 19);
    break;
  }
  case MSP_BOARD_INFO: {
    uint8_t data[6] = {'Q', 'U', 'I', 'C', 0, 0};
    send_msp(code, data, 6);
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
    send_msp(code, data, 12);
    break;
  }
  case MSP_SET_4WAY_IF: {
    const uint8_t data_size = len - full_size;

    uint8_t data[1] = {4};
    send_msp(code, data, 1);

    // this will block and handle all usb traffic while active
    extern unsigned int lastlooptime;

    esc4wayInit();
    esc4wayProcess();
    lastlooptime = gettime();

    return len;
  }
  default:
    break;
  }

  return full_size;
}

#endif