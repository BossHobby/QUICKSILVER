#pragma once

#include <cbor.h>
#include <stdint.h>

#include "blackbox.h"

#define QUIC_PROTOCOL_VERSION 4

#define USB_BUFFER_SIZE 2048

typedef enum {
  USB_MAGIC_REBOOT = 'R',
  USB_MAGIC_SOFT_REBOOT = 'S',
  USB_MAGIC_MSP = '$',
  USB_MAGIC_QUIC = '#',
} usb_magics;

typedef enum {
  QUIC_CMD_INVALID,
  QUIC_CMD_GET,
  QUIC_CMD_SET,
  QUIC_CMD_LOG,
  QUIC_CMD_CAL_IMU,
  QUIC_CMD_BLACKBOX,
  QUIC_CMD_MOTOR
} quic_command;

typedef enum {
  QUIC_BLACKBOX_RESET,
  QUIC_BLACKBOX_LIST,
  QUIC_BLACKBOX_GET
} quic_blackbox_command;

typedef enum {
  QUIC_MOTOR_TEST_STATUS,
  QUIC_MOTOR_TEST_ENABLE,
  QUIC_MOTOR_TEST_DISABLE,
  QUIC_MOTOR_TEST_SET_VALUE,
  QUIC_MOTOR_ESC4WAY_IF
} quic_motor_command;

typedef enum {
  QUIC_FLAG_NONE,
  QUIC_FLAG_ERROR,
  QUIC_FLAG_STREAMING,
  QUIC_FLAG_EXIT,
} quic_flag;

typedef enum {
  QUIC_VAL_INVALID,
  QUIC_VAL_INFO,
  QUIC_VAL_PROFILE,
  QUIC_VAL_DEFAULT_PROFILE,
  QUIC_VAL_STATE,
  QUIC_VAL_PID_RATE_PRESETS,
  QUIC_VAL_VTX_SETTINGS,
  QUIC_VAL_OSD_FONT,
  QUIC_VAL_BLHEL_SETTINGS,
  QUIC_VAL_BIND_INFO,
} quic_values;

typedef struct {
  uint8_t active;
  float value[4];
} usb_motor_test_t;

void usb_process_msp();
void usb_process_quic();

cbor_result_t send_quic_strf(quic_command cmd, quic_flag flag, const char *fmt, ...);

#ifdef DEBUG
#define quic_debugf(args...) send_quic_strf(QUIC_CMD_LOG, QUIC_FLAG_NONE, args)
#else
#define quic_debugf(args...) __NOP()
#endif