#pragma once

#include <cbor.h>
#include <stdint.h>

#include "blackbox.h"

#define USB_BUFFER_SIZE 2048

typedef enum {
  USB_MAGIC_REBOOT = 'R',
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
} quic_command;

typedef enum {
  QUIC_FLAG_NONE,
  QUIC_FLAG_ERROR,
} quic_flag;

typedef enum {
  QUIC_VAL_INVALID,
  QUIC_VAL_INFO,
  QUIC_VAL_PROFILE,
  QUIC_VAL_DEFAULT_PROFILE,
  QUIC_VAL_BLACKBOX_RATE,
  QUIC_VAL_PID_RATE_PRESETS,
} quic_values;

void usb_process_msp();
void usb_process_quic();

cbor_result_t send_quic_strf(quic_command cmd, quic_flag flag, const char *fmt, ...);
cbor_result_t quic_blackbox(const blackbox_t *blackbox);

#ifdef DEBUG
#define quic_debugf(args...) send_quic_strf(QUIC_CMD_LOG, QUIC_FLAG_NONE, args)
#else
#define quic_debugf(args...) __NOP()
#endif