#pragma once

#include "io/quic.h"

#define USB_BUFFER_SIZE (16 * 1024)

typedef enum {
  USB_MAGIC_REBOOT = 'R',
  USB_MAGIC_SOFT_REBOOT = 'S',
  USB_MAGIC_MSP = '$',
  USB_MAGIC_QUIC = '#',
} usb_magics;

void usb_process_msp();
void usb_process_quic();
void usb_quic_logf(const char *fmt, ...);

#if defined(DEBUG) && defined(DEBUG_LOGGING)
#define quic_debugf(args...) usb_quic_logf(QUIC_CMD_LOG, QUIC_FLAG_NONE, args)
#else
#define quic_debugf(args...) __NOP()
#endif