#pragma once

#include <stdint.h>

typedef enum {
  USB_MAGIC_REBOOT = 'R',
  USB_MAGIC_MSP = '$',
  USB_MAGIC_QUIC = '#',
} usb_magics;

void usb_process_msp();
void usb_process_quic();