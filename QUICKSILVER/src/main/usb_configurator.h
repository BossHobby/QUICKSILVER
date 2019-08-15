#pragma once

#include <stdint.h>

typedef enum {
  USB_MAGIC_REBOOT = 'R',
  USB_MAGIC_MSP = '$',
  USB_MAGIC_QUIC = '#',
} usb_magics;

uint8_t usb_process_msp(uint8_t *data, uint32_t len);
uint8_t usb_process_quic(uint8_t *data, uint32_t len);