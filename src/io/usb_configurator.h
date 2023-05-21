#pragma once

#include <stdbool.h>

#include "core/project.h"

#define USB_BUFFER_SIZE (4 * 1024)

typedef enum {
  USB_MAGIC_REBOOT = 'R',
  USB_MAGIC_SOFT_REBOOT = 'S',
  USB_MAGIC_MSP = '$',
  USB_MAGIC_QUIC = '#',
} usb_magics;

void usb_serial_passthrough(serial_ports_t port, uint32_t baudrate, uint8_t stop_bits, bool half_duplex);
void usb_process_msp();
void usb_process_quic();
void usb_quic_logf(const char *fmt, ...);
void usb_configurator();
