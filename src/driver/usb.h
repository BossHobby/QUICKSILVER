#pragma once

#include <stdint.h>

#include "core/project.h"
#include "util/ring_buffer.h"

#define USB_BUFFER_SIZE 4096

extern ring_buffer_t usb_tx_buffer;
extern ring_buffer_t usb_rx_buffer;

void usb_init();
uint8_t usb_detect();
uint32_t usb_serial_read(uint8_t *data, uint32_t len);
void usb_serial_write(uint8_t *data, uint32_t len);
void usb_serial_printf(const char *fmt, ...);
void usb_serial_print(char *str);