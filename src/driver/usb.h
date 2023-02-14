#pragma once

#include <stdint.h>

#include "project.h"

void usb_init();
uint8_t usb_detect();
uint32_t usb_serial_read(uint8_t *data, uint32_t len);
uint8_t usb_serial_read_byte();
void usb_serial_write(uint8_t *data, uint32_t len);
void usb_serial_printf(const char *fmt, ...);
void usb_serial_print(char *str);