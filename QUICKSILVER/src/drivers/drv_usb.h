#pragma once

#include <stdint.h>

#include "project.h"

extern void usb_configurator(void);
void usb_init(void);
void usb_detect(void);
uint32_t usb_serial_read(uint8_t *data, uint32_t len);
uint8_t usb_serial_read_byte(void);
void usb_serial_write(uint8_t *data, uint32_t len);
void usb_serial_printf(const char *fmt, ...);
void usb_serial_print(char *str);