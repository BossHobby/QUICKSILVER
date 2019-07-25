#pragma once

#include <stdint.h>

extern void usb_configurator(uint8_t *data, uint32_t len);
void usb_init(void);
void usb_detect(void);
void usb_serial_write(uint8_t *data, uint32_t len);
void usb_serial_printf(const char *fmt, ...);
void usb_serial_print(char *str);
