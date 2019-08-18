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

#ifdef DEBUG
#define usb_debug_print(str) usb_serial_print(str)
#define usb_debug_printf(args...) usb_serial_printf(args)
#else
#define usb_debug_print(str)
#define usb_debug_printf(args...)
#endif