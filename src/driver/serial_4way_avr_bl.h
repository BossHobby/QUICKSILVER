#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "core/project.h"
#include "driver/serial_4way.h"

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE

// Bootloader result codes
#define brSUCCESS 0x30
#define brERRORVERIFY 0xC0
#define brERRORCOMMAND 0xC1
#define brERRORCRC 0xC2
#define brNONE 0xFF

void avr_bl_init_pin(gpio_pins_t pin);
uint8_t avr_bl_connect(gpio_pins_t pin, uint8_t *data);
uint8_t avr_bl_send_keepalive(gpio_pins_t pin);
void avr_bl_send_restart(gpio_pins_t pin, uint8_t *data);
void avr_bl_reboot(gpio_pins_t pin);

uint8_t avr_bl_read_flash(gpio_pins_t pin, uint8_t interface_mode, uint16_t addr, uint8_t *data, uint8_t size);
uint8_t avr_bl_read_eeprom(gpio_pins_t pin, uint16_t addr, uint8_t *data, uint8_t size);
uint8_t avr_bl_page_erase(gpio_pins_t pin, uint16_t addr);
uint8_t avr_bl_write_eeprom(gpio_pins_t pin, uint16_t addr, const uint8_t *data, uint8_t size);
uint8_t avr_bl_write_flash(gpio_pins_t pin, uint16_t addr, const uint8_t *data, uint8_t size);
uint8_t avr_bl_verify_flash(gpio_pins_t pin, uint16_t addr, const uint8_t *data, uint8_t size);

#endif
