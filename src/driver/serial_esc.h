#pragma once

#include <stdint.h>

#include "driver/gpio.h"

static inline bool serial_esc_is_high(gpio_pins_t pin) {
  return gpio_pin_read(pin) > 0;
}

static inline void serial_esc_set_high(gpio_pins_t pin) {
  gpio_pin_set(pin);
}

static inline void serial_esc_set_low(gpio_pins_t pin) {
  gpio_pin_reset(pin);
}

void serial_esc_set_input(gpio_pins_t pin);
void serial_esc_set_output(gpio_pins_t pin);

bool serial_esc_read(gpio_pins_t pin, uint32_t baud, uint8_t *bt);
void serial_esc_write(gpio_pins_t pin, uint32_t baud, uint8_t data);

void serial_esc_process(uint8_t index, uint32_t baud);