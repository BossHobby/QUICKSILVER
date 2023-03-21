#pragma once

#include <target.h>

#define PIN_IDENT(port, num) PIN_##port##num
#define GPIO_PIN(port, num) PIN_IDENT(port, num),

typedef enum {
  PIN_NONE,
#include "gpio_pins.in"
  PINS_MAX,
} gpio_pins_t;

#undef GPIO_PIN
