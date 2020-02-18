#pragma once

#include "target.h"

#define GPIO_PINS PA15 PB3 PB4 PB5

#ifdef F4

#define PA15 PIN(A, 15)

#define PB3 PIN(B, 3)
#define PB4 PIN(B, 4)
#define PB5 PIN(B, 5)

#endif

#define PIN_IDENT(port, num) PIN_##port##num
#define PIN(port, num) PIN_IDENT(port, num),

typedef enum {
  GPIO_PIN_INVALID,
  GPIO_PINS
  GPIO_PINS_MAX,
} gpio_pins_t;

#undef PIN