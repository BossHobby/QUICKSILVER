#pragma once

#include "target.h"

#ifdef F4

#define PA0 GPIO_PIN(A, 0)
#define PA1 GPIO_PIN(A, 1)
#define PA2 GPIO_PIN(A, 2)
#define PA3 GPIO_PIN(A, 3)
#define PA9 GPIO_PIN(A, 9)
#define PA10 GPIO_PIN(A, 10)
#define PA15 GPIO_PIN(A, 15)

#define PB3 GPIO_PIN(B, 3)
#define PB4 GPIO_PIN(B, 4)
#define PB5 GPIO_PIN(B, 5)
#define PB6 GPIO_PIN(B, 6)
#define PB7 GPIO_PIN(B, 7)
#define PB10 GPIO_PIN(B, 10)
#define PB11 GPIO_PIN(B, 11)

#define PC6 GPIO_PIN(C, 6)
#define PC7 GPIO_PIN(C, 7)
#define PC10 GPIO_PIN(C, 10)
#define PC11 GPIO_PIN(C, 11)

#define GPIO_PINS                   \
  PA0 PA1 PA2 PA3 PA9 PA10 PA15     \
      PB3 PB4 PB5 PB6 PB7 PB10 PB11 \
          PC6 PC7 PC10 PC11

#endif

#define PIN_IDENT(port, num) PIN_##port##num
#define GPIO_PIN(port, num) PIN_IDENT(port, num),

typedef enum {
  GPIO_PIN_INVALID,
  GPIO_PINS
      GPIO_PINS_MAX,
} gpio_pins_t;

#undef GPIO_PIN