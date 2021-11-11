#pragma once

#include "target.h"

#ifdef STM32F4

#define PA0 GPIO_PIN(A, 0)
#define PA1 GPIO_PIN(A, 1)
#define PA2 GPIO_PIN(A, 2)
#define PA3 GPIO_PIN(A, 3)
#define PA4 GPIO_PIN(A, 4)
#define PA5 GPIO_PIN(A, 5)
#define PA6 GPIO_PIN(A, 6)
#define PA7 GPIO_PIN(A, 7)
#define PA8 GPIO_PIN(A, 8)
#define PA9 GPIO_PIN(A, 9)
#define PA10 GPIO_PIN(A, 10)
#define PA11 GPIO_PIN(A, 11)
#define PA12 GPIO_PIN(A, 12)
#define PA13 GPIO_PIN(A, 13)
#define PA14 GPIO_PIN(A, 14)
#define PA15 GPIO_PIN(A, 15)

#define PB0 GPIO_PIN(B, 0)
#define PB1 GPIO_PIN(B, 1)
#define PB2 GPIO_PIN(B, 2)
#define PB3 GPIO_PIN(B, 3)
#define PB4 GPIO_PIN(B, 4)
#define PB5 GPIO_PIN(B, 5)
#define PB6 GPIO_PIN(B, 6)
#define PB7 GPIO_PIN(B, 7)
#define PB8 GPIO_PIN(B, 8)
#define PB9 GPIO_PIN(B, 9)
#define PB10 GPIO_PIN(B, 10)
#define PB11 GPIO_PIN(B, 11)
#define PB12 GPIO_PIN(B, 12)
#define PB13 GPIO_PIN(B, 13)
#define PB14 GPIO_PIN(B, 14)
#define PB15 GPIO_PIN(B, 15)

#define PC0 GPIO_PIN(C, 0)
#define PC1 GPIO_PIN(C, 1)
#define PC2 GPIO_PIN(C, 2)
#define PC3 GPIO_PIN(C, 3)
#define PC4 GPIO_PIN(C, 4)
#define PC5 GPIO_PIN(C, 5)
#define PC6 GPIO_PIN(C, 6)
#define PC7 GPIO_PIN(C, 7)
#define PC8 GPIO_PIN(C, 8)
#define PC9 GPIO_PIN(C, 9)
#define PC10 GPIO_PIN(C, 10)
#define PC11 GPIO_PIN(C, 11)
#define PC12 GPIO_PIN(C, 12)
#define PC13 GPIO_PIN(C, 13)
#define PC14 GPIO_PIN(C, 14)
#define PC15 GPIO_PIN(C, 15)
#define PD2 GPIO_PIN(D, 2)

#define GPIO_PINS                                                               \
  PA0 PA1 PA2 PA3 PA4 PA5 PA6 PA7 PA8 PA9 PA10 PA11 PA12 PA13 PA14 PA15         \
      PB0 PB1 PB2 PB3 PB4 PB5 PB6 PB7 PB8 PB9 PB10 PB11 PB12 PB13 PB14 PB15     \
          PC0 PC1 PC2 PC3 PC4 PC5 PC6 PC7 PC8 PC9 PC10 PC11 PC12 PC13 PC14 PC15 \
              PD2

#endif

#define PIN_IDENT(port, num) PIN_##port##num
#define GPIO_PIN(port, num) PIN_IDENT(port, num),

typedef enum {
  GPIO_PIN_INVALID,
  GPIO_PINS
      GPIO_PINS_MAX,
} gpio_pins_t;

#undef GPIO_PIN
