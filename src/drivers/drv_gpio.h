#pragma once

#include "gpio_pins.h"

#include "project.h"

typedef struct {
  GPIO_TypeDef *port;
  uint8_t pin_index;
  uint32_t pin;
  uint32_t pin_source;
} gpio_pin_def_t;

#define MAKE_PIN_DEF(port_num, num)    \
  {                                    \
    .port = GPIO##port_num,            \
    .pin_index = num,                  \
    .pin = GPIO_Pin_##num,             \
    .pin_source = GPIO_PinSource##num, \
  }

extern const volatile gpio_pin_def_t gpio_pin_defs[GPIO_PINS_MAX];

void gpio_init(void);
int gpio_init_fpv(uint8_t rxmode);
