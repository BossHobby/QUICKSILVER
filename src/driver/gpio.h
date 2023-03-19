#pragma once

#include "core/project.h"

typedef struct {
  GPIO_TypeDef *port;
  uint8_t pin_index;
  uint32_t pin;
} gpio_pin_def_t;

#define MAKE_PIN_DEF(port_num, num) \
  {                                 \
    .port = GPIO##port_num,         \
    .pin_index = num,               \
    .pin = LL_GPIO_PIN_##num,       \
  }

extern const gpio_pin_def_t gpio_pin_defs[PINS_MAX];

void gpio_init();

void gpio_pin_init(LL_GPIO_InitTypeDef *init, gpio_pins_t pin);
void gpio_pin_init_af(LL_GPIO_InitTypeDef *init, gpio_pins_t pin, uint32_t af);
void gpio_pin_set(gpio_pins_t pin);
void gpio_pin_reset(gpio_pins_t pin);
void gpio_pin_toggle(gpio_pins_t pin);
uint32_t gpio_pin_read(gpio_pins_t pin);

int gpio_init_fpv(uint8_t mode);
