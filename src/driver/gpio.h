#pragma once

#include "core/project.h"
#include "driver/resource.h"
#include "driver/timer.h"

typedef struct {
  gpio_pins_t pin;
  resource_tag_t tag;
  uint8_t af;
} gpio_af_t;

typedef struct {
  GPIO_TypeDef *port;
  uint8_t pin_index;
  uint32_t pin;
} gpio_pin_def_t;

extern const gpio_pin_def_t gpio_pin_defs[PINS_MAX];
extern const gpio_af_t gpio_pin_afs[];
extern const uint32_t GPIO_AF_MAX;

void gpio_init();

void gpio_pin_init(LL_GPIO_InitTypeDef *init, gpio_pins_t pin);
void gpio_pin_init_af(LL_GPIO_InitTypeDef *init, gpio_pins_t pin, uint8_t af);
void gpio_pin_init_tag(LL_GPIO_InitTypeDef *init, gpio_pins_t pin, resource_tag_t tag);
void gpio_pin_set(gpio_pins_t pin);
void gpio_pin_reset(gpio_pins_t pin);
void gpio_pin_toggle(gpio_pins_t pin);
uint32_t gpio_pin_read(gpio_pins_t pin);

bool gpio_init_fpv(uint8_t mode);
