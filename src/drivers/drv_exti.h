#pragma once

#include "drv_gpio.h"

typedef struct {
  uint8_t index;
  uint32_t exti_line;
  uint32_t syscfg_exti_line;
  IRQn_Type exti_irqn;
} exti_line_def_t;

extern const exti_line_def_t exti_line_defs[16];

void exti_enable(gpio_pins_t pin, uint32_t trigger);