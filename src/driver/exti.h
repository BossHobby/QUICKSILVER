#pragma once

#include "driver/gpio.h"

typedef enum {
  EXTI_TRIG_NONE,
  EXTI_TRIG_RISING,
  EXTI_TRIG_FALLING,
  EXTI_TRIG_RISING_FALLING,
} exti_trigger_t;

typedef struct {
  uint8_t index;
  uint32_t exti_line;
  uint32_t syscfg_exti_line;
  IRQn_Type exti_irqn;
} exti_line_def_t;

extern const exti_line_def_t exti_line_defs[16];

void exti_enable(gpio_pins_t pin, exti_trigger_t trigger);

// exti_enable() will configure the external interrupt *and enable the interrupt*
// Thereafter you can enable/disable the interrupt using either of the following
void exti_interrupt_enable(gpio_pins_t pin);
void exti_interrupt_disable(gpio_pins_t pin);
