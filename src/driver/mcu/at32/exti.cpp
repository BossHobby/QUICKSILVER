#include "driver/exti.h"

#include "driver/interrupt.h"

#define EXTI_LINE(n, irq) {n, 1U << n, n, irq}
const exti_line_def_t exti_line_defs[16] = {
    EXTI_LINE(0, EXINT0_IRQn),
    EXTI_LINE(1, EXINT1_IRQn),
    EXTI_LINE(2, EXINT2_IRQn),
    EXTI_LINE(3, EXINT3_IRQn),
    EXTI_LINE(4, EXINT4_IRQn),
    EXTI_LINE(5, EXINT9_5_IRQn),
    EXTI_LINE(6, EXINT9_5_IRQn),
    EXTI_LINE(7, EXINT9_5_IRQn),
    EXTI_LINE(8, EXINT9_5_IRQn),
    EXTI_LINE(9, EXINT9_5_IRQn),
    EXTI_LINE(10, EXINT15_10_IRQn),
    EXTI_LINE(11, EXINT15_10_IRQn),
    EXTI_LINE(12, EXINT15_10_IRQn),
    EXTI_LINE(13, EXINT15_10_IRQn),
    EXTI_LINE(14, EXINT15_10_IRQn),
    EXTI_LINE(15, EXINT15_10_IRQn),
};
#undef EXTI_LINE

void exti_enable(gpio_pins_t pin, exti_trigger_t trigger) {
  if (pin == PIN_NONE)
    return;
  if (trigger == EXTI_TRIG_NONE) {
    exti_disable(pin);
    return;
  }
  const gpio_pin_def_t &gpio = gpio_pin_defs[pin];
  const exti_line_def_t &line = exti_line_defs[gpio.pin_index];
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
  // GPIO ports occupy consecutive 0x400-byte blocks in the AT32 register map.
  const auto port = static_cast<scfg_port_source_type>((reinterpret_cast<uintptr_t>(gpio.port) - GPIOA_BASE) / 0x400);
  scfg_exint_line_config(port, static_cast<scfg_pins_source_type>(gpio.pin_index));

  exint_init_type config;
  exint_default_para_init(&config);
  config.line_select = line.exti_line;
  config.line_enable = TRUE;
  config.line_mode = EXINT_LINE_INTERRUPUT;
  config.line_polarity = trigger == EXTI_TRIG_RISING ? EXINT_TRIGGER_RISING_EDGE : trigger == EXTI_TRIG_FALLING ? EXINT_TRIGGER_FALLING_EDGE
                                                                                                                : EXINT_TRIGGER_BOTH_EDGE;
  exint_flag_clear(line.exti_line);
  exint_init(&config);
  interrupt_enable(line.exti_irqn, EXTI_PRIORITY);
}

void exti_interrupt_enable(gpio_pins_t pin) {
  if (pin != PIN_NONE)
    exint_interrupt_enable(1U << gpio_pin_defs[pin].pin_index, TRUE);
}

void exti_interrupt_disable(gpio_pins_t pin) {
  if (pin != PIN_NONE)
    exint_interrupt_enable(1U << gpio_pin_defs[pin].pin_index, FALSE);
}

void exti_disable(gpio_pins_t pin) {
  if (pin == PIN_NONE)
    return;
  exti_interrupt_disable(pin);
  exint_flag_clear(1U << gpio_pin_defs[pin].pin_index);
}

bool exti_line_active(gpio_pins_t pin) {
  if (pin == PIN_NONE)
    return false;
  const uint32_t line = 1U << gpio_pin_defs[pin].pin_index;
  if (exint_interrupt_flag_get(line) == RESET)
    return false;
  exint_flag_clear(line);
  return true;
}

static void handle_exti_isr() {
  if (exti_line_active(target.gyro.exti)) {
    extern void gyro_handle_exti();
    gyro_handle_exti();
  }
}

extern "C" {
void EXINT0_IRQHandler() { handle_exti_isr(); }
void EXINT1_IRQHandler() { handle_exti_isr(); }
void EXINT2_IRQHandler() { handle_exti_isr(); }
void EXINT3_IRQHandler() { handle_exti_isr(); }
void EXINT4_IRQHandler() { handle_exti_isr(); }
void EXINT9_5_IRQHandler() { handle_exti_isr(); }
void EXINT15_10_IRQHandler() { handle_exti_isr(); }
}
