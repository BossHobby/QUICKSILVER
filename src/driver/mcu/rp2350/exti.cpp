#include "driver/exti.h"

#include "driver/interrupt.h"

extern void gyro_handle_exti();

// Events selected by exti_enable() for each pin, re-armed by
// exti_interrupt_enable().
static uint8_t exti_events[NUM_BANK0_GPIOS];

static uint32_t exti_event_mask(exti_trigger_t trigger) {
  switch (trigger) {
  case EXTI_TRIG_RISING:
    return GPIO_IRQ_EDGE_RISE;
  case EXTI_TRIG_FALLING:
    return GPIO_IRQ_EDGE_FALL;
  case EXTI_TRIG_RISING_FALLING:
    return GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
  default:
    return 0;
  }
}

void exti_enable(gpio_pins_t pin, exti_trigger_t trigger) {
  const uint32_t index = gpio_pin_defs[pin].pin_index;
  exti_events[index] = exti_event_mask(trigger);
  gpio_set_irq_enabled(index, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
  gpio_set_irq_enabled(index, exti_events[index], true);
  interrupt_enable(IO_IRQ_BANK0_IRQn, EXTI_PRIORITY);
}

void exti_interrupt_enable(gpio_pins_t pin) {
  const uint32_t index = gpio_pin_defs[pin].pin_index;
  gpio_set_irq_enabled(index, exti_events[index], true);
}

void exti_interrupt_disable(gpio_pins_t pin) {
  const uint32_t index = gpio_pin_defs[pin].pin_index;
  gpio_set_irq_enabled(index, exti_events[index], false);
}

extern "C" void IO_IRQ_BANK0_Handler() {
  if (target.gyro.exti == PIN_NONE)
    return;
  const uint32_t index = gpio_pin_defs[target.gyro.exti].pin_index;
  const uint32_t events = gpio_get_irq_event_mask(index) & exti_events[index];
  if (events == 0)
    return;
  gpio_acknowledge_irq(index, events);
  gyro_handle_exti();
}
