#include "driver/exti.h"

#include <stdbool.h>

#include "driver/interrupt.h"
#include "project.h"

#define EXTI_LINES \
  EXTI_LINE(0)     \
  EXTI_LINE(1)     \
  EXTI_LINE(2)     \
  EXTI_LINE(3)     \
  EXTI_LINE(4)     \
  EXTI_LINE(5)     \
  EXTI_LINE(6)     \
  EXTI_LINE(7)     \
  EXTI_LINE(8)     \
  EXTI_LINE(9)     \
  EXTI_LINE(10)    \
  EXTI_LINE(11)    \
  EXTI_LINE(12)    \
  EXTI_LINE(13)    \
  EXTI_LINE(14)    \
  EXTI_LINE(15)

#define LINE exti_line_defs[gpio_pin_defs[pin].pin_index]

static void exti_set_source(gpio_pins_t pin) {
  switch ((uint32_t)gpio_pin_defs[pin].port) {
#if defined(GPIOA)
  case (uint32_t)GPIOA:
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LINE.syscfg_exti_line);
    break;
#endif

#if defined(GPIOB)
  case (uint32_t)GPIOB:
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LINE.syscfg_exti_line);
    break;
#endif

#if defined(GPIOC)
  case (uint32_t)GPIOC:
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LINE.syscfg_exti_line);
    break;
#endif

#if defined(GPIOD)
  case (uint32_t)GPIOD:
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, LINE.syscfg_exti_line);
    break;
#endif

#if defined(GPIOE)
  case (uint32_t)GPIOE:
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LINE.syscfg_exti_line);
    break;
#endif

#if defined(GPIOF)
  case (uint32_t)GPIOF:
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTF, LINE.syscfg_exti_line);
    break;
#endif

#if defined(GPIOG)
  case (uint32_t)GPIOG:
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTG, LINE.syscfg_exti_line);
    break;
#endif

#if defined(GPIOH)
  case (uint32_t)GPIOH:
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTH, LINE.syscfg_exti_line);
    break;
#endif

#if defined(GPIOI)
  case (uint32_t)GPIOI:
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTI, LINE.syscfg_exti_line);
    break;
#endif

#if defined(GPIOJ)
  case (uint32_t)GPIOJ:
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTJ, LINE.syscfg_exti_line);
    break;
#endif

#if defined(GPIOK)
  case (uint32_t)GPIOK:
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTK, LINE.syscfg_exti_line);
    break;
#endif
  }
}

void exti_enable(gpio_pins_t pin, uint32_t trigger) {
  exti_set_source(pin);

  LL_EXTI_ClearFlag_0_31(LINE.exti_line);

  LL_EXTI_InitTypeDef exti_init;
  exti_init.Line_0_31 = LINE.exti_line;
  exti_init.LineCommand = ENABLE;
  exti_init.Mode = LL_EXTI_MODE_IT;
  exti_init.Trigger = trigger;
  LL_EXTI_Init(&exti_init);

  LL_EXTI_EnableIT_0_31(LINE.exti_line);

  interrupt_enable(LINE.exti_irqn, EXTI_PRIORITY);
}

void exti_interrupt_enable(gpio_pins_t pin) {
  interrupt_enable(LINE.exti_irqn, EXTI_PRIORITY);
}

void exti_interrupt_disable(gpio_pins_t pin) {
  interrupt_disable(LINE.exti_irqn);
}

bool exti_line_active(gpio_pins_t pin) {
  if (LL_EXTI_IsActiveFlag_0_31(LINE.exti_line) != RESET) {
    LL_EXTI_ClearFlag_0_31(LINE.exti_line);
    return true;
  }
  return false;
}

static void handle_exit_isr() {
  // handle specific exti lines here

#if defined(USE_SX127X) && defined(SX12XX_DIO0_PIN)
  if (exti_line_active(SX12XX_DIO0_PIN)) {
    extern void sx127x_handle_exti();
    sx127x_handle_exti();
  }
#endif

#if defined(USE_SX128X) && defined(SX12XX_DIO0_PIN)
  if (exti_line_active(SX12XX_DIO0_PIN)) {
    extern void sx128x_handle_dio0_exti(bool);
    sx128x_handle_dio0_exti(gpio_pin_read(SX12XX_DIO0_PIN));
  }
#endif

#if defined(USE_SX128X) && defined(USE_SX128X_BUSY_EXTI) && defined(SX12XX_BUSY_PIN)
  if (exti_line_active(SX12XX_BUSY_PIN)) {
    extern void sx128x_handle_busy_exti(bool);
    sx128x_handle_busy_exti(gpio_pin_read(SX12XX_BUSY_PIN));
  }
#endif

#if defined(USE_A7105) && defined(A7105_GIO1_PIN)
  if (exti_line_active(A7105_GIO1_PIN)) {
    extern void a7105_handle_busy_exti(bool);
    a7105_handle_busy_exti(gpio_pin_read(A7105_GIO1_PIN));
  }
#endif
}

void EXTI0_IRQHandler() {
  handle_exit_isr();
}
void EXTI1_IRQHandler() {
  handle_exit_isr();
}
void EXTI2_IRQHandler() {
  handle_exit_isr();
}
void EXTI3_IRQHandler() {
  handle_exit_isr();
}
void EXTI4_IRQHandler() {
  handle_exit_isr();
}
void EXTI9_5_IRQHandler() {
  handle_exit_isr();
}
void EXTI15_10_IRQHandler() {
  handle_exit_isr();
}

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)

#define EXTI5_IRQn EXTI9_5_IRQn
#define EXTI6_IRQn EXTI9_5_IRQn
#define EXTI7_IRQn EXTI9_5_IRQn
#define EXTI8_IRQn EXTI9_5_IRQn
#define EXTI9_IRQn EXTI9_5_IRQn
#define EXTI10_IRQn EXTI15_10_IRQn
#define EXTI11_IRQn EXTI15_10_IRQn
#define EXTI12_IRQn EXTI15_10_IRQn
#define EXTI13_IRQn EXTI15_10_IRQn
#define EXTI14_IRQn EXTI15_10_IRQn
#define EXTI15_IRQn EXTI15_10_IRQn

#endif

#define EXTI_LINE(num)                              \
  {                                                 \
      .index = num,                                 \
      .exti_line = LL_EXTI_LINE_##num,              \
      .syscfg_exti_line = LL_SYSCFG_EXTI_LINE##num, \
      .exti_irqn = EXTI##num##_IRQn,                \
  },

const exti_line_def_t exti_line_defs[16] = {EXTI_LINES};

#undef EXTI_LINE