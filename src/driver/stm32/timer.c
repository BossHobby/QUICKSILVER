#include "driver/timer.h"

#include "driver/rcc.h"

#define TIMER_ASSIGMENT_MAX 32

const timer_def_t timer_defs[TIMER_MAX] = {
    [TIMER_INVALID] = {},
    [TIMER1] = {
        .rcc = RCC_APB2_GRP1(TIM1),
        .irq = TIM1_UP_TIM10_IRQn,
        .instance = TIM1,
    },
    [TIMER2] = {
        .rcc = RCC_APB1_GRP1(TIM2),
        .irq = TIM2_IRQn,
        .instance = TIM2,
    },
    [TIMER3] = {
        .rcc = RCC_APB1_GRP1(TIM3),
        .irq = TIM3_IRQn,
        .instance = TIM3,
    },
    [TIMER4] = {
        .rcc = RCC_APB1_GRP1(TIM4),
        .irq = TIM4_IRQn,
        .instance = TIM4,
    },
    [TIMER5] = {
        .rcc = RCC_APB1_GRP1(TIM5),
        .irq = TIM5_IRQn,
        .instance = TIM5,
    },
#ifndef STM32F411
    [TIMER6] = {
        .rcc = RCC_APB1_GRP1(TIM6),
        .irq = TIM6_DAC_IRQn,
        .instance = TIM6,
    },
    [TIMER7] = {
        .rcc = RCC_APB1_GRP1(TIM7),
        .irq = TIM7_IRQn,
        .instance = TIM7,
    },
    [TIMER8] = {
        .rcc = RCC_APB2_GRP1(TIM8),
        .irq = TIM8_UP_TIM13_IRQn,
        .instance = TIM8,
    },
#endif
#ifndef STM32H7
    [TIMER9] = {
        .rcc = RCC_APB2_GRP1(TIM9),
        .irq = TIM1_BRK_TIM9_IRQn,
        .instance = TIM9,
    },
    [TIMER10] = {
        .rcc = RCC_APB2_GRP1(TIM10),
        .irq = TIM1_UP_TIM10_IRQn,
        .instance = TIM10,
    },
    [TIMER11] = {
        .rcc = RCC_APB2_GRP1(TIM11),
        .irq = TIM1_TRG_COM_TIM11_IRQn,
        .instance = TIM11,
    },
#endif
#ifndef STM32F411
    [TIMER12] = {
        .rcc = RCC_APB1_GRP1(TIM12),
        .irq = TIM8_BRK_TIM12_IRQn,
        .instance = TIM12,
    },
    [TIMER13] = {
        .rcc = RCC_APB1_GRP1(TIM13),
        .irq = TIM8_UP_TIM13_IRQn,
        .instance = TIM13,
    },
    [TIMER14] = {
        .rcc = RCC_APB1_GRP1(TIM14),
        .irq = TIM8_TRG_COM_TIM14_IRQn,
        .instance = TIM14,
    },
#endif
#ifdef STM32H743
    [TIMER15] = {
        .rcc = RCC_APB2_GRP1(TIM15),
        .irq = TIM15_IRQn,
        .instance = TIM15,
    },
    [TIMER16] = {
        .rcc = RCC_APB2_GRP1(TIM16),
        .irq = TIM16_IRQn,
        .instance = TIM16,
    },
    [TIMER17] = {
        .rcc = RCC_APB2_GRP1(TIM17),
        .irq = TIM17_IRQn,
        .instance = TIM17,
    },
#endif
};

void timer_up_init(timer_index_t tim, uint16_t divider, uint32_t period) {
  rcc_enable(timer_defs[tim].rcc);

  LL_TIM_InitTypeDef tim_init;
  tim_init.Prescaler = divider - 1;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  tim_init.Autoreload = period;
  tim_init.ClockDivision = 0;
  tim_init.RepetitionCounter = 0;
  LL_TIM_Init(timer_defs[tim].instance, &tim_init);
}

uint32_t timer_channel_val(timer_channel_t chan) {
  switch (chan) {
  case TIMER_CH1:
    return LL_TIM_CHANNEL_CH1;
  case TIMER_CH1N:
    return LL_TIM_CHANNEL_CH1N;
  case TIMER_CH2:
    return LL_TIM_CHANNEL_CH2;
  case TIMER_CH2N:
    return LL_TIM_CHANNEL_CH2N;
  case TIMER_CH3:
    return LL_TIM_CHANNEL_CH3;
  case TIMER_CH3N:
    return LL_TIM_CHANNEL_CH3N;
  case TIMER_CH4:
    return LL_TIM_CHANNEL_CH4;
  default:
    return 0;
  }
}

static void timer_irq_handler() {
  extern void soft_serial_timer_irq_handler();
  soft_serial_timer_irq_handler();

#if defined(USE_RX_SPI_EXPRESS_LRS)
  extern void elrs_timer_irq_handler();
  elrs_timer_irq_handler();
#endif
}

#ifdef STM32H7
#undef TIM1_BRK_TIM9_IRQHandler
#undef TIM1_TRG_COM_TIM11_IRQHandler
#endif

void TIM15_IRQHandler() {
  timer_irq_handler();
}
void TIM16_IRQHandler() {
  timer_irq_handler();
}
void TIM17_IRQHandler() {
  timer_irq_handler();
}
void TIM1_BRK_IRQHandler() {
  timer_irq_handler();
}
void TIM1_BRK_TIM9_IRQHandler() {
  timer_irq_handler();
}
void TIM1_CC_IRQHandler() {
  timer_irq_handler();
}
void TIM1_TRG_COM_IRQHandler() {
  timer_irq_handler();
}
void TIM1_TRG_COM_TIM11_IRQHandler() {
  timer_irq_handler();
}
void TIM1_UP_IRQHandler() {
  timer_irq_handler();
}
void TIM1_UP_TIM10_IRQHandler() {
  timer_irq_handler();
}
void TIM2_IRQHandler() {
  timer_irq_handler();
}
void TIM3_IRQHandler() {
  timer_irq_handler();
}
void TIM4_IRQHandler() {
  timer_irq_handler();
}
void TIM5_IRQHandler() {
  timer_irq_handler();
}
void TIM6_DAC_IRQHandler() {
  timer_irq_handler();
}
void TIM7_IRQHandler() {
  timer_irq_handler();
}
void TIM8_BRK_TIM12_IRQHandler() {
  timer_irq_handler();
}
void TIM8_CC_IRQHandler() {
  timer_irq_handler();
}
void TIM8_TRG_COM_TIM14_IRQHandler() {
  timer_irq_handler();
}
void TIM8_UP_TIM13_IRQHandler() {
  timer_irq_handler();
}
