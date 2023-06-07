#include "driver/timer.h"

#include "driver/rcc.h"

const timer_def_t timer_defs[TIMER_MAX] = {
    [TIMER_INVALID] = {},
    [TIMER1] = {
        .rcc = RCC_ENCODE(TMR1),
        .irq = TMR1_OVF_TMR10_IRQn,
        .instance = TMR1,
    },
    [TIMER2] = {
        .rcc = RCC_ENCODE(TMR2),
        .irq = TMR2_GLOBAL_IRQn,
        .instance = TMR2,
    },
    [TIMER3] = {
        .rcc = RCC_ENCODE(TMR3),
        .irq = TMR3_GLOBAL_IRQn,
        .instance = TMR3,
    },
    [TIMER4] = {
        .rcc = RCC_ENCODE(TMR4),
        .irq = TMR4_GLOBAL_IRQn,
        .instance = TMR4,
    },
    [TIMER5] = {
        .rcc = RCC_ENCODE(TMR5),
        .irq = TMR5_GLOBAL_IRQn,
        .instance = TMR5,
    },
    [TIMER6] = {
        .rcc = RCC_ENCODE(TMR6),
        .irq = TMR6_DAC_GLOBAL_IRQn,
        .instance = TMR6,
    },
    [TIMER7] = {
        .rcc = RCC_ENCODE(TMR7),
        .irq = TMR7_GLOBAL_IRQn,
        .instance = TMR7,
    },
    [TIMER8] = {
        .rcc = RCC_ENCODE(TMR8),
        .irq = TMR8_OVF_TMR13_IRQn,
        .instance = TMR8,
    },
    [TIMER9] = {
        .rcc = RCC_ENCODE(TMR9),
        .irq = TMR1_BRK_TMR9_IRQn,
        .instance = TMR9,
    },
    [TIMER10] = {
        .rcc = RCC_ENCODE(TMR10),
        .irq = TMR1_OVF_TMR10_IRQn,
        .instance = TMR10,
    },
    [TIMER11] = {
        .rcc = RCC_ENCODE(TMR11),
        .irq = TMR1_TRG_HALL_TMR11_IRQn,
        .instance = TMR11,
    },
    [TIMER12] = {
        .rcc = RCC_ENCODE(TMR12),
        .irq = TMR8_BRK_TMR12_IRQn,
        .instance = TMR12,
    },
    [TIMER13] = {
        .rcc = RCC_ENCODE(TMR13),
        .irq = TMR8_OVF_TMR13_IRQn,
        .instance = TMR13,
    },
    [TIMER14] = {
        .rcc = RCC_ENCODE(TMR14),
        .irq = TMR8_TRG_HALL_TMR14_IRQn,
        .instance = TMR14,
    },
};

void timer_up_init(timer_index_t tim, uint16_t divider, uint32_t period) {
  const timer_def_t *def = &timer_defs[tim];

  rcc_enable(def->rcc);

  tmr_base_init(def->instance, period, divider - 1);
  tmr_clock_source_div_set(def->instance, TMR_CLOCK_DIV1);
  tmr_cnt_dir_set(def->instance, TMR_COUNT_UP);
  tmr_period_buffer_enable(def->instance, TRUE);
}

uint32_t timer_channel_val(timer_channel_t chan) {
  switch (chan) {
  case TIMER_CH1:
    return TMR_SELECT_CHANNEL_1;
  case TIMER_CH1N:
    return TMR_SELECT_CHANNEL_1C;
  case TIMER_CH2:
    return TMR_SELECT_CHANNEL_2;
  case TIMER_CH2N:
    return TMR_SELECT_CHANNEL_2C;
  case TIMER_CH3:
    return TMR_SELECT_CHANNEL_3;
  case TIMER_CH3N:
    return TMR_SELECT_CHANNEL_3C;
  case TIMER_CH4:
    return TMR_SELECT_CHANNEL_4;
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

void TMR1_OVF_TMR10_IRQHandler() {
  timer_irq_handler();
}
void TMR2_GLOBAL_IRQHandler() {
  timer_irq_handler();
}
void TMR3_GLOBAL_IRQHandler() {
  timer_irq_handler();
}
void TMR4_GLOBAL_IRQHandler() {
  timer_irq_handler();
}
void TMR5_GLOBAL_IRQHandler() {
  timer_irq_handler();
}
void TMR6_DAC_GLOBAL_IRQHandler() {
  timer_irq_handler();
}
void TMR7_GLOBAL_IRQHandler() {
  timer_irq_handler();
}
void TMR8_OVF_TMR13_IRQHandler() {
  timer_irq_handler();
}
void TMR1_BRK_TMR9_IRQHandler() {
  timer_irq_handler();
}
void TMR1_TRG_HALL_TMR11_IRQHandler() {
  timer_irq_handler();
}
void TMR8_BRK_TMR12_IRQHandler() {
  timer_irq_handler();
}
void TMR8_TRG_HALL_TMR14_IRQHandler() {
  timer_irq_handler();
}