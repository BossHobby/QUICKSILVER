#include "driver/timer.h"

#include <FreeRTOS.h>
#include <hardware/pwm.h>
#include <task.h>

// Each PWM slice is a 16-bit up counter; all route their wrap interrupt to
// PWM_IRQ_WRAP_0.
#define TIMER(_num)                       \
  [TIMER##_num] = {                       \
      .rcc = RESETS_RESET_PWM_BITS,       \
      .irq = PWM_IRQ_WRAP_0_IRQn,         \
      .instance = &pwm_hw->slice[_num - 1], \
  },
const timer_def_t timer_defs[TIMER_MAX] = {
    [TIMER_INVALID] = {},
    TIMERS};
#undef TIMER

extern bool flight_timer_irq_handler();

static uint32_t timer_slice(timer_index_t tim) {
  return tim - TIMER1;
}

void timer_up_init(timer_index_t tim, uint16_t divider, uint32_t period) {
  const uint32_t slice = timer_slice(tim);
  rcc_enable(timer_defs[tim].rcc);
  pwm_set_enabled(slice, false);
  pwm_set_clkdiv_int_frac4(slice, divider, 0);
  pwm_set_wrap(slice, period);
  pwm_set_counter(slice, 0);
}

// TOP is double-buffered and takes effect at the next wrap.
void timer_up_set_period(timer_index_t tim, uint32_t period) {
  pwm_set_wrap(timer_slice(tim), period);
}

uint32_t timer_up_count(timer_index_t tim) {
  return pwm_get_counter(timer_slice(tim));
}

void timer_up_start(timer_index_t tim) {
  const uint32_t slice = timer_slice(tim);
  pwm_clear_irq(slice);
  pwm_set_irq0_enabled(slice, true);
  pwm_set_enabled(slice, true);
}

bool timer_up_pending(timer_index_t tim) {
  const uint32_t slice = timer_slice(tim);
  if (!(pwm_get_irq0_status_mask() & (1u << slice)))
    return false;
  pwm_clear_irq(slice);
  return true;
}

extern "C" void PWM_IRQ_WRAP_0_Handler() {
  const bool wake = flight_timer_irq_handler();
  portYIELD_FROM_ISR(wake);
}
