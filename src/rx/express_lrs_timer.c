#include "rx/express_lrs.h"

#include <stdbool.h>

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/timer.h"
#include "util/util.h"

#if defined(RX_EXPRESS_LRS) && defined(USE_SX128X)

#define TIMER_HZ 1000000
#define TIMER_INSTANCE TIM3
#define TIMER_IRQN TIM3_IRQn

static bool is_running = false;
static volatile bool is_tick = false;

static volatile int32_t phase_shift = 0;
static volatile uint32_t current_interval = 0;

volatile elrs_phase_lock_state_t pl_state;
volatile int32_t timer_freq_offset = 0;

extern elrs_timer_state_t elrs_timer_state;
extern volatile uint8_t ota_nonce;

extern void elrs_handle_tick();
extern void elrs_handle_tock();

bool elrs_timer_is_running() {
  return is_running;
}

void elrs_timer_stop() {
  is_running = false;

  LL_TIM_DisableIT_UPDATE(TIMER_INSTANCE);
  LL_TIM_DisableCounter(TIMER_INSTANCE);
  LL_TIM_SetCounter(TIMER_INSTANCE, 0);
}

void elrs_timer_init(uint32_t interval_us) {
  current_interval = interval_us;
  timer_init(TIMER_INSTANCE, PWM_CLOCK_FREQ_HZ / TIMER_HZ, (current_interval >> 1) - 1);

  interrupt_enable(TIMER_IRQN, TIMER_PRIORITY);

  elrs_timer_stop();
}

void elrs_timer_resume(uint32_t interval_us) {
  current_interval = interval_us;
  is_tick = false;
  is_running = true;

  LL_TIM_SetAutoReload(TIMER_INSTANCE, (current_interval >> 1) - 1);
  LL_TIM_SetCounter(TIMER_INSTANCE, 0);

  LL_TIM_ClearFlag_UPDATE(TIMER_INSTANCE);
  LL_TIM_EnableIT_UPDATE(TIMER_INSTANCE);

  LL_TIM_EnableCounter(TIMER_INSTANCE);
  LL_TIM_GenerateEvent_UPDATE(TIMER_INSTANCE);
}

void elrs_timer_set_phase_shift(int32_t shift) {
  const int32_t min = -(current_interval >> 2);
  const int32_t max = (current_interval >> 2);

  if (shift < min) {
    phase_shift = min;
  } else if (shift > max) {
    phase_shift = max;
  } else {
    phase_shift = shift;
  }
}

void elrs_phase_init() {
  elrs_lpf_init((elrs_lpf_t *)&pl_state.offset_lpf, 2);
  elrs_lpf_init((elrs_lpf_t *)&pl_state.offset_dx_lpf, 4);
}

void elrs_phase_int_event(uint32_t time) {
  pl_state.int_event_time_us = time;
  pl_state.int_event_active = true;
}

void elrs_phase_ext_event(uint32_t time) {
  pl_state.ext_event_time_us = time;
  pl_state.ext_event_active = true;
}

void elrs_phase_update(elrs_state_t state) {
  if (state <= DISCONNECTED) {
    return;
  }

  pl_state.raw_offset_us = 0;
  if (pl_state.ext_event_active && pl_state.int_event_active) {
    pl_state.raw_offset_us = (int32_t)(pl_state.ext_event_time_us - pl_state.int_event_time_us);
  }
  pl_state.ext_event_active = false;
  pl_state.int_event_active = false;

  pl_state.offset = elrs_lpf_update((elrs_lpf_t *)&pl_state.offset_lpf, pl_state.raw_offset_us);
  pl_state.offset_dx = elrs_lpf_update((elrs_lpf_t *)&pl_state.offset_dx_lpf, pl_state.raw_offset_us - pl_state.prev_raw_offset_us);

  if (elrs_timer_state == TIMER_LOCKED && elrs_lq_current_is_set()) {
    // limit rate of freq offset adjustment slightly
    if (ota_nonce % 8 == 0) {
      if (pl_state.offset > 0) {
        timer_freq_offset++;
      } else if (pl_state.offset < 0) {
        timer_freq_offset--;
      }
    }
  }

  if (state < CONNECTED) {
    elrs_timer_set_phase_shift(pl_state.raw_offset_us >> 1);
  } else {
    elrs_timer_set_phase_shift(pl_state.offset >> 2);
  }

  pl_state.prev_raw_offset_us = pl_state.raw_offset_us;
}

void elrs_phase_reset() {
  timer_freq_offset = 0;

  pl_state.ext_event_active = false;
  pl_state.int_event_active = false;

  pl_state.raw_offset_us = 0;
  pl_state.prev_raw_offset_us = 0;

  pl_state.offset = 0;
  pl_state.offset_dx = 0;

  elrs_lpf_init((elrs_lpf_t *)&pl_state.offset_lpf, 2);
  elrs_lpf_init((elrs_lpf_t *)&pl_state.offset_dx_lpf, 4);
}

void TIM3_IRQHandler() {
  if (LL_TIM_IsActiveFlag_UPDATE(TIMER_INSTANCE) == 1) {
    LL_TIM_ClearFlag_UPDATE(TIMER_INSTANCE);
  }

  if (is_tick) {
    const uint32_t adjusted_period = ((current_interval >> 1) + timer_freq_offset - 1);
    LL_TIM_SetAutoReload(TIMER_INSTANCE, adjusted_period);

    elrs_handle_tick();
  } else {
    const uint32_t adjusted_period = ((current_interval >> 1) + phase_shift + timer_freq_offset - 1);
    LL_TIM_SetAutoReload(TIMER_INSTANCE, adjusted_period);
    phase_shift = 0;

    elrs_handle_tock();
  }

  is_tick = !is_tick;
}

#endif