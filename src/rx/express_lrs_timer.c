#include "rx/express_lrs.h"

#include <stdbool.h>

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/timer.h"
#include "util/util.h"

#if defined(USE_RX_SPI_EXPRESS_LRS)

#define TIMER timer_defs[TIMER_TAG_TIM(timer_tag)]
#define TIMER_HZ 1000000

static resource_tag_t timer_tag;

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

  LL_TIM_DisableIT_UPDATE(TIMER.instance);
  LL_TIM_DisableCounter(TIMER.instance);
  LL_TIM_SetCounter(TIMER.instance, 0);
}

void elrs_timer_init(uint32_t interval_us) {
  current_interval = interval_us;

  if (timer_tag == RES_SERIAL_INVALID) {
    timer_tag = timer_alloc(TIMER_USE_ELRS);
  }
  timer_up_init(TIMER_TAG_TIM(timer_tag), PWM_CLOCK_FREQ_HZ / TIMER_HZ, (current_interval >> 1) - 1);
  interrupt_enable(TIMER.irq, TIMER_PRIORITY);

  elrs_timer_stop();
}

void elrs_timer_resume(uint32_t interval_us) {
  current_interval = interval_us;
  is_tick = false;
  is_running = true;

  LL_TIM_SetAutoReload(TIMER.instance, (current_interval >> 1) - 1);
  LL_TIM_SetCounter(TIMER.instance, 0);

  LL_TIM_ClearFlag_UPDATE(TIMER.instance);
  LL_TIM_EnableIT_UPDATE(TIMER.instance);

  LL_TIM_EnableCounter(TIMER.instance);
  LL_TIM_GenerateEvent_UPDATE(TIMER.instance);
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

void elrs_timer_irq_handler() {
  if (timer_tag == RESOURCE_INVALID) {
    return;
  }

  if (!LL_TIM_IsActiveFlag_UPDATE(TIMER.instance)) {
    return;
  }

  LL_TIM_ClearFlag_UPDATE(TIMER.instance);
  if (is_tick) {
    const uint32_t adjusted_period = ((current_interval >> 1) + timer_freq_offset - 1);
    LL_TIM_SetAutoReload(TIMER.instance, adjusted_period);

    elrs_handle_tick();
  } else {
    const uint32_t adjusted_period = ((current_interval >> 1) + phase_shift + timer_freq_offset - 1);
    LL_TIM_SetAutoReload(TIMER.instance, adjusted_period);
    phase_shift = 0;

    elrs_handle_tock();
  }

  is_tick = !is_tick;
}

#endif