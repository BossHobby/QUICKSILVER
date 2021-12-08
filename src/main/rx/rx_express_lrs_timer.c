#include "rx_express_lrs.h"

#include <stdbool.h>

#include "drv_gpio.h"
#include "drv_timer.h"
#include "project.h"
#include "util.h"

#if defined(RX_EXPRESS_LRS) && defined(USE_SX12XX)

#define TIMER_HZ 1000000
#define TIMER_INSTANCE TIM3
#define TIMER_IRQN TIM3_IRQn

#define DEBUG_PIN PIN_A10

typedef struct {
  bool int_event_active;
  uint32_t int_event_time_us;

  bool ext_event_active;
  uint32_t ext_event_time_us;

  int32_t raw_offset_us;
  int32_t prev_raw_offset_us;

} elrs_phase_lock_state_t;

static volatile bool is_tick = false;

static volatile int32_t phase_shift = 0;
static volatile uint32_t current_interval = 0;
static volatile elrs_phase_lock_state_t pl_state;

volatile int32_t timer_freq_offset = 0;

extern void elrs_handle_tick();
extern void elrs_handle_tock();

void elrs_timer_stop() {
  LL_TIM_DisableIT_UPDATE(TIMER_INSTANCE);
  LL_TIM_DisableCounter(TIMER_INSTANCE);
  LL_TIM_SetCounter(TIMER_INSTANCE, 0);
}

void elrs_timer_init(uint32_t interval_us) {
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init(&gpio_init, DEBUG_PIN);

  current_interval = interval_us;
  timer_init(TIMER_INSTANCE, PWM_CLOCK_FREQ_HZ / TIMER_HZ, (current_interval / 2) - 1);

  NVIC_SetPriority(TIMER_IRQN, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(TIMER_IRQN);

  elrs_timer_stop();
}

void elrs_timer_resume(uint32_t interval_us) {
  current_interval = interval_us;
  is_tick = false;

  LL_TIM_SetAutoReload(TIMER_INSTANCE, (current_interval / 2) - 1);
  LL_TIM_SetCounter(TIMER_INSTANCE, 0);

  LL_TIM_ClearFlag_UPDATE(TIMER_INSTANCE);
  LL_TIM_EnableIT_UPDATE(TIMER_INSTANCE);

  LL_TIM_EnableCounter(TIMER_INSTANCE);
  LL_TIM_GenerateEvent_UPDATE(TIMER_INSTANCE);
}

void elrs_timer_set_phase_shift(int32_t shift) {
  const int32_t min = -(current_interval / 4);
  const int32_t max = (current_interval / 4);

  if (shift < min) {
    phase_shift = min;
  } else if (shift > max) {
    phase_shift = max;
  } else {
    phase_shift = shift;
  }
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

  if (state < CONNECTED) {
    elrs_timer_set_phase_shift(pl_state.raw_offset_us / 2);
  } else {
  }

  pl_state.prev_raw_offset_us = pl_state.raw_offset_us;
}

void elrs_phase_reset() {
  pl_state.ext_event_active = false;
  pl_state.int_event_active = false;

  pl_state.raw_offset_us = 0;
  pl_state.prev_raw_offset_us = 0;
}

void TIM3_IRQHandler() {
  if (LL_TIM_IsActiveFlag_UPDATE(TIMER_INSTANCE) == 1) {
    LL_TIM_ClearFlag_UPDATE(TIMER_INSTANCE);
  }

  if (is_tick) {
    gpio_pin_reset(DEBUG_PIN);

    const uint32_t adjusted_period = ((current_interval / 2) + timer_freq_offset - 1);
    LL_TIM_SetAutoReload(TIMER_INSTANCE, adjusted_period);

    elrs_handle_tick();
  } else {
    gpio_pin_set(DEBUG_PIN);

    const uint32_t adjusted_period = ((current_interval / 2) + phase_shift + timer_freq_offset - 1);
    LL_TIM_SetAutoReload(TIMER_INSTANCE, adjusted_period);
    phase_shift = 0;

    elrs_handle_tock();
  }

  is_tick = !is_tick;
}

#endif