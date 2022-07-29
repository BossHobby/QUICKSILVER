#include "debug.h"

#include "drv_gpio.h"
#include "drv_time.h"
#include "flight/control.h"
#include "util/cbor_helper.h"

#ifdef DEBUG

#define SKIP_LOOPS 20000

perf_counter_t perf_counters[PERF_COUNTER_MAX];

static uint32_t perf_counter_start_time[PERF_COUNTER_MAX];
static uint32_t loop_counter = 0;

void perf_counter_init() {
  for (uint32_t i = 0; i < PERF_COUNTER_MAX; i++) {
    perf_counters[i].min = UINT32_MAX;
    perf_counters[i].max = 0;
    perf_counters[i].current = 0;
  }
}

void perf_counter_start(perf_counters_t counter) {
  perf_counter_start_time[counter] = time_cycles();
}

void perf_counter_end(perf_counters_t counter) {
  if (loop_counter < SKIP_LOOPS) {
    return;
  }

  uint32_t delta = time_cycles() - perf_counter_start_time[counter];

  if (delta > perf_counters[counter].max) {
    perf_counters[counter].max = delta;
  }

  if (delta < perf_counters[counter].min) {
    perf_counters[counter].min = delta;
  }

  perf_counters[counter].current = delta;
}

void perf_counter_update() {
  loop_counter++;
}

#define ENCODE_CYCLES(val)                                     \
  {                                                            \
    const uint32_t us = (val) / (SYS_CLOCK_FREQ_HZ / 1000000); \
    CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &us));      \
  }

cbor_result_t cbor_encode_perf_counters(cbor_value_t *enc) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_array_indefinite(enc));

  for (uint32_t i = 0; i < PERF_COUNTER_MAX; i++) {
    CBOR_CHECK_ERROR(res = cbor_encode_map_indefinite(enc));

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "min"));
    ENCODE_CYCLES(perf_counters[i].min)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "max"));
    ENCODE_CYCLES(perf_counters[i].max)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "current"));
    ENCODE_CYCLES(perf_counters[i].current)

    CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));
  }

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

void debug_pin_init() {
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;

#ifdef DEBUG_PIN0
  gpio_pin_init(&gpio_init, DEBUG_PIN0);
#endif

#ifdef DEBUG_PIN1
  gpio_pin_init(&gpio_init, DEBUG_PIN1);
#endif
}

void debug_pin_enable(uint8_t index) {
#ifdef DEBUG_PIN0
  if (index == 0) {
    gpio_pin_set(DEBUG_PIN0);
  }
#endif

#ifdef DEBUG_PIN1
  if (index == 1) {
    gpio_pin_set(DEBUG_PIN1);
  }
#endif
}

void debug_pin_disable(uint8_t index) {
#ifdef DEBUG_PIN0
  if (index == 0) {
    gpio_pin_reset(DEBUG_PIN0);
  }
#endif

#ifdef DEBUG_PIN1
  if (index == 1) {
    gpio_pin_reset(DEBUG_PIN1);
  }
#endif
}

void debug_pin_toggle(uint8_t index) {
#ifdef DEBUG_PIN0
  if (index == 0) {
    gpio_pin_toggle(DEBUG_PIN0);
  }
#endif

#ifdef DEBUG_PIN1
  if (index == 1) {
    gpio_pin_toggle(DEBUG_PIN1);
  }
#endif
}

#else

void perf_counter_start(perf_counters_t counter) {}
void perf_counter_end(perf_counters_t counter) {}

void perf_counter_init() {}
void perf_counter_update() {}

void debug_pin_init() {}

void debug_pin_enable(uint8_t index) {}
void debug_pin_disable(uint8_t index) {}
void debug_pin_toggle(uint8_t index) {}

#endif
