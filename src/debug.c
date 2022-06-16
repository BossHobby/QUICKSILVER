#include "debug.h"

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

#else

void perf_counter_start(perf_counters_t counter) {}
void perf_counter_end(perf_counters_t counter) {}

void perf_counter_init() {}
void perf_counter_update() {}

void debug_update() {}

#endif
