#include "debug.h"

#include "drv_time.h"
#include "util/cbor_helper.h"

#ifdef DEBUG

#define SKIP_LOOPS 20000

debug_type debug;
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
  perf_counter_start_time[counter] = timer_cycles();
}

void perf_counter_end(perf_counters_t counter) {
  if (loop_counter < SKIP_LOOPS) {
    return;
  }

  uint32_t delta = timer_cycles() - perf_counter_start_time[counter];

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

cbor_result_t cbor_encode_perf_counters(cbor_value_t *enc) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_array_indefinite(enc));

  for (uint32_t i = 0; i < PERF_COUNTER_MAX; i++) {
    CBOR_CHECK_ERROR(res = cbor_encode_map_indefinite(enc));

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "min"));
    CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &perf_counters[i].min));

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "max"));
    CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &perf_counters[i].max));

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "current"));
    CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &perf_counters[i].current));

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

#endif
