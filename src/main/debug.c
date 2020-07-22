#include "debug.h"

#include "drv_time.h"

#ifdef DEBUG

debug_type debug;
perf_counter_t perf_counters[PERF_COUNTER_MAX];

static uint32_t perf_counter_start_time[PERF_COUNTER_MAX];

void perf_counter_start(perf_counters_t counter) {
  perf_counter_start_time[counter] = timer_micros();
}

void perf_counter_end(perf_counters_t counter) {
  uint32_t delta = timer_micros() - perf_counter_start_time[counter];

  if (delta > perf_counters[counter].max) {
    perf_counters[counter].max = delta;
  }
  if (delta < perf_counters[counter].min) {
    perf_counters[counter].min = delta;
  }

  perf_counters[counter].current = delta;
}

#else

void perf_counter_start(perf_counters_t counter) {
}

void perf_counter_end(perf_counters_t counter) {
}

#endif
