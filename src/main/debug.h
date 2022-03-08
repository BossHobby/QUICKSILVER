#pragma once

#include <cbor.h>

#include "project.h"

typedef struct {
  float vbat_compensated;
  float adcfilt;
  float totaltime;
  float timefilt;
  float adcreffilt;
  float cpu_load;
  float max_cpu_load;
  unsigned int max_cpu_loop_number;
  unsigned int loops_between_max_cpu_load;
  float min_cpu_load;
  unsigned int min_cpu_loop_number;
  unsigned int loops_between_min_cpu_load;
} debug_t;

typedef enum {
  PERF_COUNTER_TOTAL,
  PERF_COUNTER_GYRO,
  PERF_COUNTER_CONTROL,
  PERF_COUNTER_RX,
  PERF_COUNTER_OSD,
  PERF_COUNTER_MISC,
  PERF_COUNTER_BLACKBOX,

  PERF_COUNTER_MAX
} perf_counters_t;

typedef struct {
  uint32_t min;
  uint32_t max;
  uint32_t current;
} perf_counter_t;

extern debug_t debug;
extern perf_counter_t perf_counters[PERF_COUNTER_MAX];

void perf_counter_start(perf_counters_t counter);
void perf_counter_end(perf_counters_t counter);

void perf_counter_init();
void perf_counter_update();

void debug_update();

cbor_result_t cbor_encode_perf_counters(cbor_value_t *enc);