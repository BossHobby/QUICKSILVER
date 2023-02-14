#pragma once

#include <cbor.h>

#include "project.h"

#include "io/usb_configurator.h"

typedef enum {
  PERF_COUNTER_TOTAL,
  PERF_COUNTER_GYRO,
  PERF_COUNTER_CONTROL,
  PERF_COUNTER_RX,
  PERF_COUNTER_OSD,
  PERF_COUNTER_MISC,
  PERF_COUNTER_BLACKBOX,
  PERF_COUNTER_DEBUG,

  PERF_COUNTER_MAX
} perf_counters_t;

typedef struct {
  uint32_t min;
  uint32_t max;
  uint32_t current;
} perf_counter_t;

extern perf_counter_t perf_counters[PERF_COUNTER_MAX];

void perf_counter_start(perf_counters_t counter);
void perf_counter_end(perf_counters_t counter);

void perf_counter_init();
void perf_counter_update();

cbor_result_t cbor_encode_perf_counters(cbor_value_t *enc);

void debug_pin_init();

void debug_pin_enable(uint8_t index);
void debug_pin_disable(uint8_t index);
void debug_pin_toggle(uint8_t index);

#if defined(DEBUG) && defined(DEBUG_LOGGING)
#define quic_debugf(args...) usb_quic_logf(args)
#else
#define quic_debugf(args...) __NOP()
#endif