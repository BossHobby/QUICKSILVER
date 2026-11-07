#pragma once

#include "driver/time.h"

static inline uint32_t time_cycles() {
  return DWT->CYCCNT;
}

static inline uint32_t time_millis() {
  extern volatile uint32_t systick_count;
  return systick_count;
}

static inline uint32_t time_micros() {
  extern volatile uint32_t micros_base;
  const uint32_t base = micros_base;
  return base + (DWT->CYCCNT - base * TICKS_PER_US) / TICKS_PER_US;
}
