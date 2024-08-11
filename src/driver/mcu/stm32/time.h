#pragma once

static inline uint32_t time_cycles() {
  return DWT->CYCCNT;
}

static inline uint32_t time_millis() {
  extern volatile uint32_t systick_count;
  return systick_count;
}