#include "driver/time.h"

#include <time.h>

#include "core/project.h"

// Run-time stats clock for the FreeRTOS kernel (FreeRTOSConfig.h). Uses the
// real monotonic clock even under PIO_UNIT_TESTING, where time_cycles() may
// be a frozen mock: scheduler tests need a live clock for idle-share deltas.
// Scaled to the time_cycles() domain so window math (US_TO_CYCLES) matches
// the MCU DWT clock; at the native 500 MHz scale this wraps in ~8.6 s, so a
// multi-second stall can produce one bogus sample before the window recovers.
static uint32_t rtos_monotonic_clock(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return US_TO_CYCLES(1000000UL * (uint64_t)(ts.tv_sec) + (uint64_t)(ts.tv_nsec) / 1000UL);
}

extern "C" uint32_t rtos_runtime_clock(void) {
  return rtos_monotonic_clock();
}

void time_init() {
}

#ifdef PIO_UNIT_TESTING

static uint32_t test_time_us = 0;

void time_test_reset(void) {
  test_time_us = 0;
}

void time_test_set_us(uint32_t time_us) {
  test_time_us = time_us;
}

void time_test_advance_us(uint32_t delta_us) {
  test_time_us += delta_us;
}

uint32_t time_cycles() {
  return US_TO_CYCLES(test_time_us);
}

uint32_t time_micros() {
  return test_time_us;
}

uint32_t time_millis() {
  return test_time_us / 1000U;
}

#else

uint32_t time_cycles() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return 500000000UL * (uint64_t)(ts.tv_sec) + (uint64_t)(ts.tv_nsec) / 2UL;
}

uint32_t time_micros() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return 1000000UL * (uint64_t)(ts.tv_sec) + (uint64_t)(ts.tv_nsec) / 1000UL;
}

uint32_t time_millis() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return 1000UL * (uint64_t)(ts.tv_sec) + (uint64_t)(ts.tv_nsec) / 1000000UL;
}

#endif

void time_delay_us(uint32_t us) {
#ifdef PIO_UNIT_TESTING
  time_test_advance_us(us);
#else
  volatile uint32_t delay = US_TO_CYCLES(us);
  volatile uint32_t start = time_cycles();
  while (time_cycles() - start < delay) {
    __NOP();
  }
#endif
}

void time_delay_ms(uint32_t ms) {
  while (ms--)
    time_delay_us(1000);
}
