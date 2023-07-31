#include "driver/time.h"

#include <time.h>

#include "core/project.h"

void time_init() {
}

uint32_t time_cycles() {
  struct timespec ts;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
  return 500000000UL * (uint64_t)(ts.tv_sec) + (uint64_t)(ts.tv_nsec) / 2UL;
}

uint32_t time_micros() {
  struct timespec ts;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
  return 1000000UL * (uint64_t)(ts.tv_sec) + (uint64_t)(ts.tv_nsec) / 1000UL;
}

uint32_t time_millis() {
  struct timespec ts;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
  return 1000UL * (uint64_t)(ts.tv_sec) + (uint64_t)(ts.tv_nsec) / 1000000UL;
}

void time_delay_us(uint32_t us) {
  volatile uint32_t delay = US_TO_CYCLES(us);
  volatile uint32_t start = time_cycles();
  while (time_cycles() - start < delay) {
    __NOP();
  }
}

void time_delay_ms(uint32_t ms) {
  while (ms--)
    time_delay_us(1000);
}
