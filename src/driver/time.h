#pragma once

#include <stdint.h>

#define TICKS_PER_US (SYS_CLOCK_FREQ_HZ / 1000000)

#define US_TO_CYCLES(us) ((us) * TICKS_PER_US)
#define CYCLES_TO_US(cycles) ((cycles) / TICKS_PER_US)

void time_init();

uint32_t time_micros();

void time_delay_us(uint32_t us);
void time_delay_ms(uint32_t ms);

#ifdef PIO_UNIT_TESTING
void time_test_reset(void);
void time_test_set_us(uint32_t time_us);
void time_test_advance_us(uint32_t delta_us);
#endif
