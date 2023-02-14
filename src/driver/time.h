#pragma once

#include <stdint.h>

#define TICKS_PER_US (SYS_CLOCK_FREQ_HZ / 1000000)

void time_init();

uint32_t time_cycles();

uint32_t time_micros();
uint32_t time_millis();

void time_delay_us(uint32_t us);
void time_delay_ms(uint32_t ms);