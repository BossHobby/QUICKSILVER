#pragma once

#include <stdint.h>

void time_init(void);
uint32_t gettime(void);

uint32_t debug_timer_micros();
uint32_t debug_timer_millis();
void debug_timer_delay_us(uint32_t us);

void delay(uint32_t data);
