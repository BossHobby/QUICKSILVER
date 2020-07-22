#pragma once

#include <stdint.h>

void timer_init();

uint32_t timer_cycles();

uint32_t timer_micros();
uint32_t timer_millis();

void timer_delay_us(uint32_t us);
void timer_delay_until(uint32_t us);
