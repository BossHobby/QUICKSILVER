#pragma once

#include <stdint.h>

void time_init();

uint32_t time_cycles();

uint32_t time_micros();
uint32_t time_millis();

void time_delay_us(uint32_t us);
void time_delay_ms(uint32_t ms);