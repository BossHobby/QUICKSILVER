#pragma once

#include <stdint.h>

void gpio_pin_set(uint32_t pin);
void gpio_pin_reset(uint32_t pin);
void gpio_pin_toggle(uint32_t pin);
bool gpio_pin_read(uint32_t pin);
