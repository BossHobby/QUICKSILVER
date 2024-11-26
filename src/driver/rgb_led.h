#pragma once

#include <stdbool.h>
#include <stdint.h>

#define RGB_LED_MAX 32

void rgb_led_init();
void rgb_led_set_value(uint32_t value, uint32_t count);
void rgb_led_send();
bool rgb_led_busy();