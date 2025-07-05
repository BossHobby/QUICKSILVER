#pragma once

#include <stdbool.h>
#include <stdint.h>

#define RGB_LED_MAX 32
#define RGB_LEDS_PER_UPDATE 8

void rgb_led_init();
void rgb_led_send(uint32_t count);
bool rgb_led_busy();