#pragma once

#include <stdint.h>

#define LEDALL 15

void led_init();
void led_on(uint8_t val);
void led_off(uint8_t val);
void led_pwm(float brightness, float looptime);

void led_flash();
void led_blink(uint8_t count);

void led_update();
