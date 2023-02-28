#pragma once

#include <stdint.h>

#define LEDALL 15

void led_init();
void led_on(uint8_t val);
void led_off(uint8_t val);
void led_flash(uint32_t period, int duty);

void led_update();

void led_pwm(uint8_t pwmval, float looptime);
