#pragma once

#include <stdint.h>

#define LEDALL 15

void ledon(uint8_t val);
void ledoff(uint8_t val);
void ledflash(uint32_t period, int duty);

void led_update();

uint8_t led_pwm(uint8_t pwmval);
