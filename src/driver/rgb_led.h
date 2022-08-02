#pragma once

#define RGB_LED_NUMBER profile.rgb.led_count
#define RGB_LED_MAX 32

void rgb_init();
void rgb_send(int data);
void rgb_dma_start();
