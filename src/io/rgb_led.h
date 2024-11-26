#pragma once

#define RGB(r, g, b) ((((uint32_t)g & 0xff) << 16) | (((uint32_t)r & 0xff) << 8) | ((uint32_t)b & 0xff))

void rgb_led_update();