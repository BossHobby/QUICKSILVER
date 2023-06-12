#pragma once

#define RGB(r, g, b) ((((uint32_t)g & 0xff) << 16) | (((uint32_t)r & 0xff) << 8) | ((uint32_t)b & 0xff))
#define RGB5BIT(r, g, b) ((((uint32_t)(g >> 3) & 0x1f) << 10) | (((uint32_t)(r >> 3) & 0x1f) << 5) | ((uint32_t)(b >> 3) & 0x1f))
#define RGB5TO8BIT(rgb) ((((uint32_t)rgb & 0x1f) << 3) | (((uint32_t)rgb & 0x3e0) << 6) | (((uint32_t)rgb & 0x7c00) << 9))

void rgb_led_update();
