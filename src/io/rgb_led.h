#pragma once
void rgb_led_lvc();
#define RGB(r, g, b) ((((int)g & 0xff) << 16) | (((int)r & 0xff) << 8) | ((int)b & 0xff))
#define RGB5BIT(r, g, b) ((((int)(g>>3) & 0x1f) << 10) | (((int)(r>>3) & 0x1f) << 5) | ((int)(b>>3) & 0x1f))
#define RGB5TO8BIT(rgb) ((((int)rgb & 0x1f) << 3) | (((int)rgb & 0x3e0) << 6) | (((int)rgb & 0x7c00) << 9))