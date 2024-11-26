#include "io/rgb_led.h"

#include "core/project.h"
#include "driver/rgb_led.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "util/util.h"

#define RGB_LED_COUNT 8
#define RGB_FILTER_TIME FILTERCALC(100, 1000000)

static void rgb_led_set_all(uint32_t color, float fade_coeff) {
  static float g_filt = 0;
  const uint32_t g = (color >> 16) & 0xff;
  lpf(&g_filt, g, fade_coeff);

  static float r_filt = 0;
  const uint32_t r = (color >> 8) & 0xff;
  lpf(&r_filt, r, fade_coeff);

  static float b_filt = 0;
  const uint32_t b = (color >> 0) & 0xff;
  lpf(&b_filt, b, fade_coeff);

  rgb_led_set_value(RGB(r_filt, g_filt, b_filt), RGB_LED_COUNT);
}

// flashes between 2 colours, duty cycle 1 - 16
static void rgb_ledflash(uint32_t color1, uint32_t color2, uint32_t period_ms, uint8_t duty) {
  const uint32_t period = period_ms * 1000;
  const uint32_t divider = (period * duty) >> 5;
  if (time_micros() % period > divider) {
    rgb_led_set_all(color1, lpfcalc(divider, period));
  } else {
    rgb_led_set_all(color2, lpfcalc(divider, period));
  }
}

void rgb_led_update() {
#if defined(USE_RGB_LED)
  if (rgb_led_busy()) {
    return;
  }
  if (flags.lowbatt) {
    rgb_ledflash(RGB(255, 0, 0), RGB(0, 0, 255), 500, 16);
  }
  if (flags.rx_mode == RXMODE_BIND) {
    rgb_ledflash(RGB(255, 0, 0), RGB(0, 0, 255), 100, 24);
  }
  if (flags.failsafe) {
    rgb_ledflash(RGB(255, 0, 0), RGB(0, 0, 255), 500, 30);
  }
  if (flags.arm_switch && (flags.throttle_safety == 1 || flags.arm_safety == 1)) {
    rgb_ledflash(RGB(255, 0, 0), RGB(0, 0, 255), 100, 8);
  }
  rgb_led_send();
#endif
}
