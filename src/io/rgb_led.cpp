#include "io/rgb_led.h"

#include "control/control.h"
#include "core/project.h"
#include "driver/rgb_led.h"
#include "util/util.h"

#define RGB_LED_COUNT 0

// flashes between 2 colours, duty cycle 1 - 16
static uint32_t rgb_ledflash(uint32_t color1, uint32_t color2, uint32_t period_ms, uint8_t duty) {
  const uint32_t period = period_ms * 1000;
  const uint32_t divider = (period * duty) >> 5;
  return time_micros() % period > divider ? color1 : color2;
}

#define RGB_LED_UPDATE_PERIOD_US 10000

TickType_t rgb_led_update() {
#if defined(USE_RGB_LED)
  static uint32_t last_update_us = 0;
  const uint32_t now = time_micros();
  if (now - last_update_us < RGB_LED_UPDATE_PERIOD_US) {
    return pdMS_TO_TICKS((RGB_LED_UPDATE_PERIOD_US - (now - last_update_us)) / 1000);
  }
  last_update_us = now;

  if (RGB_LED_COUNT == 0 || rgb_led_busy()) {
    return pdMS_TO_TICKS(RGB_LED_UPDATE_PERIOD_US / 1000);
  }

  uint32_t new_value = 0;

  if (flags.lowbatt) {
    new_value = rgb_ledflash(RGB(255, 0, 0), RGB(0, 0, 255), 500, 16);
  }
  if (flags.rx_mode == RXMODE_BIND) {
    new_value = rgb_ledflash(RGB(255, 0, 0), RGB(0, 0, 255), 100, 24);
  }
  if (flags.failsafe) {
    new_value = rgb_ledflash(RGB(255, 0, 0), RGB(0, 0, 255), 500, 30);
  }
  if (flags.arm_request && flags.arming_disabled_flags != ARMING_DISABLED_NONE) {
    new_value = rgb_ledflash(RGB(255, 0, 0), RGB(0, 0, 255), 100, 8);
  }

  static uint32_t current_value = 0;
  if (current_value != new_value) {
    rgb_led_set_value(new_value, RGB_LED_COUNT);
    current_value = new_value;
  } else {
    rgb_led_send();
  }
  return pdMS_TO_TICKS(RGB_LED_UPDATE_PERIOD_US / 1000);
#else
  return portMAX_DELAY;
#endif
}
