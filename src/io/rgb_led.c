#include "io/rgb_led.h"

#include <math.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/rgb_led.h"
#include "driver/time.h"
#include "flight/control.h"
#include "util/util.h"

#ifdef USE_RGB_LED

uint32_t led_values[RGB_LED_MAX];

static uint32_t rgb_led_breathe(uint32_t color, uint32_t period_ms, float phase_offset) {
  const uint32_t time = (time_micros() / 1000) % period_ms;
  const float phase = ((float)time / period_ms * 2.0f * M_PI) + phase_offset;
  const float brightness = (sinf(phase) + 1.0f) / 2.0f;
  
  const uint8_t r = ((color >> 8) & 0xff) * brightness;
  const uint8_t g = ((color >> 16) & 0xff) * brightness;
  const uint8_t b = ((color >> 0) & 0xff) * brightness;
  
  return RGB(r, g, b);
}

void rgb_led_update() {
  // Early exit if RGB LED pin not configured or count is zero
  if (target.rgb_led == PIN_NONE || profile.rgb_led.count == 0) {
    return;
  }

  uint32_t new_color = 0;
  uint32_t period_ms = 2000; // Default breathing period (2 seconds)

  // Priority order: critical errors first, then warnings, then normal states
  if (flags.lowbatt) {
    // Fast breathing red for low battery warning
    new_color = RGB(255, 0, 0); // Red
    period_ms = 800;
  } else if (flags.failsafe) {
    // Fast breathing for failsafe
    new_color = RGB(255, 0, 255); // Magenta
    period_ms = 600;
  } else if (flags.rx_mode == RXMODE_BIND) {
    // Very fast breathing for bind mode
    new_color = RGB(0, 255, 255); // Cyan
    period_ms = 400;
  } else if (flags.arm_switch && (flags.throttle_safety == 1 || flags.arm_safety == 1)) {
    // Medium breathing for safety warning
    new_color = RGB(255, 165, 0); // Orange
    period_ms = 1000;
  } else if (flags.arm_state) {
    // Slow breathing for armed state
    new_color = profile.rgb_led.color_armed;
    period_ms = 2500;
  } else {
    // Slow breathing for disarmed/normal state
    new_color = profile.rgb_led.color_disarmed;
    period_ms = 3000;
  }

  for (uint32_t i = 0; i < profile.rgb_led.count; i++) {
    const float phase_offset = (float)i / (profile.rgb_led.count - 1) * M_PI / 4.0f;
    led_values[i] = rgb_led_breathe(new_color, period_ms, phase_offset);
  }

  rgb_led_send(profile.rgb_led.count);
}

void rgb_led_set(uint32_t index, uint32_t value) {
  if (index < profile.rgb_led.count && index < RGB_LED_MAX) {
    led_values[index] = value;
  }
}

#else

void rgb_led_update() {
}

void rgb_led_set(uint32_t index, uint32_t value) {
  (void)index;
  (void)value;
}

#endif