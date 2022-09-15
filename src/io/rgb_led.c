#include "rgb_led.h"

#include <math.h>

#include "core/profile.h"
#include "driver/rgb_led.h"
#include "driver/time.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "io/rgb_led.h"
#include "project.h"
#include "util/util.h"

#ifdef RGB_LED_DMA

typedef enum {
  RGB_STATE_DELAY,
  RGB_STATE_UPDATE,
  RGB_STATE_SEND,
} rgb_led_state_t;

#define RGB_LED_COUNT profile.rgb.led_count

// maximum sequence length for LED patterns 14bytes/sequence for 1400bytes total
#define LED_MAX_SEQUENCE_LEN 100
#define LED_MAX_SEQUENCE_COLORS 3
// maximum colors for rainbow wave
#define LED_MAX_WAVE_COLORS 6

// normal flight rgb colour - LED switch OFF
#define RGB_VALUE_INFLIGHT_OFF RGB(0, 0, 0)

// fade from one color to another when changed
#define RGB_FILTER_TIME_MICROSECONDS 50e3

#define RGB_DELAY_MS 16
#define RGB_FILTER_TIME FILTERCALC(1000 * RGB_DELAY_MS, RGB_FILTER_TIME_MICROSECONDS)

// array with individual led brightnesses
static uint32_t rgb_led_value[RGB_LED_MAX];

// variables for pattern/wave sequences
static uint32_t current_step = 0;
static uint32_t last_millis = 0;
static uint8_t pattern_rev = 0;

// set all leds to a specific color
static void rgb_led_set_all(uint32_t rgb, uint32_t fade) {
  static float r_filt = 0;
  static float g_filt = 0;
  static float b_filt = 0;

  if (fade) {
    // deconstruct the colour into components
    uint32_t g = rgb >> 16;
    uint32_t r = (rgb & 0x0000FF00) >> 8;
    uint32_t b = rgb & 0xff;

    // filter individual colors
    lpf(&r_filt, r, RGB_FILTER_TIME);
    lpf(&g_filt, g, RGB_FILTER_TIME);
    lpf(&b_filt, b, RGB_FILTER_TIME);
    uint32_t temp = RGB(r_filt, g_filt, b_filt);

    for (uint32_t i = 0; i < RGB_LED_COUNT; i++)
      rgb_led_value[i] = temp;
  } else {
    for (uint32_t i = 0; i < RGB_LED_COUNT; i++)
      rgb_led_value[i] = rgb;
  }
}

// set an individual led brightness
static void rgb_led_set_one(uint32_t led_number, uint32_t rgb) {
  rgb_led_value[led_number] = rgb;
}

// flashes between 2 colours, duty cycle 1 - 15
static void rgb_ledflash(uint32_t color1, uint32_t color2, uint32_t period, uint32_t duty) {
  if (time_micros() % period > (period * duty) >> 4) {
    rgb_led_set_all(color1, 1);
  } else {
    rgb_led_set_all(color2, 1);
  }
}

static void rgb_leds_off() {
  rgb_led_set_all(RGB_VALUE_INFLIGHT_OFF, 0);
  current_step = 0;
  last_millis = 0;
  pattern_rev = 0;
}

static void rgb_pattern_sequence() {
  if (time_millis() > last_millis + profile.rgb.led_sequence.duration) {
    last_millis = time_millis();
    // copy values locally
    uint64_t led_mask = (uint64_t)profile.rgb.led_sequence.led_map[current_step].led_mask1 | ((uint64_t)profile.rgb.led_sequence.led_map[current_step].led_mask2 << 32);
    uint16_t colors[4];
    for (uint8_t i = 0; i < 3; i++)
      colors[i + 1] = profile.rgb.led_sequence.led_map[current_step].colors[i];

    for (uint16_t j = 0; j < RGB_LED_COUNT; j++) {
      uint32_t temp_color = (led_mask & (3 << (j * 2))) >> (j * 2);
      rgb_led_set_one(j, RGB5TO8BIT(colors[temp_color]));
    }

    if (pattern_rev) {
      if (current_step == 0) {
        pattern_rev = !pattern_rev;
        current_step++;
      } else {
        current_step--;
      }
    } else {
      if (current_step == profile.rgb.led_sequence.num_steps - 1) {
        if (profile.rgb.led_sequence.pattern_reverse) {
          pattern_rev = !pattern_rev;
          current_step++;
        } else {
          current_step = 0;
        }
      } else {
        current_step++;
      }
    }
  }
}

static void rgb_rainbow() {
  float factor1, factor2;
  uint16_t ind;
  uint16_t col1, col2, r1, g1, b1, r2, g2, b2;
  for (uint16_t j = 0; j < RGB_LED_COUNT; j++) {
    uint16_t k = j;
    ind = current_step;
    if (profile.rgb.wave_sequence.width > 0) {
      if (profile.rgb.wave_sequence.reverse)
        k = (RGB_LED_COUNT - 1) - j;
      ind += j * profile.rgb.wave_sequence.fade_steps / profile.rgb.wave_sequence.width;
    }
    col1 = (uint32_t)((ind % (profile.rgb.wave_sequence.fade_steps * profile.rgb.wave_sequence.num_colors)) / profile.rgb.wave_sequence.fade_steps);
    col2 = (col1 + 1) % profile.rgb.wave_sequence.num_colors;
    // deconstruct the colour into components
    g1 = profile.rgb.wave_sequence.colors[col1] >> 16;
    r1 = (profile.rgb.wave_sequence.colors[col1] & 0x0000FF00) >> 8;
    b1 = profile.rgb.wave_sequence.colors[col1] & 0xff;
    g2 = profile.rgb.wave_sequence.colors[col2] >> 16;
    r2 = (profile.rgb.wave_sequence.colors[col2] & 0x0000FF00) >> 8;
    b2 = profile.rgb.wave_sequence.colors[col2] & 0xff;

    factor1 = 1.0 - ((float)(ind % (profile.rgb.wave_sequence.fade_steps * profile.rgb.wave_sequence.num_colors) - col1 * profile.rgb.wave_sequence.fade_steps) / profile.rgb.wave_sequence.fade_steps);
    factor2 = (float)((uint32_t)(ind - (col1 * profile.rgb.wave_sequence.fade_steps)) % (profile.rgb.wave_sequence.fade_steps * profile.rgb.wave_sequence.num_colors)) / profile.rgb.wave_sequence.fade_steps;
    rgb_led_set_one(k, RGB((uint32_t)(r1 * factor1 + r2 * factor2), (uint32_t)(g1 * factor1 + g2 * factor2), (uint32_t)(b1 * factor1 + b2 * factor2)));
  }
  current_step++;
  if (current_step > (profile.rgb.wave_sequence.fade_steps * profile.rgb.wave_sequence.num_colors)) {
    current_step = 0;
  }
}

static uint32_t rgb_fade(uint32_t current, uint32_t new, float coeff) {
  // deconstruct the colour into components
  uint32_t g1 = current >> 16;
  uint32_t r1 = (current & 0x0000FF00) >> 8;
  uint32_t b1 = current & 0xff;
  uint32_t g2 = new >> 16;
  uint32_t r2 = (new & 0x0000FF00) >> 8;
  uint32_t b2 = new & 0xff;

  return RGB((r1 + (r2 - r1) * coeff), (g1 + (g2 - g1) * coeff), (b1 + (b2 - b1) * coeff));
}

static void rgb_solid_color() {
  if (profile.rgb.solid_color.color2 != 0) {
    switch (profile.rgb.solid_color.modifier_channel) {
    case 0:
      rgb_led_set_all(rgb_fade(profile.rgb.solid_color.color1, profile.rgb.solid_color.color2, (float)((state.rx_filtered.roll + 1) / 2)), 0);
      break;
    case 1:
      rgb_led_set_all(rgb_fade(profile.rgb.solid_color.color1, profile.rgb.solid_color.color2, (float)((state.rx_filtered.pitch + 1) / 2)), 0);
      break;
    case 2:
      rgb_led_set_all(rgb_fade(profile.rgb.solid_color.color1, profile.rgb.solid_color.color2, state.rx_filtered.throttle), 0);
      break;
    case 3:
      rgb_led_set_all(rgb_fade(profile.rgb.solid_color.color1, profile.rgb.solid_color.color2, (float)((state.rx_filtered.yaw + 1) / 2)), 0);
      break;
    default: // one of the aux channels
      rgb_led_set_all(state.aux[profile.rgb.solid_color.modifier_channel] ? profile.rgb.solid_color.color2 : profile.rgb.solid_color.color1, 1);
      break;
    }
  } else {
    rgb_led_set_all(profile.rgb.solid_color.color1, 0);
  }
}

static void rgb_led_pattern() {
  switch (profile.rgb.active_pattern) {
  case RGB_PATTERN_SOLID:
    rgb_solid_color();
    break;
  case RGB_PATTERN_SEQUENCE:
    rgb_pattern_sequence();
    break;
  case RGB_PATTERN_RAINBOW:
    rgb_rainbow();
    break;
  default:
    rgb_led_set_all(RGB_VALUE_INFLIGHT_OFF, 1);
    break;
  }
}

static void rgb_led_update_values() {
  // Only flash low bat if at least one colour has been selected (some may not want to flash the LEDs)
  if (flags.lowbatt && profile.rgb.low_bat1 > 0 && profile.rgb.low_bat2 > 0) {
    rgb_ledflash(profile.rgb.low_bat1, profile.rgb.low_bat2, 500000, 8);
    return;
  }

  // bind mode (fast flash, not customizable)
  if (flags.rx_mode == RXMODE_BIND) {
    rgb_ledflash(RGB(0, 128, 128), RGB(32, 32, 32), 50000, 12);
    return;
  }

  // failsafe flash
  if (flags.failsafe) {
    rgb_ledflash(profile.rgb.failsafe1, profile.rgb.failsafe2, 500000, 8);
    return;
  }

  if (rx_aux_on(AUX_LEDS))
    rgb_led_pattern();
  else
    rgb_leds_off();
}

void rgb_led_update() {
  static rgb_led_state_t rgb_state = RGB_STATE_DELAY;

  static uint32_t delay_start = 0;

  switch (rgb_state) {
  case RGB_STATE_DELAY:
    if ((time_millis() - delay_start) > RGB_DELAY_MS) {
      rgb_state = RGB_STATE_UPDATE;
    }
    break;

  case RGB_STATE_UPDATE:
    if (!rgb_led_busy()) {
      rgb_led_update_values();
      rgb_led_set_value(rgb_led_value, RGB_LED_COUNT);
      rgb_state = RGB_STATE_SEND;
    }
    break;

  case RGB_STATE_SEND:
    if (!rgb_led_busy()) {
      rgb_led_send();
      delay_start = time_millis();
      rgb_state = RGB_STATE_DELAY;
    }
    break;
  }
}

#else

void rgb_led_update() {}

#endif