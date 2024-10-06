#include "io/led.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/time.h"
#include "flight/control.h"
#include "util/util.h"

#define LED_FLASH -1

static int32_t blink_count = 0;

void led_init() {
  gpio_config_t init;
  init.mode = GPIO_OUTPUT;
  init.output = GPIO_PUSHPULL;
  init.pull = GPIO_NO_PULL;
  init.drive = GPIO_DRIVE_HIGH;

  for (uint8_t i = 0; i < LED_MAX; i++) {
    const target_led_t led = target.leds[i];
    if (led.pin == PIN_NONE) {
      continue;
    }

    gpio_pin_init(led.pin, init);
    if (led.invert) {
      gpio_pin_set(led.pin);
    } else {
      gpio_pin_reset(led.pin);
    }
  }
}

void led_on(uint8_t val) {
  for (uint8_t i = 0; i < LED_MAX; i++) {
    if ((val & (i + 1)) == 0) {
      continue;
    }

    const target_led_t led = target.leds[i];
    if (led.pin == PIN_NONE) {
      continue;
    }

    if (led.invert) {
      gpio_pin_reset(led.pin);
    } else {
      gpio_pin_set(led.pin);
    }
  }
}

void led_off(uint8_t val) {
  for (uint8_t i = 0; i < LED_MAX; i++) {
    if ((val & (i + 1)) == 0) {
      continue;
    }

    const target_led_t led = target.leds[i];
    if (led.pin == PIN_NONE) {
      continue;
    }

    if (led.invert) {
      gpio_pin_set(led.pin);
    } else {
      gpio_pin_reset(led.pin);
    }
  }
}

// delta- sigma first order modulator.
void led_pwm(float desired_brightness, float looptime) {
  static uint32_t last_time = 0;
  const uint32_t time = time_micros();
  const uint32_t ledtime = time - last_time;
  last_time = time;

  static float ds_integrator = 0;
  ds_integrator = constrain(ds_integrator, -2, 2);

  static float last_brightness = 0;
  ds_integrator += (desired_brightness - last_brightness) * ledtime * (1.0f / looptime);

  if (ds_integrator > 0.49f) {
    led_on(LEDALL);
    last_brightness = 1.0f;
  } else {
    led_off(LEDALL);
    last_brightness = 0;
  }
}

// duty 0-31
static void led_flash_duty(uint32_t period_ms, uint8_t duty) {
  const uint32_t period = period_ms * 1000;
  const uint32_t divider = (period * duty) >> 5;
  if (time_micros() % period > divider) {
    led_on(LEDALL);
  } else {
    led_off(LEDALL);
  }
}

void led_flash() {
  blink_count = LED_FLASH;
}

void led_blink(uint8_t count) {
  blink_count = count;
}

void led_update() {
  if (flags.lowbatt) {
    return led_flash_duty(500, 16);
  }

  if (flags.rx_mode == RXMODE_BIND) {
    return led_flash_duty(100, 24);
  }

  if (flags.failsafe) {
    return led_flash_duty(500, 30);
  }

  if (flags.arm_switch && (flags.throttle_safety == 1 || flags.arm_safety == 1)) {
    return led_flash_duty(100, 8);
  }

  static uint32_t last_time = 0;

  if (blink_count == LED_FLASH) {
    if (!last_time)
      last_time = time_micros();
    if (time_micros() - last_time > 500000) {
      blink_count = 0;
      last_time = 0;
    }
    led_flash_duty(100, 8);
    return;
  }

  if (blink_count) {
    uint32_t time = time_micros();
    if (!last_time) {
      last_time = time;
      led_off(LEDALL);
    }
    if (time - last_time > 500000) {
      blink_count--;
      last_time = 0;
    }
    if (time - last_time > 250000) {
      led_on(LEDALL);
    }
    return;
  }

  if (LED_BRIGHTNESS != 15)
    led_pwm(LED_BRIGHTNESS, state.looptime_autodetect);
  else
    led_on(LEDALL);
}
