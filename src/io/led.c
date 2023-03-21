#include "io/led.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/time.h"
#include "flight/control.h"
#include "util/util.h"

// for led flash on gestures
int ledcommand = 0;
int ledblink = 0;
uint32_t ledcommandtime = 0;

void led_init() {
  LL_GPIO_InitTypeDef init;
  init.Mode = LL_GPIO_MODE_OUTPUT;
  init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  init.Pull = LL_GPIO_PULL_NO;
  init.Speed = LL_GPIO_SPEED_FREQ_HIGH;

  for (uint8_t i = 0; i < LED_MAX; i++) {
    const target_led_t led = target.leds[i];
    if (led.pin == PIN_NONE) {
      continue;
    }

    gpio_pin_init(&init, led.pin);
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

void led_flash(uint32_t period, int duty) {
  if (time_micros() % period > (period * duty) >> 4) {
    led_on(LEDALL);
  } else {
    led_off(LEDALL);
  }
}

// delta- sigma first order modulator.
void led_pwm(uint8_t pwmval, float looptime) {
  static uint32_t last_time = 0;
  const uint32_t time = time_micros();
  const uint32_t ledtime = time - last_time;
  last_time = time;

  static float ds_integrator = 0;
  limitf(&ds_integrator, 2);

  static float last_brightness = 0;

  const float desired_brightness = pwmval * (1.0f / 15.0f);
  ds_integrator += (desired_brightness - last_brightness) * ledtime * (1.0f / looptime);

  if (ds_integrator > 0.49f) {
    led_on(LEDALL);
    last_brightness = 1.0f;
  } else {
    led_off(LEDALL);
    last_brightness = 0;
  }
}

void led_update() {
  // led flash logic
  if (flags.lowbatt) {
    led_flash(500000, 8);
    return;
  }

  if (flags.rx_mode == RXMODE_BIND) { // bind mode
    led_flash(100000, 12);
    return;
  }

  if (flags.failsafe) {
    led_flash(500000, 15);
    return;
  }

  if (ledcommand) {
    if (!ledcommandtime)
      ledcommandtime = time_micros();
    if (time_micros() - ledcommandtime > 500000) {
      ledcommand = 0;
      ledcommandtime = 0;
    }
    led_flash(100000, 8);
    return;
  }

  if (ledblink) {
    uint32_t time = time_micros();
    if (!ledcommandtime) {
      ledcommandtime = time;
      led_off(LEDALL);
    }
    if (time - ledcommandtime > 500000) {
      ledblink--;
      ledcommandtime = 0;
    }
    if (time - ledcommandtime > 300000) {
      led_on(LEDALL);
    }
  } else { // led is normally on
    if (LED_BRIGHTNESS != 15)
      led_pwm(LED_BRIGHTNESS, state.looptime_autodetect);
    else
      led_on(LEDALL);
  }
}
