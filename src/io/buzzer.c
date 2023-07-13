
#include "io/buzzer.h"

#include "core/profile.h"
#include "core/project.h"
#include "driver/gpio.h"
#include "driver/time.h"
#include "flight/control.h"

static void buzzer_on() {
  if (target.buzzer.pin == PIN_NONE) {
    return;
  }

  if (target.buzzer.invert) {
    gpio_pin_reset(target.buzzer.pin);
  } else {
    gpio_pin_set(target.buzzer.pin);
  }
}

static void buzzer_off() {
  if (target.buzzer.pin == PIN_NONE) {
    return;
  }

  if (target.buzzer.invert) {
    gpio_pin_set(target.buzzer.pin);
  } else {
    gpio_pin_reset(target.buzzer.pin);
  }
}

void buzzer_init() {
  if (target.buzzer.pin == PIN_NONE) {
    return;
  }

  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_OUTPUT;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_pin_init(target.buzzer.pin, gpio_init);

  buzzer_off();
}

static uint32_t buzzer_pulse_rate() {
  if (flags.lowbatt) {
    return 200000; // 1/5th second
  }

  if (flags.failsafe) {
    return 400000; // 2/5ths second
  }

  if (rx_aux_on(AUX_BUZZER_ENABLE)) {
    return 600000; // 3/5ths second
  }

  return 0;
}

static uint32_t buzzer_delay() {
  if (flags.failsafe) {
    return BUZZER_DELAY;
  }
  return 0;
}

void buzzer_update() {
  static uint32_t buzzer_time = 0;

  if (flags.usb_active) {
    // dont beep on usb
    buzzer_time = 0;
    buzzer_off();
    return;
  }

  const uint32_t pulse_rate = buzzer_pulse_rate();
  if (pulse_rate == 0) {
    // beeper not active
    buzzer_time = 0;
    buzzer_off();
    return;
  }

  const uint32_t time = time_micros();
  if (buzzer_time == 0) {
    buzzer_time = time;
    return;
  }

  const uint32_t delay = buzzer_delay();
  if (time - buzzer_time < delay) {
    buzzer_off();
    return;
  }

  // enable buzzer
  if (time % pulse_rate > pulse_rate / 2) {
    static bool toggle = false;
    if (toggle) {
      buzzer_on();
    } else {
      buzzer_off();
    }
    toggle = !toggle;
  } else {
    buzzer_off();
  }
}
