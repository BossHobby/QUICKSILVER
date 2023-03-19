
#include "io/buzzer.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/time.h"
#include "flight/control.h"

#ifdef BUZZER_ENABLE

static void buzzer_on() {
#ifdef BUZZER_INVERT
  gpio_pin_reset(BUZZER_PIN);
#else
  gpio_pin_set(BUZZER_PIN);
#endif
}

static void buzzer_off() {
#ifdef BUZZER_INVERT
  gpio_pin_set(BUZZER_PIN);
#else
  gpio_pin_reset(BUZZER_PIN);
#endif
}

void buzzer_init() {
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init(&gpio_init, BUZZER_PIN);

  buzzer_off();
}

void buzzer_update() {
  static bool toggle;
  static uint32_t buzzertime;

  uint32_t pulse_rate;

  // waits 5 seconds
  // before configuring the gpio buzzer pin to ensure
  // there is time to program the chip (if using SWDAT or SWCLK)

  if ((flags.lowbatt || flags.failsafe || rx_aux_on(AUX_BUZZER_ENABLE)) && !flags.usb_active) {
    uint32_t time = time_micros();
    if (buzzertime == 0)
      buzzertime = time;
    else {

      // rank lowbatt > failsafe > throttle
      if (flags.lowbatt)
        pulse_rate = 200000; // 1/5th second
      else if (flags.failsafe)
        pulse_rate = 400000; // 2/5ths second
      else
        pulse_rate = 600000; // 3/5ths second

      // start the buzzer if timeout has elapsed
      if (time - buzzertime > BUZZER_DELAY || flags.lowbatt || rx_aux_on(AUX_BUZZER_ENABLE)) {

        // enable buzzer
        if (time % pulse_rate > pulse_rate / 2) {
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
    }

  } else {
    buzzertime = 0;
    buzzer_off();
  }
}

#else

void buzzer_init() {}
void buzzer_update() {}

#endif
