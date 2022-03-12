
#include "buzzer.h"

#include "drv_gpio.h"
#include "drv_time.h"
#include "flight/control.h"
#include "project.h"

#ifdef BUZZER_ENABLE

#ifdef BUZZER_INVERT
#define PIN_ON(pin) gpio_pin_reset(pin)
#define PIN_OFF(pin) gpio_pin_set(pin)
#else
#define PIN_ON(pin) gpio_pin_set(pin)
#define PIN_OFF(pin) gpio_pin_reset(pin)
#endif

int gpio_init_buzzer() {
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init(&gpio_init, BUZZER_PIN);

  return 1;
}

void buzzer() {
  static int toggle;
  static uint32_t buzzertime;
  static int buzzer_init = 0;

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
        // initialize pin only after minimum 10 seconds from powerup
        if (!buzzer_init && time > 10e6) {
          buzzer_init = gpio_init_buzzer();
        }

        // don't continue if buzzer not initialized
        if (!buzzer_init)
          return;

        // enable buzzer
        if (time % pulse_rate > pulse_rate / 2) {
          if (toggle) // cycle the buzzer
          {
            PIN_ON(BUZZER_PIN); // on
          } else {
            PIN_OFF(BUZZER_PIN); // off
          }
          toggle = !toggle;
        } else {
          PIN_OFF(BUZZER_PIN);
        }
      }
    }

  } else {
    buzzertime = 0;
    // set buzzer to off if beeping condition stopped
    if (buzzer_init)
      PIN_OFF(BUZZER_PIN);
  }
}

#endif
