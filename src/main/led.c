#include "led.h"

#include "drv_gpio.h"
#include "drv_time.h"
#include "flight/control.h"
#include "project.h"
#include "util/util.h"

#define LEDALL 15

// for led flash on gestures
int ledcommand = 0;
int ledblink = 0;
uint32_t ledcommandtime = 0;

void ledon(uint8_t val) {
#if (LED_NUMBER > 0)
#ifdef LED1_INVERT
  if (val & 1)
    gpio_pin_reset(LED1PIN);
#else
  if (val & 1)
    gpio_pin_set(LED1PIN);
#endif
#endif
#if (LED_NUMBER > 1)
#ifdef LED2_INVERT
  if (val & 2)
    gpio_pin_reset(LED2PIN);
#else
  if (val & 2)
    gpio_pin_set(LED2PIN);
#endif
#endif
#if (LED_NUMBER > 2)
#ifdef LED3_INVERT
  if (val & 4)
    gpio_pin_reset(LED3PIN);
#else
  if (val & 4)
    gpio_pin_set(LED3PIN);
#endif
#endif
#if (LED_NUMBER > 3)
#ifdef LED4_INVERT
  if (val & 8)
    gpio_pin_reset(LED4PIN);
#else
  if (val & 8)
    gpio_pin_set(LED4PIN);
#endif
#endif
}

void ledoff(uint8_t val) {
#if (LED_NUMBER > 0)
#ifdef LED1_INVERT
  if (val & 1)
    gpio_pin_set(LED1PIN);
#else
  if (val & 1)
    gpio_pin_reset(LED1PIN);
#endif
#endif
#if (LED_NUMBER > 1)
#ifdef LED2_INVERT
  if (val & 2)
    gpio_pin_set(LED2PIN);
#else
  if (val & 2)
    gpio_pin_reset(LED2PIN);
#endif
#endif
#if (LED_NUMBER > 2)
#ifdef LED3_INVERT
  if (val & 4)
    gpio_pin_set(LED3PIN);
#else
  if (val & 4)
    gpio_pin_reset(LED3PIN);
#endif
#endif
#if (LED_NUMBER > 3)
#ifdef LED1_INVERT
  if (val & 8)
    gpio_pin_set(LED4PIN);
#else
  if (val & 8)
    gpio_pin_reset(LED4PIN);
#endif
#endif
}

void ledflash(uint32_t period, int duty) {
#if (LED_NUMBER > 0)
  if (time_micros() % period > (period * duty) >> 4) {
    ledon(LEDALL);
  } else {
    ledoff(LEDALL);
  }
#endif
}

// delta- sigma first order modulator.
uint8_t led_pwm(uint8_t pwmval) {
  static float lastledbrightness = 0;
  static uint32_t lastledtime = 0;
  static float ds_integrator = 0;

  uint32_t time = time_micros();
  uint32_t ledtime = time - lastledtime;

  lastledtime = time;

  float desiredbrightness = pwmval * (1.0f / 15.0f);
  limitf(&ds_integrator, 2);

  ds_integrator += (desiredbrightness - lastledbrightness) * ledtime * (1.0f / LOOPTIME);

  if (ds_integrator > 0.49f) {
    ledon(255);
    lastledbrightness = 1.0f;
  } else {
    ledoff(255);
    lastledbrightness = 0;
  }
  return 0;
}

void led_update() {
  if (LED_NUMBER <= 0) {
    return;
  }

  // led flash logic
  if (flags.lowbatt) {
    ledflash(500000, 8);
    return;
  }

  if (flags.rx_mode == RXMODE_BIND) { // bind mode
    ledflash(100000, 12);
    return;
  }

  if (flags.failsafe) {
    ledflash(500000, 15);
    return;
  }

  if (ledcommand) {
    if (!ledcommandtime)
      ledcommandtime = time_micros();
    if (time_micros() - ledcommandtime > 500000) {
      ledcommand = 0;
      ledcommandtime = 0;
    }
    ledflash(100000, 8);
    return;
  }

  if (ledblink) {
    uint32_t time = time_micros();
    if (!ledcommandtime) {
      ledcommandtime = time;
      ledoff(255);
    }
    if (time - ledcommandtime > 500000) {
      ledblink--;
      ledcommandtime = 0;
    }
    if (time - ledcommandtime > 300000) {
      ledon(255);
    }
  } else { // led is normally on
    if (LED_BRIGHTNESS != 15)
      led_pwm(LED_BRIGHTNESS);
    else
      ledon(255);
  }
}
