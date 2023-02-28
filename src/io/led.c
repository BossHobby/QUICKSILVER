#include "io/led.h"

#include "driver/gpio.h"
#include "driver/time.h"
#include "flight/control.h"
#include "project.h"
#include "util/util.h"

#define LEDALL 15

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

#if (LED_NUMBER > 0)
  gpio_pin_init(&init, LED1PIN);
#endif
#if (LED_NUMBER > 1)
  gpio_pin_init(&init, LED2PIN);
#endif
#if (LED_NUMBER > 2)
  gpio_pin_init(&init, LED3PIN);
#endif
#if (LED_NUMBER > 3)
  gpio_pin_init(&init, LED4PIN);
#endif
}

void led_on(uint8_t val) {
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

void led_off(uint8_t val) {
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

void led_flash(uint32_t period, int duty) {
#if (LED_NUMBER > 0)
  if (time_micros() % period > (period * duty) >> 4) {
    led_on(LEDALL);
  } else {
    led_off(LEDALL);
  }
#endif
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
  if (LED_NUMBER <= 0) {
    return;
  }

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
