#include "led.h"

#include "drv_gpio.h"
#include "drv_time.h"
#include "project.h"

#define LEDALL 15

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
  if (timer_micros() % period > (period * duty) >> 4) {
    ledon(LEDALL);
  } else {
    ledoff(LEDALL);
  }
#endif
}

int ledlevel = 0;

uint8_t led_pwm2(uint8_t pwmval) {
  static int loopcount = 0;

  ledlevel = pwmval;
  loopcount++;
  loopcount &= 0xF;
  if (ledlevel > loopcount) {
    ledon(255);
  } else {
    ledoff(255);
  }
  return ledlevel;
}

int ledlevel2 = 0;
unsigned long lastledtime;
float lastledbrightness = 0;

//#define DEBUG

#ifdef DEBUG
int debug_led;
#endif

#include "util.h"

// delta- sigma first order modulator.
uint8_t led_pwm(uint8_t pwmval) {
  static float ds_integrator = 0;
  unsigned int time = timer_micros();
  unsigned int ledtime = time - lastledtime;

  lastledtime = time;

  float desiredbrightness = pwmval * (1.0f / 15.0f);

  //	limitf( &lastledbrightness, 2);

  limitf(&ds_integrator, 2);

  ds_integrator += (desiredbrightness - lastledbrightness) * ledtime * (1.0f / LOOPTIME);

  if (ds_integrator > 0.49f) {
    ledon(255);
    lastledbrightness = 1.0f;
#ifdef DEBUG
    debug_led <<= 1;
    debug_led |= 1;
#endif
  } else {
    ledoff(255);
    lastledbrightness = 0;
#ifdef DEBUG
    debug_led <<= 1;
    debug_led &= 0xFFFFFFFE;
#endif
  }
  return 0;
}
