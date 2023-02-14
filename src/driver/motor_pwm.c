#include <math.h>

#include "core/profile.h"
#include "driver/gpio.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "driver/timer.h"
#include "flight/control.h"
#include "project.h"

#ifdef USE_PWM_DRIVER

#ifndef PWM_CLOCK_FREQ_HZ
#define PWM_CLOCK_FREQ_HZ 48000000
#warning PWM_CLOCK_FREQ_HZ not present
#endif

#define PWM_DIVIDER 1
#define PWMTOP ((PWM_CLOCK_FREQ_HZ / PWMFREQ) - 1)

// pwm frequency checking macros
#if (PWMTOP < 1400)
// approx 34Khz
#undef PWMTOP
#define PWMTOP 6000
#warning PWM FREQUENCY TOO HIGH
#endif

#if (PWMTOP > 65535)
// under approx 732Hz we add the divider by 4
#undef PWMTOP
#define PWMTOP (((PWM_CLOCK_FREQ_HZ / 4) / PWMFREQ) - 1)
#undef PWM_DIVIDER
#define PWM_DIVIDER 4
// #warning PWM DIVIDE BY 4 ON
#endif

#if (PWMTOP > 65535)
// approx 183Hz is min frequency
#undef PWMTOP
#undef PWM_DIVIDER
#define PWMTOP 6000
#define PWM_DIVIDER 1
#warning PWM FREQUENCY TOO LOW
#endif
// end pwm frequency macros

extern profile_t profile;

uint32_t motorbeeptime = 0;
int beepon = 0;

#define MOTOR_BEEPS_PWM_ON 0.2
#define MOTOR_BEEPS_PWM_OFF 0.0

void motor_init() {
// timer clock enable
#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel) \
  timer_init(timer, PWM_DIVIDER, PWMTOP);

  MOTOR_PINS

#undef MOTOR_PIN

  LL_TIM_OC_InitTypeDef tim_oc_init;
  tim_oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  tim_oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  tim_oc_init.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  tim_oc_init.CompareValue = 0;

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;

#ifndef TIM14
#define TIM14 NULL
#endif

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel)               \
  gpio_init.Pin = LL_GPIO_PIN_##pin;                                     \
  gpio_init.Alternate = pin_af;                                          \
  LL_GPIO_Init(GPIO##port, &gpio_init);                                  \
  LL_TIM_OC_Init(timer, LL_TIM_CHANNEL_CH##timer_channel, &tim_oc_init); \
  LL_TIM_EnableCounter(timer);                                           \
  if (timer != TIM14)                                                    \
    LL_TIM_EnableAllOutputs(timer);

  MOTOR_PINS

#undef MOTOR_PIN
}

void motor_wait_for_ready() {
}

void motor_beep() {
  uint32_t time = time_millis();
  if (!beepon && (time % 2000 < 125)) {
    for (int i = 0; i <= 3; i++) {
      motor_set(i, MOTOR_BEEPS_PWM_ON);
      beepon = 1;
    }
  } else {
    for (int i = 0; i <= 3; i++) {
      motor_set(i, MOTOR_BEEPS_PWM_OFF);
      beepon = 0;
    }
  }
}

void motor_write(float *values) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    int32_t pwm = values[i] * PWMTOP;
    if (pwm < 0)
      pwm = 0;
    if (pwm > PWMTOP)
      pwm = PWMTOP;

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel) \
  case MOTOR_PIN_IDENT(port, pin):                         \
    timer->CCR##timer_channel = (uint16_t)pwm;             \
    break;

    switch (profile.motor.motor_pins[i]) {
      MOTOR_PINS
    default:
      // handle error;
      break;
    };
#undef MOTOR_PIN
  }
}

void motor_set_direction(motor_direction_t dir) {
}

bool motor_direction_change_done() {
  return true;
}

#endif
