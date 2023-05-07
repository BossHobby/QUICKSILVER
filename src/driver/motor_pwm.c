#include <math.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/gpio.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "driver/timer.h"
#include "flight/control.h"

#define PWM_DIVIDER 1
#define PWMTOP ((PWM_CLOCK_FREQ_HZ / PWMFREQ) - 1)

#define MOTOR_BEEPS_PWM_ON 0.2
#define MOTOR_BEEPS_PWM_OFF 0.0

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
static resource_tag_t timer_tags[MOTOR_PIN_MAX];

void motor_pwm_init() {
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;

  LL_TIM_OC_InitTypeDef tim_oc_init;
  tim_oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  tim_oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  tim_oc_init.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  tim_oc_init.CompareValue = 0;

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const gpio_pins_t pin = target.motor_pins[i];

    for (uint32_t j = 0; j < GPIO_AF_MAX; j++) {
      const gpio_af_t *func = &gpio_pin_afs[j];
      if (func->pin != pin || RESOURCE_TAG_TYPE(func->tag) != RESOURCE_TIM) {
        continue;
      }

      if (timer_alloc_tag(TIMER_USE_MOTOR_PWM, func->tag)) {
        timer_tags[i] = func->tag;
        gpio_pin_init_af(&gpio_init, pin, func->af);
        break;
      }
    }

    const uint8_t tim = TIMER_TAG_TIM(timer_tags[i]);
    const uint8_t ch = TIMER_TAG_CH(timer_tags[i]);
    timer_up_init(tim, PWM_DIVIDER, PWMTOP);
    LL_TIM_OC_Init(timer_defs[tim].instance, timer_ll_channel(ch), &tim_oc_init);
    LL_TIM_EnableCounter(timer_defs[tim].instance);
#ifndef STM32F411
    if (tim != TIMER14) {
      LL_TIM_EnableAllOutputs(timer_defs[tim].instance);
    }
#endif
  }
}

void motor_pwm_beep() {
  static uint8_t beepon = 0;
  uint32_t time = time_millis();
  if (!beepon && (time % 2000 < 125)) {
    for (uint32_t i = 0; i <= 3; i++) {
      motor_set(i, MOTOR_BEEPS_PWM_ON);
      beepon = 1;
    }
  } else {
    for (uint32_t i = 0; i <= 3; i++) {
      motor_set(i, MOTOR_BEEPS_PWM_OFF);
      beepon = 0;
    }
  }
}

void motor_pwm_write(float *values) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    int32_t pwm = values[i] * PWMTOP;
    if (pwm < 0)
      pwm = 0;
    if (pwm > PWMTOP)
      pwm = PWMTOP;

    const resource_tag_t tag = timer_tags[i];
    switch (TIMER_TAG_CH(tag)) {
    case TIMER_CH1:
    case TIMER_CH1N:
      WRITE_REG(timer_defs[TIMER_TAG_TIM(tag)].instance->CCR1, (uint16_t)pwm);
      break;
    case TIMER_CH2:
    case TIMER_CH2N:
      WRITE_REG(timer_defs[TIMER_TAG_TIM(tag)].instance->CCR2, (uint16_t)pwm);
      break;
    case TIMER_CH3:
    case TIMER_CH3N:
      WRITE_REG(timer_defs[TIMER_TAG_TIM(tag)].instance->CCR3, (uint16_t)pwm);
      break;
    case TIMER_CH4:
      WRITE_REG(timer_defs[TIMER_TAG_TIM(tag)].instance->CCR4, (uint16_t)pwm);
      break;
    default:
      break;
    }
  }
}
