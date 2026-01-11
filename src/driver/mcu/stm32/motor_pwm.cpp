#include "driver/motor_pwm.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "util/util.h"

#ifdef USE_MOTOR_PWM

static resource_tag_t timer_tags[MOTOR_PIN_MAX];

void motor_pwm_init() {
  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_ALTERNATE;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;

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
        gpio_pin_init_af(pin, gpio_init, func->af);
        break;
      }
    }

    if (timer_tags[i] == 0) {
      continue;
    }

    const timer_index_t tim = TIMER_TAG_TIM(timer_tags[i]);
    const timer_channel_t ch = TIMER_TAG_CH(timer_tags[i]);
    timer_up_init(tim, PWM_DIVIDER, PWM_TOP);
    LL_TIM_OC_Init(timer_defs[tim].instance, timer_channel_val(ch), &tim_oc_init);
    LL_TIM_EnableCounter(timer_defs[tim].instance);
#if defined(TIMER14)
    if (tim != TIMER14) {
      LL_TIM_EnableAllOutputs(timer_defs[tim].instance);
    }
#endif
  }
}

void motor_pwm_write(float *values) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const uint16_t pwm = constrain(values[i] * PWM_TOP, 0, PWM_TOP);

    const resource_tag_t tag = timer_tags[i];
    switch (TIMER_TAG_CH(tag)) {
    case TIMER_CH1:
    case TIMER_CH1N:
      WRITE_REG(timer_defs[TIMER_TAG_TIM(tag)].instance->CCR1, pwm);
      break;
    case TIMER_CH2:
    case TIMER_CH2N:
      WRITE_REG(timer_defs[TIMER_TAG_TIM(tag)].instance->CCR2, pwm);
      break;
    case TIMER_CH3:
    case TIMER_CH3N:
      WRITE_REG(timer_defs[TIMER_TAG_TIM(tag)].instance->CCR3, pwm);
      break;
    case TIMER_CH4:
      WRITE_REG(timer_defs[TIMER_TAG_TIM(tag)].instance->CCR4, pwm);
      break;
    default:
      break;
    }
  }
}

#endif