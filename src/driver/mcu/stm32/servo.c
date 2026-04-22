#include "driver/servo.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "util/util.h"

#ifdef USE_SERVO

#define SERVO_TIMER_FREQ 1000000

static resource_tag_t servo_tags[SERVO_MAX];

void servo_pwm_init(const gpio_pins_t *pins, uint16_t pwm_hz) {
  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_ALTERNATE;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;

  LL_TIM_OC_InitTypeDef tim_oc_init;
  tim_oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  tim_oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  tim_oc_init.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  tim_oc_init.CompareValue = SERVO_PULSE_CENTER;

  const uint16_t divider = (PWM_CLOCK_FREQ_HZ / SERVO_TIMER_FREQ) - 1;
  const uint32_t period = SERVO_TIMER_FREQ / pwm_hz;

  for (uint32_t i = 0; i < SERVO_MAX; i++) {
    servo_tags[i] = 0;

    if (pins[i] == PIN_NONE) {
      continue;
    }

    for (uint32_t j = 0; j < GPIO_AF_MAX; j++) {
      const gpio_af_t *func = &gpio_pin_afs[j];
      if (func->pin != pins[i] || RESOURCE_TAG_TYPE(func->tag) != RESOURCE_TIM) {
        continue;
      }

      if (!timer_alloc_tag(TIMER_USE_SERVO, func->tag)) {
        continue;
      }

      const timer_index_t tim = TIMER_TAG_TIM(func->tag);
      const timer_channel_t ch = TIMER_TAG_CH(func->tag);

      gpio_pin_init_af(pins[i], gpio_init, func->af);
      timer_up_init(tim, divider, period);
      LL_TIM_OC_Init(timer_defs[tim].instance, timer_channel_val(ch), &tim_oc_init);
      LL_TIM_EnableCounter(timer_defs[tim].instance);
      LL_TIM_EnableAllOutputs(timer_defs[tim].instance);

      servo_tags[i] = func->tag;
      break;
    }
  }
}

void servo_pwm_write(const float *values) {
  for (uint32_t i = 0; i < SERVO_MAX; i++) {
    if (servo_tags[i] == 0) {
      continue;
    }

    const float val = constrain(values[i], -1.0f, 1.0f);
    const uint16_t pulse = (uint16_t)(SERVO_PULSE_CENTER + (val * (SERVO_PULSE_MAX - SERVO_PULSE_CENTER)));

    const resource_tag_t tag = servo_tags[i];
    switch (TIMER_TAG_CH(tag)) {
    case TIMER_CH1:
    case TIMER_CH1N:
      WRITE_REG(timer_defs[TIMER_TAG_TIM(tag)].instance->CCR1, pulse);
      break;
    case TIMER_CH2:
    case TIMER_CH2N:
      WRITE_REG(timer_defs[TIMER_TAG_TIM(tag)].instance->CCR2, pulse);
      break;
    case TIMER_CH3:
    case TIMER_CH3N:
      WRITE_REG(timer_defs[TIMER_TAG_TIM(tag)].instance->CCR3, pulse);
      break;
    case TIMER_CH4:
      WRITE_REG(timer_defs[TIMER_TAG_TIM(tag)].instance->CCR4, pulse);
      break;
    default:
      break;
    }
  }
}

void servo_pwm_stop() {
  gpio_config_t gpio_init = gpio_config_default();

  for (uint32_t i = 0; i < SERVO_MAX; i++) {
    const resource_tag_t tag = servo_tags[i];
    if (tag == 0) {
      continue;
    }

    LL_TIM_DisableCounter(timer_defs[TIMER_TAG_TIM(tag)].instance);

    for (uint32_t j = 0; j < GPIO_AF_MAX; j++) {
      const gpio_af_t *func = &gpio_pin_afs[j];
      if (func->tag != tag) {
        continue;
      }
      gpio_pin_init(func->pin, gpio_init);
      break;
    }

    servo_tags[i] = 0;
  }
}

#endif
