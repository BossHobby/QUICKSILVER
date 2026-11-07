#include "driver/motor_pwm.h"

#include <hardware/pwm.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "util/util.h"

#ifdef USE_MOTOR_PWM

static resource_tag_t timer_tags[MOTOR_PIN_MAX];

static uint32_t motor_pwm_channel(resource_tag_t tag) {
  return TIMER_TAG_CH(tag) == TIMER_CH2 ? PWM_CHAN_B : PWM_CHAN_A;
}

void motor_pwm_init() {
  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_ALTERNATE;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const gpio_pins_t pin = target.outputs[i].pin;
    if (pin == PIN_NONE || (target.outputs[i].caps & OUTPUT_CAP_BRUSHED) == 0 ||
        !profile_output_slot_uses_protocol(i, OUTPUT_PROTOCOL_BRUSHED)) {
      continue;
    }

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
    const uint32_t slice = tim - TIMER1;
    timer_up_init(tim, PWM_DIVIDER, PWM_TOP);
    pwm_set_chan_level(slice, motor_pwm_channel(timer_tags[i]), 0);
    pwm_set_enabled(slice, true);
  }
}

void motor_pwm_write(float *values) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const resource_tag_t tag = timer_tags[i];
    if (tag == 0) {
      continue;
    }

    const uint16_t pwm = constrain(values[i] * PWM_TOP, 0, PWM_TOP);
    pwm_set_chan_level(TIMER_TAG_TIM(tag) - TIMER1, motor_pwm_channel(tag), pwm);
  }
}

#endif
