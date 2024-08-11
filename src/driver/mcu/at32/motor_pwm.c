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

  tmr_output_config_type tim_oc_init;
  tmr_output_default_para_init(&tim_oc_init);
  tim_oc_init.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tim_oc_init.oc_idle_state = TRUE;
  tim_oc_init.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tim_oc_init.oc_output_state = TRUE;

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

    const uint8_t tim = TIMER_TAG_TIM(timer_tags[i]);
    const uint8_t ch = TIMER_TAG_CH(timer_tags[i]);
    const timer_def_t *def = &timer_defs[tim];

    tmr_counter_enable(def->instance, FALSE);

    timer_up_init(tim, PWM_DIVIDER, PWM_TOP);

    tmr_output_channel_config(def->instance, timer_channel_val(ch), &tim_oc_init);
    tmr_channel_value_set(def->instance, timer_channel_val(ch), 0);
    tmr_output_channel_buffer_enable(def->instance, timer_channel_val(ch), TRUE);

    tmr_output_enable(def->instance, TRUE);
    tmr_counter_enable(def->instance, TRUE);
  }
}

void motor_pwm_write(float *values) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const resource_tag_t tag = timer_tags[i];
    const timer_def_t *def = &timer_defs[TIMER_TAG_TIM(tag)];

    const uint16_t pwm = constrain(values[i] * PWM_TOP, 0, PWM_TOP);
    tmr_channel_value_set(def->instance, timer_channel_val(TIMER_TAG_CH(tag)), pwm);
  }
}

#endif