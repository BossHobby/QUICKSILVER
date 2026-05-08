#include "driver/servo.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "util/util.h"

#ifdef USE_SERVO

#define SERVO_TIMER_FREQ 1000000

static resource_tag_t servo_tags[MOTOR_PIN_MAX];

void servo_pwm_init(const gpio_pins_t *pins, uint16_t pwm_hz) {
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

  const uint16_t divider = PWM_CLOCK_FREQ_HZ / SERVO_TIMER_FREQ;
  const uint32_t period = SERVO_TIMER_FREQ / pwm_hz;

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (pins[i] == PIN_NONE) {
      continue;
    }

    resource_tag_t tag = servo_tags[i];
    if (tag == 0) {
      tag = servo_alloc_func(pins[i]);
      servo_tags[i] = tag;
    }
    if (tag == 0) {
      continue;
    }

    const timer_index_t tim = TIMER_TAG_TIM(tag);
    const timer_channel_t ch = TIMER_TAG_CH(tag);
    const timer_def_t *def = &timer_defs[tim];
    gpio_pin_init_tag(pins[i], gpio_init, tag);
    tmr_counter_enable(def->instance, FALSE);
    timer_up_init(tim, divider, period);
    tmr_output_channel_config(def->instance, timer_channel_val(ch), &tim_oc_init);
    tmr_channel_value_set(def->instance, timer_channel_val(ch), SERVO_PULSE_CENTER);
    tmr_output_channel_buffer_enable(def->instance, timer_channel_val(ch), TRUE);
    tmr_channel_enable(def->instance, timer_channel_val(ch), TRUE);
    tmr_output_enable(def->instance, TRUE);
    tmr_counter_enable(def->instance, TRUE);
  }
}

void servo_pwm_write(const float *values) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const resource_tag_t tag = servo_tags[i];
    if (tag == 0) {
      continue;
    }

    const float val = constrain(values[i], -1.0f, 1.0f);
    const uint16_t pulse = (uint16_t)(SERVO_PULSE_CENTER + (val * (SERVO_PULSE_MAX - SERVO_PULSE_CENTER)));

    const timer_def_t *def = &timer_defs[TIMER_TAG_TIM(tag)];
    tmr_channel_value_set(def->instance, timer_channel_val(TIMER_TAG_CH(tag)), pulse);
  }
}

void servo_pwm_stop() {
  gpio_config_t gpio_init = gpio_config_default();

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const resource_tag_t tag = servo_tags[i];
    if (tag == 0) {
      continue;
    }

    const timer_def_t *def = &timer_defs[TIMER_TAG_TIM(tag)];
    tmr_channel_enable(def->instance, timer_channel_val(TIMER_TAG_CH(tag)), FALSE);
    tmr_counter_enable(def->instance, FALSE);

    for (uint32_t j = 0; j < GPIO_AF_MAX; j++) {
      const gpio_af_t *func = &gpio_pin_afs[j];
      if (func->tag != tag) {
        continue;
      }
      gpio_pin_init(func->pin, gpio_init);
      break;
    }
  }
}

#endif
