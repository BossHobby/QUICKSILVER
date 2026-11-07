#include "driver/servo.h"

#include <hardware/pwm.h>

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "util/util.h"

#ifdef USE_SERVO

#define SERVO_TIMER_FREQ 1000000

static resource_tag_t servo_tags[MOTOR_PIN_MAX];

static uint32_t servo_pwm_channel(resource_tag_t tag) {
  return TIMER_TAG_CH(tag) == TIMER_CH2 ? PWM_CHAN_B : PWM_CHAN_A;
}

static uint16_t servo_pulse(float value) {
  const float val = constrain(value, -1.0f, 1.0f);
  return (uint16_t)(SERVO_PULSE_CENTER + (val * (SERVO_PULSE_MAX - SERVO_PULSE_CENTER)));
}

void servo_pwm_init(const gpio_pins_t *pins, uint16_t pwm_hz, const float *values) {
  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_ALTERNATE;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;

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
    const uint32_t slice = tim - TIMER1;
    gpio_pin_init_tag(pins[i], gpio_init, tag);
    timer_up_init(tim, divider, period - 1);
    pwm_set_chan_level(slice, servo_pwm_channel(tag), servo_pulse(values[i]));
    pwm_set_enabled(slice, true);
  }
}

// Channel levels are double-buffered and take effect at the next wrap.
void servo_pwm_write(const float *values) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const resource_tag_t tag = servo_tags[i];
    if (tag == 0) {
      continue;
    }
    pwm_set_chan_level(TIMER_TAG_TIM(tag) - TIMER1, servo_pwm_channel(tag), servo_pulse(values[i]));
  }
}

void servo_pwm_stop() {
  gpio_config_t gpio_init = gpio_config_default();

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const resource_tag_t tag = servo_tags[i];
    if (tag == 0) {
      continue;
    }

    pwm_set_enabled(TIMER_TAG_TIM(tag) - TIMER1, false);

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
