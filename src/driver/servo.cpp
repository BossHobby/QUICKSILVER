#include "driver/servo.h"

#include "control/output.h"
#include "core/failloop.h"
#include "core/profile.h"
#include "core/project.h"

#ifdef USE_SERVO

extern void servo_pwm_init(const gpio_pins_t *pins, uint16_t pwm_hz, const float *values);
extern void servo_pwm_write(const float *values);
extern void servo_pwm_stop();

static float servo_values[MOTOR_PIN_MAX];

resource_tag_t servo_alloc_func(gpio_pins_t pin) {
  for (uint32_t i = 0; i < GPIO_AF_MAX; i++) {
    const gpio_af_t *func = &gpio_pin_afs[i];
    if (func->pin != pin || RESOURCE_TAG_TYPE(func->tag) != RESOURCE_TIM) {
      continue;
    }

    const timer_channel_t ch = TIMER_TAG_CH(func->tag);
    if ((ch & (TIMER_CH1N | TIMER_CH2N | TIMER_CH3N | TIMER_CH4N)) != 0) {
      continue;
    }

    if (!timer_alloc_tag(TIMER_USE_SERVO, func->tag)) {
      continue;
    }

    return func->tag;
  }
  return 0;
}

void servo_set(uint8_t index, float value) {
  if (index >= MOTOR_PIN_MAX) {
    return;
  }
  servo_values[index] = value;
}

void servo_update() {
  servo_pwm_write(servo_values);
}

void servo_init() {
  const uint16_t pwm_hz = profile.servo.pwm_rate_hz;
  if (pwm_hz < 50 || pwm_hz > 333) {
    failloop(FAILLOOP_OUTPUT);
    return;
  }

  gpio_pins_t pins[MOTOR_PIN_MAX] = {PIN_NONE};
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const profile_output_t *output = &profile.outputs[i];
    if (output->protocol == OUTPUT_PROTOCOL_PWM && output->target_output < MOTOR_PIN_MAX) {
      servo_values[output->target_output] = output_pwm_stop_value(i);
    }
  }
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (!profile_output_slot_uses_protocol(i, OUTPUT_PROTOCOL_PWM)) {
      continue;
    }

    if ((target.outputs[i].caps & OUTPUT_CAP_PWM) != 0) {
      pins[i] = target.outputs[i].pin;
    }
  }
  servo_pwm_init(pins, pwm_hz, servo_values);
}

void servo_stop() {
  servo_pwm_stop();
}

#else

resource_tag_t servo_alloc_func(gpio_pins_t pin) {
  (void)pin;
  return 0;
}

void servo_set(uint8_t index, float value) {
  (void)index;
  (void)value;
}

void servo_update() {
}

void servo_init() {
}

void servo_stop() {
}

#endif
