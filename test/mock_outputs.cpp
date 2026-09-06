#include "mock_outputs.h"

float pwm_values[MOTOR_PIN_MAX];
uint16_t pwm_rate;
failloop_t fault;

void failloop(failloop_t value) { fault = value; }
void servo_pwm_write(const float *values) {
  for (unsigned i = 0; i < MOTOR_PIN_MAX; i++)
    pwm_values[i] = values[i];
}
void servo_pwm_init(const gpio_pins_t *, uint16_t rate, const float *values) {
  pwm_rate = rate;
  servo_pwm_write(values);
}
void servo_pwm_stop() {}
