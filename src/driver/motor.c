#include "driver/motor.h"

#include "core/project.h"

static float motor_values[MOTOR_PIN_MAX];

#if !defined(USE_PWM_DRIVER) && !defined(USE_DSHOT_DMA_DRIVER)
void motor_init() {}
void motor_wait_for_ready() {}
void motor_beep() {}
void motor_write(float *values) {}
#endif

void motor_set(uint8_t number, float pwm) {
  if (number > MOTOR_PIN_MAX) {
    return;
  }

  motor_values[number] = pwm;
}

void motor_set_all(float pwm) {
  motor_set(MOTOR_BL, pwm);
  motor_set(MOTOR_FL, pwm);
  motor_set(MOTOR_FR, pwm);
  motor_set(MOTOR_BR, pwm);
}

void motor_update() {
  motor_wait_for_ready();
  motor_write(motor_values);
}