#include "driver/motor.h"

#include "core/project.h"

static float motor_values[MOTOR_PIN_MAX];

extern void motor_pwm_init();
extern void motor_pwm_beep();
extern void motor_pwm_write(float *values);

extern void motor_dshot_init();
extern void motor_dshot_wait_for_ready();
extern void motor_dshot_write(float *values);
extern void motor_dshot_set_direction(motor_direction_t dir);
extern bool motor_dshot_direction_change_done();
extern void motor_dshot_beep();

void motor_init() {
  if (target.brushless) {
    target_set_feature(FEATURE_BRUSHLESS);
    motor_dshot_init();
  } else {
    target_reset_feature(FEATURE_BRUSHLESS);
    motor_pwm_init();
  }
}

void motor_wait_for_ready() {
  if (target.brushless) {
    motor_dshot_wait_for_ready();
  }
}

void motor_beep() {
  if (target.brushless) {
    motor_dshot_beep();
  } else {
    motor_pwm_beep();
  }
}

void motor_set(motor_position_t pos, float pwm) {
  if ((uint8_t)pos > MOTOR_PIN_MAX) {
    return;
  }

  motor_values[pos] = pwm;
}

void motor_set_all(float pwm) {
  motor_set(MOTOR_BL, pwm);
  motor_set(MOTOR_FL, pwm);
  motor_set(MOTOR_FR, pwm);
  motor_set(MOTOR_BR, pwm);
}

void motor_update() {
  motor_wait_for_ready();
  if (target.brushless) {
    motor_dshot_write(motor_values);
  } else {
    motor_pwm_write(motor_values);
  }
}

void motor_set_direction(motor_direction_t dir) {
  if (target.brushless) {
    motor_dshot_set_direction(dir);
  }
}

bool motor_direction_change_done() {
  if (target.brushless) {
    return motor_dshot_direction_change_done();
  }
  return true;
}
