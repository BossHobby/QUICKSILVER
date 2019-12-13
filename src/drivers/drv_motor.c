#include "drv_motor.h"

void motor_set_all(float pwm) {
  motor_set(MOTOR_BL, pwm);
  motor_set(MOTOR_FL, pwm);
  motor_set(MOTOR_FR, pwm);
  motor_set(MOTOR_BR, pwm);
}