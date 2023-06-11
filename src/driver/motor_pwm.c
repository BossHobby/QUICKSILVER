#include "driver/motor_pwm.h"

#include "core/project.h"
#include "driver/time.h"

#ifdef USE_MOTOR_PWM

void motor_pwm_beep() {
  static uint8_t beepon = 0;

  const uint32_t time = time_millis();
  if (!beepon && (time % 2000 < 125)) {
    for (uint32_t i = 0; i <= 3; i++) {
      motor_set(i, MOTOR_PWM_BEEPS_ON);
      beepon = 1;
    }
  } else {
    for (uint32_t i = 0; i <= 3; i++) {
      motor_set(i, MOTOR_PWM_BEEPS_OFF);
      beepon = 0;
    }
  }
}

#endif