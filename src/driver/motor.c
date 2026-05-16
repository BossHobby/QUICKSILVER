#include "driver/motor.h"

#include "core/profile.h"
#include "core/project.h"
#include "util/util.h"

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
#ifdef USE_MOTOR_DSHOT
  if (profile_outputs_use_protocol(OUTPUT_PROTOCOL_DSHOT)) {
    target_set_feature(FEATURE_BRUSHLESS);
    motor_dshot_init();
  } else {
    target_reset_feature(FEATURE_BRUSHLESS);
  }
#endif
#ifdef USE_MOTOR_PWM
  if (profile_outputs_use_protocol(OUTPUT_PROTOCOL_BRUSHED)) {
    target_reset_feature(FEATURE_BRUSHLESS);
    motor_pwm_init();
  }
#endif
}

void motor_wait_for_ready() {
#ifdef USE_MOTOR_DSHOT
  if (profile_outputs_use_protocol(OUTPUT_PROTOCOL_DSHOT)) {
    motor_dshot_wait_for_ready();
  }
#endif
}

void motor_beep() {
  motor_wait_for_ready();

#ifdef USE_MOTOR_DSHOT
  if (profile_outputs_use_protocol(OUTPUT_PROTOCOL_DSHOT)) {
    motor_dshot_beep();
  }
#endif
#ifdef USE_MOTOR_PWM
  if (profile_outputs_use_protocol(OUTPUT_PROTOCOL_BRUSHED)) {
    motor_pwm_beep();
  }
#endif
}

void motor_set(uint8_t pos, float pwm) {
  if (pos >= MOTOR_PIN_MAX) {
    return;
  }

  motor_values[pos] = pwm;
}

void motor_set_all(float pwm) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    motor_values[i] = pwm;
  }
}

void motor_update() {
  motor_wait_for_ready();

#ifdef USE_MOTOR_DSHOT
  if (profile_outputs_use_protocol(OUTPUT_PROTOCOL_DSHOT)) {
    motor_dshot_write(motor_values);
  }
#endif
#ifdef USE_MOTOR_PWM
  if (profile_outputs_use_protocol(OUTPUT_PROTOCOL_BRUSHED)) {
    motor_pwm_write(motor_values);
  }
#endif
#ifdef SIMULATOR
  extern float simulator_motor_values[MOTOR_PIN_MAX];
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    simulator_motor_values[i] = motor_values[i];
  }
#endif
}

void motor_set_direction(motor_direction_t dir) {
#ifdef USE_MOTOR_DSHOT
  if (profile_outputs_use_protocol(OUTPUT_PROTOCOL_DSHOT)) {
    motor_dshot_set_direction(dir);
  }
#endif
}

bool motor_direction_change_done() {
#ifdef USE_MOTOR_DSHOT
  if (profile_outputs_use_protocol(OUTPUT_PROTOCOL_DSHOT)) {
    return motor_dshot_direction_change_done();
  }
#endif
  return true;
}
