#include "control/output.h"

#include "driver/motor.h"
#include "driver/servo.h"
#include "osd/render.h"
#include "util/util.h"

static float output_apply_config(const profile_output_t *output, float value) {
  if (output->invert) {
    value = -value;
  }
  value += (float)output->trim / 1000.0f;
  const float min = (float)output->min / 1000.0f;
  const float max = (float)output->max / 1000.0f;
  return constrain(value, min, max);
}

static bool output_servo_allowed(const profile_output_t *output) {
#ifdef VEHICLE_ROVER
  return output->protocol != OUTPUT_PROTOCOL_SERVO_PWM || osd_state.screen_history_size == 0;
#else
  return true;
#endif
}

void output_set_role(output_role_t role, float value) {
  const profile_output_t *output = profile_output_for_role(role);
  if (!output || output->target_output >= MOTOR_PIN_MAX || !output_servo_allowed(output)) {
    return;
  }

  value = output_apply_config(output, value);

#ifdef VEHICLE_ROVER
  if (output->protocol == OUTPUT_PROTOCOL_SERVO_PWM) {
    servo_set(output->target_output, value);
  } else {
    motor_set(output->target_output, value);
  }
#else
  motor_set(output->target_output, value);
#endif
}

void output_write_all() {
  motor_update();
#ifdef VEHICLE_ROVER
  servo_update();
#endif
}

void output_stop_all() {
  motor_set_all(MOTOR_OFF);
  motor_update();
#ifdef VEHICLE_ROVER
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    servo_set(i, 0.0f);
  }
  servo_update();
#endif
}

bool output_role_configured(output_role_t role) {
  const profile_output_t *output = profile_output_for_role(role);
  if (!output || output->target_output >= MOTOR_PIN_MAX) {
    return false;
  }
  const target_output_t *target_output = &target.outputs[output->target_output];
  if (target_output->pin == PIN_NONE) {
    return false;
  }
  if (output->protocol == OUTPUT_PROTOCOL_DSHOT || output->protocol == OUTPUT_PROTOCOL_MOTOR_PWM) {
    return (target_output->caps & OUTPUT_CAP_MOTOR) != 0;
  }
  if (output->protocol == OUTPUT_PROTOCOL_SERVO_PWM) {
    return (target_output->caps & OUTPUT_CAP_SERVO) != 0;
  }
  return false;
}
