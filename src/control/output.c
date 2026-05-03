#include "control/output.h"

#include "driver/motor.h"
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

  motor_set(output->target_output, output_apply_config(output, value));
}

void output_write_all() {
  motor_update();
}

void output_stop_all() {
  motor_set_all(MOTOR_OFF);
  motor_update();
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
  return output->protocol == OUTPUT_PROTOCOL_DSHOT || output->protocol == OUTPUT_PROTOCOL_MOTOR_PWM
             ? (target_output->caps & OUTPUT_CAP_MOTOR) != 0
             : false;
}
