#include "control/output.h"

#include "control/control.h"
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

static void output_set_slot(const profile_output_t *output, float value) {
  if (!output || output->target_output >= MOTOR_PIN_MAX) {
    return;
  }

  if (value == MOTOR_OFF && output->protocol != OUTPUT_PROTOCOL_PWM) {
    motor_set(output->target_output, MOTOR_OFF);
    return;
  }

  value = output_apply_config(output, value);

  if (output->protocol == OUTPUT_PROTOCOL_PWM) {
    servo_set(output->target_output, value);
  } else {
    motor_set(output->target_output, value);
  }
}

static void output_stop_slot(const profile_output_t *output) {
  if (!output || output->target_output >= MOTOR_PIN_MAX) {
    return;
  }

  if (output->protocol == OUTPUT_PROTOCOL_PWM) {
    servo_set(output->target_output, 0.0f);
  } else {
    motor_set(output->target_output, MOTOR_OFF);
  }
}

static bool output_source_value(const profile_output_t *output, float *value) {
  switch (output->source) {
  case OUTPUT_SOURCE_NONE:
    return false;

  case OUTPUT_SOURCE_RX_CHANNEL: {
    if (output->source_index >= RX_CHANNEL_MAX) {
      return false;
    }
    const float channel = (float)state.rx_channels[output->source_index] / (float)AUX_VALUE_MAX;
    *value = channel * 2.0f - 1.0f;
    return true;
  }

#ifdef VEHICLE_ROVER
  case OUTPUT_SOURCE_THROTTLE:
    *value = state.motor_mix.axis[0];
    return true;

  case OUTPUT_SOURCE_STEERING:
    *value = state.motor_mix.axis[1];
    return true;
#else
  case OUTPUT_SOURCE_MOTOR_1:
  case OUTPUT_SOURCE_MOTOR_2:
  case OUTPUT_SOURCE_MOTOR_3:
  case OUTPUT_SOURCE_MOTOR_4:
    *value = state.motor_mix.axis[output->source - OUTPUT_SOURCE_MOTOR_1];
    return true;
#endif

  default:
    return false;
  }
}

static bool output_allowed(const profile_output_t *output) {
  if (flags.failsafe && !flags.motortest_override) {
    return false;
  }
  if (output->protocol == OUTPUT_PROTOCOL_PWM) {
    return osd_state.screen_history_size == 0;
  }
  return flags.arm_state || flags.motortest_override;
}

void output_apply_mapped_sources() {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const profile_output_t *output = &profile.outputs[i];
    if (output->protocol == OUTPUT_PROTOCOL_NONE || output->target_output >= MOTOR_PIN_MAX) {
      continue;
    }

    if (!output_allowed(output)) {
      output_stop_slot(output);
      continue;
    }

    float value = 0.0f;
    if (output_source_value(output, &value)) {
      output_set_slot(output, value);
    } else {
      output_stop_slot(output);
    }
  }
}

void output_write_all() {
  motor_update();
  servo_update();
}

void output_stop_all() {
  motor_set_all(MOTOR_OFF);
  motor_update();
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    servo_set(i, 0.0f);
  }
  servo_update();
}

bool output_source_configured(output_source_t source) {
  const profile_output_t *output = profile_output_for_source(source);
  if (!output || output->target_output >= MOTOR_PIN_MAX) {
    return false;
  }
  const target_output_t *target_output = &target.outputs[output->target_output];
  if (target_output->pin == PIN_NONE) {
    return false;
  }
  if (output->protocol == OUTPUT_PROTOCOL_DSHOT) {
    return target.brushless && (target_output->caps & OUTPUT_CAP_MOTOR) != 0;
  }
  if (output->protocol == OUTPUT_PROTOCOL_BRUSHED) {
    return !target.brushless && (target_output->caps & OUTPUT_CAP_MOTOR) != 0;
  }
  if (output->protocol == OUTPUT_PROTOCOL_PWM) {
    return (target_output->caps & (OUTPUT_CAP_MOTOR | OUTPUT_CAP_SERVO)) != 0;
  }
  return false;
}
