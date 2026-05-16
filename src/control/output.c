#include "control/output.h"

#include <math.h>

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

void output_apply_mixer_rules(void) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    state.output[i] = 0.0f;
    state.output_active[i] = false;
  }

  for (uint32_t i = 0; i < MIXER_RULE_MAX; i++) {
    const profile_mixer_rule_t *rule = &profile.mixer[i];
    const uint8_t output_index = rule->output_index;
    if (rule->source == OUTPUT_SOURCE_NONE || rule->source >= OUTPUT_SOURCE_MAX || output_index >= MOTOR_PIN_MAX || rule->weight == 0) {
      continue;
    }

    float value = state.mixer_source[rule->source];
    if (rule->source == OUTPUT_SOURCE_RX_CHANNEL) {
      if (rule->source_index >= RX_CHANNEL_MAX) {
        continue;
      }
      value = (float)state.rx_channels[rule->source_index] / (float)AUX_VALUE_MAX;
      value = value * 2.0f - 1.0f;
    }

    state.output[output_index] += value * ((float)rule->weight / 100.0f);
    state.output_active[output_index] = true;
  }
}

void output_activate_count(uint8_t count) {
  if (count > MOTOR_PIN_MAX) {
    count = MOTOR_PIN_MAX;
  }

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    state.output_active[i] = i < count;
  }
}

static bool output_has_mixer_source(uint8_t output_index, output_source_t source) {
  for (uint32_t i = 0; i < MIXER_RULE_MAX; i++) {
    const profile_mixer_rule_t *rule = &profile.mixer[i];
    if (rule->output_index == output_index && rule->source == source && rule->weight != 0) {
      return true;
    }
  }
  return false;
}

static bool output_is_motor_value(uint8_t index, const profile_output_t *output) {
  if (!state.output_active[index] || output->protocol == OUTPUT_PROTOCOL_NONE || output->target_output >= MOTOR_PIN_MAX) {
    return false;
  }

  const target_output_t *target_output = &target.outputs[output->target_output];
  if (target_output->pin == PIN_NONE) {
    return false;
  }

  if (output->protocol == OUTPUT_PROTOCOL_DSHOT) {
    return (target_output->caps & OUTPUT_CAP_DSHOT) != 0;
  }
  if (output->protocol == OUTPUT_PROTOCOL_BRUSHED) {
    return (target_output->caps & OUTPUT_CAP_BRUSHED) != 0;
  }
  if (output->protocol == OUTPUT_PROTOCOL_PWM) {
    return (target_output->caps & OUTPUT_CAP_PWM) != 0 && output_has_mixer_source(index, OUTPUT_SOURCE_THROTTLE);
  }
  return false;
}

static float output_finalize_pwm_motor(float *value, float motor_limit) {
#ifdef VEHICLE_ROVER
  if (profile.rover.reversible) {
    *value = constrain(*value, -1.0f, 1.0f) * motor_limit;
    return fabsf(*value);
  }

  *value = constrain(*value, -1.0f, (motor_limit * 2.0f) - 1.0f);
  return (*value + 1.0f) * 0.5f;
#else
  *value = constrain(*value, -1.0f, 1.0f) * motor_limit;
  return fabsf(*value);
#endif
}

void output_finalize_motor_values(void) {
  state.thrsum = 0.0f;
  uint8_t motor_count = 0;

  float motor_min_value = 0.0f;
  if (!flags.on_ground && flags.arm_state && !flags.motortest_override) {
    // 0.0001 for legacy purposes, force motor drivers downstream to round up
    motor_min_value = 0.0001f + (float)profile.motor.digital_idle * 0.01f;
  }

  const float motor_limit = profile.motor.motor_limit * 0.01f;
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    profile_output_t *output = &profile.outputs[i];
    if (!output_is_motor_value(i, output)) {
      continue;
    }

    if (output->protocol == OUTPUT_PROTOCOL_PWM) {
      state.thrsum += output_finalize_pwm_motor(&state.output[i], motor_limit);
    } else if (!flags.motortest_override) {
      state.output[i] = constrain(state.output[i], 0.0f, 1.0f);
      state.output[i] = mapf(state.output[i], 0.0f, 1.0f, motor_min_value, motor_limit);
      state.thrsum += state.output[i];
    } else if (state.output[i] <= 0.0f) {
      // Zero throttle in motor test must send stop rather than min throttle.
      state.output[i] = MOTOR_OFF;
    } else {
      state.thrsum += state.output[i];
    }

    motor_count++;
  }

  if (motor_count > 0) {
    state.thrsum = state.thrsum / motor_count;
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

void output_write_values() {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const profile_output_t *output = &profile.outputs[i];
    if (output->protocol == OUTPUT_PROTOCOL_NONE || output->target_output >= MOTOR_PIN_MAX) {
      continue;
    }

    if (!output_allowed(output) || !state.output_active[i]) {
      output_stop_slot(output);
      continue;
    }

    output_set_slot(output, state.output[i]);
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
  for (uint32_t i = 0; i < MIXER_RULE_MAX; i++) {
    if (profile.mixer[i].source != source || profile.mixer[i].output_index >= MOTOR_PIN_MAX) {
      continue;
    }
    const profile_output_t *output = &profile.outputs[profile.mixer[i].output_index];
    if (output->protocol == OUTPUT_PROTOCOL_NONE || output->target_output >= MOTOR_PIN_MAX) {
      continue;
    }
    const target_output_t *target_output = &target.outputs[output->target_output];
    if (target_output->pin == PIN_NONE) {
      continue;
    }
    output_caps_t cap = 0;
    if (output->protocol == OUTPUT_PROTOCOL_DSHOT) {
      cap = OUTPUT_CAP_DSHOT;
    } else if (output->protocol == OUTPUT_PROTOCOL_BRUSHED) {
      cap = OUTPUT_CAP_BRUSHED;
    } else if (output->protocol == OUTPUT_PROTOCOL_PWM) {
      cap = OUTPUT_CAP_PWM;
    }
    if ((target_output->caps & cap) != 0) {
      return true;
    }
  }
  return false;
}
