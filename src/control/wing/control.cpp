#include "control/wing/control.h"

#include <math.h>
#include <stddef.h>

#include "control/angle_pid.h"
#include "control/control.h"
#include "control/imu.h"
#include "control/output.h"
#include "control/pid.h"
#include "control/rates.h"
#include "core/flash.h"
#include "core/profile.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "osd/render.h"
#include "util/util.h"

typedef enum {
  WING_MODE_MANUAL,
  WING_MODE_ACRO,
  WING_MODE_LEVEL,
} wing_mode_t;

#define WING_AUTOTRIM_CAPTURE_MS 2000
#define WING_LAUNCH_IDLE_SPINUP_MS 1500
#define WING_FLYING_MIN_SPEED 3.5f
#define WING_FLYING_MIN_ALTITUDE 5.0f
#define WING_FLYING_ACCEL_THRESHOLD 0.3f
#define WING_FLYING_ACCEL_TIME_MS 250

static struct {
  uint32_t state_start_ms;
  uint32_t detect_start_ms;
  float throttle_start;
  float takeoff_altitude;
  bool takeoff_altitude_valid;
  bool flying_detected;
  uint32_t flying_detect_start_ms;
} launch;

static struct {
  bool ready = true;
  uint32_t start_ms;
  uint32_t samples;
  struct {
    bool selected;
    int16_t backup;
    float sum;
  } outputs[MOTOR_PIN_MAX];
} autotrim;

motor_test_t motor_test = {
    .active = 0,
    .value = {0.0f},
};

static void wing_apply_outputs(float throttle, float aileron, float elevator, float rudder) {
  state.mixer_source[OUTPUT_SOURCE_THROTTLE] = throttle;
  state.mixer_source[OUTPUT_SOURCE_ROLL] = constrain(aileron, -1.0f, 1.0f);
  state.mixer_source[OUTPUT_SOURCE_PITCH] = constrain(elevator, -1.0f, 1.0f);
  state.mixer_source[OUTPUT_SOURCE_YAW] = constrain(rudder, -1.0f, 1.0f);
  output_apply_mixer_rules();
  output_finalize_motor_values();
  output_write_values();
  output_write_all();
}

static void wing_apply_test_outputs() {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    state.output[i] = motor_test.value[i];
    state.output_active[i] = profile.outputs[i].protocol != OUTPUT_PROTOCOL_NONE;
  }
  output_finalize_motor_values();
  output_write_values();
  output_write_all();
}

static wing_mode_t wing_active_mode() {
  if (rx_aux_on(AUX_LEVELMODE)) {
    return WING_MODE_LEVEL;
  }
  if (rx_aux_on(AUX_ACROMODE)) {
    return WING_MODE_ACRO;
  }
  return WING_MODE_MANUAL;
}

static void wing_calc_stabilized(wing_mode_t mode) {
  if (state.wing_launch_state == WING_LAUNCH_ACTIVE || state.wing_launch_state == WING_LAUNCH_FINISH) {
    const float target_pitch = profile.wing.autolaunch.pitch_angle * DEGTORAD;
    float pitch = target_pitch;
    if (state.wing_launch_state == WING_LAUNCH_FINISH) {
      const uint32_t elapsed_ms = time_millis() - launch.state_start_ms;
      const float t = constrain((float)elapsed_ms / (float)MAX(profile.wing.autolaunch.finish_ms, 1), 0.0f, 1.0f);
      pitch = target_pitch * (1.0f - t) + state.rx_filtered.pitch * profile.rate.level_max_angle * DEGTORAD * t;
    }

    const float max_angle = profile.rate.level_max_angle * DEGTORAD;
    float angle_input[3] = {0.0f, constrain(pitch / max_angle, -1.0f, 1.0f), 0.0f};
    state.angle_error = input_stick_vector(angle_input);
    state.setpoint.roll = angle_pid(0);
    state.setpoint.pitch = angle_pid(1);
    state.setpoint.yaw = 0.0f;
  } else if (mode == WING_MODE_LEVEL) {
    state.angle_error = input_stick_vector(state.rx_filtered.axis);
    state.setpoint.roll = angle_pid(0);
    state.setpoint.pitch = angle_pid(1);
    state.setpoint.yaw = input_rates_calc().yaw;
  } else {
    state.setpoint = input_rates_calc();
  }

  state.error.roll = state.setpoint.roll - state.gyro.roll;
  state.error.pitch = state.setpoint.pitch - state.gyro.pitch;
  state.error.yaw = state.setpoint.yaw - state.gyro.yaw;
  pid_calc();
  state.mixer_source[OUTPUT_SOURCE_THROTTLE] = state.throttle;
  state.mixer_source[OUTPUT_SOURCE_ROLL] = constrain(state.pidoutput.roll, -1.0f, 1.0f);
  state.mixer_source[OUTPUT_SOURCE_PITCH] = constrain(state.pidoutput.pitch, -1.0f, 1.0f);
  state.mixer_source[OUTPUT_SOURCE_YAW] = constrain(state.pidoutput.yaw, -1.0f, 1.0f);
}

static bool wing_launch_sticks_moved() {
  const float deadband = profile.wing.autolaunch.stick_deadband;
  return fabsf(state.rx_filtered.roll) >= deadband ||
         fabsf(state.rx_filtered.pitch) >= deadband;
}

static bool wing_autotrim_surface(uint8_t index) {
  const profile_output_t *output = &profile.outputs[index];
  if (output->protocol != OUTPUT_PROTOCOL_PWM || output->target_output >= MOTOR_PIN_MAX ||
      target.outputs[output->target_output].pin == PIN_NONE ||
      !(target.outputs[output->target_output].caps & OUTPUT_CAP_PWM) || !state.output_active[index]) {
    return false;
  }
  bool surface = false;
  for (uint8_t i = 0; i < MIXER_RULE_MAX; i++) {
    const profile_mixer_rule_t *rule = &profile.mixer[i];
    if (rule->output_index != index || rule->weight == 0) {
      continue;
    }
    if (rule->source == OUTPUT_SOURCE_THROTTLE) {
      return false;
    }
    surface |= rule->source == OUTPUT_SOURCE_ROLL || rule->source == OUTPUT_SOURCE_PITCH || rule->source == OUTPUT_SOURCE_YAW;
  }
  return surface;
}

static void wing_cancel_autotrim() {
  if (state.wing_autotrim_state == WING_AUTOTRIM_SAVE_PENDING) {
    for (uint8_t i = 0; i < MOTOR_PIN_MAX; i++) {
      if (autotrim.outputs[i].selected) {
        profile.outputs[i].trim = autotrim.outputs[i].backup;
      }
    }
  }
  state.wing_autotrim_state = WING_AUTOTRIM_IDLE;
}

static void wing_update_autotrim() {
  if (!rx_aux_on(AUX_AUTOTRIM)) {
    wing_cancel_autotrim();
    autotrim.ready = true;
    return;
  }
  if (flags.failsafe || flags.failsafe_outputs_blocked || flags.motortest_override || osd_state.screen_history_size != 0) {
    wing_cancel_autotrim();
    autotrim.ready = false;
    return;
  }
  if (state.wing_autotrim_state == WING_AUTOTRIM_SAVE_PENDING) {
    if (!flags.arm_state) {
      flash_save();
      state.wing_autotrim_state = WING_AUTOTRIM_SAVED;
    }
    return;
  }
  if (!flags.arm_state) {
    if (state.wing_autotrim_state == WING_AUTOTRIM_ACTIVE) {
      wing_cancel_autotrim();
    }
    return;
  }
  const uint32_t now_ms = time_millis();
  if (autotrim.ready) {
    autotrim.ready = false;
    autotrim.start_ms = now_ms;
    autotrim.samples = 0;
    bool has_surface = false;
    for (uint8_t i = 0; i < MOTOR_PIN_MAX; i++) {
      autotrim.outputs[i].selected = wing_autotrim_surface(i);
      has_surface |= autotrim.outputs[i].selected;
      autotrim.outputs[i].backup = profile.outputs[i].trim;
      autotrim.outputs[i].sum = 0.0f;
    }
    if (!has_surface) {
      return;
    }
    state.wing_autotrim_state = WING_AUTOTRIM_ACTIVE;
  }
  if (state.wing_autotrim_state != WING_AUTOTRIM_ACTIVE) {
    return;
  }

  // Sample each surface once, after mixing, inversion, trim and travel limits.
  // These are normalized commanded positions, not measured servo feedback.
  for (uint8_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (autotrim.outputs[i].selected) {
      autotrim.outputs[i].sum += output_apply_config(&profile.outputs[i], state.output[i]);
    }
  }
  autotrim.samples++;
  if (now_ms - autotrim.start_ms >= WING_AUTOTRIM_CAPTURE_MS) {
    for (uint8_t i = 0; i < MOTOR_PIN_MAX; i++) {
      if (autotrim.outputs[i].selected) {
        profile.outputs[i].trim = constrain((int32_t)lrintf(1000.0f * autotrim.outputs[i].sum / autotrim.samples), -500, 500);
      }
    }
    pid_reset_i();
    state.wing_autotrim_state = WING_AUTOTRIM_SAVE_PENDING;
  }
}

static void wing_launch_set_state(wing_launch_state_t next_state) {
  state.wing_launch_state = next_state;
  launch.state_start_ms = time_millis();
  if (next_state != WING_LAUNCH_WAIT) {
    launch.detect_start_ms = 0;
  }
  if (next_state == WING_LAUNCH_IDLE) {
    launch.flying_detected = false;
    launch.flying_detect_start_ms = 0;
  }
  if (next_state == WING_LAUNCH_MOTOR_DELAY || next_state == WING_LAUNCH_ACTIVE) {
    launch.takeoff_altitude = state.gps_altitude;
    launch.takeoff_altitude_valid = state.gps_lock;
  }
}

static bool wing_launch_abort_allowed(uint32_t elapsed_ms) {
  return elapsed_ms >= profile.wing.autolaunch.min_time_ms && wing_launch_sticks_moved();
}

static bool wing_launch_is_flying() {
  if (launch.flying_detected || state.wing_launch_state == WING_LAUNCH_DONE) {
    return true;
  }

  const bool throttle_condition = state.throttle >= profile.wing.autolaunch.throttle;
  const bool velocity_condition = state.gps_speed > WING_FLYING_MIN_SPEED;
  const bool altitude_condition = launch.takeoff_altitude_valid && fabsf(state.gps_altitude - launch.takeoff_altitude) > WING_FLYING_MIN_ALTITUDE;

  return state.gps_lock && throttle_condition && velocity_condition && altitude_condition;
}

static void wing_launch_update_flying_detected(uint32_t now_ms) {
  if (state.wing_launch_state < WING_LAUNCH_SPINUP || state.wing_launch_state >= WING_LAUNCH_DONE || launch.flying_detected) {
    return;
  }

  if (wing_launch_is_flying()) {
    launch.flying_detected = true;
    return;
  }

  const bool accel_condition = state.accel_raw.pitch > (WING_FLYING_ACCEL_THRESHOLD * ACC_1G);
  if (!accel_condition) {
    launch.flying_detect_start_ms = 0;
    return;
  }
  if (launch.flying_detect_start_ms == 0) {
    launch.flying_detect_start_ms = now_ms;
  }
  if (now_ms - launch.flying_detect_start_ms >= WING_FLYING_ACCEL_TIME_MS) {
    launch.flying_detected = true;
  }
}

static bool wing_launch_probably_not_flying() {
  return !wing_launch_is_flying();
}

static void wing_launch_reset_pids_if_needed() {
  if (!rx_aux_on(AUX_AUTOLAUNCH) || !state.wing_launch_available || state.wing_launch_state >= WING_LAUNCH_DONE) {
    return;
  }
  if (state.wing_launch_state < WING_LAUNCH_SPINUP || wing_launch_probably_not_flying()) {
    pid_reset_i();
  }
}

static float wing_launch_ramp(float start, float end, uint32_t elapsed_ms, uint16_t duration_ms) {
  const float t = constrain((float)elapsed_ms / (float)MAX(duration_ms, 1), 0.0f, 1.0f);
  return start + (end - start) * t;
}

static bool wing_launch_detected() {
  const bool accel_launched = state.accel_raw.pitch > profile.wing.autolaunch.accel_threshold * ACC_1G;
  const bool gps_launched = state.gps_lock && state.gps_speed > profile.wing.autolaunch.velocity_threshold && state.accel_raw.pitch > 0.0f;

  return accel_launched || gps_launched;
}

static bool wing_launch_max_altitude_reached() {
  return profile.wing.autolaunch.max_altitude > 0.0f &&
         launch.takeoff_altitude_valid &&
         state.gps_lock &&
          state.gps_altitude - launch.takeoff_altitude >= profile.wing.autolaunch.max_altitude;
}

static float wing_autolaunch_throttle() {
  const bool launch_aux_on = rx_aux_on(AUX_AUTOLAUNCH);
  if (!flags.arm_state) {
    state.wing_launch_available = launch_aux_on;
  }

  if (!launch_aux_on || !flags.arm_state || flags.failsafe) {
    wing_launch_set_state(WING_LAUNCH_IDLE);
    return state.throttle;
  }
  if (!state.wing_launch_available) {
    wing_launch_set_state(WING_LAUNCH_IDLE);
    return state.throttle;
  }

  const uint32_t now_ms = time_millis();
  const uint32_t elapsed_ms = now_ms - launch.state_start_ms;
  wing_launch_update_flying_detected(now_ms);
  switch (state.wing_launch_state) {
  case WING_LAUNCH_IDLE:
    if (state.rx_filtered.throttle < THROTTLE_SAFETY) {
      return 0.0f;
    }
    if (profile.wing.autolaunch.idle_delay_ms > 0 || profile.wing.autolaunch.idle_throttle > 0.0f) {
      wing_launch_set_state(WING_LAUNCH_IDLE_DELAY);
    } else {
      wing_launch_set_state(WING_LAUNCH_WAIT);
    }
    return 0.0f;

  case WING_LAUNCH_IDLE_DELAY: {
    if (state.rx_filtered.throttle < THROTTLE_SAFETY) {
      wing_launch_set_state(WING_LAUNCH_IDLE);
      return 0.0f;
    }
    const uint32_t idle_wait_ms = profile.wing.autolaunch.idle_delay_ms + (profile.wing.autolaunch.idle_throttle > 0.0f ? WING_LAUNCH_IDLE_SPINUP_MS : 0);
    if (elapsed_ms >= idle_wait_ms) {
      wing_launch_set_state(WING_LAUNCH_WAIT);
    }
    if (profile.wing.autolaunch.idle_throttle <= 0.0f) {
      return 0.0f;
    }
    if (elapsed_ms <= profile.wing.autolaunch.idle_delay_ms) {
      return 0.0f;
    }
    return wing_launch_ramp(0.0f, profile.wing.autolaunch.idle_throttle, elapsed_ms - profile.wing.autolaunch.idle_delay_ms, WING_LAUNCH_IDLE_SPINUP_MS);

  }
  case WING_LAUNCH_WAIT: {
    if (state.rx_filtered.throttle < THROTTLE_SAFETY) {
      wing_launch_set_state(WING_LAUNCH_IDLE);
      return 0.0f;
    }
    const bool detected = wing_launch_detected();
    if (detected) {
      if (launch.detect_start_ms == 0) {
        launch.detect_start_ms = now_ms;
      }
      if (now_ms - launch.detect_start_ms >= profile.wing.autolaunch.detect_time_ms) {
        wing_launch_set_state(WING_LAUNCH_DETECTED);
      }
    } else {
      launch.detect_start_ms = 0;
    }
    return profile.wing.autolaunch.idle_throttle;

  }
  case WING_LAUNCH_DETECTED:
    wing_launch_set_state(WING_LAUNCH_MOTOR_DELAY);
    return profile.wing.autolaunch.idle_throttle;

  case WING_LAUNCH_MOTOR_DELAY:
    if (wing_launch_abort_allowed(elapsed_ms)) {
      wing_launch_set_state(WING_LAUNCH_ABORTED);
      return 0.0f;
    }
    if (elapsed_ms >= profile.wing.autolaunch.motor_delay_ms) {
      launch.throttle_start = profile.wing.autolaunch.idle_throttle;
      wing_launch_set_state(WING_LAUNCH_SPINUP);
    }
    return profile.wing.autolaunch.idle_throttle;

  case WING_LAUNCH_SPINUP:
    if (wing_launch_abort_allowed(profile.wing.autolaunch.motor_delay_ms + elapsed_ms)) {
      wing_launch_set_state(WING_LAUNCH_ABORTED);
      return 0.0f;
    }
    if (elapsed_ms >= profile.wing.autolaunch.spinup_ms) {
      wing_launch_set_state(WING_LAUNCH_ACTIVE);
      return profile.wing.autolaunch.throttle;
    }
    return wing_launch_ramp(launch.throttle_start, profile.wing.autolaunch.throttle, elapsed_ms, profile.wing.autolaunch.spinup_ms);

  case WING_LAUNCH_ACTIVE:
    if (wing_launch_abort_allowed(profile.wing.autolaunch.motor_delay_ms + profile.wing.autolaunch.spinup_ms + elapsed_ms)) {
      wing_launch_set_state(WING_LAUNCH_ABORTED);
      return state.throttle;
    }
    if (elapsed_ms >= profile.wing.autolaunch.timeout_ms || wing_launch_max_altitude_reached()) {
      launch.throttle_start = profile.wing.autolaunch.throttle;
      wing_launch_set_state(WING_LAUNCH_FINISH);
      return launch.throttle_start;
    }
    return profile.wing.autolaunch.throttle;

  case WING_LAUNCH_FINISH:
    if (elapsed_ms >= profile.wing.autolaunch.finish_ms || wing_launch_sticks_moved()) {
      wing_launch_set_state(WING_LAUNCH_DONE);
      return state.throttle;
    }
    return wing_launch_ramp(launch.throttle_start, state.throttle, elapsed_ms, profile.wing.autolaunch.finish_ms);

  case WING_LAUNCH_DONE:
  case WING_LAUNCH_ABORTED:
    state.wing_launch_available = false;
    return state.throttle;
  }
  return state.throttle;
}

static void wing_update_throttle() {
  if (!flags.arm_state) {
    state.throttle = 0.0f;
    flags.in_air = 0;
    return;
  }

  if (state.rx_filtered.throttle < 0.05f) {
    state.throttle = 0.0f;
  } else {
    state.throttle = (state.rx_filtered.throttle - 0.05f) * 1.0526316f;
  }

  if (state.rx_filtered.throttle > THROTTLE_SAFETY) {
    flags.in_air = 1;
  }
}

void control() {
  bool motortest_usb = false;
  if (flags.usb_active && motor_test.active) {
    flags.arm_state = 1;
    flags.on_ground = 0;
    flags.motortest_override = 1;
    flags.controls_override = 0;
    motortest_usb = true;
  } else {
    flags.motortest_override = 0;
  }

  control_failsafe_update();
  if (flags.controls_override) {
    state.rx_filtered = state.rx_override;
  }

  control_update_arming();
  wing_update_throttle();
  state.throttle = wing_autolaunch_throttle();
  wing_launch_reset_pids_if_needed();
  const wing_mode_t wing_mode = wing_active_mode();
  if (wing_mode == WING_MODE_MANUAL && state.wing_launch_state != WING_LAUNCH_ACTIVE && state.wing_launch_state != WING_LAUNCH_FINISH) {
    pid_reset_i();
    state.mixer_source[OUTPUT_SOURCE_THROTTLE] = state.throttle;
    state.mixer_source[OUTPUT_SOURCE_ROLL] = constrain(state.rx_filtered.roll, -1.0f, 1.0f);
    state.mixer_source[OUTPUT_SOURCE_PITCH] = constrain(state.rx_filtered.pitch, -1.0f, 1.0f);
    state.mixer_source[OUTPUT_SOURCE_YAW] = constrain(state.rx_filtered.yaw, -1.0f, 1.0f);
  } else {
    wing_calc_stabilized(wing_mode);
  }

  if (flags.motortest_override) {
    if (motortest_usb) {
      wing_apply_test_outputs();
    } else {
      wing_apply_outputs(state.throttle, state.rx_filtered.roll, state.rx_filtered.pitch, state.rx_filtered.yaw);
    }
  } else if (!flags.arm_state || flags.failsafe_outputs_blocked) {
    flags.on_ground = 1;
    state.throttle = 0.0f;
    wing_apply_outputs(
        MOTOR_OFF,
        state.mixer_source[OUTPUT_SOURCE_ROLL],
        state.mixer_source[OUTPUT_SOURCE_PITCH],
        state.mixer_source[OUTPUT_SOURCE_YAW]);
  } else {
    // Once airborne, retain flight IMU filtering through unpowered glides.
    flags.on_ground = !flags.in_air;
    wing_apply_outputs(
        state.mixer_source[OUTPUT_SOURCE_THROTTLE],
        state.mixer_source[OUTPUT_SOURCE_ROLL],
        state.mixer_source[OUTPUT_SOURCE_PITCH],
        state.mixer_source[OUTPUT_SOURCE_YAW]);
  }
  // Update centers after writing this loop's outputs, as in INAV's switched trim.
  wing_update_autotrim();
}
