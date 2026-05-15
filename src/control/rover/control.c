#include "control/rover/control.h"

#include <math.h>

#include "control/control.h"
#include "control/output.h"
#include "core/profile.h"
#include "core/tasks.h"
#include "driver/motor.h"
#include "io/blackbox.h"
#include "io/usb_configurator.h"
#include "util/filter.h"
#include "util/util.h"

enum {
  ROVER_DEBUG_MODE,
  ROVER_DEBUG_STEERING_CLAMP,
  ROVER_DEBUG_STEERING_SCALE,
  ROVER_DEBUG_STEERING_THROTTLE,
  ROVER_DEBUG_DECEL_FACTOR,
  ROVER_DEBUG_THROTTLE_ASSIST,
  ROVER_DEBUG_ACCEL_FACTOR,
};

#define ROVER_PID_KP_SCALE (1.0f / 628.0f)
#define ROVER_PID_KI_SCALE (1.0f / 100.0f)
#define ROVER_PID_KD_SCALE (1.0f / 37500.0f)
#define ROVER_PID_ITERM_LIMIT 0.35f
#define ROVER_PID_ITERM_RELAX 0.995f
#define ROVER_RATE_THROTTLE_BOOST_MAX 0.15f
#define ROVER_RATE_THROTTLE_CUT_MAX 0.35f
#define ROVER_RATE_THROTTLE_FILTER_HZ 10.0f
#define ROVER_STEERING_THROTTLE_ACCEL_HZ 2.0f
#define ROVER_STEERING_THROTTLE_DECEL_HZ 0.75f
#define ROVER_STEERING_ACCEL_START 0.10f
#define ROVER_STEERING_ACCEL_END 0.60f
#define ROVER_STEERING_DECEL_START 0.10f
#define ROVER_STEERING_DECEL_END 0.60f

static float rover_last_gyro_yaw = 0.0f;
static float rover_ierror_yaw = 0.0f;
static float rover_last_error_yaw = 0.0f;
static float rover_last_error_yaw2 = 0.0f;
static float rover_steering_throttle = 0.0f;
static filter_t rover_dterm_filter;
static filter_state_t rover_dterm_filter_state;
static filter_lp_pt1 rover_rate_throttle_filter;
static filter_state_t rover_rate_throttle_filter_state;

motor_test_t motor_test = {
    .active = 0,
    .value = {0.0f, 0.0f, 0.0f, 0.0f},
};

static rover_steer_mode_t rover_active_steer_mode() {
  if (rx_aux_on(AUX_RATE_THROTTLE)) {
    return ROVER_STEER_MODE_RATE_THROTTLE;
  }
  if (rx_aux_on(AUX_RATE_ASSIST)) {
    return ROVER_STEER_MODE_RATE_ASSIST;
  }
  return ROVER_STEER_MODE_MANUAL;
}

static uint8_t rover_output_index_for_source(output_source_t source) {
  for (uint8_t i = 0; i < MIXER_RULE_MAX; i++) {
    if (profile.mixer[i].source == source) {
      return profile.mixer[i].output_index;
    }
  }
  return MOTOR_PIN_MAX;
}

static float rover_throttle_scale(float throttle) {
  const float breakpoint = constrain(profile.rover.throttle_scale_breakpoint, 0.0f, 1.0f);
  if (breakpoint >= 1.0f) {
    return 1.0f;
  }

  const float factor = constrain(profile.rover.throttle_scale_factor, 0.0f, 1.0f);
  const float throttle_abs = constrain(fabsf(throttle), 0.0f, 1.0f);
  if (throttle_abs <= breakpoint) {
    return 1.0f;
  }

  const float t = (throttle_abs - breakpoint) / (1.0f - breakpoint);
  const float scale = 1.0f - (1.0f - factor) * t * t;
  return constrain(scale, 0.0f, 1.0f);
}

static int16_t rover_debug_scale(float value) {
  return lrintf(constrain(value, -32.0f, 32.0f) * BLACKBOX_SCALE);
}

static float rover_smoothstep(float value, float min, float max) {
  const float t = constrain((value - min) / (max - min), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

static float rover_steering_scale() {
  const float throttle_abs = constrain(fabsf(state.throttle), 0.0f, 1.0f);
  float accel_factor = 0.0f;
  float decel_factor = 0.0f;

  if (!flags.arm_state || flags.failsafe) {
    rover_steering_throttle = 0.0f;
  } else if (throttle_abs > rover_steering_throttle) {
    accel_factor = rover_smoothstep(-state.accel.pitch, ROVER_STEERING_ACCEL_START, ROVER_STEERING_ACCEL_END);
    const float attack_hz = ROVER_STEERING_THROTTLE_ACCEL_HZ * accel_factor;
    const float alpha = constrain(state.looptime * attack_hz, 0.0f, 1.0f);
    rover_steering_throttle += (throttle_abs - rover_steering_throttle) * alpha;
  } else {
    decel_factor = rover_smoothstep(state.accel.pitch, ROVER_STEERING_DECEL_START, ROVER_STEERING_DECEL_END);
    const float decay_hz = ROVER_STEERING_THROTTLE_DECEL_HZ * decel_factor;
    const float alpha = constrain(state.looptime * decay_hz, 0.0f, 1.0f);
    rover_steering_throttle += (throttle_abs - rover_steering_throttle) * alpha;
  }

  const float steering_scale = rover_throttle_scale(rover_steering_throttle);
  blackbox_set_debug(BBOX_DEBUG_ROVER, ROVER_DEBUG_STEERING_SCALE, rover_debug_scale(steering_scale));
  blackbox_set_debug(BBOX_DEBUG_ROVER, ROVER_DEBUG_STEERING_THROTTLE, rover_debug_scale(rover_steering_throttle));
  blackbox_set_debug(BBOX_DEBUG_ROVER, ROVER_DEBUG_DECEL_FACTOR, rover_debug_scale(decel_factor));
  blackbox_set_debug(BBOX_DEBUG_ROVER, ROVER_DEBUG_ACCEL_FACTOR, rover_debug_scale(accel_factor));

  return steering_scale;
}

static float rover_yaw_rate(float yaw_stick) {
  return yaw_stick * profile.rover.yaw_rate * DEGTORAD;
}

static float rover_update_iterm(float yaw_stick, float pid_no_i, float steering_scale) {
  if (!flags.arm_state || flags.failsafe) {
    rover_ierror_yaw = 0.0f;
    rover_last_error_yaw = 0.0f;
    rover_last_error_yaw2 = 0.0f;
    return rover_ierror_yaw;
  }

  const float steer_amount = constrain(fabsf(yaw_stick), 0.0f, 1.0f);
  const float iterm_gain = (1.0f - steer_amount) * (1.0f - steer_amount);
  const float iterm_error = state.error.yaw * iterm_gain;
  const float simpson_sum = rover_last_error_yaw2 + 4.0f * rover_last_error_yaw + iterm_error;
  const float i_delta = simpson_sum * profile.rover.pid.ki * ROVER_PID_KI_SCALE * state.looptime * (1.0f / 3.0f);
  const float next_ierror = constrain(rover_ierror_yaw + i_delta, -ROVER_PID_ITERM_LIMIT, ROVER_PID_ITERM_LIMIT);
  const bool iterm_windup = (pid_no_i + next_ierror >= steering_scale && i_delta > 0.0f) ||
                            (pid_no_i + next_ierror <= -steering_scale && i_delta < 0.0f);

  if (!iterm_windup) {
    rover_ierror_yaw = next_ierror;
  } else {
    rover_ierror_yaw *= ROVER_PID_ITERM_RELAX;
  }
  rover_ierror_yaw *= 1.0f - ((1.0f - ROVER_PID_ITERM_RELAX) * steer_amount);
  rover_last_error_yaw2 = rover_last_error_yaw;
  rover_last_error_yaw = iterm_error;

  return rover_ierror_yaw;
}

static void rover_calc_steering() {
  const rover_steer_mode_t steer_mode = rover_active_steer_mode();
  int16_t debug_throttle_assist = 0;
  int16_t debug_steering_clamp = 0;

  const float drive_direction = state.throttle < 0.0f ? -1.0f : 1.0f;
  const float yaw_stick = state.rx_filtered.yaw;
  const float steering_scale = rover_steering_scale();

  const float gyro_yaw = state.gyro.yaw * drive_direction;
  const float gyro_delta = (state.gyro.yaw - rover_last_gyro_yaw) * drive_direction;
  rover_last_gyro_yaw = state.gyro.yaw;

  state.setpoint = (vec3_t){0};
  state.error = (vec3_t){0};
  state.pid_p_term = (vec3_t){0};
  state.pid_i_term = (vec3_t){0};
  state.pid_d_term = (vec3_t){0};
  state.pidoutput = (vec3_t){0};

  state.setpoint.yaw = rover_yaw_rate(yaw_stick);
  state.error.yaw = state.setpoint.yaw - gyro_yaw;

  switch (steer_mode) {
  case ROVER_STEER_MODE_MANUAL:
    rover_ierror_yaw = 0.0f;
    rover_last_error_yaw = 0.0f;
    rover_last_error_yaw2 = 0.0f;
    state.pidoutput.yaw = yaw_stick * steering_scale;
    break;

  case ROVER_STEER_MODE_RATE_THROTTLE:
  case ROVER_STEER_MODE_RATE_ASSIST: {
    state.pid_p_term.yaw = state.error.yaw * profile.rover.pid.kp * ROVER_PID_KP_SCALE * steering_scale;

    filter_coeff(profile.filter.dterm[0].type, &rover_dterm_filter, profile.filter.dterm[0].cutoff_freq, task_get_period_us(TASK_PID));
    state.pid_d_term.yaw = -filter_step(profile.filter.dterm[0].type, &rover_dterm_filter, &rover_dterm_filter_state, gyro_delta) * profile.rover.pid.kd * ROVER_PID_KD_SCALE * state.looptime_inverse * steering_scale;

    const float pid_no_i = state.pid_p_term.yaw + state.pid_d_term.yaw;
    state.pid_i_term.yaw = rover_update_iterm(yaw_stick, pid_no_i, steering_scale);

    state.pidoutput.yaw = pid_no_i + state.pid_i_term.yaw;
    break;
  }
  }

  if (steer_mode == ROVER_STEER_MODE_RATE_THROTTLE) {
    if (fabsf(state.throttle) > 0.0f) {
      const float yaw_rate_error = fabsf(state.setpoint.yaw) - fabsf(gyro_yaw);
      const float throttle_assist_unclamped = yaw_rate_error * profile.rover.pid.kp * ROVER_PID_KP_SCALE;
      const float throttle_assist_target = constrain(throttle_assist_unclamped, -ROVER_RATE_THROTTLE_CUT_MAX, ROVER_RATE_THROTTLE_BOOST_MAX);
      const float throttle_assist = filter_lp_pt1_step(&rover_rate_throttle_filter, &rover_rate_throttle_filter_state, throttle_assist_target);

      debug_throttle_assist = rover_debug_scale(throttle_assist);

      state.throttle += throttle_assist * copysignf(1.0f, state.throttle);
      state.throttle = constrain(state.throttle, -1.0f, 1.0f);
    } else {
      rover_rate_throttle_filter_state.delay_element[0] = 0.0f;
    }
  } else {
    rover_rate_throttle_filter_state.delay_element[0] = 0.0f;
  }

  const float unconstrained_steering = state.pidoutput.yaw;
  state.pidoutput.yaw = constrain(state.pidoutput.yaw, -steering_scale, steering_scale);
  if (unconstrained_steering < -steering_scale) {
    debug_steering_clamp = -1;
  } else if (unconstrained_steering > steering_scale) {
    debug_steering_clamp = 1;
  }

  blackbox_set_debug(BBOX_DEBUG_ROVER, ROVER_DEBUG_MODE, steer_mode);
  blackbox_set_debug(BBOX_DEBUG_ROVER, ROVER_DEBUG_STEERING_CLAMP, debug_steering_clamp);
  blackbox_set_debug(BBOX_DEBUG_ROVER, ROVER_DEBUG_THROTTLE_ASSIST, debug_throttle_assist);
}

static void rover_apply_outputs(float throttle, float steering) {
  float drive_value = (fabsf(throttle) < 0.001f) ? 0.0f : throttle;
  if (!profile.rover.reversible) {
    drive_value = drive_value * 2.0f - 1.0f;
  }
  state.mixer_source[OUTPUT_SOURCE_THROTTLE] = drive_value;
  state.mixer_source[OUTPUT_SOURCE_ROLL] = 0.0f;
  state.mixer_source[OUTPUT_SOURCE_PITCH] = 0.0f;
  state.mixer_source[OUTPUT_SOURCE_YAW] = constrain(steering, -1.0f, 1.0f);

  output_apply_mixer_rules();
  output_finalize_motor_values();
  output_write_values();
  output_write_all();
}

static float rover_test_value(output_source_t source, float fallback) {
  const uint8_t index = rover_output_index_for_source(source);
  if (index < MOTOR_PIN_MAX) {
    return motor_test.value[index];
  }
  return fallback;
}

static void rover_update_throttle() {
  if (!flags.arm_state) {
    state.throttle = 0.0f;
    flags.in_air = 0;
    return;
  }

  const float rx_thr = state.rx_filtered.throttle;
  const float deadband = profile.rover.center_deadband;

  if (fabsf(rx_thr - 0.5f) < deadband) {
    state.throttle = 0.0f;
  } else if (rx_thr > 0.5f) {
    state.throttle = (rx_thr - 0.5f - deadband) / (0.5f - deadband);
    state.throttle = constrain(state.throttle, 0.0f, 1.0f);
  } else if (profile.rover.reversible) {
    state.throttle = -((0.5f - deadband - rx_thr) / (0.5f - deadband));
    state.throttle = constrain(state.throttle, -1.0f, 0.0f);
  } else {
    state.throttle = 0.0f;
  }

  flags.in_air = 1;
}

void pid_init() {
  filter_init(profile.filter.dterm[0].type, &rover_dterm_filter, &rover_dterm_filter_state, 1,
              profile.filter.dterm[0].cutoff_freq, task_get_period_us(TASK_PID));
  filter_lp_pt1_init(&rover_rate_throttle_filter, &rover_rate_throttle_filter_state, 1,
                     ROVER_RATE_THROTTLE_FILTER_HZ, task_get_period_us(TASK_PID));
  rover_last_gyro_yaw = state.gyro.yaw;
  rover_ierror_yaw = 0.0f;
  rover_last_error_yaw = 0.0f;
  rover_last_error_yaw2 = 0.0f;
  rover_steering_throttle = 0.0f;
}

void control() {
  bool motortest_usb = false;
  if (flags.usb_active && motor_test.active) {
    flags.arm_state = 1;
    flags.on_ground = 0;
    flags.motortest_override = 1;
    motortest_usb = true;
  } else {
    flags.motortest_override = 0;
  }

  control_update_arming();
  rover_update_throttle();

  if (flags.motortest_override) {
    const float throttle = motortest_usb ? rover_test_value(OUTPUT_SOURCE_THROTTLE, MOTOR_OFF) : state.throttle;
    const float steering = motortest_usb ? rover_test_value(OUTPUT_SOURCE_YAW, 0.0f) : 0.0f;
    rover_apply_outputs(throttle, steering);
  } else if (!flags.arm_state || flags.failsafe) {
    flags.on_ground = 1;
    state.throttle = 0.0f;
    rover_calc_steering();
    rover_apply_outputs(0.0f, state.pidoutput.yaw);
  } else {
    flags.on_ground = 0;
    rover_calc_steering();
    rover_apply_outputs(state.throttle, state.pidoutput.yaw);
  }
}
