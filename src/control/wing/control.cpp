#include "control/wing/control.h"

#include <math.h>
#include <stddef.h>

#include "control/angle_pid.h"
#include "control/control.h"
#include "control/imu.h"
#include "control/navigation.h"
#include "control/output.h"
#include "control/pid.h"
#include "control/rates.h"
#include "control/wing/navigation.h"
#include "core/flash.h"
#include "core/profile.h"
#include "core/tasks.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "io/blackbox.h"
#include "util/util.h"

enum {
  WING_DEBUG_MODE,
  WING_DEBUG_LAUNCH_STATE,
  WING_DEBUG_AUTOTRIM_STATE,
  WING_DEBUG_FAILSAFE_PHASE,
};

typedef enum {
  WING_MODE_MANUAL,
  WING_MODE_ACRO,
  WING_MODE_LEVEL,
  WING_MODE_NAV,
} wing_mode_t;

#define WING_AUTOTRIM_CAPTURE_MS 2000
#define WING_AUTOTRIM_MAX_RATE (20.0f * DEGTORAD)
#define WING_LAUNCH_IDLE_SPINUP_MS 1500
#define WING_LAUNCH_GPS_MAX_PITCH (45.0f * DEGTORAD)
#define WING_LAUNCH_MAX_THROW_MS 500
#define WING_FLYING_MIN_SPEED 3.5f
#define WING_FLYING_MIN_ALTITUDE 5.0f
#define WING_FLYING_ACCEL_THRESHOLD 0.3f
#define WING_FLYING_ACCEL_TIME_MS 250

static constexpr float GRAVITY = 9.80665f; // m/s^2
// Cruise-to-current airspeed ratio limits; the throttle proxy is coarse. Idle
// glides read slow although they keep near cruise airspeed, so the slow limit
// stays close.
static constexpr float SPEED_SCALE_MIN = 0.6f;
static constexpr float SPEED_SCALE_MAX = 1.25f;
// Airspeed lags throttle and attitude changes by about this time constant.
static constexpr float SPEED_THROTTLE_FILTER_TIME = 2.0f; // seconds
// Turn coordination stops growing at this bank and fades out between the
// fade angles; it also stops beyond the pitch limit, where bank is undefined.
static constexpr float TURN_BANK_LIMIT = 60.0f * DEGTORAD;
static constexpr float TURN_FADE_START = 70.0f * DEGTORAD;
static constexpr float TURN_FADE_END = 80.0f * DEGTORAD;
static constexpr float TURN_PITCH_LIMIT = 70.0f * DEGTORAD;
// Cruise speed learning compares GPS velocity across a turn of at least this
// much heading change, within this long, while the throttle model's speed
// stays within this fraction.
static constexpr float SPEED_LEARN_HEADING_CHANGE = 20.0f * DEGTORAD;
static constexpr uint32_t SPEED_LEARN_MAX_TURN_MS = 10000;
static constexpr float SPEED_LEARN_MAX_SCALE_CHANGE = 0.05f;
// The throttle model only converts measured airspeed to cruise speed near
// cruise; idle glides pin it at its limit and would bias the result.
static constexpr float SPEED_LEARN_MAX_SCALE_OFFSET = 0.2f;
// The value follows the median of the latest turns, so isolated bad turns
// never reach it, and needs this many turns in a flight before it moves.
static constexpr uint8_t SPEED_LEARN_SAMPLES = 5;
static constexpr uint8_t SPEED_LEARN_MIN_SAMPLES = 3;
static constexpr float SPEED_LEARN_GAIN = 0.2f; // per accepted turn
// The learned value stays within the range a user can set.
static constexpr float SPEED_LEARN_MIN = 5.0f;         // m/s, 18 km/h
static constexpr float SPEED_LEARN_MAX = 50.0f;        // m/s, 180 km/h
static constexpr float SPEED_LEARN_SAVE_CHANGE = 0.5f; // m/s since the last save

static struct {
  uint32_t state_start_ms;
  uint32_t detect_start_ms;
  uint32_t detected_ms;
  uint32_t release_ms; // 0 while the throw is still accelerating.
  float throttle_start;
  float takeoff_altitude;
  bool takeoff_altitude_valid;
  bool flying_detected;
  uint32_t flying_detect_start_ms;
} launch;

static struct {
  bool ready = true;
  bool steady;
  uint32_t last_ms;
  uint32_t capture_ms;
  uint32_t samples;
  struct {
    bool selected;
    int16_t backup;
    float sum;
  } outputs[MOTOR_PIN_MAX];
} autotrim;

static float speed_throttle; // filtered level-flight throttle equivalent

static struct {
  // Gyro-integrated earth heading change since the turn start, radians. Kept
  // relative so float resolution holds for the small per-loop increments.
  float heading_change;
  uint32_t last_gps_ms; // latest GPS fix compared, so heading pairs with its velocity
  bool turn_start_valid;
  uint32_t turn_start_gps_ms;
  float turn_start_north; // GPS velocity, m/s
  float turn_start_east;
  float turn_start_scale;
  float samples[SPEED_LEARN_SAMPLES]; // cruise speed per turn this flight, m/s
  uint8_t sample_count;
  uint8_t sample_next;
  float saved_speed; // profile.navigation.cruise_speed in flash
  bool save_pending;
} speed_learn;

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
  // Navigation runs only when armed, airborne and out of launch. It owns a
  // failsafe when failsafe RTH is enabled, otherwise it has already stopped.
  if (state.wing_nav_state != WING_NAV_INACTIVE) {
    return WING_MODE_NAV;
  }
  // Stage 1 centers the sticks; level the wings rather than hold an attitude.
  if (state.failsafe_phase == FAILSAFE_PHASE_STAGE1_GUARD) {
    return WING_MODE_LEVEL;
  }
  if (rx_aux_on(AUX_LEVELMODE)) {
    return WING_MODE_LEVEL;
  }
  if (rx_aux_on(AUX_ACROMODE)) {
    return WING_MODE_ACRO;
  }
  return WING_MODE_MANUAL;
}

static bool wing_launch_in_progress() {
  return rx_aux_on(AUX_AUTOLAUNCH) && state.wing_launch_available &&
         !flags.failsafe && state.wing_launch_state < WING_LAUNCH_DONE;
}

// Body rates of a coordinated turn at the current bank: the heading turns at
// g * tan(bank) / V about earth-up. The angle loop only sees attitude, so
// without these it holds that rotation off, sagging the nose and opposing the
// rudder through every turn.
static vec3_t wing_turn_rates() {
  const float cruise_speed = profile.navigation.cruise_speed;
  const float roll = state.attitude.roll;
  const float pitch = state.attitude.pitch;
  if (!flags.in_air || cruise_speed <= 0.0f || fabsf(pitch) > TURN_PITCH_LIMIT)
    return (vec3_t){};

  const float fade = constrain((TURN_FADE_END - fabsf(roll)) / (TURN_FADE_END - TURN_FADE_START), 0.0f, 1.0f);
  const float bank = constrain(roll, -TURN_BANK_LIMIT, TURN_BANK_LIMIT);
  const float sin_bank = fastsin(bank);
  const float cos_bank = fastcos(bank);
  const float speed = cruise_speed / state.wing_speed_scale;
  const float turn_rate = fade * GRAVITY * sin_bank / (cos_bank * speed);

  // Earth-up rotation in body axes; pitch is positive nose-down.
  const float cos_pitch = fastcos(pitch);
  return (vec3_t){{
      turn_rate * fastsin(pitch),
      -turn_rate * sin_bank * cos_pitch,
      turn_rate * cos_bank * cos_pitch,
  }};
}

static void wing_calc_stabilized(wing_mode_t mode, bool launch_stabilized) {
  if (launch_stabilized) {
    // pitch_angle is a climb angle; attitude targets are positive nose-down.
    const float target_pitch = -profile.wing.autolaunch.pitch_angle * DEGTORAD;
    float pitch = target_pitch;
    if (state.wing_launch_state == WING_LAUNCH_FINISH) {
      const uint32_t elapsed_ms = time_millis() - launch.state_start_ms;
      const float t = constrain((float)elapsed_ms / (float)MAX(profile.wing.autolaunch.finish_ms, 1), 0.0f, 1.0f);
      pitch = target_pitch * (1.0f - t) + state.rx_filtered.pitch * profile.rate.level_max_angle * DEGTORAD * t;
    }

    // The launch angle is independent of the pilot's level-mode limit.
    state.angle_error = input_angle_vector(0.0f, pitch);
    state.setpoint = vec3_add((vec3_t){{angle_pid(0), angle_pid(1), 0.0f}}, wing_turn_rates());
  } else if (mode == WING_MODE_LEVEL) {
    const float max_angle = profile.rate.level_max_angle * DEGTORAD;
    state.angle_error = input_angle_vector(state.rx_filtered.roll * max_angle, state.rx_filtered.pitch * max_angle);
    state.setpoint = (vec3_t){{angle_pid(0), angle_pid(1), input_rates_calc().yaw}};
    if (profile.wing.turn_coordination)
      state.setpoint = vec3_add(state.setpoint, wing_turn_rates());
  } else if (mode == WING_MODE_NAV) {
    // Until cruise locks its course, the pilot banks and yaws as in Level.
    const bool steer = state.wing_nav_state == WING_NAV_CRUISE_STEER || state.wing_nav_state == WING_NAV_CRUISE_NO_GPS;
    const float roll = steer ? state.rx_filtered.roll * profile.rate.level_max_angle * DEGTORAD : wing_nav_command.roll;
    state.angle_error = input_angle_vector(roll, wing_nav_command.pitch);
    state.setpoint = vec3_add((vec3_t){{angle_pid(0), angle_pid(1), steer ? input_rates_calc().yaw : 0.0f}}, wing_turn_rates());
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

static bool wing_autotrim_steady() {
  // Trim is the command that holds settled flight: not on the ground, during
  // autolaunch, while navigating or while the airframe is still rotating.
  if (!flags.in_air || wing_launch_in_progress() || state.wing_nav_state != WING_NAV_INACTIVE) {
    return false;
  }
  if (state.gps_lock && state.gps_speed < WING_FLYING_MIN_SPEED) {
    return false;
  }
  return fabsf(state.gyro.roll) < WING_AUTOTRIM_MAX_RATE &&
         fabsf(state.gyro.pitch) < WING_AUTOTRIM_MAX_RATE &&
         fabsf(state.gyro.yaw) < WING_AUTOTRIM_MAX_RATE;
}

static void wing_update_autotrim(bool armed_at_start) {
  if (!rx_aux_on(AUX_AUTOTRIM)) {
    wing_cancel_autotrim();
    autotrim.ready = true;
    return;
  }
  if (flags.failsafe || flags.failsafe_outputs_blocked || flags.motortest_override) {
    wing_cancel_autotrim();
    autotrim.ready = false;
    return;
  }
  if (state.wing_autotrim_state == WING_AUTOTRIM_SAVE_PENDING) {
    // Flight holds profile_mutex only for passes that start disarmed. The save
    // blocks with interrupts masked, so wait while GPS shows a gliding wing.
    const bool moving = state.gps_lock && state.gps_speed >= WING_FLYING_MIN_SPEED;
    if (!flags.arm_state && !armed_at_start && !moving) {
      flash_save();
      flight_reset_runtime();
      state.wing_autotrim_state = WING_AUTOTRIM_SAVED;
      // The profile write includes the learned cruise speed.
      speed_learn.saved_speed = profile.navigation.cruise_speed;
      speed_learn.save_pending = false;
    }
    return;
  }
  if (!flags.arm_state) {
    if (state.wing_autotrim_state == WING_AUTOTRIM_ACTIVE) {
      // Landing mid-capture keeps the request: capture again on the next arm.
      wing_cancel_autotrim();
      autotrim.ready = true;
    }
    return;
  }
  if (autotrim.ready) {
    autotrim.ready = false;
    autotrim.steady = false;
    autotrim.capture_ms = 0;
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

  // Capture time only accumulates across consecutive steady passes.
  const uint32_t now_ms = time_millis();
  const bool steady = wing_autotrim_steady();
  if (steady && autotrim.steady) {
    autotrim.capture_ms += now_ms - autotrim.last_ms;
  }
  autotrim.steady = steady;
  autotrim.last_ms = now_ms;
  if (!steady) {
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
  if (autotrim.capture_ms >= WING_AUTOTRIM_CAPTURE_MS) {
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
  if (next_state == WING_LAUNCH_DETECTED) {
    launch.detected_ms = launch.state_start_ms;
    launch.release_ms = 0;
  }
  if (next_state == WING_LAUNCH_MOTOR_DELAY || next_state == WING_LAUNCH_ACTIVE) {
    launch.takeoff_altitude = state.gps_altitude;
    launch.takeoff_altitude_valid = state.gps_lock;
  }
}

static bool wing_launch_abort_allowed(uint32_t now_ms) {
  return now_ms - launch.detected_ms >= profile.wing.autolaunch.min_time_ms && wing_launch_sticks_moved();
}

static bool wing_launch_is_flying() {
  if (launch.flying_detected || state.wing_launch_state == WING_LAUNCH_DONE) {
    return true;
  }

  // GPS speed is ground speed and wind shifts it either way, so climb at
  // launch throttle is the GPS evidence of flight.
  const bool throttle_condition = state.throttle >= profile.wing.autolaunch.throttle;
  const bool altitude_condition = launch.takeoff_altitude_valid && fabsf(state.gps_altitude - launch.takeoff_altitude) > WING_FLYING_MIN_ALTITUDE;

  return state.gps_lock && throttle_condition && altitude_condition;
}

static void wing_launch_update_flying_detected(uint32_t now_ms) {
  if (state.wing_launch_state < WING_LAUNCH_SPINUP || state.wing_launch_state >= WING_LAUNCH_DONE || launch.flying_detected) {
    return;
  }

  if (wing_launch_is_flying()) {
    launch.flying_detected = true;
    return;
  }

  // accel_raw.pitch reads positive nose-down at rest, so forward acceleration is negative.
  const bool accel_condition = -state.accel_raw.pitch > (WING_FLYING_ACCEL_THRESHOLD * ACC_1G);
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

static bool wing_launch_throw_accelerating() {
  // A forward throw reads negative on accel_raw.pitch (positive nose-down at rest).
  return -state.accel_raw.pitch > profile.wing.autolaunch.accel_threshold * ACC_1G;
}

static bool wing_launch_detected() {
  // Attitude is positive nose-down: accept level to 45 degrees nose-up.
  const bool launch_attitude = state.attitude.pitch <= 0.0f && state.attitude.pitch >= -WING_LAUNCH_GPS_MAX_PITCH;
  // A held airframe has no ground speed regardless of wind.
  const bool gps_launched = state.gps_lock && state.gps_speed > profile.wing.autolaunch.velocity_threshold && launch_attitude;

  return wing_launch_throw_accelerating() || gps_launched;
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
    wing_launch_set_state(WING_LAUNCH_IDLE);
    return state.throttle;
  }
  // Launch runs at most once per arm: DONE or ABORTED holds until disarm, so
  // neither a recovered link nor a re-enabled switch may restart it in flight.
  if (state.wing_launch_state >= WING_LAUNCH_DONE) {
    state.wing_launch_available = false;
    return state.throttle;
  }
  if (flags.failsafe || !launch_aux_on) {
    if (state.wing_launch_available) {
      // Throttle already latched in_air before release. A link loss must not
      // let navigation bypass the throw detection or hand-clear delay.
      if (flags.failsafe && state.wing_launch_state < WING_LAUNCH_SPINUP)
        control_failsafe_disarm();
      wing_launch_set_state(WING_LAUNCH_ABORTED);
    }
    state.wing_launch_available = false;
    return state.throttle;
  }
  if (!state.wing_launch_available) {
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
    if (wing_launch_abort_allowed(now_ms)) {
      wing_launch_set_state(WING_LAUNCH_ABORTED);
      return 0.0f;
    }
    // The delay clears the throwing hand, so it starts at release: when the
    // throw stops accelerating, or after a capped throw duration.
    if (launch.release_ms == 0 &&
        (!wing_launch_throw_accelerating() || now_ms - launch.detected_ms >= WING_LAUNCH_MAX_THROW_MS)) {
      launch.release_ms = now_ms;
    }
    if (launch.release_ms != 0 && now_ms - launch.release_ms >= profile.wing.autolaunch.motor_delay_ms) {
      launch.throttle_start = profile.wing.autolaunch.idle_throttle;
      wing_launch_set_state(WING_LAUNCH_SPINUP);
    }
    return profile.wing.autolaunch.idle_throttle;

  case WING_LAUNCH_SPINUP:
    if (wing_launch_abort_allowed(now_ms)) {
      wing_launch_set_state(WING_LAUNCH_ABORTED);
      return 0.0f;
    }
    if (elapsed_ms >= profile.wing.autolaunch.spinup_ms) {
      wing_launch_set_state(WING_LAUNCH_ACTIVE);
      return profile.wing.autolaunch.throttle;
    }
    return wing_launch_ramp(launch.throttle_start, profile.wing.autolaunch.throttle, elapsed_ms, profile.wing.autolaunch.spinup_ms);

  case WING_LAUNCH_ACTIVE:
    if (wing_launch_abort_allowed(now_ms)) {
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

// Estimate cruise / current airspeed without a sensor. Steady thrust balances
// drag (proportional to V^2) plus the climb component, so V^2 follows throttle
// less the climb share navigation feeds forward. 1 on the ground, while
// cruise throttle is unset and through autolaunch: in_air latches in the hand,
// where idle throttle would read as a very slow wing at the throw.
static void wing_update_speed_scale() {
  const float cruise_throttle = profile.navigation.cruise_throttle;
  if (!flags.arm_state || !flags.in_air || cruise_throttle <= 0.0f || wing_launch_in_progress()) {
    speed_throttle = cruise_throttle;
    state.wing_speed_scale = 1.0f;
    return;
  }

  // Attitude pitch is positive nose-down.
  const float climb_deg = -state.attitude.pitch * RADTODEG;
  const float level_throttle = state.throttle - WING_THROTTLE_PER_CLIMB_DEG * climb_deg;
  lpf(&speed_throttle, level_throttle, constrain(1.0f - state.looptime / SPEED_THROTTLE_FILTER_TIME, 0.0f, 1.0f));

  const float speed_sq_ratio = constrain(speed_throttle / cruise_throttle,
                                         1.0f / (SPEED_SCALE_MAX * SPEED_SCALE_MAX),
                                         1.0f / (SPEED_SCALE_MIN * SPEED_SCALE_MIN));
  state.wing_speed_scale = 1.0f / sqrtf(speed_sq_ratio);
}

static float wing_speed_learn_median() {
  float sorted[SPEED_LEARN_SAMPLES];
  const uint8_t count = speed_learn.sample_count;
  for (uint8_t i = 0; i < count; i++) {
    float value = speed_learn.samples[i];
    uint8_t j = i;
    for (; j > 0 && sorted[j - 1] > value; j--)
      sorted[j] = sorted[j - 1];
    sorted[j] = value;
  }
  return count % 2 ? sorted[count / 2] : 0.5f * (sorted[count / 2 - 1] + sorted[count / 2]);
}

// Learn the airspeed at cruise throttle from GPS. Across a turn a steady wind
// cancels from the change in ground velocity, which is airspeed times the
// change in the nose direction. The gyro measures that heading change, so the
// GPS-course-corrected absolute heading never enters.
static void wing_learn_cruise_speed() {
  if (!flags.in_air) {
    speed_learn.turn_start_valid = false;
    speed_learn.sample_count = 0;
    speed_learn.sample_next = 0;
    if (!speed_learn.save_pending)
      speed_learn.saved_speed = profile.navigation.cruise_speed;
    return;
  }

  const float roll = state.attitude.roll;
  const float pitch = state.attitude.pitch;
  if (fabsf(pitch) > TURN_PITCH_LIMIT) {
    speed_learn.turn_start_valid = false;
    return;
  }
  // Earth-up rate from body rates; pitch is positive nose-down.
  const float heading_rate = (fastcos(roll) * state.gyro.yaw - fastsin(roll) * state.gyro.pitch) / fastcos(pitch);
  speed_learn.heading_change += heading_rate * state.looptime;

  const uint32_t gps_ms = state.gps_last_update_ms;
  if (!state.gps_lock || time_millis() - gps_ms > NAV_GPS_STALE_MS || wing_launch_in_progress()) {
    speed_learn.turn_start_valid = false;
    return;
  }
  // Compare only as each fix arrives; between fixes the gyro heading has moved
  // on from the heading its velocity was measured at.
  if (speed_learn.turn_start_valid && gps_ms == speed_learn.last_gps_ms)
    return;
  speed_learn.last_gps_ms = gps_ms;

  const float heading_change = speed_learn.heading_change;
  const bool restart = !speed_learn.turn_start_valid ||
                       fabsf(state.wing_speed_scale - 1.0f) > SPEED_LEARN_MAX_SCALE_OFFSET ||
                       gps_ms - speed_learn.turn_start_gps_ms > SPEED_LEARN_MAX_TURN_MS ||
                       fabsf(state.wing_speed_scale - speed_learn.turn_start_scale) > SPEED_LEARN_MAX_SCALE_CHANGE * speed_learn.turn_start_scale;
  if (!restart && fabsf(heading_change) < SPEED_LEARN_HEADING_CHANGE)
    return;

  if (!restart) {
    const float velocity_change = hypotf(state.gps_vel_north - speed_learn.turn_start_north, state.gps_vel_east - speed_learn.turn_start_east);
    const float nose_change = 2.0f * fabsf(fastsin(0.5f * heading_change));
    const float scale = 0.5f * (state.wing_speed_scale + speed_learn.turn_start_scale);
    speed_learn.samples[speed_learn.sample_next] = velocity_change / nose_change * scale;
    speed_learn.sample_next = (speed_learn.sample_next + 1) % SPEED_LEARN_SAMPLES;
    speed_learn.sample_count = MIN(speed_learn.sample_count + 1, SPEED_LEARN_SAMPLES);
    if (speed_learn.sample_count >= SPEED_LEARN_MIN_SAMPLES) {
      float *cruise_speed = &profile.navigation.cruise_speed;
      const float current = constrain(*cruise_speed, SPEED_LEARN_MIN, SPEED_LEARN_MAX);
      const float median = wing_speed_learn_median();
      *cruise_speed = constrain(current + SPEED_LEARN_GAIN * (median - current), SPEED_LEARN_MIN, SPEED_LEARN_MAX);
      speed_learn.save_pending |= fabsf(*cruise_speed - speed_learn.saved_speed) >= SPEED_LEARN_SAVE_CHANGE;
    }
  }

  // Each turn sample starts the next.
  speed_learn.turn_start_valid = true;
  speed_learn.turn_start_gps_ms = gps_ms;
  speed_learn.heading_change = 0.0f;
  speed_learn.turn_start_north = state.gps_vel_north;
  speed_learn.turn_start_east = state.gps_vel_east;
  speed_learn.turn_start_scale = state.wing_speed_scale;
}

// Store a learned cruise speed once landed, like autotrim. Flight holds
// profile_mutex only for passes that start disarmed, and the save blocks with
// interrupts masked, so wait while GPS shows a gliding wing.
static void wing_save_cruise_speed(bool armed_at_start) {
  const bool moving = state.gps_lock && state.gps_speed >= WING_FLYING_MIN_SPEED;
  if (!speed_learn.save_pending || flags.arm_state || armed_at_start || moving)
    return;
  flash_save();
  flight_reset_runtime();
  speed_learn.saved_speed = profile.navigation.cruise_speed;
  speed_learn.save_pending = false;
}

bool control_failsafe_active() {
  return flags.failsafe_outputs_blocked;
}

void control() {
  const bool armed_at_start = flags.arm_state;
  const float previous_throttle = state.throttle;
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
  // Stage 1 centers the sticks before navigation's next 10 ms update. Keep
  // the applied throttle for that handoff, including navigation's slew seed.
  if (flags.arm_state && flags.in_air && flags.failsafe && profile.navigation.rth_on_failsafe)
    state.throttle = previous_throttle;
  wing_launch_reset_pids_if_needed();
  const wing_mode_t wing_mode = wing_active_mode();
  if (wing_mode == WING_MODE_NAV)
    state.throttle = wing_nav_command.throttle;
  wing_update_speed_scale();
  wing_learn_cruise_speed();
  const bool launch_stabilized = wing_launch_in_progress();
  if (wing_mode == WING_MODE_MANUAL && !launch_stabilized) {
    pid_reset_i();
    state.mixer_source[OUTPUT_SOURCE_THROTTLE] = state.throttle;
    state.mixer_source[OUTPUT_SOURCE_ROLL] = constrain(state.rx_filtered.roll, -1.0f, 1.0f);
    state.mixer_source[OUTPUT_SOURCE_PITCH] = constrain(state.rx_filtered.pitch, -1.0f, 1.0f);
    state.mixer_source[OUTPUT_SOURCE_YAW] = constrain(state.rx_filtered.yaw, -1.0f, 1.0f);
  } else {
    wing_calc_stabilized(wing_mode, launch_stabilized);
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
  wing_update_autotrim(armed_at_start);
  wing_save_cruise_speed(armed_at_start);

  blackbox_set_debug(BBOX_DEBUG_WING, WING_DEBUG_MODE, wing_mode);
  blackbox_set_debug(BBOX_DEBUG_WING, WING_DEBUG_LAUNCH_STATE, state.wing_launch_state);
  blackbox_set_debug(BBOX_DEBUG_WING, WING_DEBUG_AUTOTRIM_STATE, state.wing_autotrim_state);
  blackbox_set_debug(BBOX_DEBUG_WING, WING_DEBUG_FAILSAFE_PHASE, state.failsafe_phase);
}
