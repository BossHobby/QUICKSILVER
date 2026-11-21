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
#include "core/project.h"
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
  WING_DEBUG_AIRSPEED,        // state.airspeed, cm/s
  WING_DEBUG_AIRSPEED_MEASURED, // state.airspeed is measured, not the throttle model
  WING_DEBUG_WIND_NORTH,      // estimated wind, cm/s
  WING_DEBUG_WIND_EAST,
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
#define WING_FLYING_ACCEL_THRESHOLD 0.3f
#define WING_FLYING_ACCEL_TIME_MS 250

// The pilot chooses the trimmed attitude by turning the switch on in straight
// flight; turns and corrections show as rotation. Outside Manual the
// integrator holds the trim, so the sticks must also be centered.
static constexpr float AUTOTRIM_STICK_CENTER = 0.05f;

// The throttle model is coarse and stays within the WING_SPEED_SCALE limits.
// Idle glides read slow although they keep near cruise airspeed, so the slow
// limit stays close.
// Airspeed lags throttle and attitude changes by about this time constant.
static constexpr float SPEED_THROTTLE_FILTER_TIME = 2.0f; // seconds
// Turn coordination stops growing at this bank and fades out between the
// fade angles; it also stops beyond the pitch limit, where bank is undefined.
static constexpr float TURN_BANK_LIMIT = 60.0f * DEGTORAD;
static constexpr float TURN_FADE_START = 70.0f * DEGTORAD;
static constexpr float TURN_FADE_END = 80.0f * DEGTORAD;
static constexpr float TURN_PITCH_LIMIT = 70.0f * DEGTORAD;
// A climb the airframe cannot power trades airspeed for height. With a
// measured airspeed, navigation's nose-up limit falls from the whole climb
// angle at CLIMB_FULL_SPEED of the cruise speed to level at CLIMB_LEVEL_SPEED,
// so the climb settles where the throttle holds the airspeed between them.
static constexpr float CLIMB_FULL_SPEED = 0.8f;
static constexpr float CLIMB_LEVEL_SPEED = 0.6f;
// A slower airspeed estimate does not turn the wing any faster.
static constexpr float TURN_MIN_SPEED = 5.0f; // m/s
// Wind without an airspeed sensor: a coordinated wing flies along its nose, so
// GPS velocity less the wind has no component across it, as ArduPilot's
// synthetic sideslip. Turning shows the wind from every side; each GPS fix
// corrects the wind and the offset of the gyro heading from the nose.
static constexpr float WIND_WALK = 0.05f;                    // m/s per sqrt(s) the wind may change
static constexpr float WIND_HEADING_WALK = 0.5f * DEGTORAD;  // rad per sqrt(s) the gyro heading may drift
static constexpr float WIND_VELOCITY_NOISE = 0.7f;           // m/s across the nose: GPS velocity and sideslip
static constexpr float WIND_INITIAL_SIGMA = 5.0f;            // m/s per axis
static constexpr float WIND_INITIAL_HEADING_SIGMA = 0.7f;    // rad, of the ground track as the nose direction
static constexpr float WIND_START_MIN_SPEED = 3.0f;          // m/s ground speed that gives the first nose direction
static constexpr float WIND_GATE = 4.0f;                     // innovation limit in standard deviations
static constexpr uint8_t WIND_MAX_REJECTS = 30;              // consecutive fixes, 3 s at 10 Hz, before starting over
// A wing cannot change airspeed faster than this; a GPS velocity that says so
// along the nose is a glitch the gate across it cannot see.
static constexpr float WIND_MAX_ACCELERATION = 10.0f;        // m/s^2
// The wind gives the airspeed once turns have narrowed it to this, or this
// share of the airspeed on a fast wing, whose heading drift moves more wind;
// and the nose has swept this far since the estimate started: straight flight
// and a launch's wobble fit any wind along the track.
static constexpr float WIND_VALID_SIGMA = 1.5f;              // m/s, both axes together
static constexpr float WIND_VALID_SHARE = 0.1f;
static constexpr float WIND_VALID_TURN = 90.0f * DEGTORAD;
// Cruise speed learning converts a measured airspeed to cruise throttle with
// the throttle model, which only holds near cruise: idle glides pin it at its
// limit and would bias the result.
static constexpr float SPEED_LEARN_MAX_SCALE_OFFSET = 0.2f;
// Its throttle per degree of climb is a rough share too: flown, climbs at the
// throttle it calls cruise lost a quarter of the airspeed. Learn in level flight.
static constexpr float SPEED_LEARN_MAX_CLIMB_RATE = 1.0f; // m/s
// The cruise speed stays put through a flight, so gains and the climb limit
// keep their reference; once landed it moves this share of the way to the
// flight's average, measured over at least this many GPS fixes.
static constexpr float SPEED_LEARN_SHARE = 0.5f;
static constexpr uint32_t SPEED_LEARN_MIN_SAMPLES = 100; // 10 s at 10 Hz
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
  // The switch was seen off while armed, so turning it on asks for a capture.
  bool ready;
  bool flown; // flight shown since arming; held through slow headwind legs
  bool steady;
  uint32_t last_ms;
  uint32_t capture_ms;
  uint32_t samples;
  struct {
    bool selected;
    int16_t backup;
    int16_t trim; // captured, applied once the sticks center
    float sum;
  } outputs[MOTOR_PIN_MAX];
} autotrim;

static float speed_throttle;             // filtered level-flight throttle equivalent
static float throttle_speed_scale = 1.0f; // cruise / airspeed by the throttle model
// state.airspeed is measured. The throttle model's cannot show a slow wing: it
// reads the airspeed from the throttle navigation commands.
static bool airspeed_measured;

static struct {
  bool running;
  float heading;      // gyro-integrated earth heading, radians; its offset to the nose is estimated
  float x[3];         // wind north and east, m/s, and that offset, radians
  float p[3][3];      // their covariance
  float airspeed;     // GPS velocity less the wind along the nose at the latest fix, m/s
  float heading_change; // gyro heading change since that fix, kept apart so float resolution holds
  float turned;         // heading change since the estimate started, radians, and its range
  float turned_min;
  float turned_max;
  uint32_t gps_ms;    // latest GPS fix taken
  uint8_t rejects;    // consecutive fixes beyond the gate
} wind;

static struct {
  uint32_t gps_ms;   // latest GPS fix measured at
  uint32_t samples;  // measurements this flight
  float sum;         // of this flight's cruise speed measurements, m/s
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

COLD_CODE static void wing_apply_test_outputs() {
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

// Launch takes over from arming, so an always-on switch leaves the disarmed
// surfaces on the sticks for preflight checks and OSD menus.
static bool wing_launch_in_progress() {
  return flags.arm_state && rx_aux_on(AUX_AUTOLAUNCH) && flags.wing_launch_available &&
         !flags.failsafe && state.wing_launch_state < WING_LAUNCH_DONE;
}

// Body rates of a coordinated turn at the current bank: the heading turns at
// g * tan(bank) / V about earth-up. The angle loop only sees attitude, so
// without these it holds that rotation off, sagging the nose and opposing the
// rudder through every turn.
static vec3_t wing_turn_rates() {
  const float roll = state.attitude.roll;
  const float pitch = state.attitude.pitch;
  // Without a measured airspeed an unset cruise speed leaves none.
  if (!flags.in_air || state.airspeed <= 0.0f || fabsf(pitch) > TURN_PITCH_LIMIT)
    return (vec3_t){};

  const float fade = constrain((TURN_FADE_END - fabsf(roll)) / (TURN_FADE_END - TURN_FADE_START), 0.0f, 1.0f);
  const float bank = constrain(roll, -TURN_BANK_LIMIT, TURN_BANK_LIMIT);
  const float sin_bank = fastsin(bank);
  const float cos_bank = fastcos(bank);
  const float speed = MAX(state.airspeed, TURN_MIN_SPEED);
  const float turn_rate = fade * GRAVITY * sin_bank / (cos_bank * speed);

  // Earth-up rotation in body axes; pitch is positive nose-down.
  const float cos_pitch = fastcos(pitch);
  return (vec3_t){{
      turn_rate * fastsin(pitch),
      -turn_rate * sin_bank * cos_pitch,
      turn_rate * cos_bank * cos_pitch,
  }};
}

// Navigation's pitch target, positive nose-down, held down at low airspeed.
static float wing_nav_pitch() {
  const float cruise_speed = profile.navigation.cruise_speed;
  if (!airspeed_measured || cruise_speed <= 0.0f)
    return wing_nav_command.pitch;
  const float margin = (state.airspeed / cruise_speed - CLIMB_LEVEL_SPEED) / (CLIMB_FULL_SPEED - CLIMB_LEVEL_SPEED);
  return MAX(wing_nav_command.pitch, -WING_MAX_CLIMB_ANGLE * constrain(margin, 0.0f, 1.0f));
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
    state.setpoint = (vec3_t){{angle_pid(0), angle_pid(1), 0.0f}} + wing_turn_rates();
  } else if (mode == WING_MODE_LEVEL) {
    const float max_angle = profile.rate.level_max_angle * DEGTORAD;
    state.angle_error = input_angle_vector(state.rx_filtered.roll * max_angle, state.rx_filtered.pitch * max_angle);
    state.setpoint = (vec3_t){{angle_pid(0), angle_pid(1), input_rates_calc().yaw}};
    if (profile.wing.turn_coordination)
      state.setpoint += wing_turn_rates();
  } else if (mode == WING_MODE_NAV) {
    // Until cruise locks its course, the pilot banks and yaws as in Level.
    const bool steer = state.wing_nav_state == WING_NAV_CRUISE_STEER || state.wing_nav_state == WING_NAV_CRUISE_NO_GPS;
    const float roll = steer ? state.rx_filtered.roll * profile.rate.level_max_angle * DEGTORAD : wing_nav_command.roll;
    state.angle_error = input_angle_vector(roll, wing_nav_pitch());
    state.setpoint = (vec3_t){{angle_pid(0), angle_pid(1), steer ? input_rates_calc().yaw : 0.0f}} + wing_turn_rates();
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

// Roll, pitch and yaw share of a surface command. Channel mixes such as flaps
// remain in the mix after trimming, so the trim must not absorb them.
static float wing_autotrim_surface_command(uint8_t index) {
  float value = 0.0f;
  for (uint8_t i = 0; i < MIXER_RULE_MAX; i++) {
    const profile_mixer_rule_t *rule = &profile.mixer[i];
    if (rule->output_index != index)
      continue;
    if (rule->source == OUTPUT_SOURCE_ROLL || rule->source == OUTPUT_SOURCE_PITCH || rule->source == OUTPUT_SOURCE_YAW)
      value += state.mixer_source[rule->source] * ((float)rule->weight / 100.0f);
  }
  return value;
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

// in_air latches on a throttle run-up, so flight needs further evidence: ground
// speed, a climb or a finished autolaunch since arming. The latch holds because
// a headwind can slow ground speed below the threshold.
static void wing_autotrim_update_flown() {
  if (!flags.arm_state || !flags.in_air) {
    autotrim.flown = false;
    return;
  }
  if ((state.gps_health.lock && state.gps_speed >= WING_FLYING_MIN_SPEED) ||
      (flags.altitude_valid && fabsf(state.altitude) > WING_FLYING_MIN_ALTITUDE) ||
      state.wing_launch_state == WING_LAUNCH_DONE) {
    autotrim.flown = true;
  }
}

static bool wing_autotrim_sticks_centered() {
  return fabsf(state.rx_filtered.roll) < AUTOTRIM_STICK_CENTER &&
         fabsf(state.rx_filtered.pitch) < AUTOTRIM_STICK_CENTER &&
         fabsf(state.rx_filtered.yaw) < AUTOTRIM_STICK_CENTER;
}

static bool wing_autotrim_steady(wing_mode_t mode) {
  // Trim is the command that holds straight flight: not on the ground, during
  // autolaunch, while navigating, turning or still rotating.
  if (!autotrim.flown || wing_launch_in_progress() || state.wing_nav_state != WING_NAV_INACTIVE) {
    return false;
  }
  // Stabilized modes capture the integrator's correction, not stick commands.
  // In Manual the sticks themselves hold the trim.
  if (mode != WING_MODE_MANUAL && !wing_autotrim_sticks_centered()) {
    return false;
  }
  return fabsf(state.gyro.roll) < WING_AUTOTRIM_MAX_RATE &&
         fabsf(state.gyro.pitch) < WING_AUTOTRIM_MAX_RATE &&
         fabsf(state.gyro.yaw) < WING_AUTOTRIM_MAX_RATE;
}

// In Manual the held sticks are the captured trim, so applying it under them
// would double the deflection until they center.
static void wing_autotrim_apply() {
  if (flags.arm_state && !wing_autotrim_sticks_centered()) {
    return;
  }
  for (uint8_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (autotrim.outputs[i].selected) {
      profile.outputs[i].trim = autotrim.outputs[i].trim;
    }
  }
  // The trim now holds the correction the integrator carried.
  pid_reset_i();
  state.wing_autotrim_state = WING_AUTOTRIM_SAVE_PENDING;
}

static void wing_update_autotrim(bool armed_at_start, wing_mode_t mode) {
  wing_autotrim_update_flown();
  if (!rx_aux_on(AUX_AUTOTRIM)) {
    wing_cancel_autotrim();
    // A switch already on at arming must be cycled, so the pilot chooses
    // the moment in flight.
    autotrim.ready = flags.arm_state;
    return;
  }
  if (flags.failsafe || flags.failsafe_outputs_blocked || flags.motortest_override) {
    // A captured trim survives a link loss; an interrupted capture needs the
    // switch cycled.
    if (state.wing_autotrim_state == WING_AUTOTRIM_ACTIVE) {
      wing_cancel_autotrim();
    }
    autotrim.ready = false;
    return;
  }
  if (state.wing_autotrim_state == WING_AUTOTRIM_CENTER_STICKS) {
    wing_autotrim_apply();
  }
  if (state.wing_autotrim_state == WING_AUTOTRIM_SAVE_PENDING) {
    // Flight holds profile_mutex only for passes that start disarmed. The save
    // blocks with interrupts masked, so wait while GPS shows a gliding wing.
    const bool moving = state.gps_health.lock && state.gps_speed >= WING_FLYING_MIN_SPEED;
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
    // Disarming mid-capture ends the request; cycle the switch in flight.
    if (state.wing_autotrim_state == WING_AUTOTRIM_ACTIVE) {
      wing_cancel_autotrim();
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
    state.wing_autotrim_state = has_surface ? WING_AUTOTRIM_ACTIVE : WING_AUTOTRIM_NO_SURFACE;
  }
  if (state.wing_autotrim_state != WING_AUTOTRIM_ACTIVE) {
    return;
  }

  // Capture time only accumulates across consecutive steady passes.
  const uint32_t now_ms = time_millis();
  const bool steady = wing_autotrim_steady(mode);
  if (steady && autotrim.steady) {
    autotrim.capture_ms += now_ms - autotrim.last_ms;
  }
  autotrim.steady = steady;
  autotrim.last_ms = now_ms;
  if (!steady) {
    return;
  }

  // Sample each surface's attitude command once, after inversion, trim and
  // travel limits. These are commanded positions, not servo feedback.
  for (uint8_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (autotrim.outputs[i].selected) {
      autotrim.outputs[i].sum += output_apply_config(&profile.outputs[i], wing_autotrim_surface_command(i));
    }
  }
  autotrim.samples++;
  if (autotrim.capture_ms >= WING_AUTOTRIM_CAPTURE_MS) {
    for (uint8_t i = 0; i < MOTOR_PIN_MAX; i++) {
      if (autotrim.outputs[i].selected) {
        autotrim.outputs[i].trim = constrain((int32_t)lrintf(1000.0f * autotrim.outputs[i].sum / autotrim.samples), -500, 500);
      }
    }
    state.wing_autotrim_state = WING_AUTOTRIM_CENTER_STICKS;
    wing_autotrim_apply();
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
    launch.takeoff_altitude_valid = state.gps_health.lock;
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

  return state.gps_health.lock && throttle_condition && altitude_condition;
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
  if (!rx_aux_on(AUX_AUTOLAUNCH) || !flags.wing_launch_available || state.wing_launch_state >= WING_LAUNCH_DONE) {
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
  const bool gps_launched = state.gps_health.lock && state.gps_speed > profile.wing.autolaunch.velocity_threshold && launch_attitude;

  return wing_launch_throw_accelerating() || gps_launched;
}

static bool wing_launch_max_altitude_reached() {
  return profile.wing.autolaunch.max_altitude > 0.0f &&
         launch.takeoff_altitude_valid &&
         state.gps_health.lock &&
          state.gps_altitude - launch.takeoff_altitude >= profile.wing.autolaunch.max_altitude;
}

// Disarmed, launch only tracks the switch and stays idle.
COLD_CODE static void wing_launch_disarmed(bool launch_aux_on) {
  flags.wing_launch_available = launch_aux_on;
  wing_launch_set_state(WING_LAUNCH_IDLE);
}

static float wing_autolaunch_throttle() {
  const bool launch_aux_on = rx_aux_on(AUX_AUTOLAUNCH);
  if (!flags.arm_state) {
    wing_launch_disarmed(launch_aux_on);
    return state.throttle;
  }
  // Launch runs at most once per arm: DONE or ABORTED holds until disarm, so
  // neither a recovered link nor a re-enabled switch may restart it in flight.
  if (state.wing_launch_state >= WING_LAUNCH_DONE) {
    flags.wing_launch_available = false;
    return state.throttle;
  }
  if (flags.failsafe || !launch_aux_on) {
    if (flags.wing_launch_available) {
      // Throttle already latched in_air before release. A link loss must not
      // let navigation bypass the throw detection or hand-clear delay.
      if (flags.failsafe && state.wing_launch_state < WING_LAUNCH_SPINUP)
        control_failsafe_disarm();
      wing_launch_set_state(WING_LAUNCH_ABORTED);
    }
    flags.wing_launch_available = false;
    return state.throttle;
  }
  if (!flags.wing_launch_available) {
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

// Earth-up turn rate from body rates, rad/s; pitch is positive nose-down.
// Meaningless beyond TURN_PITCH_LIMIT, where its users stop.
static float wing_heading_rate() {
  const float roll = state.attitude.roll;
  return (fastcos(roll) * state.gyro.yaw - fastsin(roll) * state.gyro.pitch) / fastcos(state.attitude.pitch);
}

static bool wing_gps_velocity_fresh() {
  return state.gps_health.lock && time_millis() - state.gps_last_update_ms <= NAV_GPS_STALE_MS;
}

// No wind, and the nose along the ground track, until turns tell them apart.
static void wing_wind_start() {
  const vec2_t velocity = vec3_horizontal(state.gps_velocity);
  wind.x[0] = wind.x[1] = 0.0f;
  wind.x[2] = wrap_rad(atan2approx_rad(velocity.east, velocity.north) - wind.heading);
  for (auto &row : wind.p)
    row[0] = row[1] = row[2] = 0.0f;
  wind.p[0][0] = wind.p[1][1] = WIND_INITIAL_SIGMA * WIND_INITIAL_SIGMA;
  wind.p[2][2] = WIND_INITIAL_HEADING_SIGMA * WIND_INITIAL_HEADING_SIGMA;
  wind.airspeed = vec2_magnitude(velocity);
  wind.turned = wind.turned_min = wind.turned_max = 0.0f;
  wind.rejects = 0;
  wind.running = true;
}

// Correct the wind with each GPS fix in flight. Without fixes it keeps its
// estimate; on the ground and through autolaunch it starts over.
static void wing_update_wind() {
  const bool flying = flags.in_air && !wing_launch_in_progress() && fabsf(state.attitude.pitch) <= TURN_PITCH_LIMIT;
  if (!flying) {
    wind.running = false;
    wind.heading_change = 0.0f;
    return;
  }
  wind.heading_change += wing_heading_rate() * state.looptime;

  const uint32_t gps_ms = state.gps_last_update_ms;
  if (!wing_gps_velocity_fresh() || gps_ms == wind.gps_ms)
    return;
  const float dt = MIN((gps_ms - wind.gps_ms) * 0.001f, 1.0f);
  wind.gps_ms = gps_ms;
  wind.heading = wrap_rad(wind.heading + wind.heading_change);
  wind.turned += wind.heading_change;
  wind.turned_min = MIN(wind.turned_min, wind.turned);
  wind.turned_max = MAX(wind.turned_max, wind.turned);
  wind.heading_change = 0.0f;
  if (!wind.running) {
    // A slower ground track has no direction to start the nose from.
    if (state.gps_speed >= WIND_START_MIN_SPEED)
      wing_wind_start();
    return;
  }

  wind.p[0][0] += WIND_WALK * WIND_WALK * dt;
  wind.p[1][1] += WIND_WALK * WIND_WALK * dt;
  wind.p[2][2] += WIND_HEADING_WALK * WIND_HEADING_WALK * dt;

  // Air velocity along and across the nose; across is the innovation.
  const float nose = wind.heading + wind.x[2];
  const float c = fastcos(nose), s = fastsin(nose);
  const float north = state.gps_velocity.north - wind.x[0];
  const float east = state.gps_velocity.east - wind.x[1];
  const float along = north * c + east * s;
  const float across = north * s - east * c;
  const float h[3] = {-s, c, along};
  float ph[3];
  for (uint32_t i = 0; i < 3; i++)
    ph[i] = wind.p[i][0] * h[0] + wind.p[i][1] * h[1] + wind.p[i][2] * h[2];
  const float variance = h[0] * ph[0] + h[1] * ph[1] + h[2] * ph[2] + WIND_VELOCITY_NOISE * WIND_VELOCITY_NOISE;
  if (across * across > WIND_GATE * WIND_GATE * variance) {
    // Sustained disagreement means the estimate settled on a wrong nose direction.
    if (++wind.rejects > WIND_MAX_REJECTS) {
      wind.running = false;
      return;
    }
  } else {
    wind.rejects = 0;
    for (uint32_t i = 0; i < 3; i++) {
      const float gain = ph[i] / variance;
      wind.x[i] -= gain * across;
      for (uint32_t j = 0; j < 3; j++)
        wind.p[i][j] -= gain * ph[j];
    }
    wind.x[2] = wrap_rad(wind.x[2]);
  }

  // A fix beyond the gate leaves the wind as it was and still gives the
  // airspeed along the nose.
  const float corrected = wind.heading + wind.x[2];
  const float airspeed = (state.gps_velocity.north - wind.x[0]) * fastcos(corrected) + (state.gps_velocity.east - wind.x[1]) * fastsin(corrected);
  wind.airspeed = slew(wind.airspeed, airspeed, WIND_MAX_ACCELERATION * dt);
  // Flying backward through the air is the nose estimate turned around.
  if (wind.airspeed <= 0.0f)
    wind.running = false;
}

// The wind is narrow enough to give the airspeed from GPS velocity.
static bool wing_wind_valid() {
  const float sigma = MAX(WIND_VALID_SIGMA, WIND_VALID_SHARE * wind.airspeed);
  return wind.running && wing_gps_velocity_fresh() && wind.turned_max - wind.turned_min >= WIND_VALID_TURN &&
         wind.p[0][0] + wind.p[1][1] <= sigma * sigma;
}

// Airspeed from the best source: GPS velocity less the wind once turns have
// shown it, else a throttle model. Steady thrust balances
// drag (proportional to V^2) plus the climb component, so the model's V^2
// follows throttle less the climb share navigation feeds forward. On the
// ground the airspeed is zero. Through autolaunch it is the cruise speed:
// in_air latches in the hand, where idle throttle would read as a very slow
// wing at the throw. With cruise throttle unset the model stays at the cruise
// speed.
static void wing_update_airspeed() {
  const float cruise_throttle = profile.navigation.cruise_throttle;
  const float cruise_speed = profile.navigation.cruise_speed;
  airspeed_measured = false;
  flags.wind_valid = wing_wind_valid();
  if (!flags.arm_state || !flags.in_air || wing_launch_in_progress()) {
    speed_throttle = cruise_throttle;
    throttle_speed_scale = 1.0f;
    state.airspeed = flags.arm_state && flags.in_air ? cruise_speed : 0.0f;
    return;
  }

  throttle_speed_scale = 1.0f;
  if (cruise_throttle > 0.0f) {
    // Attitude pitch is positive nose-down.
    const float climb_deg = -state.attitude.pitch * RADTODEG;
    const float level_throttle = state.throttle - WING_THROTTLE_PER_CLIMB_DEG * climb_deg;
    lpf(&speed_throttle, level_throttle, constrain(1.0f - state.looptime / SPEED_THROTTLE_FILTER_TIME, 0.0f, 1.0f));

    const float speed_sq_ratio = constrain(speed_throttle / cruise_throttle,
                                           1.0f / (WING_SPEED_SCALE_MAX * WING_SPEED_SCALE_MAX),
                                           1.0f / (WING_SPEED_SCALE_MIN * WING_SPEED_SCALE_MIN));
    throttle_speed_scale = 1.0f / sqrtf(speed_sq_ratio);
  } else {
    speed_throttle = cruise_throttle;
  }
  state.airspeed = cruise_speed / throttle_speed_scale;

  if (flags.wind_valid) {
    state.wind_speed = {{wind.x[0], wind.x[1]}};
    airspeed_measured = true;
    state.airspeed = wind.airspeed;
  }
}

// Learn the airspeed at cruise throttle from a measured airspeed, converted
// with the throttle model's share of cruise speed. The throttle model's own
// airspeed is the cruise speed itself and teaches nothing. Each flight's
// average moves the cruise speed once the flight ends.
static void wing_learn_cruise_speed() {
  float *cruise_speed = &profile.navigation.cruise_speed;
  if (!flags.in_air) {
    if (speed_learn.samples >= SPEED_LEARN_MIN_SAMPLES) {
      const float current = constrain(*cruise_speed, SPEED_LEARN_MIN, SPEED_LEARN_MAX);
      const float flown = speed_learn.sum / speed_learn.samples;
      *cruise_speed = constrain(current + SPEED_LEARN_SHARE * (flown - current), SPEED_LEARN_MIN, SPEED_LEARN_MAX);
      speed_learn.save_pending |= fabsf(*cruise_speed - speed_learn.saved_speed) >= SPEED_LEARN_SAVE_CHANGE;
    }
    speed_learn.samples = 0;
    speed_learn.sum = 0.0f;
    if (!speed_learn.save_pending)
      speed_learn.saved_speed = *cruise_speed;
    return;
  }
  // Once per GPS fix, the rate the wind airspeed changes at.
  const uint32_t gps_ms = state.gps_last_update_ms;
  if (!airspeed_measured || gps_ms == speed_learn.gps_ms ||
      fabsf(throttle_speed_scale - 1.0f) > SPEED_LEARN_MAX_SCALE_OFFSET || fabsf(state.velocity.up) > SPEED_LEARN_MAX_CLIMB_RATE)
    return;
  speed_learn.gps_ms = gps_ms;
  speed_learn.sum += state.airspeed * throttle_speed_scale;
  speed_learn.samples++;
}

// Store a learned cruise speed once landed, like autotrim. Flight holds
// profile_mutex only for passes that start disarmed, and the save blocks with
// interrupts masked, so wait while GPS shows a gliding wing. The caller checks
// save_pending.
COLD_CODE static void wing_save_cruise_speed(bool armed_at_start) {
  const bool moving = state.gps_health.lock && state.gps_speed >= WING_FLYING_MIN_SPEED;
  if (flags.arm_state || armed_at_start || moving)
    return;
  flash_save();
  flight_reset_runtime();
  speed_learn.saved_speed = profile.navigation.cruise_speed;
  speed_learn.save_pending = false;
}

bool control_failsafe_active() {
  return flags.failsafe_outputs_blocked;
}

HOT_CODE void control() {
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
  wing_update_wind();
  wing_update_airspeed();
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
  wing_update_autotrim(armed_at_start, wing_mode);
  if (speed_learn.save_pending)
    wing_save_cruise_speed(armed_at_start);

  blackbox_set_debug(BBOX_DEBUG_WING, WING_DEBUG_MODE, wing_mode);
  blackbox_set_debug(BBOX_DEBUG_WING, WING_DEBUG_LAUNCH_STATE, state.wing_launch_state);
  blackbox_set_debug(BBOX_DEBUG_WING, WING_DEBUG_AUTOTRIM_STATE, state.wing_autotrim_state);
  blackbox_set_debug(BBOX_DEBUG_WING, WING_DEBUG_FAILSAFE_PHASE, state.failsafe_phase);
  blackbox_set_debug(BBOX_DEBUG_WING, WING_DEBUG_AIRSPEED, (int16_t)constrain(state.airspeed * 100.0f, 0.0f, 32767.0f));
  blackbox_set_debug(BBOX_DEBUG_WING, WING_DEBUG_AIRSPEED_MEASURED, airspeed_measured);
  blackbox_set_debug(BBOX_DEBUG_WING, WING_DEBUG_WIND_NORTH, (int16_t)constrain(state.wind_speed.north * 100.0f, -32768.0f, 32767.0f));
  blackbox_set_debug(BBOX_DEBUG_WING, WING_DEBUG_WIND_EAST, (int16_t)constrain(state.wind_speed.east * 100.0f, -32768.0f, 32767.0f));
}
