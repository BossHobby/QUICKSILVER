#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "control/control.h"
#include "control/gyro.h"
#include "control/imu.h"
#include "core/profile.h"
#include "core/project.h"
#include "core/tasks.h"
#include "driver/time.h"
#include "io/blackbox.h"
#include "util/filter.h"
#include "util/quaternion.h"
#include "util/util.h"
#include "util/vector.h"

// Gravity correction time constants in seconds.
#define GRAVITY_GROUND_FILTER_TIME 0.15f
#define GRAVITY_FLIGHT_FILTER_TIME 6.0f
#define ACCEL_FILTER_HZ 10.0f

// accel magnitude limits for drift correction
#define ACC_MIN 0.7f
#define ACC_MAX 1.3f

// mahony ahrs parameters
#define MAHONY_KP 0.25f // accelerometer correction gain
#define MAHONY_DISARMED_KP_MULTIPLIER 10.0f

// gps heading fusion parameters
#define GPS_HEADING_KP 1.0f // gps heading correction gain per second
#define GPS_MIN_SPEED 1.0f // m/s minimum speed for gps
#define GPS_TRUST_SPEED 5.0f // m/s full trust speed
#define GPS_FULL_HEADING_ACCURACY 5.0f // degrees, full course weight
#define GPS_MAX_HEADING_ACCURACY 15.0f // degrees, reject course at or above this uncertainty
#define GPS_HEADING_STALE_MS 500U
#define CONFIDENCE_TIME_CONSTANT 2.0f // seconds of consistent forward motion

#ifdef VEHICLE_WING
#define GPS_HEADING_ROLL_LIMIT 25.0f // degrees
#else
#define GPS_HEADING_ROLL_LIMIT 10.0f // degrees
#endif

typedef struct {
  quat_t orientation;
  float yaw_suppression;
  uint32_t last_gps_heading_update_ms;
  bool gps_heading_initialized;
  bool gps_heading_sample_seen;
  bool armed;
} attitude_state_t;

static attitude_state_t attitude_state;
static bool gps_enabled; // Latched at init, like GPS task registration.
static filter_lp_pt1 accel_filter;
static filter_state_t accel_filter_state[2][3];

static vec3_t mahony_correct(vec3_t gyro, vec3_t gravity_estimate) {
  // Compare normalized measured gravity with the quaternion's predicted gravity.
  const quat_t &q = attitude_state.orientation;
  const vec3_t gravity_body = {{
      2.0f * (q.x * q.z - q.w * q.y),
      2.0f * (q.w * q.x + q.y * q.z),
      q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
  }};

  // error is cross product between estimated and measured gravity
  const vec3_t error = vec3_cross(gravity_estimate, gravity_body);

  const float kp = flags.arm_state ? MAHONY_KP : MAHONY_KP * MAHONY_DISARMED_KP_MULTIPLIER;
  return vec3_add(gyro, vec3_mul(error, kp));
}

static bool imu_value_finite(float value) {
  // Hardware builds use -Ofast; isfinite() may be optimized away there.
  const union { float value; uint32_t bits; } sample = {.value = value};
  return (sample.bits & 0x7f800000U) != 0x7f800000U;
}

static uint8_t gps_heading_sensor_blocks() {
  uint8_t sensor_blocks = 0;
  if (!state.gps_lock) sensor_blocks |= HEADING_NO_FIX;
  if (time_millis() - state.gps_last_update_ms > GPS_HEADING_STALE_MS)
    sensor_blocks |= HEADING_STALE;
  if (!imu_value_finite(state.gps_speed) || state.gps_speed <= GPS_MIN_SPEED)
    sensor_blocks |= HEADING_LOW_SPEED;
  if (!imu_value_finite(state.gps_heading) || !imu_value_finite(state.gps_heading_accuracy) || state.gps_heading_accuracy < 0 || state.gps_heading_accuracy >= GPS_MAX_HEADING_ACCURACY)
    sensor_blocks |= HEADING_POOR_ACCURACY;
  return sensor_blocks;
}

// Publish all motion-related rejection reasons, then weight usable course data.
static float gps_heading_gain() {
  state.heading_correction_flags = 0;
  if (state.rth_active && state.rth_state != RTH_STATE_ACQUIRE_HEADING) {
    if (state.rth_state != RTH_STATE_NAVIGATE)
      state.heading_correction_flags |= HEADING_RTH_PHASE;
    // Course is only a nose reference while commanding forward motion.
    if (state.rx_override.pitch <= 0.02f || fabsf(state.rx_override.roll) >= state.rx_override.pitch || state.attitude.pitch <= 0)
      state.heading_correction_flags |= HEADING_NOT_FORWARD;
    if (state.heading_correction_flags) return 0.0f;

    const float bearing = state.home_bearing * DEGTORAD;
    const float speed_home = state.gps_vel_north * cosf(bearing) + state.gps_vel_east * sinf(bearing);
    const float recovery = constrain((state.gps_speed - speed_home) / GPS_TRUST_SPEED, 0.0f, 3.5f);
    const float speed_quality = constrain((state.gps_speed - GPS_MIN_SPEED) / (GPS_TRUST_SPEED - GPS_MIN_SPEED), 0.0f, 1.0f);
    return GPS_HEADING_KP * speed_quality * (constrain(state.attitude.pitch / (30.0f * DEGTORAD), 0.0f, 2.0f) + recovery);
  }

  // Weight course by how likely motion is to follow the nose.
  // Ground speed alone cannot distinguish forward flight from wind drift or tail-first flight.
  const float roll_angle_deg = fabsf(state.attitude.roll * RADTODEG);
  const float roll_suppression = MAX(1.0f - roll_angle_deg / GPS_HEADING_ROLL_LIMIT, 0.0f);
  float pitch_gain = 1.0f;
#ifdef VEHICLE_MULTI
  pitch_gain = constrain(state.attitude.pitch / (10.0f * DEGTORAD), 0.0f, 5.0f);
#endif

  if (roll_suppression <= 0) state.heading_correction_flags |= HEADING_ROLL;
  if (attitude_state.yaw_suppression < 0.1f)
    state.heading_correction_flags |= HEADING_YAW;
  if (pitch_gain <= 0 && (attitude_state.gps_heading_initialized || state.rth_active))
    state.heading_correction_flags |= HEADING_NOT_FORWARD;
  if (flags.controls_override && !state.rth_active)
    state.heading_correction_flags |= HEADING_NOT_FORWARD;
  if (state.rth_active && (state.rx_override.pitch <= 0.02f || state.attitude.pitch < 5.0f * DEGTORAD))
    state.heading_correction_flags |= HEADING_NOT_FORWARD;
  if (state.heading_correction_flags) return 0.0f;

  const float speed_ratio = state.gps_speed / 2.0f;
  const float speed_gain = speed_ratio < 1.0f ? speed_ratio * speed_ratio : MIN(speed_ratio, 10.0f);
  return GPS_HEADING_KP * attitude_state.yaw_suppression * roll_suppression * speed_gain * pitch_gain;
}

static void gps_heading_update(float dt) {
  // Track yaw transients every loop, including between GPS packets.
  const float yaw_ratio = MIN(fabsf(state.gyro.yaw) * RADTODEG / 10.0f, 1.0f);
  const float yaw_factor = 1.0f - yaw_ratio * yaw_ratio;
  attitude_state.yaw_suppression = MIN(yaw_factor, attitude_state.yaw_suppression + dt / (1.0f + dt) * (yaw_factor - attitude_state.yaw_suppression));

  const uint8_t sensor_blocks = gps_heading_sensor_blocks();
  if (sensor_blocks) {
    state.heading_correction_flags = sensor_blocks;
    // Low speed alone does not invalidate integrated yaw; an unavailable GPS does.
    if (sensor_blocks & (HEADING_NO_FIX | HEADING_STALE))
      state.heading_confidence = fmaxf(state.heading_confidence - dt * 0.02f, 0.0f);
    return;
  }
  if (attitude_state.gps_heading_sample_seen && state.gps_last_update_ms == attitude_state.last_gps_heading_update_ms) return;

  float gps_dt = dt;
  if (attitude_state.gps_heading_sample_seen) {
    const uint32_t sample_delta_ms = state.gps_last_update_ms - attitude_state.last_gps_heading_update_ms;
    gps_dt = constrain(sample_delta_ms * 0.001f, dt, 0.2f);
  }
  attitude_state.gps_heading_sample_seen = true;
  attitude_state.last_gps_heading_update_ms = state.gps_last_update_ms;

  const float motion_gain = gps_heading_gain();
  if (state.heading_correction_flags) return;
  const float accuracy_weight = constrain((GPS_MAX_HEADING_ACCURACY - state.gps_heading_accuracy) / (GPS_MAX_HEADING_ACCURACY - GPS_FULL_HEADING_ACCURACY), 0.0f, 1.0f);
  const float fusion_gain = motion_gain * accuracy_weight;

  // Seed once during ordinary flight; never snap yaw during an override.
  if (!attitude_state.gps_heading_initialized && !flags.controls_override) {
    attitude_state.gps_heading_initialized = true;
    // Seeding a yaw reference is not proof of its reliability.
    // Confidence still needs multiple consistent GPS observations after this sample.
    float roll, pitch, yaw;
    quat_to_euler(&attitude_state.orientation, &roll, &pitch, &yaw);
    attitude_state.orientation = quat_from_euler(roll, pitch, state.gps_heading * DEGTORAD);
    return;
  }

  // calculate heading error
  float heading_error = state.gps_heading * DEGTORAD - state.attitude.yaw;
  while (heading_error > M_PI_F) {
    heading_error -= 2.0f * M_PI_F;
  }
  while (heading_error < -M_PI_F) {
    heading_error += 2.0f * M_PI_F;
  }

  // Like Betaflight, require alignment within about 15 degrees and weight confidence by useful forward motion, rather than merely passing the gates.
  const float alignment = MAX(1.0f - fabsf(heading_error) / (15.0f * DEGTORAD), 0.0f);
  const float confidence = constrain(fusion_gain * alignment, 0.0f, 1.0f);
  state.heading_confidence += gps_dt / (CONFIDENCE_TIME_CONSTANT + gps_dt) * (confidence - state.heading_confidence);

  // apply gps heading correction if suppression allows
  if (fusion_gain > 0.01f) {
    attitude_state.gps_heading_initialized = true;

    // convert heading error to rotation around z axis (gravity) and apply
    const vec3_t z_axis = {{0, 0, 1}};
    const float correction_limit = (state.rth_active ? 45.0f : 10.0f) * DEGTORAD * gps_dt;
    const float correction_angle = constrain(heading_error * (1.0f - expf(-fusion_gain * gps_dt)), -correction_limit, correction_limit);
    const quat_t correction = quat_from_axis_angle(&z_axis, correction_angle);
    attitude_state.orientation = quat_multiply(&correction, &attitude_state.orientation);
    quat_normalize(&attitude_state.orientation);
  }
}

static void imu_update_attitude(bool gravity_valid) {
  if (!gps_enabled) {
    state.attitude.roll = atan2approx_rad(state.GEstG.roll, state.GEstG.yaw);
    const float horizontal = sqrtf(state.GEstG.roll * state.GEstG.roll + state.GEstG.yaw * state.GEstG.yaw);
    state.attitude.pitch = atan2approx_rad(state.GEstG.pitch, horizontal);
    return;
  }

  // The legacy IMU stores gravity as {right, forward, up} and integrates it with {-pitch, roll, yaw}.
  // In the quaternion body frame gravity evolves as -omega x gravity: swap its horizontal axes and negate gyro pitch.
  const vec3_t gravity = {{state.GEstG.pitch, state.GEstG.roll, state.GEstG.yaw}};
  vec3_t gyro = {{state.gyro.roll, -state.gyro.pitch, state.gyro.yaw}};
  if (gravity_valid)
    gyro = mahony_correct(gyro, gravity);
  attitude_state.orientation = quat_integrate(&attitude_state.orientation, &gyro, state.looptime);

  if (flags.arm_state && !attitude_state.armed)
    state.heading_confidence = 0.0f;
  attitude_state.armed = flags.arm_state;
  gps_heading_update(state.looptime);

  // Publish the corrected orientation, after GPS heading correction.
  quat_to_euler(&attitude_state.orientation, &state.attitude.roll, &state.attitude.pitch, &state.attitude.yaw);
  // Preserve Quicksilver's positive nose-down pitch convention.
  state.attitude.pitch = -state.attitude.pitch;

  // Euler extraction already bounds yaw to +/-180 degrees.
  const float heading = state.attitude.yaw * RADTODEG;
  state.heading = heading < 0.0f ? heading + 360.0f : heading;
  // Publish rejected samples too, so diagnostics do not freeze.
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 0, (int16_t)(state.heading * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 1, (int16_t)(state.gps_heading * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 3, (int16_t)(state.heading_confidence * 1000));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 7, state.heading_correction_flags);
  // 0.1 degrees; -1 means the receiver supplied a non-finite accuracy.
  const int16_t heading_accuracy = imu_value_finite(state.gps_heading_accuracy) ? (int16_t)constrain(state.gps_heading_accuracy * 10.0f, 0.0f, 32767.0f) : -1;
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 8, heading_accuracy);
}

static void imu_reset_attitude() {
  gps_enabled = profile.serial.gps != SERIAL_PORT_INVALID;
  state.attitude = {};
  attitude_state = {};
  state.heading = 0.0f;
  state.heading_confidence = 0.0f;
  state.heading_correction_flags = HEADING_NO_FIX;
  attitude_state.orientation = quat_identity();
  attitude_state.yaw_suppression = 1.0f;
}

void imu_filter_update() {
  filter_lp_pt1_coeff(&accel_filter, ACCEL_FILTER_HZ, task_get_period_us(TASK_IMU));
}

void imu_init() {
  // init the gravity vector with accel values
  for (int sample = 0; sample < 100; sample++) {
    gyro_update();

    for (int axis = 0; axis < 3; axis++) {
      lpf(&state.GEstG.axis[axis], state.accel_raw.axis[axis], 0.85f);
    }
    time_delay_us(1000);
  }

  for (auto &pass : accel_filter_state) {
    filter_lp_pt1_init(&accel_filter, pass, 3, ACCEL_FILTER_HZ, task_get_period_us(TASK_IMU));
  }
  imu_reset_attitude();
}

void imu_update() {
  // Predict gravity from the gyro delta, then correct it with filtered acceleration.
  const vec3_t rot = {{
      -state.gyro_delta_angle.axis[1],
      state.gyro_delta_angle.axis[0],
      state.gyro_delta_angle.axis[2],
  }};
  state.GEstG = vec3_rotate(state.GEstG, rot);

  for (int axis = 0; axis < 3; axis++) {
    const float first_pass = filter_lp_pt1_step(&accel_filter, &accel_filter_state[0][axis], state.accel_raw.axis[axis]);
    state.accel.axis[axis] = filter_lp_pt1_step(&accel_filter, &accel_filter_state[1][axis], first_pass);
  }

  const float accmag_squared = vec3_dot(state.accel, state.accel);
  if (accmag_squared > (ACC_MIN * ACC_1G) * (ACC_MIN * ACC_1G) && accmag_squared < (ACC_MAX * ACC_1G) * (ACC_MAX * ACC_1G)) {
    const float accel_scale = ACC_1G / sqrtf(accmag_squared);
    state.accel = vec3_mul(state.accel, accel_scale);

    const float filter_time = flags.on_ground ? GRAVITY_GROUND_FILTER_TIME : GRAVITY_FLIGHT_FILTER_TIME;
    const float filtcoeff = lpfcalc(state.looptime, filter_time);
    lpf(&state.GEstG.roll, state.accel.roll, filtcoeff);
    lpf(&state.GEstG.pitch, state.accel.pitch, filtcoeff);
    lpf(&state.GEstG.yaw, state.accel.yaw, filtcoeff);
  }

  // Normalize once for both control consumers and attitude correction.
  const float gravity_magnitude = vec3_magnitude(state.GEstG);
  const bool gravity_valid = imu_value_finite(gravity_magnitude) && gravity_magnitude > 0.01f;
  if (gravity_valid) {
    state.GEstG = vec3_mul(state.GEstG, ACC_1G / gravity_magnitude);
  }

  imu_update_attitude(gravity_valid);
}

#ifdef PIO_UNIT_TESTING
void imu_test_attitude_init() { imu_reset_attitude(); }

void imu_test_attitude_update() {
  // Isolated attitude tests supply gravity directly, bypassing the IMU stage.
  const float magnitude = vec3_magnitude(state.GEstG);
  const bool valid = imu_value_finite(magnitude) && magnitude > 0.01f;
  if (valid)
    state.GEstG = vec3_mul(state.GEstG, ACC_1G / magnitude);
  imu_update_attitude(valid);
}
#endif
