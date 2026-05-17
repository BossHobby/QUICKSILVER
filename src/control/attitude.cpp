#include "control/attitude.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "core/project.h"
#include "driver/time.h"
#include "control/control.h"
#include "io/blackbox.h"
#include "util/quaternion.h"
#include "util/util.h"

#define ACC_1G 2048.0f

// mahony ahrs parameters
#define MAHONY_KP 0.25f // accelerometer correction gain
#define MAHONY_KI 0.0f  // gyro bias estimation gain (disabled)
#define MAHONY_DISARMED_KP_MULTIPLIER 10.0f

// gps heading fusion parameters
#define GPS_HEADING_KP 1.0f             // gps heading correction gain per second
#define GPS_MIN_SPEED 1.0f              // m/s minimum speed for gps
#define GPS_TRUST_SPEED 5.0f            // m/s full trust speed
#define GPS_FULL_HEADING_ACCURACY 5.0f // degrees, full course weight
#define GPS_MAX_HEADING_ACCURACY 15.0f // degrees, reject course at or above this uncertainty
#define GPS_HEADING_STALE_MS 500U
#define CONFIDENCE_TIME_CONSTANT 2.0f  // seconds of consistent forward motion

// limits
#define INTEGRAL_LIMIT 0.5f // rad/s max integral

typedef struct {
  quat_t orientation;
  vec3_t gyro_bias;
  vec3_t integral_fb;
  float yaw_suppression;
  uint32_t last_gps_heading_update_ms;
  bool gps_heading_initialized;
  bool gps_heading_sample_seen;
  bool armed;
} attitude_state_t;

static attitude_state_t attitude_state;

static float mahony_kp_gain(void) {
  return flags.arm_state ? MAHONY_KP : MAHONY_KP * MAHONY_DISARMED_KP_MULTIPLIER;
}

static void mahony_update(vec3_t gyro, vec3_t gravity_estimate, float dt) {
  const float gravity_mag = vec3_magnitude(gravity_estimate);
  if (gravity_mag <= 0.01f) {
    attitude_state.orientation = quat_integrate(&attitude_state.orientation, &gyro, dt);
    return;
  }
  gravity_estimate = vec3_mul(gravity_estimate, 1.0f / gravity_mag);

  // estimated gravity direction from quaternion
  const vec3_t gravity_body = {{
      2.0f * (attitude_state.orientation.x * attitude_state.orientation.z - attitude_state.orientation.w * attitude_state.orientation.y),
      2.0f * (attitude_state.orientation.w * attitude_state.orientation.x + attitude_state.orientation.y * attitude_state.orientation.z),
      attitude_state.orientation.w * attitude_state.orientation.w - attitude_state.orientation.x * attitude_state.orientation.x - attitude_state.orientation.y * attitude_state.orientation.y + attitude_state.orientation.z * attitude_state.orientation.z,
  }};

  // error is cross product between estimated and measured gravity
  const vec3_t error = vec3_cross(gravity_estimate, gravity_body);

  // apply feedback terms
  if (MAHONY_KI > 0.0f) {
    // accumulate integral feedback
    attitude_state.integral_fb = vec3_add(attitude_state.integral_fb, vec3_mul(error, MAHONY_KI * dt));

    // anti-windup: limit integral
    for (int axis = 0; axis < 3; axis++) {
      attitude_state.integral_fb.axis[axis] = constrain(attitude_state.integral_fb.axis[axis], -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    }
  }

  // apply proportional and integral feedback
  gyro = vec3_add(gyro, vec3_mul(error, mahony_kp_gain()));
  gyro = vec3_add(gyro, attitude_state.integral_fb);

  // integrate quaternion
  attitude_state.orientation = quat_integrate(&attitude_state.orientation, &gyro, dt);
}

static bool heading_value_finite(float value) {
  // Hardware builds use -Ofast; isfinite() may be optimized away there.
  const union { float value; uint32_t bits; } sample = {.value = value};
  return (sample.bits & 0x7f800000U) != 0x7f800000U;
}

static void gps_heading_update(float dt) {
  // Follow actual motion, including yaw transients between GPS packets.
  // Betaflight 2026.6 uses a fast suppression / one-second recovery envelope.
  const float yaw_ratio = MIN(fabsf(state.gyro.yaw) * RADTODEG / 10.0f, 1.0f);
  const float yaw_factor = 1.0f - yaw_ratio * yaw_ratio;
  attitude_state.yaw_suppression = MIN(yaw_factor,
      attitude_state.yaw_suppression + dt / (1.0f + dt) * (yaw_factor - attitude_state.yaw_suppression));
  const uint32_t now_ms = time_millis();
  const uint32_t gps_age_ms = now_ms - state.gps_last_update_ms;
  const bool gps_sample_fresh = gps_age_ms <= GPS_HEADING_STALE_MS;
  const bool gps_sample_new = !attitude_state.gps_heading_sample_seen ||
                              state.gps_last_update_ms != attitude_state.last_gps_heading_update_ms;

  uint8_t sensor_blocks = 0;
  if (!state.gps_lock) sensor_blocks |= HEADING_NO_FIX;
  if (!gps_sample_fresh) sensor_blocks |= HEADING_STALE;
  if (!heading_value_finite(state.gps_speed) || state.gps_speed <= GPS_MIN_SPEED)
    sensor_blocks |= HEADING_LOW_SPEED;
  if (!heading_value_finite(state.gps_heading) || !heading_value_finite(state.gps_heading_accuracy) ||
      state.gps_heading_accuracy < 0 || state.gps_heading_accuracy >= GPS_MAX_HEADING_ACCURACY)
    sensor_blocks |= HEADING_POOR_ACCURACY;
  if (sensor_blocks) {
    state.heading_correction_flags = sensor_blocks;
    // Hover alone is not evidence of a bad heading. Retain integrated yaw
    // confidence at low speed; decay it if the receiver becomes unavailable.
    if (sensor_blocks & (HEADING_NO_FIX | HEADING_STALE))
      state.heading_confidence = fmaxf(state.heading_confidence - dt * 0.02f, 0.0f);
    return;
  }
  if (!gps_sample_new) return;
  state.heading_correction_flags = 0;

  float gps_dt = dt;
  if (attitude_state.gps_heading_sample_seen) {
    const uint32_t sample_delta_ms = state.gps_last_update_ms - attitude_state.last_gps_heading_update_ms;
    gps_dt = constrain(sample_delta_ms * 0.001f, dt, 0.2f);
  }
  attitude_state.gps_heading_sample_seen = true;
  attitude_state.last_gps_heading_update_ms = state.gps_last_update_ms;
  // Weight course by how likely motion is to follow the nose. Ground speed
  // alone cannot distinguish forward flight from wind drift or tail-first flight.
  const float roll_angle_deg = fabsf(state.attitude.roll * RADTODEG);
#ifdef VEHICLE_WING
  const float roll_limit = 25.0f;
#else
  const float roll_limit = 10.0f;
#endif
  const float roll_suppression = MAX(1.0f - roll_angle_deg / roll_limit, 0.0f);
  const float speed_ratio = state.gps_speed / 2.0f;
  const float speed_gain = speed_ratio < 1.0f ? speed_ratio * speed_ratio : MIN(speed_ratio, 10.0f);
  float pitch_gain = 1.0f;
#ifdef VEHICLE_MULTI
  pitch_gain = constrain(state.attitude.pitch / (10.0f * DEGTORAD), 0.0f, 5.0f);
#endif

  // speed-based quality factor
  const float speed_quality = (state.gps_speed < GPS_TRUST_SPEED) ?
    constrain((state.gps_speed - GPS_MIN_SPEED) / (GPS_TRUST_SPEED - GPS_MIN_SPEED), 0.0f, 1.0f) : 1.0f;

  float fusion_gain;
  if (state.rth_active && state.rth_state != RTH_STATE_ACQUIRE_HEADING) {
    if (state.rth_state != RTH_STATE_NAVIGATE)
      state.heading_correction_flags |= HEADING_RTH_PHASE;
    // Only forward commands make course a useful nose reference. Level drift
    // and deliberate backwards/sideways braking must not redefine heading.
    if (state.rx_override.pitch <= 0.02f ||
        fabsf(state.rx_override.roll) >= state.rx_override.pitch || state.attitude.pitch <= 0)
      state.heading_correction_flags |= HEADING_NOT_FORWARD;
    const float bearing = state.home_bearing * DEGTORAD;
    const float speed_home = state.gps_vel_north * cosf(bearing) + state.gps_vel_east * sinf(bearing);
    const float recovery = constrain((state.gps_speed - speed_home) / GPS_TRUST_SPEED, 0.0f, 3.5f);
    fusion_gain = GPS_HEADING_KP * speed_quality *
        (constrain(state.attitude.pitch / (30.0f * DEGTORAD), 0.0f, 2.0f) + recovery);
  } else {
    if (roll_suppression <= 0) state.heading_correction_flags |= HEADING_ROLL;
    if (attitude_state.yaw_suppression < 0.1f)
      state.heading_correction_flags |= HEADING_YAW;
    if (pitch_gain <= 0 && (attitude_state.gps_heading_initialized || state.rth_active))
      state.heading_correction_flags |= HEADING_NOT_FORWARD;
    if (flags.controls_override && !state.rth_active)
      state.heading_correction_flags |= HEADING_NOT_FORWARD;
    if (state.rth_active && (state.rx_override.pitch <= 0.02f || state.attitude.pitch < 5.0f * DEGTORAD))
      state.heading_correction_flags |= HEADING_NOT_FORWARD;
    fusion_gain = GPS_HEADING_KP * attitude_state.yaw_suppression * roll_suppression * speed_gain * pitch_gain;
  }
  if (state.heading_correction_flags) return;
  const float accuracy_weight = constrain(
      (GPS_MAX_HEADING_ACCURACY - state.gps_heading_accuracy) /
          (GPS_MAX_HEADING_ACCURACY - GPS_FULL_HEADING_ACCURACY),
      0.0f, 1.0f);
  fusion_gain *= accuracy_weight;

  // Seed once during ordinary flight; never snap yaw during an override.
  if (!attitude_state.gps_heading_initialized) {
    if (!flags.controls_override) {
      attitude_state.gps_heading_initialized = true;
      // Seeding a yaw reference is not proof of its reliability. Confidence
      // still needs multiple consistent GPS observations after this sample.
      float roll, pitch, yaw;
      quat_to_euler(&attitude_state.orientation, &roll, &pitch, &yaw);
      attitude_state.orientation = quat_from_euler(roll, pitch, state.gps_heading * DEGTORAD);
      return;
    }
  }

  // calculate heading error
  float heading_error = state.gps_heading * DEGTORAD - state.attitude.yaw;
  while (heading_error > M_PI_F) {
    heading_error -= 2.0f * M_PI_F;
  }
  while (heading_error < -M_PI_F) {
    heading_error += 2.0f * M_PI_F;
  }

  // Like Betaflight, require alignment within about 15 degrees and weight
  // confidence by useful forward motion, rather than merely passing the gates.
  const float alignment = MAX(1.0f - fabsf(heading_error) / (15.0f * DEGTORAD), 0.0f);
  const float confidence = constrain(fusion_gain * alignment, 0.0f, 1.0f);
  state.heading_confidence += gps_dt / (CONFIDENCE_TIME_CONSTANT + gps_dt) * (confidence - state.heading_confidence);

  // apply gps heading correction if suppression allows
  if (fusion_gain > 0.01f) {
    attitude_state.gps_heading_initialized = true;

    // convert heading error to rotation around z axis (gravity) and apply
    const vec3_t z_axis = {{0, 0, 1}};
    const float correction_limit = (state.rth_active ? 45.0f : 10.0f) * DEGTORAD * gps_dt;
    const float correction_angle = constrain(heading_error * (1.0f - expf(-fusion_gain * gps_dt)),
                                            -correction_limit, correction_limit);
    const quat_t correction = quat_from_axis_angle(&z_axis, correction_angle);
    attitude_state.orientation = quat_multiply(&correction, &attitude_state.orientation);
    quat_normalize(&attitude_state.orientation);
  }

}

void attitude_init(void) {
  memset(&attitude_state, 0, sizeof(attitude_state));
  state.heading_confidence = 0.0f;
  state.heading_correction_flags = HEADING_NO_FIX;

  // initialize to level orientation
  attitude_state.orientation = quat_identity();
  attitude_state.yaw_suppression = 1.0f;
}

void attitude_update(void) {
  if (flags.arm_state && !attitude_state.armed)
    state.heading_confidence = 0.0f; // Re-establish evidence for each flight.
  attitude_state.armed = flags.arm_state;
  // The legacy IMU stores gravity as {right, forward, up} and integrates it
  // with {-pitch, roll, yaw}. In the quaternion body frame gravity evolves
  // as -omega x gravity: swap its horizontal axes and negate gyro pitch.
  const vec3_t gravity = {{state.GEstG.pitch, state.GEstG.roll, state.GEstG.yaw}};
  const vec3_t gyro = {{state.gyro.roll, -state.gyro.pitch, state.gyro.yaw}};
  mahony_update(gyro, gravity, state.looptime);

  // gps heading fusion
  gps_heading_update(state.looptime);

  // extract euler angles and update state
  quat_to_euler(&attitude_state.orientation,
                &state.attitude.roll,
                &state.attitude.pitch,
                &state.attitude.yaw);
  // Preserve Quicksilver's positive nose-down pitch convention.
  state.attitude.pitch = -state.attitude.pitch;

  // update heading (0-360 degrees)
  state.heading = normalize_deg(state.attitude.yaw * RADTODEG);
  // Publish even when GPS fusion rejects a sample, so diagnostics do not freeze.
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 0, (int16_t)(state.heading * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 1, (int16_t)(state.gps_heading * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 3, (int16_t)(state.heading_confidence * 1000));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 7, state.heading_correction_flags);
  // 0.1 degrees; -1 means the receiver supplied a non-finite accuracy.
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 8, heading_value_finite(state.gps_heading_accuracy)
      ? (int16_t)constrain(state.gps_heading_accuracy * 10.0f, 0.0f, 32767.0f) : -1);
}
