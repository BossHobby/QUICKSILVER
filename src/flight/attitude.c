#include "flight/attitude.h"

#include <math.h>
#include <string.h>

#include "core/project.h"
#include "driver/time.h"
#include "flight/control.h"
#include "io/blackbox.h"
#include "util/quaternion.h"
#include "util/util.h"

#define ACC_1G 2048.0f

// mahony ahrs parameters
#define MAHONY_KP 0.25f // accelerometer correction gain
#define MAHONY_KI 0.0f  // gyro bias estimation gain (disabled)

// gps heading fusion parameters
#define GPS_HEADING_KP 0.1f             // gps heading correction gain
#define GPS_MIN_SPEED 2.0f              // m/s minimum speed for gps
#define GPS_TRUST_SPEED 5.0f            // m/s full trust speed
#define GPS_MAX_HEADING_ACCURACY 5.0f   // degrees max accuracy
#define CONFIDENCE_BUILD_RATE 0.5f      // per second
#define CONFIDENCE_DECAY_RATE 2.0f      // per second

// suppression parameters
#define YAW_STICK_DEADBAND 0.05f        // 5% stick deadband
#define YAW_STICK_SUPPRESSION_TIME 2.5f // seconds to decay
#define YAW_STICK_SUPPRESSION_POWER 5.0f // suppression curve exponent
#define ROLL_SUPPRESSION_ANGLE 15.0f    // degrees roll threshold

// limits
#define INTEGRAL_LIMIT 0.5f // rad/s max integral

typedef struct {
  quat_t orientation;
  vec3_t gyro_bias;
  vec3_t integral_fb;
  float heading_confidence;
  float yaw_stick_suppression;
  float last_yaw_stick;
  uint32_t last_stick_time;
  bool gps_heading_initialized;
} attitude_state_t;

static attitude_state_t attitude_state;

static void mahony_update(vec3_t gyro, vec3_t gravity_estimate, float dt) {
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
  gyro = vec3_add(gyro, vec3_mul(error, MAHONY_KP));
  gyro = vec3_add(gyro, attitude_state.integral_fb);

  // integrate quaternion
  attitude_state.orientation = quat_integrate(&attitude_state.orientation, &gyro, dt);
}

static void gps_heading_update(float dt) {
  // check if we should use gps heading
  if (!state.gps_lock ||
      state.gps_speed < GPS_MIN_SPEED ||
      state.gps_heading_accuracy > GPS_MAX_HEADING_ACCURACY) {
    attitude_state.gps_heading_initialized = false;
    attitude_state.heading_confidence *= 0.95f; // decay confidence
    return;
  }

  // update yaw stick suppression
  const float yaw_input = fabsf(state.rx_filtered.yaw);
  if (yaw_input > YAW_STICK_DEADBAND) {
    // immediate suppression using power curve
    attitude_state.yaw_stick_suppression = powf(1.0f - min(yaw_input, 1.0f), YAW_STICK_SUPPRESSION_POWER);
    attitude_state.last_stick_time = time_micros();
  } else {
    // decay suppression back to 1.0
    const float decay_factor = expf(-(time_micros() - attitude_state.last_stick_time) * 1e-6f / YAW_STICK_SUPPRESSION_TIME);
    attitude_state.yaw_stick_suppression = 1.0f - (1.0f - attitude_state.yaw_stick_suppression) * decay_factor;
  }

  // roll angle suppression
  const float roll_angle_deg = fabsf(state.attitude.roll * RADTODEG);
  const float roll_suppression = (roll_angle_deg > ROLL_SUPPRESSION_ANGLE) ? 0.0f :
    (roll_angle_deg > ROLL_SUPPRESSION_ANGLE * 0.5f) ? 
    1.0f - (roll_angle_deg - ROLL_SUPPRESSION_ANGLE * 0.5f) / (ROLL_SUPPRESSION_ANGLE * 0.5f) : 1.0f;

  // speed-based quality factor
  const float speed_quality = (state.gps_speed < GPS_TRUST_SPEED) ?
    constrain((state.gps_speed - GPS_MIN_SPEED) / (GPS_TRUST_SPEED - GPS_MIN_SPEED), 0.0f, 1.0f) : 1.0f;

  // build confidence

  // initialize on first valid gps heading
  if (!attitude_state.gps_heading_initialized) {
    // reset yaw to gps heading
    float roll, pitch, yaw;
    quat_to_euler(&attitude_state.orientation, &roll, &pitch, &yaw);
    attitude_state.orientation = quat_from_euler(roll, pitch, state.gps_heading * DEGTORAD);
    attitude_state.gps_heading_initialized = true;
    attitude_state.heading_confidence = 0.5f;
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

  // update confidence based on error magnitude
  if (fabsf(heading_error) < 30.0f * DEGTORAD && state.gps_speed > GPS_MIN_SPEED) {
    attitude_state.heading_confidence = fminf(attitude_state.heading_confidence + dt * CONFIDENCE_BUILD_RATE, 1.0f);
  } else {
    attitude_state.heading_confidence = fmaxf(attitude_state.heading_confidence - dt * CONFIDENCE_DECAY_RATE, 0.0f);
  }

  // calculate total suppression
  const float total_suppression = attitude_state.yaw_stick_suppression *
                                  roll_suppression *
                                  speed_quality *
                                  attitude_state.heading_confidence;

  // apply gps heading correction if suppression allows
  if (total_suppression > 0.01f) {
    float fusion_gain = GPS_HEADING_KP * total_suppression;

    // boost gain at higher speeds for faster convergence
    if (state.gps_speed >= GPS_TRUST_SPEED) {
      fusion_gain *= 2.0f;
    }

    // convert heading error to rotation around z axis (gravity) and apply
    const vec3_t z_axis = {{0, 0, 1}};
    const quat_t correction = quat_from_axis_angle(&z_axis, heading_error * fusion_gain);
    attitude_state.orientation = quat_multiply(&correction, &attitude_state.orientation);
    quat_normalize(&attitude_state.orientation);
  }

  // log gps heading fusion to blackbox (part of navigation debug)
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 0, (int16_t)(state.heading * 10)); // Current heading (fused)
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 1, (int16_t)(state.gps_heading * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 2, (int16_t)(heading_error * RADTODEG * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 3, (int16_t)(attitude_state.heading_confidence * 1000)); // More useful than suppression
}

void attitude_init(void) {
  memset(&attitude_state, 0, sizeof(attitude_state));

  // initialize to level orientation
  attitude_state.orientation = quat_identity();
  attitude_state.yaw_stick_suppression = 1.0f;
}

void attitude_update(void) {
  // run mahony ahrs update using filtered gravity estimate from imu
  mahony_update(state.gyro, state.GEstG, state.looptime);

  // gps heading fusion
  gps_heading_update(state.looptime);

  // extract euler angles and update state
  quat_to_euler(&attitude_state.orientation,
                &state.attitude.roll,
                &state.attitude.pitch,
                &state.attitude.yaw);

  // update heading (0-360 degrees)
  state.heading = normalize_deg(state.attitude.yaw * RADTODEG);
}
