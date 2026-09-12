#include <unity.h>
#include <math.h>
#include <string.h>

#include "control/imu.h"

#include "control/control.h"
#include "core/profile.h"
#include "driver/time.h"
#include "util/quaternion.h"
#include "mock_helpers.h"

extern void imu_test_attitude_init();
extern void imu_test_attitude_update();

#define SCALED_ACC_1G 2048.0f

// Global control state is defined in control.c
extern control_state_t state;

// Helper to set gyro and accel data
static void set_imu_data(float gyro_x, float gyro_y, float gyro_z,
                        float accel_x, float accel_y, float accel_z) {
  state.gyro.roll = gyro_x;
  state.gyro.pitch = gyro_y;
  state.gyro.yaw = gyro_z;

  state.gyro_delta_angle.roll = gyro_x * state.looptime;
  state.gyro_delta_angle.pitch = gyro_y * state.looptime;
  state.gyro_delta_angle.yaw = gyro_z * state.looptime;

  // Fixtures use X=-sin(pitch), Y=sin(roll). Convert to the legacy IMU
  // layout rather than passing Cartesian gravity straight to the AHRS.
  state.GEstG.roll = accel_y;
  state.GEstG.pitch = -accel_x;
  state.GEstG.yaw = accel_z;
}

// Helper to check if angles are close
static void assert_angles_close(float roll, float pitch, float yaw,
                               float expected_roll, float expected_pitch, float expected_yaw,
                               float tolerance) {
  TEST_ASSERT_FLOAT_WITHIN(tolerance, expected_roll, roll);
  TEST_ASSERT_FLOAT_WITHIN(tolerance, expected_pitch, pitch);

  // Handle yaw wraparound
  float yaw_diff = yaw - expected_yaw;
  while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;
  TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0f, yaw_diff);
}

// Test-specific setup for attitude tests
static void attitude_test_setup(void) {
  profile.serial.gps = SERIAL_PORT1;
  memset(&state, 0, sizeof(state));
  memset(&flags, 0, sizeof(flags));
  state.looptime = 0.001f; // 1ms loop time
  time_test_reset();

  // Initialize attitude estimation
  imu_test_attitude_init();
}

static void attitude_advance_with_gps_sample(uint32_t loop_index) {
  time_test_advance_us(1000);
  if ((loop_index % 100U) == 0) {
    state.gps_last_update_ms = time_millis();
  }
}

// Test 1: Initial state should be level
void test_attitude_initial_state(void) {
  attitude_test_setup();
  state.heading_confidence = 1.0f;
  imu_test_attitude_init();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.heading_confidence);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, state.attitude.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, state.attitude.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, state.attitude.yaw);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, state.heading);
}

// Test 2: Level flight with gravity pointing down
void test_attitude_level_flight(void) {
  attitude_test_setup();
  // Set level accelerometer (gravity down)
  set_imu_data(0, 0, 0, 0, 0, 1.0f);

  // Run several updates
  for (int i = 0; i < 100; i++) {
    imu_test_attitude_update();
  }

  // Should remain level
  assert_angles_close(state.attitude.roll, state.attitude.pitch, state.attitude.yaw,
                     0.0f, 0.0f, 0.0f, 0.01f);
}

void test_attitude_accepts_scaled_gravity_vector(void) {
  attitude_test_setup();
  set_imu_data(0, 0, 0, 0, 0, SCALED_ACC_1G);

  for (int i = 0; i < 100; i++) {
    imu_test_attitude_update();
  }

  assert_angles_close(state.attitude.roll, state.attitude.pitch, state.attitude.yaw,
                     0.0f, 0.0f, 0.0f, 0.01f);
}

void test_attitude_ignores_invalid_gravity_vector(void) {
  attitude_test_setup();
  const float yaw_rate = M_PI / 2.0f;

  for (int i = 0; i < 1000; i++) {
    set_imu_data(0, 0, yaw_rate, 0, 0, 0);
    imu_test_attitude_update();
  }

  assert_angles_close(state.attitude.roll, state.attitude.pitch, state.attitude.yaw,
                     0.0f, 0.0f, M_PI / 2.0f, 0.05f);
}

void test_attitude_converges_faster_when_disarmed(void) {
  attitude_test_setup();
  const float target_pitch = 20.0f * DEGTORAD;

  flags.arm_state = 1;
  for (int i = 0; i < 250; i++) {
    set_imu_data(0, 0, 0, -sinf(target_pitch), 0, cosf(target_pitch));
    imu_test_attitude_update();
  }
  const float armed_pitch = state.attitude.pitch;

  attitude_test_setup();
  flags.arm_state = 0;
  for (int i = 0; i < 250; i++) {
    set_imu_data(0, 0, 0, -sinf(target_pitch), 0, cosf(target_pitch));
    imu_test_attitude_update();
  }

  TEST_ASSERT_TRUE(state.attitude.pitch > armed_pitch * 3.0f);
}

// Test 3: Roll rotation
void test_attitude_roll_rotation(void) {
  attitude_test_setup();
  // Apply constant roll rate for 250ms to get 45 degrees
  float roll_rate = M_PI / 4.0f / 0.25f; // 45 deg in 250ms

  for (int i = 0; i < 250; i++) {
    // Accelerometer shows gravity rotated by current roll
    float roll = state.attitude.roll;
    set_imu_data(roll_rate, 0, 0,
                 0, sin(roll), cos(roll));
    imu_test_attitude_update();
  }

  // Should be close to 45 degrees roll
  assert_angles_close(state.attitude.roll, state.attitude.pitch, state.attitude.yaw,
                     M_PI / 4.0f, 0.0f, 0.0f, 0.05f);
}

// Test 4: Pitch rotation
void test_attitude_pitch_rotation(void) {
  attitude_test_setup();
  // Apply constant pitch rate for 250ms to get 30 degrees
  float pitch_rate = M_PI / 6.0f / 0.25f; // 30 deg in 250ms

  for (int i = 0; i < 250; i++) {
    // Accelerometer shows gravity rotated by current pitch
    float pitch = state.attitude.pitch;
    set_imu_data(0, pitch_rate, 0,
                 -sin(pitch), 0, cos(pitch));
    imu_test_attitude_update();
  }

  // Should be close to 30 degrees pitch
  assert_angles_close(state.attitude.roll, state.attitude.pitch, state.attitude.yaw,
                     0.0f, M_PI / 6.0f, 0.0f, 0.05f);
}

// Test 5: Yaw rotation (no accelerometer correction)
void test_attitude_yaw_rotation(void) {
  attitude_test_setup();
  // Apply constant yaw rate for 500ms to get 90 degrees
  float yaw_rate = M_PI / 2.0f / 0.5f; // 90 deg in 500ms

  for (int i = 0; i < 500; i++) {
    set_imu_data(0, 0, yaw_rate, 0, 0, 1.0f);
    imu_test_attitude_update();
  }

  // Should be close to 90 degrees yaw
  assert_angles_close(state.attitude.roll, state.attitude.pitch, state.attitude.yaw,
                     0.0f, 0.0f, M_PI / 2.0f, 0.05f);

  // Check heading conversion
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 90.0f, state.heading);
}

// Test 6: Accelerometer correction for drift
void test_attitude_accel_correction(void) {
  attitude_test_setup();
  // Start with 10 degree roll error
  state.attitude.roll = 10.0f * M_PI / 180.0f;

  // Level accelerometer should correct it
  for (int i = 0; i < 1000; i++) {
    set_imu_data(0, 0, 0, 0, 0, 1.0f);
    imu_test_attitude_update();
  }

  // Should converge back to level
  assert_angles_close(state.attitude.roll, state.attitude.pitch, state.attitude.yaw,
                     0.0f, 0.0f, 0.0f, 0.01f);
}

// Test 7: GPS heading fusion when moving
void test_attitude_gps_heading_fusion(void) {
  attitude_test_setup();
  // Set GPS data
  state.gps_lock = true;
  state.gps_speed = 10.0f; // 10 m/s
  state.gps_heading = 45.0f; // 45 degrees
  state.gps_heading_accuracy = 2.0f; // Good accuracy

  // No yaw stick input
  state.rx_filtered.yaw = 0.0f;

  // Run updates to fuse GPS heading
  for (int i = 0; i < 2000; i++) {
    attitude_advance_with_gps_sample(i);
    set_imu_data(0, 0, 0, 0, 0, 1.0f);
    imu_test_attitude_update();
  }

  // Should converge to GPS heading
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 45.0f, state.heading);
}

// Test 8: GPS heading suppression during yaw stick input
void test_attitude_gps_suppression_yaw_stick(void) {
  attitude_test_setup();
  // Set GPS data
  state.gps_lock = true;
  state.gps_speed = 10.0f;
  state.gps_heading = 90.0f;
  state.gps_heading_accuracy = 2.0f;

  // Apply yaw stick
  state.rx_filtered.yaw = 0.5f; // 50% yaw

  // Apply yaw rotation
  float yaw_rate = 1.0f; // 1 rad/s

  for (int i = 0; i < 1000; i++) {
    attitude_advance_with_gps_sample(i);
    set_imu_data(0, 0, yaw_rate, 0, 0, 1.0f);
    imu_test_attitude_update();
  }

  // GPS correction only runs on fresh GPS samples and is strongly suppressed
  // by yaw stick input, so the result should stay close to gyro integration.
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, state.attitude.yaw);
}

// Test 9: GPS heading suppression during roll
void test_attitude_gps_suppression_roll(void) {
  attitude_test_setup();
  // Set GPS data
  state.gps_lock = true;
  state.gps_speed = 10.0f;
  state.gps_heading = 180.0f;
  state.gps_heading_accuracy = 2.0f;

  // No yaw stick
  state.rx_filtered.yaw = 0.0f;

  // Apply roll rotation to get to 20 degrees
  float target_roll = 20.0f * DEGTORAD;
  float roll_rate = target_roll / 0.1f; // 20 degrees in 100ms

  for (int i = 0; i < 100; i++) {
    attitude_advance_with_gps_sample(i);
    set_imu_data(roll_rate, 0, 0, 0, 0, 1.0f);
    imu_test_attitude_update();
  }

  // Verify we achieved the roll angle
  TEST_ASSERT_FLOAT_WITHIN(5.0f * DEGTORAD, target_roll, state.attitude.roll);

  // Now maintain the roll angle with appropriate accelerometer readings
  float initial_heading = state.heading;

  for (int i = 0; i < 1000; i++) {
    attitude_advance_with_gps_sample(i);
    float roll = state.attitude.roll;
    set_imu_data(0, 0, 0, 0, sinf(roll), cosf(roll));
    imu_test_attitude_update();
  }

  // With 20° roll (above 15° threshold), GPS fusion should be suppressed
  // Heading should not have changed significantly
  float heading_change = fabsf(state.heading - initial_heading);
  if (heading_change > 180.0f) heading_change = 360.0f - heading_change;

  // Should have minimal heading change (less than 30 degrees)
  TEST_ASSERT_TRUE(heading_change < 30.0f);
}

// Test 10: No GPS fusion when stationary
void test_attitude_no_gps_when_stationary(void) {
  attitude_test_setup();
  // Set GPS data but low speed
  state.gps_lock = true;
  state.gps_speed = 1.0f; // Below minimum threshold
  state.gps_heading = 270.0f;
  state.gps_heading_accuracy = 2.0f;

  // Start with different heading
  state.attitude.yaw = 0.0f;

  for (int i = 0; i < 1000; i++) {
    set_imu_data(0, 0, 0, 0, 0, 1.0f);
    imu_test_attitude_update();
  }

  // Should not fuse GPS heading
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 0.0f, state.heading);
}

void test_attitude_gps_heading_requires_fresh_sample(void) {
  attitude_test_setup();
  state.gps_lock = true;
  state.gps_speed = 10.0f;
  state.gps_heading = 90.0f;
  state.gps_heading_accuracy = 2.0f;
  state.gps_last_update_ms = time_millis();

  set_imu_data(0, 0, 0, 0, 0, 1.0f);
  imu_test_attitude_update();
  const float heading_after_sample = state.heading;

  state.gps_heading = 180.0f;
  for (int i = 0; i < 100; i++) {
    time_test_advance_us(1000);
    set_imu_data(0, 0, 0, 0, 0, 1.0f);
    imu_test_attitude_update();
  }

  TEST_ASSERT_FLOAT_WITHIN(5.0f, heading_after_sample, state.heading);
}

// Test 11: Quaternion normalization
void test_attitude_quaternion_normalization(void) {
  attitude_test_setup();
  // Apply large rotations to test normalization
  for (int i = 0; i < 10000; i++) {
    set_imu_data(1.0f, 0.5f, 0.2f, 0.1f, 0.1f, 0.98f);
    imu_test_attitude_update();
  }

  // Attitude should still be valid (not NaN or infinite)
  TEST_ASSERT_TRUE(isfinite(state.attitude.roll));
  TEST_ASSERT_TRUE(isfinite(state.attitude.pitch));
  TEST_ASSERT_TRUE(isfinite(state.attitude.yaw));

  // Angles should be within valid range
  TEST_ASSERT_TRUE(fabs(state.attitude.roll) <= M_PI);
  TEST_ASSERT_TRUE(fabs(state.attitude.pitch) <= M_PI / 2.0f);
  TEST_ASSERT_TRUE(fabs(state.attitude.yaw) <= M_PI);
}

// Test 12: Heading wraparound
void test_attitude_heading_wraparound(void) {
  attitude_test_setup();
  // Rotate past 360 degrees
  float yaw_rate = 2.0f * M_PI / 1.0f; // Full rotation in 1 second

  for (int i = 0; i < 1500; i++) {
    set_imu_data(0, 0, yaw_rate, 0, 0, 1.0f);
    imu_test_attitude_update();
  }

  // Heading should wrap around properly
  TEST_ASSERT_TRUE(state.heading >= 0.0f);
  TEST_ASSERT_TRUE(state.heading < 360.0f);
}

void test_attitude_gps_reacquisition_does_not_snap_yaw(void) {
  attitude_test_setup();
  state.gps_lock = true;
  state.gps_speed = 10.0f;
  state.gps_heading_accuracy = 1.0f;
  set_imu_data(0, 0, 0, 0, 0, 1);
  imu_test_attitude_update(); // initialise north
  state.gps_speed = 0;
  for (int i = 0; i < 1000; i++) {
    time_test_advance_us(1000);
    imu_test_attitude_update();
  }
  state.gps_speed = 10;
  state.gps_heading = 90;
  state.gps_last_update_ms = time_millis();
  state.attitude.pitch = 15 * DEGTORAD;
  imu_test_attitude_update();
  TEST_ASSERT_TRUE(state.heading < 3.0f);
  TEST_ASSERT_TRUE(state.heading > 0.0f);
}

void test_attitude_rth_sideways_motion_does_not_correct_heading(void) {
  attitude_test_setup();
  flags.controls_override = 1;
  state.rth_active = true;
  state.rth_state = RTH_STATE_NAVIGATE;
  state.rx_override.roll = 0.15f;
  state.gps_lock = true;
  state.gps_speed = 4;
  state.gps_heading = 90;
  state.gps_heading_accuracy = 1;
  for (int i = 0; i < 2000; i++) {
    attitude_advance_with_gps_sample(i);
    set_imu_data(0, 0, 0, 0, 0, 1);
    imu_test_attitude_update();
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, state.heading);
}

void test_attitude_rth_large_heading_error_can_recover(void) {
  attitude_test_setup();
  flags.controls_override = 1;
  state.rth_active = true;
  state.rth_state = RTH_STATE_NAVIGATE;
  state.home_bearing = 180;
  state.gps_vel_east = 4;
  state.rx_override.pitch = 0.15f;
  state.gps_lock = true;
  state.gps_speed = 4;
  state.gps_heading = 90;
  state.gps_heading_accuracy = 1;
  const float pitch = 8 * DEGTORAD;
  for (int i = 0; i < 20000; i++) {
    attitude_advance_with_gps_sample(i);
    set_imu_data(0, 0, 0, -sinf(pitch), 0, cosf(pitch));
    imu_test_attitude_update();
  }
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 90, state.heading);
  TEST_ASSERT_TRUE(state.heading_confidence > 0.5f);
}

void test_attitude_heading_correction_is_sample_rate_independent(void) {
  float headings[2];
  for (int run = 0; run < 2; run++) {
    attitude_test_setup();
    state.gps_lock = true;
    state.gps_speed = 10;
    state.gps_heading_accuracy = 1;
    set_imu_data(0, 0, 0, 0, 0, 1);
    imu_test_attitude_update(); // initialise north
    state.gps_heading = 10;
    for (int i = 1; i <= 2000; i++) {
      time_test_advance_us(1000);
      if (i % (run ? 100 : 200) == 0) {
        state.gps_last_update_ms = time_millis();
      }
      state.attitude.pitch = 60 * DEGTORAD;
      imu_test_attitude_update();
    }
    headings[run] = state.heading;
  }
  TEST_ASSERT_TRUE(headings[0] > 9);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, headings[0], headings[1]);
}

static float run_rth_heading_recovery(uint8_t phase, float speed_home, float accuracy, bool fresh) {
  attitude_test_setup();
  flags.controls_override = 1;
  state.rth_active = true;
  state.rth_state = phase;
  state.rx_override.pitch = 0.1f;
  state.gps_lock = true;
  state.gps_speed = 4;
  state.gps_vel_north = -2;
  state.gps_vel_east = sqrtf(12);
  state.home_bearing = speed_home > 0 ? 120 : 300;
  state.gps_heading = 120;
  state.gps_heading_accuracy = accuracy;
  const float pitch = 8 * DEGTORAD;
  for (int i = 0; i < 5000; i++) {
    time_test_advance_us(1000);
    if (fresh && i % 100 == 0) state.gps_last_update_ms = time_millis();
    set_imu_data(0, 0, 0, -sinf(pitch), 0, cosf(pitch));
    imu_test_attitude_update();
  }
  return state.heading;
}

void test_attitude_acquisition_rejects_bad_course() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, run_rth_heading_recovery(RTH_STATE_ACQUIRE_HEADING, -4, 20, true));
  TEST_ASSERT_TRUE(state.heading_correction_flags & HEADING_POOR_ACCURACY);
  TEST_ASSERT_EQUAL_FLOAT(0, state.heading_confidence);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, run_rth_heading_recovery(RTH_STATE_ACQUIRE_HEADING, -4, 1, false));
  TEST_ASSERT_TRUE(state.heading_correction_flags & HEADING_STALE);
  TEST_ASSERT_EQUAL_FLOAT(0, state.heading_confidence);
}

void test_attitude_forward_flight_builds_and_hover_retains_confidence() {
  attitude_test_setup();
  state.gps_lock = true;
  state.gps_speed = 4;
  state.gps_heading = 90;
  state.gps_heading_accuracy = 1;
  // Establish tilt before testing GPS alignment. Stick offsets without actual
  // yaw motion must not inhibit useful observations indefinitely.
  state.rx_filtered.yaw = 0.08f;
  const float pitch = 10 * DEGTORAD;
  for (int i = 0; i < 8000; i++) {
    attitude_advance_with_gps_sample(i);
    set_imu_data(0, 0, 0, -sinf(pitch), 0, cosf(pitch));
    imu_test_attitude_update();
    if (i < 100) TEST_ASSERT_TRUE(state.heading_confidence < 0.2f);
  }
  TEST_ASSERT_TRUE(state.heading_confidence > 0.8f);
  const float confidence = state.heading_confidence;
  state.gps_speed = 0;
  for (int i = 0; i < 60000; i++) {
    attitude_advance_with_gps_sample(i);
    set_imu_data(0, 0, 0, 0, 0, 1);
    imu_test_attitude_update();
  }
  TEST_ASSERT_EQUAL_FLOAT(confidence, state.heading_confidence);
  flags.arm_state = true;
  imu_test_attitude_update();
  TEST_ASSERT_EQUAL_FLOAT(0, state.heading_confidence);
}

void test_attitude_yaw_between_gps_samples_suppresses_correction() {
  attitude_test_setup();
  state.gps_lock = true;
  state.gps_speed = 4;
  state.gps_heading_accuracy = 1;
  set_imu_data(0, 0, 0, 0, 0, 1);
  imu_test_attitude_update();
  // Actual motion without stick input, entirely between two GPS packets.
  time_test_advance_us(1000);
  state.gyro.yaw = 30 * DEGTORAD;
  imu_test_attitude_update();
  time_test_advance_us(1000);
  state.gyro.yaw = 0;
  state.gps_heading = 90;
  state.gps_last_update_ms = time_millis();
  state.attitude.pitch = 10 * DEGTORAD;
  imu_test_attitude_update();
  TEST_ASSERT_TRUE(state.heading_correction_flags & HEADING_YAW);
  TEST_ASSERT_TRUE(state.heading < 1);
}

void test_attitude_rth_recovery_strengthens_when_moving_away(void) {
  const float toward = run_rth_heading_recovery(RTH_STATE_NAVIGATE, 4, 1, true);
  const float away = run_rth_heading_recovery(RTH_STATE_NAVIGATE, -4, 1, true);
  TEST_ASSERT_TRUE(away > toward + 30);
  TEST_ASSERT_FLOAT_WITHIN(5, 120, away);
}

void test_attitude_rth_recovery_requires_valid_course_and_phase(void) {
  const uint8_t phases[] = {RTH_STATE_CLIMB, RTH_STATE_TURN, RTH_STATE_HOVER_HOME};
  for (const uint8_t phase : phases) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, run_rth_heading_recovery(phase, -4, 1, true));
    TEST_ASSERT_TRUE(state.heading_correction_flags & HEADING_RTH_PHASE);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, run_rth_heading_recovery(RTH_STATE_NAVIGATE, -4, 20, true));
  TEST_ASSERT_TRUE(state.heading_correction_flags & HEADING_POOR_ACCURACY);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, run_rth_heading_recovery(RTH_STATE_NAVIGATE, -4, NAN, true));
  TEST_ASSERT_TRUE(state.heading_correction_flags & HEADING_POOR_ACCURACY);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, run_rth_heading_recovery(RTH_STATE_NAVIGATE, -4, 1, false));
  TEST_ASSERT_TRUE(state.heading_correction_flags & HEADING_STALE);
  state.gps_lock = false;
  imu_test_attitude_update();
  TEST_ASSERT_TRUE(state.heading_correction_flags & HEADING_NO_FIX);
  state.gps_speed = 0;
  imu_test_attitude_update();
  TEST_ASSERT_TRUE(state.heading_correction_flags & HEADING_LOW_SPEED);

  // Match QUIC's 4096-byte state payload capacity, including the new field.
  uint8_t buffer[4096];
  cbor_value_t encoder;
  cbor_encoder_init(&encoder, buffer, sizeof(buffer));
  TEST_ASSERT_TRUE(cbor_encode_control_state_t(&encoder, &state) >= CBOR_OK);
}

void test_attitude_normal_heading_pitch_weight_matches_vehicle(void) {
  attitude_test_setup();
  state.gps_lock = true;
  state.gps_speed = 4;
  state.gps_heading_accuracy = 1;
  set_imu_data(0, 0, 0, 0, 0, 1);
  imu_test_attitude_update(); // Initial motion reference is north.
  state.gps_heading = 180;
  const float pitch = -15 * DEGTORAD;
  for (int i = 0; i < 5000; i++) {
    attitude_advance_with_gps_sample(i);
    set_imu_data(0, 0, 0, -sinf(pitch), 0, cosf(pitch));
    imu_test_attitude_update();
  }
#ifdef VEHICLE_MULTI
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, state.heading);
#else
  TEST_ASSERT_TRUE(state.heading > 20);
#endif
}

static void imu_pipeline_setup(float roll, float pitch) {
  attitude_test_setup();
  state.looptime_autodetect = 1000;
  // Sensor values after sixaxis_read(): positive roll -> positive accel.roll,
  // positive nose-down pitch -> positive accel.pitch.
  state.accel_raw = (vec3_t){{sinf(roll) * cosf(pitch), sinf(pitch), cosf(roll) * cosf(pitch)}};
  imu_init();
}

void test_attitude_imu_pipeline_roll_and_pitch(void) {
  for (int axis = 0; axis < 2; axis++) {
    imu_pipeline_setup(0, 0);
    flags.arm_state = 1;
    const float rate = 60 * DEGTORAD;
    for (int i = 0; i < 5500; i++) {
      const float angle = MIN(i + 1, 500) * state.looptime * rate;
      const float roll = axis == 0 ? angle : 0;
      const float pitch = axis == 1 ? angle : 0;
      state.accel_raw = (vec3_t){{sinf(roll) * cosf(pitch), sinf(pitch), cosf(roll) * cosf(pitch)}};
      state.gyro = (vec3_t){0};
      state.gyro.axis[axis] = i < 500 ? rate : 0;
      state.gyro_delta_angle = vec3_mul(state.gyro, state.looptime);
      imu_calc();
      TEST_ASSERT_FLOAT_WITHIN(0.5f * DEGTORAD, roll, state.attitude.roll);
      TEST_ASSERT_FLOAT_WITHIN(0.5f * DEGTORAD, pitch, state.attitude.pitch);
      TEST_ASSERT_FLOAT_WITHIN(0.5f * DEGTORAD, 0, state.attitude.yaw);
    }
  }
}

void test_attitude_imu_pipeline_yaw_while_tilted(void) {
  const float roll = 30 * DEGTORAD;
  const float pitch = 20 * DEGTORAD;
  imu_pipeline_setup(roll, pitch);
  flags.on_ground = 1;
  // Acquire the actual IMU gravity estimate before rotating around earth-up.
  for (int i = 0; i < 5000; i++) {
    imu_calc();
  }
  const float initial_yaw = state.attitude.yaw;
  flags.arm_state = 1;
  flags.on_ground = 0;
  const float yaw_rate = 30 * DEGTORAD;
  state.gyro = (vec3_t){{yaw_rate * sinf(pitch), -yaw_rate * sinf(roll) * cosf(pitch), yaw_rate * cosf(roll) * cosf(pitch)}};
  state.gyro_delta_angle = vec3_mul(state.gyro, state.looptime);
  for (int i = 0; i < 3000; i++) {
    imu_calc();
  }
  TEST_ASSERT_FLOAT_WITHIN(0.5f * DEGTORAD, roll, state.attitude.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.5f * DEGTORAD, pitch, state.attitude.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.5f * DEGTORAD, 90 * DEGTORAD, state.attitude.yaw - initial_yaw);
}

void test_attitude_course_accuracy_tapers_recovery() {
  const float good = run_rth_heading_recovery(RTH_STATE_ACQUIRE_HEADING, -4, 5, true);
  const float good_confidence = state.heading_confidence;
  const float moderate = run_rth_heading_recovery(RTH_STATE_ACQUIRE_HEADING, -4, 10, true);
  TEST_ASSERT_FALSE(state.heading_correction_flags & HEADING_POOR_ACCURACY);
  TEST_ASSERT_TRUE(moderate > 20);
  TEST_ASSERT_TRUE(moderate < good);
  TEST_ASSERT_TRUE(state.heading_confidence > 0);
  TEST_ASSERT_TRUE(state.heading_confidence < good_confidence);
  const float poor = run_rth_heading_recovery(RTH_STATE_ACQUIRE_HEADING, -4, 14, true);
  TEST_ASSERT_TRUE(poor > 0);
  TEST_ASSERT_TRUE(poor < moderate);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, run_rth_heading_recovery(RTH_STATE_ACQUIRE_HEADING, -4, 15, true));
  TEST_ASSERT_TRUE(state.heading_correction_flags & HEADING_POOR_ACCURACY);
}

void test_imu_without_gps_publishes_tilt_without_heading_estimator() {
  const auto saved_gps = profile.serial.gps;
  const float poses[][2] = {{30, 20}, {-45, 30}, {170, -40}, {0, 89}, {0, -89}};
  for (const auto &pose : poses) {
    memset(&state, 0, sizeof(state));
    memset(&flags, 0, sizeof(flags));
    profile.serial.gps = SERIAL_PORT_INVALID;
    state.looptime_autodetect = 125;
    state.looptime = 0.000125f;
    const float roll = pose[0] * DEGTORAD;
    const float pitch = pose[1] * DEGTORAD;
    state.accel_raw = {{sinf(roll) * cosf(pitch), sinf(pitch), cosf(roll) * cosf(pitch)}};
    state.GEstG = state.accel_raw;
    imu_init();
    // A stray fix/course must not enable the GPS path without configuration.
    state.gps_lock = true;
    state.gps_heading = 90;
    state.gps_speed = 10;
    for (unsigned i = 0; i < 100; i++) {
      imu_calc();
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f * DEGTORAD, roll, state.attitude.roll);
    TEST_ASSERT_FLOAT_WITHIN(0.5f * DEGTORAD, pitch, state.attitude.pitch);
    TEST_ASSERT_EQUAL_FLOAT(0, state.attitude.yaw);
    TEST_ASSERT_EQUAL_FLOAT(0, state.heading);
    TEST_ASSERT_EQUAL_FLOAT(0, state.heading_confidence);
    TEST_ASSERT_EQUAL_UINT8(HEADING_NO_FIX, state.heading_correction_flags);
  }
  profile.serial.gps = saved_gps;
}
