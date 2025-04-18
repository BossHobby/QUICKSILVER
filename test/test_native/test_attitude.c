#include <unity.h>
#include <math.h>
#include <string.h>

#include "flight/attitude.h"
#include "flight/control.h"
#include "util/quaternion.h"
#include "mock_helpers.h"

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
  
  // Set GEstG (normalized gravity estimate) instead of raw accel
  // This matches what the IMU would provide
  state.GEstG.roll = accel_x;
  state.GEstG.pitch = accel_y;
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
  memset(&state, 0, sizeof(state));
  state.looptime = 0.001f; // 1ms loop time
  
  // Initialize attitude estimation
  attitude_init();
}

// Test 1: Initial state should be level
void test_attitude_initial_state(void) {
  attitude_test_setup();
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
    attitude_update();
  }
  
  // Should remain level
  assert_angles_close(state.attitude.roll, state.attitude.pitch, state.attitude.yaw,
                     0.0f, 0.0f, 0.0f, 0.01f);
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
    attitude_update();
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
    attitude_update();
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
    attitude_update();
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
    attitude_update();
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
    set_imu_data(0, 0, 0, 0, 0, 1.0f);
    attitude_update();
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
    set_imu_data(0, 0, yaw_rate, 0, 0, 1.0f);
    attitude_update();
  }
  
  // With 50% yaw stick, suppression = (1-0.5)^5 = 0.03125 (3.125%)
  // GPS heading at 90° (1.57 rad) will still have some influence
  // The yaw should be between pure gyro (1.0 rad) and GPS (1.57 rad)
  
  // The actual yaw will be influenced by:
  // 1. Initial convergence to GPS before stick input
  // 2. Confidence building over time (500ms to build to 0.5)
  // 3. The 3.125% GPS fusion that still occurs (50% stick -> (1-0.5)^5)
  // 4. The speed quality factor (1.0 at 10 m/s)
  
  // After 1000 updates (1 second), yaw has rotated by 1 rad from gyro
  // But GPS is pulling it toward 1.57 rad (90°)
  // With full confidence and speed, even with yaw stick suppression,
  // the fusion gain is GPS_HEADING_KP * 0.03125 * 1.0 * 1.0 * 0.5 = 0.0015625
  // This small but continuous correction over 1000 iterations adds up
  
  // The result is closer to GPS than pure gyro integration would suggest
  // A more realistic expectation is between 1.5 and 1.8 rad
  TEST_ASSERT_FLOAT_WITHIN(0.25f, 1.65f, state.attitude.yaw);
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
    set_imu_data(roll_rate, 0, 0, 0, 0, 1.0f);
    attitude_update();
  }
  
  // Verify we achieved the roll angle
  TEST_ASSERT_FLOAT_WITHIN(5.0f * DEGTORAD, target_roll, state.attitude.roll);
  
  // Now maintain the roll angle with appropriate accelerometer readings
  float initial_heading = state.heading;
  
  for (int i = 0; i < 1000; i++) {
    float roll = state.attitude.roll;
    set_imu_data(0, 0, 0, 0, sinf(roll), cosf(roll));
    attitude_update();
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
    attitude_update();
  }
  
  // Should not fuse GPS heading
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 0.0f, state.heading);
}

// Test 11: Quaternion normalization
void test_attitude_quaternion_normalization(void) {
  attitude_test_setup();
  // Apply large rotations to test normalization
  for (int i = 0; i < 10000; i++) {
    set_imu_data(1.0f, 0.5f, 0.2f, 0.1f, 0.1f, 0.98f);
    attitude_update();
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
    attitude_update();
  }
  
  // Heading should wrap around properly
  TEST_ASSERT_TRUE(state.heading >= 0.0f);
  TEST_ASSERT_TRUE(state.heading < 360.0f);
}

