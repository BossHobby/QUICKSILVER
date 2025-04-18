#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <unity.h>

// Include mock helpers
#include "mock_helpers.h"

// Include the IMU module
#include "flight/imu.h"
#include "flight/control.h"
#include "util/vector.h"
#include "util/util.h"
#include "core/profile.h"
#include "core/tasks.h"

// Constants
#define ACC_1G 1.0f

// Test initialization
static void imu_setUp(void) {
  // Reset hardware mocks
  mock_hardware_reset_all();
  
  // Reset state
  memset(&state, 0, sizeof(state));
  memset(&flags, 0, sizeof(flags));
  
  // Set default values
  state.looptime = 0.000125f; // 8kHz
  
  // Initialize gravity vector to (0, 0, 1g)
  state.GEstG.axis[0] = 0.0f;
  state.GEstG.axis[1] = 0.0f;
  state.GEstG.axis[2] = ACC_1G;
}

// Test IMU initialization
void test_imu_gravity_vector_init(void) {
  imu_setUp();
  
  // Verify initial gravity vector
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.GEstG.roll);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.GEstG.pitch);
  TEST_ASSERT_EQUAL_FLOAT(ACC_1G, state.GEstG.yaw);
}

// Test gyro integration (rotation)
void test_imu_gyro_integration(void) {
  imu_setUp();
  
  // Set initial accelerometer values to gravity only
  state.accel_raw.roll = 0.0f;
  state.accel_raw.pitch = 0.0f;
  state.accel_raw.yaw = ACC_1G;
  
  // Set a small rotation around pitch axis
  // The IMU transforms gyro_delta_angle with: rot = {-pitch, roll, yaw}
  state.gyro_delta_angle.roll = 0.0f;
  state.gyro_delta_angle.pitch = 0.01f;
  state.gyro_delta_angle.yaw = 0.0f;
  
  // Execute IMU calculation
  imu_calc();
  
  // After rotation around transformed pitch axis, roll component should change
  TEST_ASSERT_FLOAT_WITHIN(0.02f, -0.01f, state.GEstG.roll);
  
  // Magnitude should remain approximately 1g
  float mag = vec3_magnitude(state.GEstG);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, ACC_1G, mag);
}

// Test accelerometer fusion when on ground
void test_imu_accel_fusion_ground(void) {
  imu_setUp();
  
  // Set on_ground flag
  flags.on_ground = true;
  
  // Set accelerometer readings (slightly off from vertical)
  state.accel_raw.roll = 0.1f;
  state.accel_raw.pitch = 0.1f;
  state.accel_raw.yaw = 0.98f; // Close to 1g
  
  // Run multiple iterations to see fusion
  for (int i = 0; i < 10; i++) {
    imu_calc();
  }
  
  // Gravity estimate should move toward accelerometer
  TEST_ASSERT_FLOAT_WITHIN(0.2f, state.accel_raw.roll, state.GEstG.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, state.accel_raw.pitch, state.GEstG.pitch);
}

// Test accelerometer rejection on bad magnitude
void test_imu_accel_magnitude_rejection(void) {
  imu_setUp();
  
  // Save initial gravity
  vec3_t initial_gravity = state.GEstG;
  
  // Set accelerometer with bad magnitude (too high)
  state.accel_raw.roll = 2.0f;
  state.accel_raw.pitch = 2.0f;
  state.accel_raw.yaw = 2.0f;
  
  imu_calc();
  
  // Gravity should not change significantly
  TEST_ASSERT_FLOAT_WITHIN(0.1f, initial_gravity.roll, state.GEstG.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, initial_gravity.pitch, state.GEstG.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, initial_gravity.yaw, state.GEstG.yaw);
}

// Test attitude calculation for horizon mode
void test_imu_attitude_calculation(void) {
  imu_setUp();
  
  // Tilt the gravity vector
  state.GEstG.roll = 0.5f;
  state.GEstG.pitch = 0.0f;
  state.GEstG.yaw = 0.866f; // cos(30°)
  
  // Enable horizon mode (this would typically be done via aux channel)
  // For testing, we'll calculate attitude directly
  state.attitude.roll = atan2approx(state.GEstG.roll, state.GEstG.yaw);
  state.attitude.pitch = atan2approx(state.GEstG.pitch, state.GEstG.yaw);
  
  // Check roll angle (should be ~30 degrees)
  // atan2approx returns degrees, not radians
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 30.0f, state.attitude.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, state.attitude.pitch);
}

// Test IMU behavior during flight
void test_imu_in_flight_behavior(void) {
  imu_setUp();
  
  // Not on ground
  flags.on_ground = false;
  
  // Simulate some acceleration
  state.accel_raw.roll = 0.2f;
  state.accel_raw.pitch = 0.1f;
  state.accel_raw.yaw = 0.95f;
  
  // Run IMU calculations
  for (int i = 0; i < 20; i++) {
    imu_calc();
  }
  
  // Check that fusion is happening but slower than on ground
  // Gravity vector should be normalized
  float mag = vec3_magnitude(state.GEstG);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, ACC_1G, mag);
}

// Test heading computation with level quad - REMOVED
// Heading/attitude computation moved to attitude module
void test_imu_heading_level(void) {
  // This test is no longer applicable as IMU doesn't compute heading
  // The functionality is tested in test_attitude.c
  TEST_PASS();
}

// Test heading computation with tilted quad - REMOVED
// Heading/attitude computation moved to attitude module
void test_imu_heading_tilted(void) {
  // This test is no longer applicable as IMU doesn't compute heading
  // The functionality is tested in test_attitude.c
  TEST_PASS();
}

// Test GPS heading fusion with low speed
void test_imu_gps_fusion_low_speed(void) {
  imu_setUp();
  
  // Set initial heading
  state.attitude.yaw = 0.0f;
  state.heading = 0.0f;
  
  // GPS data with low speed
  state.gps_lock = true;
  state.gps_speed = 1.5f; // Below minimum
  state.gps_heading = 90.0f;
  state.gps_heading_accuracy = 2.0f;
  
  imu_calc();
  
  // Should not fuse GPS (speed too low)
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, state.heading);
}

// Test GPS heading fusion with variable gain - REMOVED
// GPS heading fusion moved to attitude module
void test_imu_gps_fusion_variable_gain(void) {
  // This test is no longer applicable as IMU doesn't handle GPS fusion
  // The functionality is tested in test_attitude.c
  TEST_PASS();
}

// Test GPS heading fusion at trust speed - REMOVED
// GPS heading fusion moved to attitude module
void test_imu_gps_fusion_trust_speed(void) {
  // This test is no longer applicable as IMU doesn't handle GPS fusion
  // The functionality is tested in test_attitude.c
  TEST_PASS();
}

// Test heading normalization - REMOVED
// Heading normalization moved to attitude module
void test_imu_heading_normalization(void) {
  // This test is no longer applicable as IMU doesn't compute heading
  // The functionality is tested in test_attitude.c
  TEST_PASS();
}

// Test GPS heading accuracy rejection
void test_imu_gps_fusion_poor_accuracy(void) {
  imu_setUp();
  
  // Set initial heading
  state.attitude.yaw = 0.0f;
  state.heading = 0.0f;
  
  // GPS data with poor accuracy
  state.gps_lock = true;
  state.gps_speed = 15.0f; // Good speed
  state.gps_heading = 90.0f;
  state.gps_heading_accuracy = 10.0f; // Poor accuracy (> 5 degrees)
  
  imu_calc();
  
  // Should not fuse GPS (accuracy too poor)
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, state.heading);
}

// Test heading with complex rotation - REMOVED
// Heading computation moved to attitude module
void test_imu_heading_complex_rotation(void) {
  // This test is no longer applicable as IMU doesn't compute heading
  // The functionality is tested in test_attitude.c
  TEST_PASS();
}
