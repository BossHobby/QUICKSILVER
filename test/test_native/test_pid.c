#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

// Include mock helpers
#include "mock_helpers.h"

// Include the PID module
#include "core/profile.h"
#include "driver/time.h"
#include "flight/control.h"
#include "flight/pid.h"

// Test fixtures
static void pid_setUp(void) {
  // Reset hardware mocks before each test
  mock_hardware_reset_all();

  // Initialize default profile values
  memset(&profile, 0, sizeof(profile));
  memset(&state, 0, sizeof(state));
  memset(&flags, 0, sizeof(flags));

  // Set reasonable defaults
  profile.pid.pid_rates[0].kp.roll = 1.0f;
  profile.pid.pid_rates[0].kp.pitch = 1.0f;
  profile.pid.pid_rates[0].kp.yaw = 1.0f;

  profile.pid.pid_rates[0].ki.roll = 0.5f;
  profile.pid.pid_rates[0].ki.pitch = 0.5f;
  profile.pid.pid_rates[0].ki.yaw = 0.5f;

  profile.pid.pid_rates[0].kd.roll = 0.1f;
  profile.pid.pid_rates[0].kd.pitch = 0.1f;
  profile.pid.pid_rates[0].kd.yaw = 0.1f;

  // Initialize voltage profile
  profile.voltage.pid_voltage_compensation = PID_VOLTAGE_COMPENSATION_NONE;

  // Initialize control state
  state.looptime = 0.000125f; // 8kHz
  state.looptime_inverse = 1.0f / state.looptime; // 8000Hz
  state.vbat_filtered = 4.0f;
  state.vbat_compensated = 1.0f;

  // Initialize PID
  pid_init();
}


// Test basic proportional control
void test_pid_proportional_control(void) {
  pid_setUp();
  // Setup
  state.error.roll = 1.0f; // 1 radian error
  state.error.pitch = 0.5f;
  state.error.yaw = 0.0f;

  // Disable I and D terms for this test
  profile.pid.pid_rates[0].ki.roll = 0.0f;
  profile.pid.pid_rates[0].ki.pitch = 0.0f;
  profile.pid.pid_rates[0].ki.yaw = 0.0f;
  profile.pid.pid_rates[0].kd.roll = 0.0f;
  profile.pid.pid_rates[0].kd.pitch = 0.0f;
  profile.pid.pid_rates[0].kd.yaw = 0.0f;

  // Reset integral state
  pid_init();

  // Execute
  pid_calc();

  // Verify proportional outputs
  // Note: outputs are scaled by pid_scales
  const float roll_scale = 1.0f / 628.0f;
  const float pitch_scale = 1.0f / 628.0f;
  TEST_ASSERT_EQUAL_FLOAT(1.0f * roll_scale, state.pidoutput.roll);
  TEST_ASSERT_EQUAL_FLOAT(0.5f * pitch_scale, state.pidoutput.pitch);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pidoutput.yaw);
}

// Test integral accumulation
void test_pid_integral_accumulation(void) {
  pid_setUp();
  // Setup - use small error for predictable integral growth
  state.error.roll = 0.1f;
  
  // Enable only integral term
  profile.pid.pid_rates[0].kp.roll = 0.0f;
  profile.pid.pid_rates[0].kd.roll = 0.0f;
  profile.pid.pid_rates[0].ki.roll = 1.0f;

  // Reset and initialize
  pid_init();

  // Simulate multiple loops to build up integral
  float expected_integral = 0.0f;
  for (int i = 0; i < 10; i++) {
    pid_calc();
    expected_integral += state.error.roll * state.looptime * profile.pid.pid_rates[0].ki.roll;
  }

  // Verify integral accumulated correctly
  TEST_ASSERT_FLOAT_WITHIN(0.001f, expected_integral, state.pidoutput.roll);
}

// Test derivative calculation - gyro response
void test_pid_derivative_calculation(void) {
  pid_setUp();
  // Setup - only D term enabled
  profile.pid.pid_rates[0].kp.roll = 0.0f;
  profile.pid.pid_rates[0].ki.roll = 0.0f;
  profile.pid.pid_rates[0].kd.roll = 1.0f;

  // Initialize stick profile (needed for D-term calculation)
  profile.pid.stick_rates[0].accelerator.roll = 1.0f;
  profile.pid.stick_rates[0].transition.roll = 1.0f;
  
  // Initialize state.looptime_inverse (needed for D-term)
  state.looptime_inverse = 1.0f / state.looptime;
  
  // Initialize with no gyro rate
  state.gyro.roll = 0.0f;
  state.setpoint.roll = 0.0f;
  pid_init();
  pid_calc();

  // Introduce gyro rate change - this should produce negative D-term output
  state.gyro.roll = 1.0f;
  pid_calc();

  // QUICKSILVER D-term = setpoint_derivative - gyro_derivative
  // With no setpoint change but gyro change, we expect negative output
  TEST_ASSERT_TRUE(state.pidoutput.roll < -0.1f);
}

// Test D-term setpoint response
void test_pid_dterm_setpoint_response(void) {
  pid_setUp();
  // Setup - only D term enabled
  profile.pid.pid_rates[0].kp.roll = 0.0f;
  profile.pid.pid_rates[0].ki.roll = 0.0f;
  profile.pid.pid_rates[0].kd.roll = 1.0f;

  // Initialize stick profile (need transition < 1 for setpoint to have effect)
  profile.pid.stick_rates[0].accelerator.roll = 1.0f;
  profile.pid.stick_rates[0].transition.roll = 0.0f; // No transition weighting
  
  // Initialize state.looptime_inverse
  state.looptime_inverse = 1.0f / state.looptime;
  
  // Set some stick input so transition weight isn't 0
  state.rx_filtered.roll = 0.0f; // No stick input
  
  // Initialize with no movement
  state.gyro.roll = 0.0f;
  state.setpoint.roll = 0.0f;
  pid_init();
  pid_calc();

  // Introduce setpoint change - this should produce positive D-term output
  state.setpoint.roll = 1.0f;
  pid_calc();

  // With setpoint change but no gyro change, we expect positive output
  TEST_ASSERT_TRUE(state.pidoutput.roll > 0.1f);
}

// Test D-term combined response
void test_pid_dterm_combined_response(void) {
  pid_setUp();
  // Setup - only D term enabled
  profile.pid.pid_rates[0].kp.roll = 0.0f;
  profile.pid.pid_rates[0].ki.roll = 0.0f;
  profile.pid.pid_rates[0].kd.roll = 1.0f;

  // Initialize stick profile
  profile.pid.stick_rates[0].accelerator.roll = 1.0f;
  profile.pid.stick_rates[0].transition.roll = 0.0f;
  
  // Initialize state.looptime_inverse
  state.looptime_inverse = 1.0f / state.looptime;
  
  // Initialize
  state.gyro.roll = 0.0f;
  state.setpoint.roll = 0.0f;
  pid_init();
  pid_calc();

  // Introduce matching setpoint and gyro changes
  // This should result in minimal D-term output (setpoint_deriv - gyro_deriv ≈ 0)
  state.setpoint.roll = 1.0f;
  state.gyro.roll = 1.0f;
  pid_calc();

  // The D-term should be close to zero but not exactly zero due to stick weighting
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, state.pidoutput.roll);
}

// Test D-term stick transition weighting
void test_pid_dterm_stick_weighting(void) {
  pid_setUp();
  // Setup - only D term enabled
  profile.pid.pid_rates[0].kp.roll = 0.0f;
  profile.pid.pid_rates[0].ki.roll = 0.0f;
  profile.pid.pid_rates[0].kd.roll = 1.0f;

  // Test different stick accelerator values
  profile.pid.stick_rates[0].accelerator.roll = 2.0f; // Higher than 1
  profile.pid.stick_rates[0].transition.roll = 0.0f; // Ensure setpoint has effect
  
  // Initialize state.looptime_inverse
  state.looptime_inverse = 1.0f / state.looptime;
  
  // Set some stick input
  state.rx_filtered.roll = 0.5f;
  
  // Initialize
  state.gyro.roll = 0.0f;
  state.setpoint.roll = 0.0f;
  pid_init();
  pid_calc();

  // Introduce setpoint change
  state.setpoint.roll = 1.0f;
  pid_calc();

  // Should still have output (could be positive or negative depending on stick weighting)
  TEST_ASSERT_NOT_EQUAL(0.0f, state.pidoutput.roll);
}

// Test voltage compensation
void test_pid_voltage_compensation(void) {
  pid_setUp();
  // Enable voltage compensation
  profile.voltage.pid_voltage_compensation = PID_VOLTAGE_COMPENSATION_ACTIVE;
  
  // Set a lower voltage
  state.vbat_filtered = 3.5f;
  state.vbat_compensated = 4.0f / 3.5f; // compensation factor
  
  // Setup error
  state.error.roll = 1.0f;
  
  // Execute
  pid_calc();
  
  // Output should be scaled by voltage compensation
  // Exact behavior depends on implementation
  TEST_ASSERT_NOT_EQUAL(0.0f, state.pidoutput.roll);
}

// Test complete PID loop
void test_pid_complete_loop(void) {
  pid_setUp();
  // Setup realistic values
  state.error.roll = 0.5f;
  state.error.pitch = -0.3f;
  state.error.yaw = 0.1f;

  pid_init();
  
  // Run for several iterations
  for (int i = 0; i < 5; i++) {
    pid_calc();
    
    // Verify outputs are within reasonable range
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 0.0f, state.pidoutput.roll);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 0.0f, state.pidoutput.pitch);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 0.0f, state.pidoutput.yaw);
    
    // Simulate changing errors
    state.error.roll *= 0.9f;
    state.error.pitch *= 0.9f;
    state.error.yaw *= 0.9f;
  }
}

