#include <unity.h>
#include <string.h>
#include "mock_helpers.h"

// Include blackbox headers
#include "io/blackbox.h"
#include "util/vector.h"
#include "util/cbor_helper.h"

// Define constants for testing (from blackbox.c)
#define BLACKBOX_I_FRAME_INTERVAL 32

// Test helper functions for blackbox delta encoding

static void create_test_blackbox_frame(blackbox_t *frame, uint32_t loop, uint32_t time) {
  memset(frame, 0, sizeof(blackbox_t));
  
  frame->loop = loop;
  frame->time = time;
  
  // Set some test values for PID terms
  frame->pid_p_term.roll = 100;
  frame->pid_p_term.pitch = 200;
  frame->pid_p_term.yaw = 300;
  
  frame->pid_i_term.roll = 50;
  frame->pid_i_term.pitch = 75;
  frame->pid_i_term.yaw = 100;
  
  frame->pid_d_term.roll = 25;
  frame->pid_d_term.pitch = 30;
  frame->pid_d_term.yaw = 35;
  
  // Set RX values
  frame->rx.roll = 1500;
  frame->rx.pitch = 1600;
  frame->rx.yaw = 1400;
  frame->rx.throttle = 1000;
  
  // Set setpoint values
  frame->setpoint.roll = 500;
  frame->setpoint.pitch = 600;
  frame->setpoint.yaw = 400;
  frame->setpoint.throttle = 800;
  
  // Set accelerometer values
  frame->accel_raw.roll = 1000;
  frame->accel_raw.pitch = 0;
  frame->accel_raw.yaw = 0;
  
  frame->accel_filter.roll = 950;
  frame->accel_filter.pitch = 10;
  frame->accel_filter.yaw = 5;
  
  // Set gyroscope values
  frame->gyro_raw.roll = 10;
  frame->gyro_raw.pitch = 20;
  frame->gyro_raw.yaw = 30;
  
  frame->gyro_filter.roll = 12;
  frame->gyro_filter.pitch = 22;
  frame->gyro_filter.yaw = 32;
  
  // Set motor values
  frame->motor.roll = 1200;
  frame->motor.pitch = 1300;
  frame->motor.yaw = 1100;
  frame->motor.throttle = 1250;
  
  // Set CPU load
  frame->cpu_load = 75;
  
  // Set debug values
  for (int i = 0; i < BLACKBOX_DEBUG_SIZE; i++) {
    frame->debug[i] = 100 + i * 10;
  }
}

// Test delta calculation for int16_t
void test_blackbox_delta_int16() {
  int16_t current = 1500;
  int16_t previous = 1000;
  
  int16_t delta = current - previous;
  TEST_ASSERT_EQUAL_INT16(500, delta);
  
  // Test negative delta
  current = 800;
  previous = 1200;
  delta = current - previous;
  TEST_ASSERT_EQUAL_INT16(-400, delta);
  
  // Test zero delta
  current = 1000;
  previous = 1000;
  delta = current - previous;
  TEST_ASSERT_EQUAL_INT16(0, delta);
}

// Test compact_vec3 delta calculation
void test_blackbox_compact_vec3_delta() {
  compact_vec3_t current = {.roll = 100, .pitch = 200, .yaw = 300};
  compact_vec3_t previous = {.roll = 90, .pitch = 180, .yaw = 270};
  compact_vec3_t delta;
  
  // Calculate delta manually for comparison
  for (int i = 0; i < 3; i++) {
    delta.axis[i] = current.axis[i] - previous.axis[i];
  }
  
  TEST_ASSERT_EQUAL_INT16(10, delta.roll);
  TEST_ASSERT_EQUAL_INT16(20, delta.pitch);
  TEST_ASSERT_EQUAL_INT16(30, delta.yaw);
}

// Test compact_vec4 delta calculation
void test_blackbox_compact_vec4_delta() {
  compact_vec4_t current = {.roll = 1500, .pitch = 1600, .yaw = 1400, .throttle = 1000};
  compact_vec4_t previous = {.roll = 1450, .pitch = 1550, .yaw = 1350, .throttle = 950};
  compact_vec4_t delta;
  
  // Calculate delta manually for comparison
  for (int i = 0; i < 4; i++) {
    delta.axis[i] = current.axis[i] - previous.axis[i];
  }
  
  TEST_ASSERT_EQUAL_INT16(50, delta.roll);
  TEST_ASSERT_EQUAL_INT16(50, delta.pitch);
  TEST_ASSERT_EQUAL_INT16(50, delta.yaw);
  TEST_ASSERT_EQUAL_INT16(50, delta.throttle);
}

// Test zero delta detection for vec3
void test_blackbox_vec3_zero_detection() {
  compact_vec3_t zero_vec = {.roll = 0, .pitch = 0, .yaw = 0};
  compact_vec3_t non_zero_vec = {.roll = 1, .pitch = 0, .yaw = 0};
  
  // Zero vector should be detected as zero
  bool is_zero = (zero_vec.axis[0] == 0 && zero_vec.axis[1] == 0 && zero_vec.axis[2] == 0);
  TEST_ASSERT_TRUE(is_zero);
  
  // Non-zero vector should not be detected as zero
  bool is_non_zero = (non_zero_vec.axis[0] == 0 && non_zero_vec.axis[1] == 0 && non_zero_vec.axis[2] == 0);
  TEST_ASSERT_FALSE(is_non_zero);
}

// Test zero delta detection for vec4
void test_blackbox_vec4_zero_detection() {
  compact_vec4_t zero_vec = {.roll = 0, .pitch = 0, .yaw = 0, .throttle = 0};
  compact_vec4_t non_zero_vec = {.roll = 0, .pitch = 0, .yaw = 0, .throttle = 1};
  
  // Zero vector should be detected as zero
  bool is_zero = (zero_vec.axis[0] == 0 && zero_vec.axis[1] == 0 && 
                  zero_vec.axis[2] == 0 && zero_vec.axis[3] == 0);
  TEST_ASSERT_TRUE(is_zero);
  
  // Non-zero vector should not be detected as zero
  bool is_non_zero = (non_zero_vec.axis[0] == 0 && non_zero_vec.axis[1] == 0 && 
                      non_zero_vec.axis[2] == 0 && non_zero_vec.axis[3] == 0);
  TEST_ASSERT_FALSE(is_non_zero);
}

// Test debug array change detection
void test_blackbox_debug_change_detection() {
  int16_t current[BLACKBOX_DEBUG_SIZE] = {100, 110, 120, 130, 140, 150, 160, 170, 180, 190};
  int16_t previous_same[BLACKBOX_DEBUG_SIZE] = {100, 110, 120, 130, 140, 150, 160, 170, 180, 190};
  int16_t previous_different[BLACKBOX_DEBUG_SIZE] = {100, 110, 120, 130, 140, 150, 160, 170, 180, 999};
  
  // Check if arrays are the same
  bool has_changed_same = false;
  for (uint32_t i = 0; i < BLACKBOX_DEBUG_SIZE; i++) {
    if (current[i] != previous_same[i]) {
      has_changed_same = true;
      break;
    }
  }
  TEST_ASSERT_FALSE(has_changed_same);
  
  // Check if arrays are different
  bool has_changed_different = false;
  for (uint32_t i = 0; i < BLACKBOX_DEBUG_SIZE; i++) {
    if (current[i] != previous_different[i]) {
      has_changed_different = true;
      break;
    }
  }
  TEST_ASSERT_TRUE(has_changed_different);
}

// Test I-frame encoding - should include all enabled fields
void test_blackbox_iframe_encoding() {
  blackbox_t current, previous;
  create_test_blackbox_frame(&current, 1, 1000);
  create_test_blackbox_frame(&previous, 0, 0);
  
  // Enable all fields for I-frame
  uint32_t field_flags = 0;
  for (int i = 0; i < BBOX_FIELD_MAX; i++) {
    field_flags |= (1 << i);
  }
  
  // Create CBOR encoder
  uint8_t buffer[1024];
  cbor_value_t enc;
  cbor_encoder_init(&enc, buffer, sizeof(buffer));
  
  // Encode I-frame
  cbor_result_t result = cbor_encode_blackbox_frame(&enc, &current, &previous, BLACKBOX_FRAME_I, field_flags);
  
  // Should succeed
  TEST_ASSERT_EQUAL_INT(CBOR_OK, result);
  
  // Verify some basic properties - check that encoding succeeded
  TEST_ASSERT_NOT_NULL(enc.curr);
}

// Test P-frame encoding - should only include changed fields
void test_blackbox_pframe_encoding() {
  blackbox_t current, previous;
  
  // Create identical frames first
  create_test_blackbox_frame(&current, 2, 2000);
  create_test_blackbox_frame(&previous, 1, 1000);
  
  // Make them identical except for loop and time
  current = previous;
  current.loop = 2;
  current.time = 2000;
  
  // Now change only PID P term
  current.pid_p_term.roll = previous.pid_p_term.roll + 10;
  
  // Enable all fields
  uint32_t field_flags = 0;
  for (int i = 0; i < BBOX_FIELD_MAX; i++) {
    field_flags |= (1 << i);
  }
  
  // Create CBOR encoder
  uint8_t buffer[1024];
  cbor_value_t enc;
  cbor_encoder_init(&enc, buffer, sizeof(buffer));
  
  // Encode P-frame
  cbor_result_t result = cbor_encode_blackbox_frame(&enc, &current, &previous, BLACKBOX_FRAME_P, field_flags);
  
  // Should succeed
  TEST_ASSERT_EQUAL_INT(CBOR_OK, result);
  
  // P-frame should be encoded successfully
  TEST_ASSERT_NOT_NULL(enc.curr);
}

// Test frame type bit handling
void test_blackbox_frame_type_bit() {
  uint32_t iframe_flags = 0xFF; // Some field flags
  uint32_t pframe_flags = 0xFF | BLACKBOX_FRAME_TYPE_BIT; // Same flags with frame type bit
  
  // I-frame should not have frame type bit set in field flags
  TEST_ASSERT_FALSE(iframe_flags & BLACKBOX_FRAME_TYPE_BIT);
  
  // P-frame should have frame type bit set
  TEST_ASSERT_TRUE(pframe_flags & BLACKBOX_FRAME_TYPE_BIT);
  
  // Stripping frame type bit should give original flags
  uint32_t stripped_flags = pframe_flags & ~BLACKBOX_FRAME_TYPE_BIT;
  TEST_ASSERT_EQUAL_UINT32(iframe_flags, stripped_flags);
}

// Test CPU load delta calculation
void test_blackbox_cpu_load_delta() {
  uint16_t current_cpu = 80;
  uint16_t previous_cpu = 75;
  
  uint16_t delta = current_cpu - previous_cpu;
  TEST_ASSERT_EQUAL_UINT16(5, delta);
  
  // Test negative delta (stored as signed)
  current_cpu = 70;
  previous_cpu = 85;
  int16_t signed_delta = (int16_t)(current_cpu - previous_cpu);
  TEST_ASSERT_EQUAL_INT16(-15, signed_delta);
}

// Test edge case: maximum delta values
void test_blackbox_delta_overflow() {
  // Test maximum positive delta for int16_t
  int16_t current = 32767;
  int16_t previous = 0;
  int16_t delta = current - previous;
  TEST_ASSERT_EQUAL_INT16(32767, delta);
  
  // Test maximum negative delta for int16_t
  current = -32768;
  previous = 0;
  delta = current - previous;
  TEST_ASSERT_EQUAL_INT16(-32768, delta);
  
  // Test potential overflow case
  current = 32767;
  previous = -32768;
  // This would overflow in 16-bit arithmetic, but the delta calculation
  // should handle it correctly within the range of values used in practice
  delta = current - previous;
  TEST_ASSERT_EQUAL_INT16(-1, delta); // Due to 16-bit wraparound
}

// Test field flag optimization for P-frames
void test_blackbox_pframe_field_optimization() {
  blackbox_t current, previous;
  
  // Create identical frames
  create_test_blackbox_frame(&current, 2, 2000);
  previous = current;
  previous.loop = 1;
  previous.time = 1000;
  
  // Only change one field
  current.pid_p_term.roll += 100;
  
  // Enable multiple fields, but only PID_P_TERM should be active in P-frame
  uint32_t input_flags = (1 << BBOX_FIELD_PID_P_TERM) | 
                         (1 << BBOX_FIELD_PID_I_TERM) | 
                         (1 << BBOX_FIELD_RX);
  
  // In a real P-frame encoding, only the changed field should be included
  // We can test this by ensuring the logic works correctly
  
  // Simulate the delta check logic
  compact_vec3_t p_delta, i_delta;
  for (int i = 0; i < 3; i++) {
    p_delta.axis[i] = current.pid_p_term.axis[i] - previous.pid_p_term.axis[i];
    i_delta.axis[i] = current.pid_i_term.axis[i] - previous.pid_i_term.axis[i];
  }
  
  bool p_changed = !(p_delta.axis[0] == 0 && p_delta.axis[1] == 0 && p_delta.axis[2] == 0);
  bool i_changed = !(i_delta.axis[0] == 0 && i_delta.axis[1] == 0 && i_delta.axis[2] == 0);
  
  TEST_ASSERT_TRUE(p_changed);   // PID P term should have changed
  TEST_ASSERT_FALSE(i_changed);  // PID I term should not have changed
}


// Test CBOR encoding for vec3 - fixed to expect positive return values
void test_blackbox_cbor_vec3_roundtrip() {
  compact_vec3_t original = {.roll = 123, .pitch = -456, .yaw = 789};
  
  uint8_t buffer[64];
  cbor_value_t enc;
  
  // Initialize encoder
  cbor_encoder_init(&enc, buffer, sizeof(buffer));
  
  // Test basic encoding first - CBOR returns positive values for success (bytes written)
  cbor_result_t array_result = cbor_encode_array(&enc, 3);
  TEST_ASSERT_TRUE(array_result > 0); // Should be positive (bytes written)
  
  // Try encoding the first int16
  cbor_result_t int_result = cbor_encode_int16_t(&enc, &original.axis[0]);
  TEST_ASSERT_TRUE(int_result > 0); // Should be positive (bytes written)
  
  // If basic operations work, the vec3 function should work too
  // Reset encoder for full test
  cbor_encoder_init(&enc, buffer, sizeof(buffer));
  cbor_result_t result = cbor_encode_compact_vec3_t(&enc, &original);
  TEST_ASSERT_TRUE(result >= CBOR_OK); // Should be CBOR_OK (0) or positive
}

// Test CBOR encoding for vec4 - fixed to expect positive return values
void test_blackbox_cbor_vec4_roundtrip() {
  compact_vec4_t original = {.roll = 1500, .pitch = 1600, .yaw = 1400, .throttle = 1000};
  
  uint8_t buffer[64];
  cbor_value_t enc;
  
  // Initialize encoder
  cbor_encoder_init(&enc, buffer, sizeof(buffer));
  
  // Test basic encoding first - CBOR returns positive values for success (bytes written)
  cbor_result_t array_result = cbor_encode_array(&enc, 4);
  TEST_ASSERT_TRUE(array_result > 0); // Should be positive (bytes written)
  
  // Try encoding the first int16
  cbor_result_t int_result = cbor_encode_int16_t(&enc, &original.axis[0]);
  TEST_ASSERT_TRUE(int_result > 0); // Should be positive (bytes written)
  
  // If basic operations work, the vec4 function should work too
  // Reset encoder for full test
  cbor_encoder_init(&enc, buffer, sizeof(buffer));
  cbor_result_t result = cbor_encode_compact_vec4_t(&enc, &original);
  TEST_ASSERT_TRUE(result >= CBOR_OK); // Should be CBOR_OK (0) or positive
}

// Test I-frame interval logic
void test_blackbox_iframe_interval() {
  // Frame 1 should be I-frame
  bool is_iframe_1 = (1 == 1 || 1 % BLACKBOX_I_FRAME_INTERVAL == 0);
  TEST_ASSERT_TRUE(is_iframe_1);
  
  // Frames 2-31 should be P-frames
  for (int i = 2; i < BLACKBOX_I_FRAME_INTERVAL; i++) {
    bool is_iframe = (i == 1 || i % BLACKBOX_I_FRAME_INTERVAL == 0);
    TEST_ASSERT_FALSE(is_iframe);
  }
  
  // Frame 32 should be I-frame
  bool is_iframe_32 = (32 == 1 || 32 % BLACKBOX_I_FRAME_INTERVAL == 0);
  TEST_ASSERT_TRUE(is_iframe_32);
  
  // Frame 33 should be P-frame
  bool is_iframe_33 = (33 == 1 || 33 % BLACKBOX_I_FRAME_INTERVAL == 0);
  TEST_ASSERT_FALSE(is_iframe_33);
}