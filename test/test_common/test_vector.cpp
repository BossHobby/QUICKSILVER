#include <math.h>
#include <string.h>
#include <unity.h>

// Include the vector utilities
#include "util/vector.h"

// Test vec3 magnitude calculation
void test_vec3_magnitude(void) {
  vec3_t v1 = {{3.0f, 4.0f, 0.0f}};
  TEST_ASSERT_EQUAL_FLOAT(5.0f, vec3_magnitude(v1));
  
  vec3_t v2 = {{1.0f, 1.0f, 1.0f}};
  TEST_ASSERT_EQUAL_FLOAT(sqrtf(3.0f), vec3_magnitude(v2));
  
  vec3_t v3 = {{0.0f, 0.0f, 0.0f}};
  TEST_ASSERT_EQUAL_FLOAT(0.0f, vec3_magnitude(v3));
  
  vec3_t v4 = {{2.0f, 2.0f, 2.0f}};
  TEST_ASSERT_EQUAL_FLOAT(sqrtf(12.0f), vec3_magnitude(v4));
}

// Test vec3 rotation (Rodrigues' formula)
void test_vec3_rotate(void) {
  // Test rotation around small angles
  vec3_t original = {{1.0f, 0.0f, 0.0f}};
  vec3_t rotation = {{0.0f, 0.0f, 0.1f}}; // Small rotation around yaw
  
  vec3_t result = vec3_rotate(original, rotation);
  
  // After small rotation around Z, X component should decrease slightly
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, result.axis[0]);
  // Y component should increase slightly
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, result.axis[1]);
  // Z component should remain nearly 0
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, result.axis[2]);
}

// Test vec3_from_array
void test_vec3_from_array(void) {
  float array[] = {1.5f, 2.5f, 3.5f};
  vec3_t vec;
  
  vec3_from_array(&vec, array);
  
  TEST_ASSERT_EQUAL_FLOAT(1.5f, vec.roll);
  TEST_ASSERT_EQUAL_FLOAT(2.5f, vec.pitch);
  TEST_ASSERT_EQUAL_FLOAT(3.5f, vec.yaw);
  
  // Test with named access
  TEST_ASSERT_EQUAL_FLOAT(1.5f, vec.axis[0]);
  TEST_ASSERT_EQUAL_FLOAT(2.5f, vec.axis[1]);
  TEST_ASSERT_EQUAL_FLOAT(3.5f, vec.axis[2]);
}

// Test vec3_compress
void test_vec3_compress(void) {
  vec3_t input = {{1.0f, 2.0f, 3.0f}};
  compact_vec3_t output;
  
  vec3_compress(&output, &input, 100.0f);
  
  TEST_ASSERT_EQUAL_INT16(100, output.roll);
  TEST_ASSERT_EQUAL_INT16(200, output.pitch);
  TEST_ASSERT_EQUAL_INT16(300, output.yaw);
  
  // Test with negative values
  vec3_t input2 = {{-0.5f, -1.5f, -2.5f}};
  vec3_compress(&output, &input2, 100.0f);
  
  TEST_ASSERT_EQUAL_INT16(-50, output.roll);
  TEST_ASSERT_EQUAL_INT16(-150, output.pitch);
  TEST_ASSERT_EQUAL_INT16(-250, output.yaw);
}

// Test vec4_from_array
void test_vec4_from_array(void) {
  float array[] = {1.1f, 2.2f, 3.3f, 4.4f};
  vec4_t vec;
  
  vec4_from_array(&vec, array);
  
  TEST_ASSERT_EQUAL_FLOAT(1.1f, vec.roll);
  TEST_ASSERT_EQUAL_FLOAT(2.2f, vec.pitch);
  TEST_ASSERT_EQUAL_FLOAT(3.3f, vec.yaw);
  TEST_ASSERT_EQUAL_FLOAT(4.4f, vec.throttle);
}

// Test vec4_compress
void test_vec4_compress(void) {
  vec4_t input = {{0.5f, 1.0f, 1.5f, 2.0f}};
  compact_vec4_t output;
  
  vec4_compress(&output, &input, 1000.0f);
  
  TEST_ASSERT_EQUAL_INT16(500, output.roll);
  TEST_ASSERT_EQUAL_INT16(1000, output.pitch);
  TEST_ASSERT_EQUAL_INT16(1500, output.yaw);
  TEST_ASSERT_EQUAL_INT16(2000, output.throttle);
}

// Test union access patterns
void test_vector_union_access(void) {
  vec3_t v3;
  
  // Set via named members
  v3.roll = 1.0f;
  v3.pitch = 2.0f;
  v3.yaw = 3.0f;
  
  // Access via array
  TEST_ASSERT_EQUAL_FLOAT(1.0f, v3.axis[0]);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, v3.axis[1]);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, v3.axis[2]);
  
  // Set via array
  v3.axis[0] = 4.0f;
  v3.axis[1] = 5.0f;
  v3.axis[2] = 6.0f;
  
  // Access via named members
  TEST_ASSERT_EQUAL_FLOAT(4.0f, v3.roll);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, v3.pitch);
  TEST_ASSERT_EQUAL_FLOAT(6.0f, v3.yaw);
}

// Test edge cases for rotation
void test_vec3_rotate_edge_cases(void) {
  // Zero rotation
  vec3_t vec = {{1.0f, 2.0f, 3.0f}};
  vec3_t zero_rot = {{0.0f, 0.0f, 0.0f}};
  
  vec3_t result = vec3_rotate(vec, zero_rot);
  
  TEST_ASSERT_EQUAL_FLOAT(vec.roll, result.roll);
  TEST_ASSERT_EQUAL_FLOAT(vec.pitch, result.pitch);
  TEST_ASSERT_EQUAL_FLOAT(vec.yaw, result.yaw);
  
  // Zero vector
  vec3_t zero_vec = {{0.0f, 0.0f, 0.0f}};
  vec3_t rotation = {{0.1f, 0.2f, 0.3f}};
  
  result = vec3_rotate(zero_vec, rotation);
  
  TEST_ASSERT_EQUAL_FLOAT(0.0f, result.roll);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, result.pitch);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, result.yaw);
}