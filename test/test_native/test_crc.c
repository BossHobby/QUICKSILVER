#include <string.h>
#include <unity.h>

// Include the CRC utilities
#include "util/crc.h"

// Test single byte CRC calculation
void test_crc8_single_byte(void) {
  // Test with initial CRC of 0
  uint8_t crc = 0;
  crc = crc8_dvb_s2_calc(crc, 0x01);
  TEST_ASSERT_NOT_EQUAL(0, crc);
  
  // Test with different input
  crc = 0;
  crc = crc8_dvb_s2_calc(crc, 0xFF);
  TEST_ASSERT_NOT_EQUAL(0, crc);
  
  // Test chaining calculations
  crc = 0;
  crc = crc8_dvb_s2_calc(crc, 0xAA);
  crc = crc8_dvb_s2_calc(crc, 0x55);
  TEST_ASSERT_NOT_EQUAL(0, crc);
}

// Test CRC calculation over data array
void test_crc8_data_array(void) {
  // Test with simple data
  uint8_t data1[] = {0x01, 0x02, 0x03, 0x04};
  uint8_t crc1 = crc8_dvb_s2_data(0, data1, sizeof(data1));
  TEST_ASSERT_NOT_EQUAL(0, crc1);
  
  // Test with same data should give same CRC
  uint8_t crc2 = crc8_dvb_s2_data(0, data1, sizeof(data1));
  TEST_ASSERT_EQUAL_UINT8(crc1, crc2);
  
  // Test with different data should give different CRC
  uint8_t data2[] = {0x01, 0x02, 0x03, 0x05};
  uint8_t crc3 = crc8_dvb_s2_data(0, data2, sizeof(data2));
  TEST_ASSERT_NOT_EQUAL(crc1, crc3);
}

// Test empty data
void test_crc8_empty_data(void) {
  uint8_t *empty_data = NULL;
  uint8_t crc = crc8_dvb_s2_data(0, empty_data, 0);
  TEST_ASSERT_EQUAL_UINT8(0, crc); // CRC of empty data with init 0 should be 0
  
  // CRC of empty data with different init should be the init value
  crc = crc8_dvb_s2_data(0x42, empty_data, 0);
  TEST_ASSERT_EQUAL_UINT8(0x42, crc);
}

// Test CRC properties
void test_crc8_properties(void) {
  // CRC should detect single bit errors
  uint8_t data1[] = {0b10101010, 0b11110000};
  uint8_t crc1 = crc8_dvb_s2_data(0, data1, sizeof(data1));
  
  uint8_t data2[] = {0b10101010, 0b11110001}; // Last bit flipped
  uint8_t crc2 = crc8_dvb_s2_data(0, data2, sizeof(data2));
  
  TEST_ASSERT_NOT_EQUAL(crc1, crc2);
  
  // CRC should detect byte order changes
  uint8_t data3[] = {0xAB, 0xCD};
  uint8_t crc3 = crc8_dvb_s2_data(0, data3, sizeof(data3));
  
  uint8_t data4[] = {0xCD, 0xAB}; // Bytes swapped
  uint8_t crc4 = crc8_dvb_s2_data(0, data4, sizeof(data4));
  
  TEST_ASSERT_NOT_EQUAL(crc3, crc4);
}

// Test with different initial CRC values
void test_crc8_initial_values(void) {
  uint8_t data[] = {0x12, 0x34, 0x56, 0x78};
  
  // Different initial values should produce different results
  uint8_t crc_init0 = crc8_dvb_s2_data(0x00, data, sizeof(data));
  uint8_t crc_init1 = crc8_dvb_s2_data(0xFF, data, sizeof(data));
  
  TEST_ASSERT_NOT_EQUAL(crc_init0, crc_init1);
  
  // Same initial value should produce same result
  uint8_t crc_init42_a = crc8_dvb_s2_data(0x42, data, sizeof(data));
  uint8_t crc_init42_b = crc8_dvb_s2_data(0x42, data, sizeof(data));
  
  TEST_ASSERT_EQUAL_UINT8(crc_init42_a, crc_init42_b);
}

// Test sequential vs all-at-once calculation
void test_crc8_sequential_calculation(void) {
  uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
  
  // Calculate all at once
  uint8_t crc_all = crc8_dvb_s2_data(0, data, sizeof(data));
  
  // Calculate sequentially
  uint8_t crc_seq = 0;
  for (size_t i = 0; i < sizeof(data); i++) {
    crc_seq = crc8_dvb_s2_calc(crc_seq, data[i]);
  }
  
  // Results should be identical
  TEST_ASSERT_EQUAL_UINT8(crc_all, crc_seq);
}

// Test with known test vectors (if available)
void test_crc8_known_values(void) {
  // Test with all zeros
  uint8_t zeros[10] = {0};
  uint8_t crc_zeros = crc8_dvb_s2_data(0, zeros, sizeof(zeros));
  TEST_ASSERT_EQUAL_UINT8(0, crc_zeros);
  
  // Test with all ones
  uint8_t ones[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t crc_ones = crc8_dvb_s2_data(0, ones, sizeof(ones));
  TEST_ASSERT_NOT_EQUAL(0, crc_ones);
  
  // Test with pattern
  uint8_t pattern[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
  uint8_t crc_pattern = crc8_dvb_s2_data(0, pattern, sizeof(pattern));
  // DVB-S2 CRC of "123456789" with init 0 should be a specific value
  // This would need to be verified with a reference implementation
  TEST_ASSERT_NOT_EQUAL(0, crc_pattern);
}