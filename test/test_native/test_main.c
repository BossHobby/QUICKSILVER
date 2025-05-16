#include <unity.h>
#include "mock_helpers.h"

// Test declarations
// Filter tests
extern void test_filter_init(void);
extern void test_filter_lowpass_pt1(void);
extern void test_filter_highfreq_attenuation(void);
extern void test_filter_reset(void);
extern void test_filter_types(void);
extern void test_filter_cascade(void);

// PID tests  
extern void test_pid_proportional_control(void);
extern void test_pid_integral_accumulation(void);
extern void test_pid_derivative_calculation(void);
extern void test_pid_voltage_compensation(void);
extern void test_pid_complete_loop(void);

// IMU tests
extern void test_imu_gravity_vector_init(void);
extern void test_imu_gyro_integration(void);
extern void test_imu_accel_fusion_ground(void);
extern void test_imu_accel_magnitude_rejection(void);
extern void test_imu_attitude_calculation(void);
extern void test_imu_in_flight_behavior(void);

// Vector tests
extern void test_vec3_magnitude(void);
extern void test_vec3_rotate(void);
extern void test_vec3_from_array(void);
extern void test_vec3_compress(void);
extern void test_vec4_from_array(void);
extern void test_vec4_compress(void);
extern void test_vector_union_access(void);
extern void test_vec3_rotate_edge_cases(void);

// CRC tests
extern void test_crc8_single_byte(void);
extern void test_crc8_data_array(void);
extern void test_crc8_empty_data(void);
extern void test_crc8_properties(void);
extern void test_crc8_initial_values(void);
extern void test_crc8_sequential_calculation(void);
extern void test_crc8_known_values(void);

// Ring buffer tests
extern void test_ring_buffer_init(void);
extern void test_ring_buffer_single_write_read(void);
extern void test_ring_buffer_multiple_write_read(void);
extern void test_ring_buffer_multi_operations(void);
extern void test_ring_buffer_full(void);
extern void test_ring_buffer_empty_read(void);
extern void test_ring_buffer_wrap_around(void);
extern void test_ring_buffer_clear(void);
extern void test_ring_buffer_partial_multi_write(void);
extern void test_ring_buffer_partial_multi_read(void);

// SPI tests
extern void test_spi_init(void);
extern void test_spi_txn_queue(void);
extern void test_spi_dma_ready(void);
extern void test_spi_reconfigure(void);

// ADC tests
extern void test_adc_init(void);
extern void test_adc_read_temperature(void);
extern void test_adc_read_vbat(void);
extern void test_adc_read_ibat(void);

// Serial tests
extern void test_serial_init(void);
extern void test_serial_write_bytes(void);
extern void test_serial_write_bytes_null_port(void);
extern void test_serial_write_bytes_null_data(void);
extern void test_serial_write_bytes_zero_count(void);
extern void test_serial_port_defs(void);
extern void test_serial_read_bytes(void);
extern void test_serial_drain(void);

// Common setUp and tearDown
void setUp(void) {
  // Reset hardware mocks before each test
  mock_hardware_reset_all();
}

void tearDown(void) {
  // No specific cleanup needed
}

// Main test runner
int main(int argc, char **argv) {
  UNITY_BEGIN();

  // Filter tests
  RUN_TEST(test_filter_init);
  RUN_TEST(test_filter_lowpass_pt1);
  RUN_TEST(test_filter_highfreq_attenuation);
  RUN_TEST(test_filter_reset);
  RUN_TEST(test_filter_types);
  RUN_TEST(test_filter_cascade);

  // PID tests
  RUN_TEST(test_pid_proportional_control);
  RUN_TEST(test_pid_integral_accumulation);
  RUN_TEST(test_pid_derivative_calculation);
  RUN_TEST(test_pid_voltage_compensation);
  RUN_TEST(test_pid_complete_loop);

  // IMU tests
  RUN_TEST(test_imu_gravity_vector_init);
  RUN_TEST(test_imu_gyro_integration);
  RUN_TEST(test_imu_accel_fusion_ground);
  RUN_TEST(test_imu_accel_magnitude_rejection);
  RUN_TEST(test_imu_attitude_calculation);
  RUN_TEST(test_imu_in_flight_behavior);

  // Vector tests
  RUN_TEST(test_vec3_magnitude);
  RUN_TEST(test_vec3_rotate);
  RUN_TEST(test_vec3_from_array);
  RUN_TEST(test_vec3_compress);
  RUN_TEST(test_vec4_from_array);
  RUN_TEST(test_vec4_compress);
  RUN_TEST(test_vector_union_access);
  RUN_TEST(test_vec3_rotate_edge_cases);

  // CRC tests
  RUN_TEST(test_crc8_single_byte);
  RUN_TEST(test_crc8_data_array);
  RUN_TEST(test_crc8_empty_data);
  RUN_TEST(test_crc8_properties);
  RUN_TEST(test_crc8_initial_values);
  RUN_TEST(test_crc8_sequential_calculation);
  RUN_TEST(test_crc8_known_values);

  // Ring buffer tests
  RUN_TEST(test_ring_buffer_init);
  RUN_TEST(test_ring_buffer_single_write_read);
  RUN_TEST(test_ring_buffer_multiple_write_read);
  RUN_TEST(test_ring_buffer_multi_operations);
  RUN_TEST(test_ring_buffer_full);
  RUN_TEST(test_ring_buffer_empty_read);
  RUN_TEST(test_ring_buffer_wrap_around);
  RUN_TEST(test_ring_buffer_clear);
  RUN_TEST(test_ring_buffer_partial_multi_write);
  RUN_TEST(test_ring_buffer_partial_multi_read);

  // SPI tests
  RUN_TEST(test_spi_init);
  RUN_TEST(test_spi_txn_queue);
  RUN_TEST(test_spi_dma_ready);
  RUN_TEST(test_spi_reconfigure);

  // ADC tests
  RUN_TEST(test_adc_init);
  RUN_TEST(test_adc_read_temperature);
  RUN_TEST(test_adc_read_vbat);
  RUN_TEST(test_adc_read_ibat);

  // Serial tests
  RUN_TEST(test_serial_init);
  RUN_TEST(test_serial_write_bytes);
  RUN_TEST(test_serial_write_bytes_null_port);
  RUN_TEST(test_serial_write_bytes_null_data);
  RUN_TEST(test_serial_write_bytes_zero_count);
  RUN_TEST(test_serial_port_defs);
  RUN_TEST(test_serial_read_bytes);
  RUN_TEST(test_serial_drain);

  return UNITY_END();
}