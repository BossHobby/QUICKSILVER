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
extern void test_pid_dterm_setpoint_response(void);
extern void test_pid_dterm_combined_response(void);
extern void test_pid_dterm_stick_weighting(void);
extern void test_pid_voltage_compensation(void);
extern void test_pid_complete_loop(void);

// IMU tests
extern void test_imu_gravity_vector_init(void);
extern void test_imu_gyro_integration(void);
extern void test_imu_accel_fusion_ground(void);
extern void test_imu_accel_magnitude_rejection(void);
extern void test_imu_attitude_calculation(void);
extern void test_imu_in_flight_behavior(void);
extern void test_imu_heading_level(void);
extern void test_imu_heading_tilted(void);
extern void test_imu_gps_fusion_low_speed(void);
extern void test_imu_gps_fusion_variable_gain(void);
extern void test_imu_gps_fusion_trust_speed(void);
extern void test_imu_heading_normalization(void);
extern void test_imu_gps_fusion_poor_accuracy(void);
extern void test_imu_heading_complex_rotation(void);

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

// Blackbox tests
extern void test_blackbox_delta_int16(void);
extern void test_blackbox_compact_vec3_delta(void);
extern void test_blackbox_compact_vec4_delta(void);
extern void test_blackbox_vec3_zero_detection(void);
extern void test_blackbox_vec4_zero_detection(void);
extern void test_blackbox_debug_change_detection(void);
extern void test_blackbox_iframe_encoding(void);
extern void test_blackbox_pframe_encoding(void);
extern void test_blackbox_frame_type_bit(void);
extern void test_blackbox_cpu_load_delta(void);
extern void test_blackbox_delta_overflow(void);
extern void test_blackbox_pframe_field_optimization(void);
extern void test_blackbox_cbor_vec3_roundtrip(void);
extern void test_blackbox_cbor_vec4_roundtrip(void);
extern void test_blackbox_iframe_interval(void);

// Attitude tests
extern void test_attitude_initial_state(void);
extern void test_attitude_level_flight(void);
extern void test_attitude_roll_rotation(void);
extern void test_attitude_pitch_rotation(void);
extern void test_attitude_yaw_rotation(void);
extern void test_attitude_accel_correction(void);
extern void test_attitude_gps_heading_fusion(void);
extern void test_attitude_gps_suppression_yaw_stick(void);
extern void test_attitude_gps_suppression_roll(void);
extern void test_attitude_no_gps_when_stationary(void);
extern void test_attitude_quaternion_normalization(void);
extern void test_attitude_heading_wraparound(void);

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
  RUN_TEST(test_pid_dterm_setpoint_response);
  RUN_TEST(test_pid_dterm_combined_response);
  RUN_TEST(test_pid_dterm_stick_weighting);
  RUN_TEST(test_pid_voltage_compensation);
  RUN_TEST(test_pid_complete_loop);

  // IMU tests
  RUN_TEST(test_imu_gravity_vector_init);
  RUN_TEST(test_imu_gyro_integration);
  RUN_TEST(test_imu_accel_fusion_ground);
  RUN_TEST(test_imu_accel_magnitude_rejection);
  RUN_TEST(test_imu_attitude_calculation);
  RUN_TEST(test_imu_in_flight_behavior);
  RUN_TEST(test_imu_heading_level);
  RUN_TEST(test_imu_heading_tilted);
  RUN_TEST(test_imu_gps_fusion_low_speed);
  RUN_TEST(test_imu_gps_fusion_variable_gain);
  RUN_TEST(test_imu_gps_fusion_trust_speed);
  RUN_TEST(test_imu_heading_normalization);
  RUN_TEST(test_imu_gps_fusion_poor_accuracy);
  RUN_TEST(test_imu_heading_complex_rotation);

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

  // Blackbox tests
  RUN_TEST(test_blackbox_delta_int16);
  RUN_TEST(test_blackbox_compact_vec3_delta);
  RUN_TEST(test_blackbox_compact_vec4_delta);
  RUN_TEST(test_blackbox_vec3_zero_detection);
  RUN_TEST(test_blackbox_vec4_zero_detection);
  RUN_TEST(test_blackbox_debug_change_detection);
  RUN_TEST(test_blackbox_iframe_encoding);
  RUN_TEST(test_blackbox_pframe_encoding);
  RUN_TEST(test_blackbox_frame_type_bit);
  RUN_TEST(test_blackbox_cpu_load_delta);
  RUN_TEST(test_blackbox_delta_overflow);
  RUN_TEST(test_blackbox_pframe_field_optimization);
  RUN_TEST(test_blackbox_cbor_vec3_roundtrip);
  RUN_TEST(test_blackbox_cbor_vec4_roundtrip);
  RUN_TEST(test_blackbox_iframe_interval);

  // Attitude tests
  RUN_TEST(test_attitude_initial_state);
  RUN_TEST(test_attitude_level_flight);
  RUN_TEST(test_attitude_roll_rotation);
  RUN_TEST(test_attitude_pitch_rotation);
  RUN_TEST(test_attitude_yaw_rotation);
  RUN_TEST(test_attitude_accel_correction);
  RUN_TEST(test_attitude_gps_heading_fusion);
  RUN_TEST(test_attitude_gps_suppression_yaw_stick);
  RUN_TEST(test_attitude_gps_suppression_roll);
  RUN_TEST(test_attitude_no_gps_when_stationary);
  RUN_TEST(test_attitude_quaternion_normalization);
  RUN_TEST(test_attitude_heading_wraparound);

  return UNITY_END();
}