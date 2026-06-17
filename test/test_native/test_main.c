#include <unity.h>
#include "driver/time.h"
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
extern void test_angle_pid_uses_legacy_dterm_timefactor(void);

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

// Failsafe tests
extern void test_failsafe_holds_last_values_before_hold_timeout(void);
extern void test_failsafe_blocks_outputs_while_rx_not_ready(void);
extern void test_failsafe_stage1_applies_centered_zero_throttle_fallback(void);
extern void test_failsafe_stage2_drop_blocks_outputs_and_disarms(void);
extern void test_failsafe_recovery_clears_with_arm_switch_high_but_blocks_rearm(void);
extern void test_failsafe_repeated_loss_keeps_outputs_blocked_after_stage2(void);
extern void test_failsafe_blocks_new_arming_while_outputs_are_allowed(void);
extern void test_failsafe_hold_keeps_existing_arm_until_stage2(void);
extern void test_regular_rearm_requires_prearm_cycle_after_disarm(void);
extern void test_default_prearm_allows_regular_rearm_when_always_on(void);
extern void test_failsafe_rearm_allows_held_prearm_after_drop(void);
extern void test_failsafe_recovery_blocks_automatic_rearm_when_arm_stays_held(void);
extern void test_sbus_failsafe_frame_does_not_refresh_last_frame_time(void);
extern void test_crsf_no_frame_timeout_forces_rssi_to_zero(void);
extern void test_crsf_channel_frame_clears_signal_lost_on_recovery(void);
extern void test_crsf_channel_frame_allows_stage2_failsafe_recovery(void);
extern void test_crsf_v3_subset_channel_frame_clears_signal_lost_on_recovery(void);
extern void test_crsf_v3_subset_channel_frame_handles_offset_10bit_channels(void);
extern void test_crsf_speed_proposal_accepts_baudrate_after_response_flush_and_delay(void);
extern void test_crsf_speed_proposal_rejects_baudrate_above_limit(void);
extern void test_crsf_negotiated_baudrate_falls_back_to_default_on_parse_errors(void);
extern void test_crsf_negotiated_baudrate_falls_back_to_default_on_serial_errors(void);
extern void test_crsf_negotiated_baudrate_does_not_fallback_on_silence(void);
extern void test_crsf_device_ping_queues_device_info_immediately(void);
extern void test_crsf_device_ping_uses_ping_origin_as_device_info_destination(void);
extern void test_crsf_gps_extended_frame_uses_gps_status(void);
extern void test_crsf_flight_mode_frame_reports_mode_text(void);
extern void test_crsf_msp_request_queues_response_immediately(void);
extern void test_crsf_link_statistics_updates_collected_stats(void);
extern void test_crsf_tx_link_statistics_updates_direct_lqi(void);
extern void test_crsf_rx_link_statistics_updates_direct_lqi_and_downlink_stats(void);

// Common setUp and tearDown
void setUp(void) {
  // Reset hardware mocks before each test
  mock_hardware_reset_all();
  time_test_reset();
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
  RUN_TEST(test_angle_pid_uses_legacy_dterm_timefactor);

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

  // Failsafe tests
  RUN_TEST(test_failsafe_holds_last_values_before_hold_timeout);
  RUN_TEST(test_failsafe_blocks_outputs_while_rx_not_ready);
  RUN_TEST(test_failsafe_stage1_applies_centered_zero_throttle_fallback);
  RUN_TEST(test_failsafe_stage2_drop_blocks_outputs_and_disarms);
  RUN_TEST(test_failsafe_recovery_clears_with_arm_switch_high_but_blocks_rearm);
  RUN_TEST(test_failsafe_repeated_loss_keeps_outputs_blocked_after_stage2);
  RUN_TEST(test_failsafe_blocks_new_arming_while_outputs_are_allowed);
  RUN_TEST(test_failsafe_hold_keeps_existing_arm_until_stage2);
  RUN_TEST(test_regular_rearm_requires_prearm_cycle_after_disarm);
  RUN_TEST(test_default_prearm_allows_regular_rearm_when_always_on);
  RUN_TEST(test_failsafe_rearm_allows_held_prearm_after_drop);
  RUN_TEST(test_failsafe_recovery_blocks_automatic_rearm_when_arm_stays_held);
  RUN_TEST(test_sbus_failsafe_frame_does_not_refresh_last_frame_time);
  RUN_TEST(test_crsf_no_frame_timeout_forces_rssi_to_zero);
  RUN_TEST(test_crsf_channel_frame_clears_signal_lost_on_recovery);
  RUN_TEST(test_crsf_channel_frame_allows_stage2_failsafe_recovery);
  RUN_TEST(test_crsf_v3_subset_channel_frame_clears_signal_lost_on_recovery);
  RUN_TEST(test_crsf_v3_subset_channel_frame_handles_offset_10bit_channels);
  RUN_TEST(test_crsf_speed_proposal_accepts_baudrate_after_response_flush_and_delay);
  RUN_TEST(test_crsf_speed_proposal_rejects_baudrate_above_limit);
  RUN_TEST(test_crsf_negotiated_baudrate_falls_back_to_default_on_parse_errors);
  RUN_TEST(test_crsf_negotiated_baudrate_falls_back_to_default_on_serial_errors);
  RUN_TEST(test_crsf_negotiated_baudrate_does_not_fallback_on_silence);
  RUN_TEST(test_crsf_device_ping_queues_device_info_immediately);
  RUN_TEST(test_crsf_device_ping_uses_ping_origin_as_device_info_destination);
  RUN_TEST(test_crsf_gps_extended_frame_uses_gps_status);
  RUN_TEST(test_crsf_flight_mode_frame_reports_mode_text);
  RUN_TEST(test_crsf_msp_request_queues_response_immediately);
  RUN_TEST(test_crsf_link_statistics_updates_collected_stats);
  RUN_TEST(test_crsf_tx_link_statistics_updates_direct_lqi);
  RUN_TEST(test_crsf_rx_link_statistics_updates_direct_lqi_and_downlink_stats);

  return UNITY_END();
}
