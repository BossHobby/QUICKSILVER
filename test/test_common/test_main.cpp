#include "driver/time.h"
#include "mock_helpers.h"
#include <unity.h>

// Test declarations
extern void test_osd_transfer_is_bounded_and_retries();
extern void test_osd_render_completes_before_transfer();
extern void test_resource_timer_tag_roundtrip();
extern void test_sdcard_csd_capacity();
extern void test_sdcard_target_cbor();
extern void test_sdcard_transport_initialization();
extern void test_sdcard_transport_late_insertion();
extern void test_sdcard_transport_reads();
extern void test_sdcard_transport_writes();
extern void test_sdcard_transport_errors();
extern void test_sdcard_transport_busy_and_timer_wrap();
extern void test_sdcard_device_samples_during_write();
extern void test_blackbox_navigation_roundtrip_and_unchanged_home(void);
extern void test_blackbox_full_frame_fits_device_buffer(void);
extern void test_attitude_rth_recovery_strengthens_when_moving_away(void);
extern void test_attitude_rth_recovery_requires_valid_course_and_phase(void);
extern void test_attitude_normal_heading_pitch_weight_matches_vehicle(void);
extern void test_attitude_imu_pipeline_roll_and_pitch(void);
extern void test_attitude_imu_pipeline_yaw_while_tilted(void);
extern void test_bmp280_raw_reference_sample_returns_pascals(void);
extern void test_bmp280_discards_unused_sample_nibbles(void);
extern void test_bmp388_raw_reference_sample_retains_cubic_correction(void);
extern void test_sdft_explicit_period_update();

// Filter tests
extern void test_filter_init(void);
extern void test_atan2approx_accuracy(void);
extern void test_filter_reconfigure_type(void);
extern void test_notch_matches_reference_with_changing_frequency(void);
extern void test_filter_lowpass_pt1(void);
extern void test_filter_highfreq_attenuation(void);
extern void test_filter_reset(void);
extern void test_filter_types(void);
extern void test_filter_cascade(void);

// IMU tests
extern void test_imu_gravity_vector_init(void);
extern void test_imu_gyro_integration(void);
extern void test_imu_accel_fusion_ground(void);
extern void test_imu_accel_magnitude_rejection(void);
extern void test_imu_zero_gravity_vector_does_not_nan(void);
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

// CBOR tests
extern void test_cbor_profile_servo_rate_roundtrip(void);
extern void test_cbor_enum_wire_format(void);
extern void test_cbor_failed_enum_decode_preserves_value(void);

// SPI tests
extern void test_spi_initial_state(void);
extern void test_spi_init(void);
extern void test_spi_txn_queue(void);
extern void test_spi_txn_full_queue(void);
extern void test_spi_sdcard_block_transfers(void);
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
extern void test_failsafe_stage1_applies_neutral_fallback(void);
extern void test_failsafe_stage2_drop_blocks_outputs_and_disarms(void);
extern void test_failsafe_rth_keeps_outputs_allowed_past_stage2_time(void);
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

// Attitude tests
extern void test_attitude_initial_state(void);
extern void test_attitude_level_flight(void);
extern void test_attitude_accepts_scaled_gravity_vector(void);
extern void test_attitude_ignores_invalid_gravity_vector(void);
extern void test_attitude_converges_faster_when_disarmed(void);
extern void test_attitude_roll_rotation(void);
extern void test_attitude_pitch_rotation(void);
extern void test_attitude_yaw_rotation(void);
extern void test_attitude_accel_correction(void);
extern void test_attitude_gps_heading_fusion(void);
extern void test_attitude_gps_suppression_yaw_stick(void);
extern void test_attitude_gps_suppression_roll(void);
extern void test_attitude_no_gps_when_stationary(void);
extern void test_attitude_gps_heading_requires_fresh_sample(void);
extern void test_attitude_quaternion_normalization(void);
extern void test_attitude_heading_wraparound(void);
extern void test_attitude_acquisition_rejects_bad_course(void);
extern void test_attitude_course_accuracy_tapers_recovery(void);
extern void test_attitude_forward_flight_builds_and_hover_retains_confidence(void);
extern void test_attitude_yaw_between_gps_samples_suppresses_correction(void);
extern void test_gps_configuration_and_fix_validity(void);
extern void test_gps_ground_and_airborne_configuration(void);
extern void test_gps_satellite_and_fix_loss_remain_visible_armed(void);

// Navigation tests
extern void test_attitude_gps_reacquisition_does_not_snap_yaw(void);
extern void test_attitude_rth_sideways_motion_does_not_correct_heading(void);
extern void test_attitude_rth_large_heading_error_can_recover(void);
extern void test_attitude_heading_correction_is_sample_rate_independent(void);

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
  RUN_TEST(test_osd_transfer_is_bounded_and_retries);
  RUN_TEST(test_osd_render_completes_before_transfer);
  RUN_TEST(test_resource_timer_tag_roundtrip);
  RUN_TEST(test_sdcard_csd_capacity);
  RUN_TEST(test_sdcard_target_cbor);
  RUN_TEST(test_sdcard_transport_initialization);
  RUN_TEST(test_sdcard_transport_late_insertion);
  RUN_TEST(test_sdcard_transport_reads);
  RUN_TEST(test_sdcard_transport_writes);
  RUN_TEST(test_sdcard_transport_errors);
  RUN_TEST(test_sdcard_transport_busy_and_timer_wrap);
  RUN_TEST(test_sdcard_device_samples_during_write);

  RUN_TEST(test_sdft_explicit_period_update);

  // Filter tests
  RUN_TEST(test_filter_init);
  RUN_TEST(test_atan2approx_accuracy);
  RUN_TEST(test_filter_reconfigure_type);
  RUN_TEST(test_notch_matches_reference_with_changing_frequency);
  RUN_TEST(test_filter_lowpass_pt1);
  RUN_TEST(test_filter_highfreq_attenuation);
  RUN_TEST(test_filter_reset);
  RUN_TEST(test_filter_types);
  RUN_TEST(test_filter_cascade);

  // IMU tests
  RUN_TEST(test_imu_gravity_vector_init);
  RUN_TEST(test_imu_gyro_integration);
  RUN_TEST(test_imu_accel_fusion_ground);
  RUN_TEST(test_imu_accel_magnitude_rejection);
  RUN_TEST(test_imu_zero_gravity_vector_does_not_nan);
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

  // CBOR tests
  RUN_TEST(test_cbor_profile_servo_rate_roundtrip);
  RUN_TEST(test_cbor_enum_wire_format);
  RUN_TEST(test_cbor_failed_enum_decode_preserves_value);

  // SPI tests
  RUN_TEST(test_spi_initial_state);
  RUN_TEST(test_spi_init);
  RUN_TEST(test_spi_txn_queue);
  RUN_TEST(test_spi_txn_full_queue);
  RUN_TEST(test_spi_sdcard_block_transfers);
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
  RUN_TEST(test_blackbox_navigation_roundtrip_and_unchanged_home);
  RUN_TEST(test_blackbox_full_frame_fits_device_buffer);
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
  RUN_TEST(test_failsafe_stage1_applies_neutral_fallback);
  RUN_TEST(test_failsafe_stage2_drop_blocks_outputs_and_disarms);
#ifdef VEHICLE_MULTI
  RUN_TEST(test_failsafe_rth_keeps_outputs_allowed_past_stage2_time);
#endif
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

  // Attitude tests
  RUN_TEST(test_attitude_initial_state);
  RUN_TEST(test_attitude_level_flight);
  RUN_TEST(test_attitude_accepts_scaled_gravity_vector);
  RUN_TEST(test_attitude_ignores_invalid_gravity_vector);
  RUN_TEST(test_attitude_converges_faster_when_disarmed);
  RUN_TEST(test_attitude_roll_rotation);
  RUN_TEST(test_attitude_pitch_rotation);
  RUN_TEST(test_attitude_yaw_rotation);
  RUN_TEST(test_attitude_accel_correction);
  RUN_TEST(test_attitude_gps_heading_fusion);
  RUN_TEST(test_attitude_gps_suppression_yaw_stick);
  RUN_TEST(test_attitude_gps_suppression_roll);
  RUN_TEST(test_attitude_no_gps_when_stationary);
  RUN_TEST(test_attitude_gps_heading_requires_fresh_sample);
  RUN_TEST(test_attitude_quaternion_normalization);
  RUN_TEST(test_attitude_heading_wraparound);
  RUN_TEST(test_attitude_acquisition_rejects_bad_course);
  RUN_TEST(test_attitude_course_accuracy_tapers_recovery);
  RUN_TEST(test_attitude_forward_flight_builds_and_hover_retains_confidence);
  RUN_TEST(test_attitude_yaw_between_gps_samples_suppresses_correction);
  RUN_TEST(test_gps_configuration_and_fix_validity);
  RUN_TEST(test_gps_ground_and_airborne_configuration);
  RUN_TEST(test_gps_satellite_and_fix_loss_remain_visible_armed);
  RUN_TEST(test_attitude_gps_reacquisition_does_not_snap_yaw);
  RUN_TEST(test_attitude_rth_sideways_motion_does_not_correct_heading);
  RUN_TEST(test_attitude_rth_large_heading_error_can_recover);
  RUN_TEST(test_attitude_heading_correction_is_sample_rate_independent);
  RUN_TEST(test_attitude_rth_recovery_strengthens_when_moving_away);
  RUN_TEST(test_attitude_rth_recovery_requires_valid_course_and_phase);
  RUN_TEST(test_attitude_normal_heading_pitch_weight_matches_vehicle);
  RUN_TEST(test_attitude_imu_pipeline_roll_and_pitch);
  RUN_TEST(test_attitude_imu_pipeline_yaw_while_tilted);
  RUN_TEST(test_bmp280_raw_reference_sample_returns_pascals);
  RUN_TEST(test_bmp280_discards_unused_sample_nibbles);
  RUN_TEST(test_bmp388_raw_reference_sample_retains_cubic_correction);
  return UNITY_END();
}
