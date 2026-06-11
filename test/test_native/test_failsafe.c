#include <unity.h>

#include <string.h>

#include "control/control.h"
#include "core/profile.h"
#include "core/target.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "rx/crsf.h"
#include "rx/rx.h"
#include "rx/unified_serial.h"
#include "util/crc.h"
#include "util/ring_buffer.h"

static void failsafe_reset(uint32_t now_us) {
  time_test_reset();
  time_test_set_us(now_us);

  flags.rx_ready = 1;
  flags.failsafe_signal_lost = 0;
  flags.failsafe = 0;
  flags.failsafe_outputs_blocked = 0;
  flags.arm_request = 0;
  flags.controls_override = 0;
  flags.arm_state = 1;
  flags.usb_active = 0;
  flags.arming_disabled_flags = ARMING_DISABLED_NONE;
  flags.turtle = 0;
  flags.turtle_ready = 0;

  profile.receiver.aux[AUX_PREARM] = (aux_function_map_t){RX_CHANNEL_ON, 0, AUX_VALUE_MAX};

  state.aux_active = 0;
  state.failsafe_time_ms = 0;
  state.failsafe_phase = FAILSAFE_PHASE_IDLE;
  state.last_frame_time_us = now_us;
  state.rx_filtered.throttle = 0.0f;
  state.rx_override.roll = 0.5f;
  state.rx_override.pitch = -0.5f;
  state.rx_override.yaw = 0.25f;
  state.rx_override.throttle = 0.75f;
}

static void failsafe_clear_arm_switch_latch(void) {
  flags.rx_ready = 1;
  flags.failsafe = 0;
  flags.failsafe_outputs_blocked = 0;
  flags.failsafe_signal_lost = 0;
  flags.usb_active = 0;
  flags.arm_state = 0;
  state.aux_active = 0;
  state.rx_filtered.throttle = 0.0f;

  control_update_arming();
  control_update_arming();

  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_NONE, flags.arming_disabled_flags);
}

static void failsafe_use_prearm_switch(void) {
  profile.receiver.aux[AUX_PREARM] = (aux_function_map_t){RX_CHANNEL_5, 1, AUX_VALUE_MAX};
}

static void serial_test_reset(uint32_t now_us, rx_serial_protocol_t protocol, uint32_t baudrate, serial_direction_t direction, serial_stop_bits_t stop_bits) {
  failsafe_reset(now_us);

  ring_buffer_clear(serial_rx.rx_buffer);
  ring_buffer_clear(serial_rx.tx_buffer);
  target.serial_ports[SERIAL_PORT1] = (target_serial_port_t){
      .index = 1,
      .rx = PIN_A10,
      .tx = PIN_A9,
      .inverter = PIN_NONE,
  };
  serial_rx.config = (serial_port_config_t){
      .port = SERIAL_PORT1,
      .baudrate = baudrate,
      .direction = direction,
      .stop_bits = stop_bits,
      .invert = false,
      .half_duplex = false,
      .half_duplex_pp = false,
  };
  serial_rx.tx_done = true;
  profile.serial.rx = SERIAL_PORT1;
  profile.serial.smart_audio = SERIAL_PORT_INVALID;
  serial_rx_detected_protcol = protocol;
  state.rx_rssi = 0.0f;
}

static void crsf_test_reset(uint32_t now_us) {
  serial_test_reset(now_us, RX_SERIAL_PROTOCOL_CRSF, 420000, SERIAL_DIR_TX_RX, SERIAL_STOP_BITS_1);
  memset(&crsf_stats, 0, sizeof(crsf_stats));
}

static void crsf_test_write_frame(uint8_t type, const uint8_t *payload, uint8_t payload_length) {
  uint8_t frame[CRSF_FRAME_SIZE_MAX] = {0};
  frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame[1] = payload_length + CRSF_FRAME_LENGTH_TYPE_CRC;
  frame[2] = type;
  memcpy(&frame[3], payload, payload_length);
  frame[3 + payload_length] = crc8_dvb_s2_data(0, &frame[2], payload_length + 1);
  ring_buffer_write_multi(serial_rx.rx_buffer, frame, payload_length + 4);
}

static void sbus_test_write_frame(uint8_t frame_flags) {
  uint8_t frame[25] = {0};
  frame[0] = 0x0F;
  frame[23] = frame_flags;
  ring_buffer_write_multi(serial_rx.rx_buffer, frame, sizeof(frame));
}

static crsf_channels_t crsf_test_centered_channels(void) {
  return (crsf_channels_t){
      .chan0 = 992,
      .chan1 = 992,
      .chan2 = 992,
      .chan3 = 992,
      .chan4 = 992,
      .chan5 = 992,
      .chan6 = 992,
      .chan7 = 992,
      .chan8 = 992,
      .chan9 = 992,
      .chan10 = 992,
      .chan11 = 992,
      .chan12 = 992,
      .chan13 = 992,
      .chan14 = 992,
      .chan15 = 992,
  };
}

void test_failsafe_holds_last_values_before_hold_timeout(void) {
  failsafe_reset(1000000);
  flags.failsafe_signal_lost = 1;
  time_test_set_us(1000000 + FAILSAFE_DETECT_TIME_US + 1000);

  control_failsafe_update();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_HOLD_LAST, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_NOT_EQUAL(0U, state.failsafe_time_ms);
  TEST_ASSERT_FALSE(flags.controls_override);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
}

void test_failsafe_blocks_outputs_while_rx_not_ready(void) {
  failsafe_reset(1000000);
  flags.rx_ready = 0;

  control_failsafe_update();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_IDLE, state.failsafe_phase);
  TEST_ASSERT_TRUE(flags.failsafe);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_FALSE(flags.controls_override);

  flags.rx_ready = 1;
  flags.failsafe_signal_lost = 0;
  control_failsafe_update();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_IDLE, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
}

void test_failsafe_stage1_applies_centered_zero_throttle_fallback(void) {
  failsafe_reset(1000000);
  flags.failsafe_signal_lost = 1;
  time_test_set_us(1000000 + FAILSAFE_HOLD_TIME_US + 1000);

  control_failsafe_update();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_STAGE1_GUARD, state.failsafe_phase);
  TEST_ASSERT_TRUE(flags.failsafe);
  TEST_ASSERT_TRUE(flags.controls_override);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.rx_override.roll);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.rx_override.pitch);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.rx_override.yaw);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.rx_override.throttle);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
}

void test_failsafe_stage2_drop_blocks_outputs_and_disarms(void) {
  failsafe_reset(1000000);
  flags.failsafe_signal_lost = 1;
  time_test_set_us(1000000 + FAILSAFE_HOLD_TIME_US + 1000);
  control_failsafe_update();
  TEST_ASSERT_TRUE(flags.controls_override);

  time_test_set_us(1000000 + FAILSAFE_HOLD_TIME_US + FAILSAFE_STAGE2_TIME_US);

  control_failsafe_update();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_STAGE2_DROP, state.failsafe_phase);
  TEST_ASSERT_TRUE(flags.failsafe);
  TEST_ASSERT_FALSE(flags.controls_override);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);
}

void test_failsafe_recovery_clears_with_arm_switch_high_but_blocks_rearm(void) {
  failsafe_reset(1000000);
  flags.arm_state = 0;
  flags.failsafe_signal_lost = 1;
  time_test_set_us(1000000 + FAILSAFE_HOLD_TIME_US + FAILSAFE_STAGE2_TIME_US);
  control_failsafe_update();

  flags.failsafe_signal_lost = 0;
  state.aux_active = 1U << AUX_ARMING;
  time_test_advance_us(1000);
  control_failsafe_update();
  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_RECOVERY, state.failsafe_phase);
  TEST_ASSERT_TRUE(flags.failsafe);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);

  time_test_advance_us(FAILSAFE_RECOVERY_TIME_US);
  control_failsafe_update();
  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_IDLE, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_EQUAL_UINT32(0U, state.failsafe_time_ms);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);

  control_update_arming();
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_TRUE((flags.arming_disabled_flags & ARMING_DISABLED_ARM_SWITCH) != 0);

  state.aux_active = 0;
  control_update_arming();
  control_update_arming();
  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_NONE, flags.arming_disabled_flags);
}

void test_failsafe_recovery_blocks_automatic_rearm_when_arm_stays_held(void) {
  failsafe_reset(1000000);
  flags.failsafe_signal_lost = 1;
  time_test_set_us(1000000 + FAILSAFE_HOLD_TIME_US + FAILSAFE_STAGE2_TIME_US);
  control_failsafe_update();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_STAGE2_DROP, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.arm_state);

  flags.failsafe_signal_lost = 0;
  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  time_test_advance_us(1000);
  control_failsafe_update();
  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_RECOVERY, state.failsafe_phase);
  TEST_ASSERT_TRUE(flags.failsafe);

  time_test_advance_us(FAILSAFE_RECOVERY_TIME_US);
  control_failsafe_update();
  control_update_arming();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_IDLE, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_TRUE((flags.arming_disabled_flags & ARMING_DISABLED_ARM_SWITCH) != 0);

  state.aux_active = 1U << AUX_PREARM;
  control_update_arming();
  control_update_arming();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_IDLE, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_NONE, flags.arming_disabled_flags);

  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);

  control_update_arming();
  TEST_ASSERT_TRUE(flags.arm_state);
}

void test_failsafe_repeated_loss_keeps_outputs_blocked_after_stage2(void) {
  failsafe_reset(1000000);
  flags.failsafe_signal_lost = 1;
  time_test_set_us(1000000 + FAILSAFE_HOLD_TIME_US + FAILSAFE_STAGE2_TIME_US);
  control_failsafe_update();
  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_STAGE2_DROP, state.failsafe_phase);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);

  flags.failsafe_signal_lost = 0;
  state.aux_active = 1U << AUX_ARMING;
  time_test_advance_us(1000);
  control_failsafe_update();
  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_RECOVERY, state.failsafe_phase);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);

  state.last_frame_time_us = time_micros();
  flags.failsafe_signal_lost = 1;
  time_test_advance_us(1000);
  control_failsafe_update();
  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_STAGE2_DROP, state.failsafe_phase);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_FALSE(flags.arm_state);

  flags.failsafe_signal_lost = 0;
  state.aux_active = 0;
  time_test_advance_us(1000);
  control_failsafe_update();
  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_RECOVERY, state.failsafe_phase);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);

  time_test_advance_us(FAILSAFE_RECOVERY_TIME_US);
  control_failsafe_update();
  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_IDLE, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
}

void test_failsafe_blocks_new_arming_while_outputs_are_allowed(void) {
  failsafe_reset(1000000);
  failsafe_clear_arm_switch_latch();

  flags.failsafe_signal_lost = 1;
  time_test_set_us(1000000 + FAILSAFE_DETECT_TIME_US + 1000);
  control_failsafe_update();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_HOLD_LAST, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);

  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  control_update_arming();

  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_TRUE((flags.arming_disabled_flags & ARMING_DISABLED_FAILSAFE) != 0);
}

void test_failsafe_hold_keeps_existing_arm_until_stage2(void) {
  failsafe_reset(1000000);
  failsafe_clear_arm_switch_latch();

  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  control_update_arming();
  TEST_ASSERT_TRUE(flags.arm_state);

  flags.failsafe_signal_lost = 1;
  time_test_set_us(1000000 + FAILSAFE_DETECT_TIME_US + 1000);
  control_failsafe_update();
  control_update_arming();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_HOLD_LAST, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_TRUE(flags.arm_state);
}

void test_regular_rearm_requires_prearm_cycle_after_disarm(void) {
  failsafe_reset(1000000);
  failsafe_use_prearm_switch();
  failsafe_clear_arm_switch_latch();

  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  control_update_arming();
  TEST_ASSERT_TRUE(flags.arm_state);

  state.aux_active = 1U << AUX_PREARM;
  control_update_arming();
  control_update_arming();
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_NONE, flags.arming_disabled_flags);

  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  control_update_arming();
  TEST_ASSERT_FALSE(flags.arm_state);

  state.aux_active = 0;
  control_update_arming();
  control_update_arming();
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_NONE, flags.arming_disabled_flags);

  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  control_update_arming();
  TEST_ASSERT_TRUE(flags.arm_state);
}

void test_default_prearm_allows_regular_rearm_when_always_on(void) {
  failsafe_reset(1000000);
  failsafe_clear_arm_switch_latch();

  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  control_update_arming();
  TEST_ASSERT_TRUE(flags.arm_state);

  state.aux_active = 1U << AUX_PREARM;
  control_update_arming();
  control_update_arming();
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_NONE, flags.arming_disabled_flags);

  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  control_update_arming();
  TEST_ASSERT_TRUE(flags.arm_state);
}

void test_failsafe_rearm_allows_held_prearm_after_drop(void) {
  failsafe_reset(1000000);
  failsafe_use_prearm_switch();
  failsafe_clear_arm_switch_latch();

  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  control_update_arming();
  TEST_ASSERT_TRUE(flags.arm_state);

  flags.failsafe_signal_lost = 1;
  time_test_set_us(1000000 + FAILSAFE_HOLD_TIME_US + FAILSAFE_STAGE2_TIME_US);
  control_failsafe_update();
  control_update_arming();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_STAGE2_DROP, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);

  flags.failsafe_signal_lost = 0;
  state.aux_active = 1U << AUX_PREARM;
  time_test_advance_us(1000);
  control_failsafe_update();
  control_update_arming();

  time_test_advance_us(FAILSAFE_RECOVERY_TIME_US);
  control_failsafe_update();
  control_update_arming();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_IDLE, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_NONE, flags.arming_disabled_flags);

  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  control_update_arming();
  TEST_ASSERT_TRUE(flags.arm_state);
}

void test_sbus_failsafe_frame_does_not_refresh_last_frame_time(void) {
  serial_test_reset(1000000, RX_SERIAL_PROTOCOL_SBUS, 100000, SERIAL_DIR_RX, SERIAL_STOP_BITS_2);

  sbus_test_write_frame(1U << 3);
  time_test_advance_us(5000);

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_TRUE(flags.failsafe_signal_lost);
  TEST_ASSERT_EQUAL_UINT32(1000000U, state.last_frame_time_us);

  sbus_test_write_frame(0);
  time_test_advance_us(5000);

  TEST_ASSERT_TRUE(rx_serial_check());
  TEST_ASSERT_FALSE(flags.failsafe_signal_lost);
  TEST_ASSERT_EQUAL_UINT32(1010000U, state.last_frame_time_us);
}

void test_crsf_no_frame_timeout_forces_rssi_to_zero(void) {
  crsf_test_reset(1000000);
  state.rx_rssi = 75.0f;
  time_test_set_us(1000000 + FAILSAFE_DETECT_TIME_US + 1000);

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_TRUE(flags.failsafe_signal_lost);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.rx_rssi);
}

void test_crsf_channel_frame_clears_signal_lost_on_recovery(void) {
  crsf_test_reset(1000000);
  time_test_set_us(1000000 + FAILSAFE_DETECT_TIME_US + 1000);
  flags.failsafe_signal_lost = 1;

  crsf_channels_t channels_frame = crsf_test_centered_channels();
  crsf_test_write_frame(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, (const uint8_t *)&channels_frame, CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE);

  TEST_ASSERT_TRUE(rx_serial_check());
  TEST_ASSERT_FALSE(flags.failsafe_signal_lost);
  TEST_ASSERT_EQUAL_UINT32(time_micros(), state.last_frame_time_us);
}

void test_crsf_channel_frame_allows_stage2_failsafe_recovery(void) {
  crsf_test_reset(1000000);
  flags.failsafe_signal_lost = 1;
  time_test_set_us(1000000 + FAILSAFE_HOLD_TIME_US + FAILSAFE_STAGE2_TIME_US);
  control_failsafe_update();

  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_STAGE2_DROP, state.failsafe_phase);
  TEST_ASSERT_TRUE(flags.failsafe);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);

  crsf_channels_t channels_frame = crsf_test_centered_channels();
  crsf_test_write_frame(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, (const uint8_t *)&channels_frame, CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE);

  TEST_ASSERT_TRUE(rx_serial_check());
  TEST_ASSERT_FALSE(flags.failsafe_signal_lost);

  control_failsafe_update();
  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_RECOVERY, state.failsafe_phase);
  TEST_ASSERT_TRUE(flags.failsafe);

  time_test_advance_us(FAILSAFE_RECOVERY_TIME_US);
  control_failsafe_update();
  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PHASE_IDLE, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
}

void test_crsf_tx_link_statistics_updates_direct_lqi(void) {
  crsf_test_reset(2000000);
  profile.receiver.lqi_source = RX_LQI_SOURCE_DIRECT;

  const uint8_t tx_link_stats[CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE] = {70, 80, 63, 5, 20, 50};
  crsf_test_write_frame(CRSF_FRAMETYPE_LINK_STATISTICS_TX, tx_link_stats, sizeof(tx_link_stats));

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_FLOAT(63.0f, state.rx_rssi);
  TEST_ASSERT_EQUAL_UINT8(70U, crsf_stats.uplink_rssi_1);
  TEST_ASSERT_EQUAL_UINT8(70U, crsf_stats.uplink_rssi_2);
  TEST_ASSERT_EQUAL_UINT8(63U, crsf_stats.uplink_link_quality);
  TEST_ASSERT_EQUAL_INT8(5, crsf_stats.uplink_snr);
}

void test_crsf_rx_link_statistics_updates_direct_lqi_and_downlink_stats(void) {
  crsf_test_reset(3000000);
  profile.receiver.lqi_source = RX_LQI_SOURCE_DIRECT;

  const uint8_t rx_link_stats[CRSF_FRAME_LINK_STATISTICS_RX_PAYLOAD_SIZE] = {71, 82, 64, 6, 21};
  crsf_test_write_frame(CRSF_FRAMETYPE_LINK_STATISTICS_RX, rx_link_stats, sizeof(rx_link_stats));

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_FLOAT(64.0f, state.rx_rssi);
  TEST_ASSERT_EQUAL_UINT8(71U, crsf_stats.downlink_rssi);
  TEST_ASSERT_EQUAL_UINT8(64U, crsf_stats.downlink_link_quality);
  TEST_ASSERT_EQUAL_INT8(6, crsf_stats.downlink_snr);
}
