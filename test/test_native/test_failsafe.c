#include <unity.h>

#include <string.h>

#include "control/control.h"
#include "core/profile.h"
#include "core/target.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "io/msp.h"
#include "rx/crsf.h"
#include "rx/rx.h"
#include "rx/unified_serial.h"
#include "util/crc.h"
#include "util/ring_buffer.h"
#include "util/util.h"

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
  serial_rx.rx_error_count = 0;
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

static uint8_t crsf_test_read_tx_frame(uint8_t *frame) {
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(2U, ring_buffer_available(serial_rx.tx_buffer));
  TEST_ASSERT_EQUAL_UINT32(2U, ring_buffer_read_multi(serial_rx.tx_buffer, frame, 2));

  const uint8_t frame_size = frame[1] + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;
  TEST_ASSERT_TRUE(frame_size <= CRSF_FRAME_SIZE_MAX);
  TEST_ASSERT_EQUAL_UINT32(frame_size - 2U, ring_buffer_read_multi(serial_rx.tx_buffer, frame + 2, frame_size - 2U));
  return frame_size;
}

static void crsf_test_assert_frame_crc(const uint8_t *frame, uint8_t frame_size) {
  TEST_ASSERT_EQUAL_UINT8(CRSF_SYNC_BYTE, frame[0]);
  TEST_ASSERT_EQUAL_UINT8(frame[1] + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, frame_size);
  TEST_ASSERT_EQUAL_UINT8(crc8_dvb_s2_data(0, frame + 2, frame[1] - CRSF_FRAME_LENGTH_CRC), frame[frame_size - 1]);
}

static uint32_t crsf_test_read_u32(const uint8_t *data) {
  return ((uint32_t)data[0] << 24) |
         ((uint32_t)data[1] << 16) |
         ((uint32_t)data[2] << 8) |
         data[3];
}

static int16_t crsf_test_read_i16(const uint8_t *data) {
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void crsf_test_assert_device_info_tail(const uint8_t *frame, uint8_t frame_size) {
  uint8_t tail_offset = 5;
  while (tail_offset < frame_size && frame[tail_offset] != 0)
    tail_offset++;

  TEST_ASSERT_TRUE(tail_offset < frame_size);
  tail_offset++;
  TEST_ASSERT_EQUAL_UINT8(frame_size - CRSF_FRAME_LENGTH_CRC, tail_offset + 14);
  TEST_ASSERT_EQUAL_UINT32(get_chip_uid(), crsf_test_read_u32(frame + tail_offset));
  TEST_ASSERT_EQUAL_UINT32(CRSF_DEVICEINFO_HARDWARE_ID, crsf_test_read_u32(frame + tail_offset + 4));
  TEST_ASSERT_EQUAL_UINT32(target_info.firmware_version, crsf_test_read_u32(frame + tail_offset + 8));
  TEST_ASSERT_EQUAL_UINT8(CRSF_DEVICEINFO_PARAMETER_COUNT, frame[tail_offset + 12]);
  TEST_ASSERT_EQUAL_UINT8(CRSF_DEVICEINFO_VERSION, frame[tail_offset + 13]);
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

static uint16_t crsf_test_write_bits(uint8_t *payload, uint16_t bit_offset, uint16_t value, uint8_t bit_count) {
  for (uint8_t bit = 0; bit < bit_count; bit++) {
    const uint16_t payload_bit = bit_offset + bit;
    if (value & (1U << bit))
      payload[payload_bit / 8] |= 1U << (payload_bit % 8);
  }

  return bit_offset + bit_count;
}

static uint8_t crsf_test_pack_subset_payload(uint8_t *payload, uint8_t start_channel, uint8_t channel_bits, const uint16_t *values, uint8_t count) {
  payload[0] = (start_channel & 0x1F) | ((channel_bits - 10) << 5);

  uint16_t bit_offset = 8;
  for (uint8_t i = 0; i < count; i++) {
    bit_offset = crsf_test_write_bits(payload, bit_offset, values[i], channel_bits);
  }

  return (bit_offset + 7) / 8;
}

static uint16_t crsf_test_scale_subset_channel(uint16_t value, uint8_t channel_bits) {
  return (uint16_t)((uint32_t)value * AUX_VALUE_MAX / ((1U << channel_bits) - 1));
}

static void crsf_test_write_speed_proposal(uint8_t port_id, uint32_t baudrate) {
  crsf_speed_proposal_payload_t payload = {
      .header = {
          .destination = CRSF_ADDRESS_FLIGHT_CONTROLLER,
          .origin = CRSF_ADDRESS_CRSF_RECEIVER,
          .subcommand = CRSF_COMMAND_SUBCMD_GENERAL,
          .command = CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL,
      },
      .port_id = port_id,
      .baudrate = __builtin_bswap32(baudrate),
  };
  const uint8_t proposal_size_without_crc = sizeof(payload) - CRSF_FRAME_LENGTH_CRC;
  uint8_t command_data[CRSF_FRAME_LENGTH_TYPE + sizeof(payload) - CRSF_FRAME_LENGTH_CRC] = {
      CRSF_FRAMETYPE_COMMAND,
  };
  memcpy(&command_data[CRSF_FRAME_LENGTH_TYPE], &payload, proposal_size_without_crc);
  payload.command_crc = crsf_command_crc8(command_data, sizeof(command_data));

  crsf_test_write_frame(CRSF_FRAMETYPE_COMMAND, (const uint8_t *)&payload, sizeof(payload));
}

static void crsf_test_assert_speed_response(uint8_t port_id, bool expected_response) {
  crsf_speed_response_frame_t response = {0};

  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(sizeof(response), ring_buffer_available(serial_rx.tx_buffer));
  TEST_ASSERT_EQUAL_UINT32(sizeof(response), ring_buffer_read_multi(serial_rx.tx_buffer, (uint8_t *)&response, sizeof(response)));
  TEST_ASSERT_EQUAL_UINT8(CRSF_SYNC_BYTE, response.address);
  TEST_ASSERT_EQUAL_UINT8(sizeof(response) - CRSF_FRAME_LENGTH_ADDRESS - CRSF_FRAME_LENGTH_FRAMELENGTH, response.frame_length);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_COMMAND, response.type);
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_CRSF_RECEIVER, response.payload.header.destination);
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, response.payload.header.origin);
  TEST_ASSERT_EQUAL_UINT8(CRSF_COMMAND_SUBCMD_GENERAL, response.payload.header.subcommand);
  TEST_ASSERT_EQUAL_UINT8(CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE, response.payload.header.command);
  TEST_ASSERT_EQUAL_UINT8(port_id, response.payload.port_id);
  TEST_ASSERT_EQUAL_UINT8(expected_response ? 1U : 0U, response.payload.response);
  const uint8_t command_crc_size = CRSF_FRAME_LENGTH_TYPE + sizeof(response.payload);
  const uint8_t frame_crc_size = command_crc_size + CRSF_FRAME_LENGTH_CRC;
  TEST_ASSERT_EQUAL_UINT8(crsf_command_crc8(&response.type, command_crc_size), response.command_crc);
  TEST_ASSERT_EQUAL_UINT8(crc8_dvb_s2_data(0, &response.type, frame_crc_size), response.crc);
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

void test_crsf_v3_subset_channel_frame_clears_signal_lost_on_recovery(void) {
  crsf_test_reset(4000000);
  time_test_set_us(4000000 + FAILSAFE_DETECT_TIME_US + 1000);
  flags.failsafe_signal_lost = 1;

  const uint16_t values[16] = {
      0,
      1024,
      2047,
      512,
      256,
      768,
      1536,
      1800,
      100,
      200,
      300,
      400,
      500,
      600,
      700,
      800,
  };
  uint8_t payload[CRSF_PAYLOAD_SIZE_MAX] = {0};
  const uint8_t payload_length = crsf_test_pack_subset_payload(payload, 0, 11, values, 16);
  crsf_test_write_frame(CRSF_FRAMETYPE_RC_CHANNELS_SUBSET_PACKED, payload, payload_length);

  TEST_ASSERT_TRUE(rx_serial_check());
  TEST_ASSERT_FALSE(flags.failsafe_signal_lost);
  TEST_ASSERT_EQUAL_UINT32(time_micros(), state.last_frame_time_us);
  TEST_ASSERT_EQUAL_UINT16(0U, state.rx_channels[0]);
  TEST_ASSERT_EQUAL_UINT16(crsf_test_scale_subset_channel(1024, 11), state.rx_channels[1]);
  TEST_ASSERT_EQUAL_UINT16(AUX_VALUE_MAX, state.rx_channels[2]);
  TEST_ASSERT_EQUAL_UINT16(crsf_test_scale_subset_channel(800, 11), state.rx_channels[15]);
}

void test_crsf_v3_subset_channel_frame_handles_offset_10bit_channels(void) {
  crsf_test_reset(5000000);
  state.rx_channels[3] = 1234;

  const uint16_t values[4] = {0, 341, 682, 1023};
  uint8_t payload[CRSF_PAYLOAD_SIZE_MAX] = {0};
  const uint8_t payload_length = crsf_test_pack_subset_payload(payload, 4, 10, values, 4);
  crsf_test_write_frame(CRSF_FRAMETYPE_RC_CHANNELS_SUBSET_PACKED, payload, payload_length);

  TEST_ASSERT_TRUE(rx_serial_check());
  TEST_ASSERT_EQUAL_UINT16(1234U, state.rx_channels[3]);
  TEST_ASSERT_EQUAL_UINT16(0U, state.rx_channels[4]);
  TEST_ASSERT_EQUAL_UINT16(crsf_test_scale_subset_channel(341, 10), state.rx_channels[5]);
  TEST_ASSERT_EQUAL_UINT16(crsf_test_scale_subset_channel(682, 10), state.rx_channels[6]);
  TEST_ASSERT_EQUAL_UINT16(AUX_VALUE_MAX, state.rx_channels[7]);
}

void test_crsf_speed_proposal_accepts_baudrate_after_response_flush_and_delay(void) {
  crsf_test_reset(6000000);
  crsf_test_write_speed_proposal(2, 1500000);

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_UINT32(420000U, serial_rx.config.baudrate);
  TEST_ASSERT_FALSE(serial_rx.tx_done);

  serial_rx.tx_done = true;
  time_test_advance_us(4000);

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_UINT32(420000U, serial_rx.config.baudrate);
  TEST_ASSERT_TRUE(serial_rx.tx_done);
  crsf_test_assert_speed_response(2, true);

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_UINT32(1500000U, serial_rx.config.baudrate);
  TEST_ASSERT_FALSE(serial_rx.tx_done);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1U, ring_buffer_available(serial_rx.tx_buffer));
}

void test_crsf_speed_proposal_rejects_baudrate_above_limit(void) {
  crsf_test_reset(7000000);
  crsf_test_write_speed_proposal(3, 3000000);

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_UINT32(420000U, serial_rx.config.baudrate);
  TEST_ASSERT_FALSE(serial_rx.tx_done);
  crsf_test_assert_speed_response(3, false);

  serial_rx.tx_done = true;

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_UINT32(420000U, serial_rx.config.baudrate);
}

void test_crsf_negotiated_baudrate_falls_back_to_default_on_parse_errors(void) {
  crsf_test_reset(7050000);
  crsf_test_write_speed_proposal(2, 1500000);

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_UINT32(CRSF_BAUDRATE_DEFAULT, serial_rx.config.baudrate);
  TEST_ASSERT_FALSE(serial_rx.tx_done);

  serial_rx.tx_done = true;
  crsf_test_assert_speed_response(2, true);
  time_test_advance_us(4000);

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_UINT32(1500000U, serial_rx.config.baudrate);

  serial_rx.tx_done = true;
  ring_buffer_clear(serial_rx.tx_buffer);

  for (uint32_t i = 0; i < 199; i++) {
    const uint8_t garbage = 0;
    ring_buffer_write_multi(serial_rx.rx_buffer, &garbage, sizeof(garbage));
    TEST_ASSERT_FALSE(rx_serial_check());
    TEST_ASSERT_EQUAL_UINT32(1500000U, serial_rx.config.baudrate);
  }

  const uint8_t garbage = 0;
  ring_buffer_write_multi(serial_rx.rx_buffer, &garbage, sizeof(garbage));

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_UINT32(CRSF_BAUDRATE_DEFAULT, serial_rx.config.baudrate);
}

void test_crsf_negotiated_baudrate_falls_back_to_default_on_serial_errors(void) {
  crsf_test_reset(7060000);
  crsf_test_write_speed_proposal(2, 1500000);

  TEST_ASSERT_FALSE(rx_serial_check());
  serial_rx.tx_done = true;
  crsf_test_assert_speed_response(2, true);
  time_test_advance_us(4000);

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_UINT32(1500000U, serial_rx.config.baudrate);

  serial_rx.tx_done = true;
  ring_buffer_clear(serial_rx.tx_buffer);

  for (uint32_t i = 0; i < 199; i++) {
    serial_rx.rx_error_count++;
    TEST_ASSERT_FALSE(rx_serial_check());
    TEST_ASSERT_EQUAL_UINT32(1500000U, serial_rx.config.baudrate);
  }

  serial_rx.rx_error_count++;

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_UINT32(CRSF_BAUDRATE_DEFAULT, serial_rx.config.baudrate);
}

void test_crsf_negotiated_baudrate_does_not_fallback_on_silence(void) {
  crsf_test_reset(7070000);
  crsf_test_write_speed_proposal(2, 1500000);

  TEST_ASSERT_FALSE(rx_serial_check());
  serial_rx.tx_done = true;
  crsf_test_assert_speed_response(2, true);
  time_test_advance_us(4000);

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_UINT32(1500000U, serial_rx.config.baudrate);

  serial_rx.tx_done = true;
  ring_buffer_clear(serial_rx.tx_buffer);
  time_test_advance_us(FAILSAFE_DETECT_TIME_US + 1000);

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_TRUE(flags.failsafe_signal_lost);
  TEST_ASSERT_EQUAL_UINT32(1500000U, serial_rx.config.baudrate);
}

void test_crsf_device_ping_queues_device_info_immediately(void) {
  crsf_test_reset(8000000);

  const uint8_t empty_payload[1] = {0};
  crsf_test_write_frame(CRSF_FRAMETYPE_DEVICE_PING, empty_payload, 0);

  TEST_ASSERT_FALSE(rx_serial_check());

  uint8_t frame[CRSF_FRAME_SIZE_MAX] = {0};
  const uint8_t frame_size = crsf_test_read_tx_frame(frame);
  crsf_test_assert_frame_crc(frame, frame_size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_DEVICE_INFO, frame[2]);
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_RADIO_TRANSMITTER, frame[3]);
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame[4]);
  crsf_test_assert_device_info_tail(frame, frame_size);
}

void test_crsf_device_ping_uses_ping_origin_as_device_info_destination(void) {
  crsf_test_reset(8050000);

  const uint8_t payload[] = {
      CRSF_ADDRESS_FLIGHT_CONTROLLER,
      CRSF_ADDRESS_CRSF_RECEIVER,
  };
  crsf_test_write_frame(CRSF_FRAMETYPE_DEVICE_PING, payload, sizeof(payload));

  TEST_ASSERT_FALSE(rx_serial_check());

  uint8_t frame[CRSF_FRAME_SIZE_MAX] = {0};
  const uint8_t frame_size = crsf_test_read_tx_frame(frame);
  crsf_test_assert_frame_crc(frame, frame_size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_DEVICE_INFO, frame[2]);
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_CRSF_RECEIVER, frame[3]);
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame[4]);
}

void test_crsf_gps_extended_frame_uses_gps_status(void) {
  crsf_test_reset(8100000);

  gps_status.fix_type = GPS_FIX_3D;
  gps_status.vel_n = 12340;
  gps_status.vel_e = -5670;
  gps_status.vel_d = -890;
  gps_status.speed_acc = 2345;
  gps_status.head_acc = 120000;
  gps_status.height = 1234500;
  gps_status.h_acc = 3456;
  gps_status.v_acc = 4567;
  gps_status.pdop = 123;

  uint8_t frame[CRSF_FRAME_SIZE_MAX] = {0};
  const uint8_t frame_size = crsf_tlm_frame_gps_extended(frame);

  crsf_test_assert_frame_crc(frame, frame_size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_GPS_EXTENDED, frame[2]);
  TEST_ASSERT_EQUAL_UINT8(GPS_FIX_3D, frame[3]);
  TEST_ASSERT_EQUAL_INT16(1234, crsf_test_read_i16(frame + 4));
  TEST_ASSERT_EQUAL_INT16(-567, crsf_test_read_i16(frame + 6));
  TEST_ASSERT_EQUAL_INT16(89, crsf_test_read_i16(frame + 8));
  TEST_ASSERT_EQUAL_INT16(234, crsf_test_read_i16(frame + 10));
  TEST_ASSERT_EQUAL_INT16(12, crsf_test_read_i16(frame + 12));
  TEST_ASSERT_EQUAL_INT16(1234, crsf_test_read_i16(frame + 14));
  TEST_ASSERT_EQUAL_INT16(345, crsf_test_read_i16(frame + 16));
  TEST_ASSERT_EQUAL_INT16(456, crsf_test_read_i16(frame + 18));
  TEST_ASSERT_EQUAL_UINT8(0U, frame[20]);
  TEST_ASSERT_EQUAL_UINT8(12U, frame[21]);
  TEST_ASSERT_EQUAL_UINT8(12U, frame[22]);
}

void test_crsf_flight_mode_frame_reports_mode_text(void) {
  crsf_test_reset(8200000);
  state.aux_active = (1U << AUX_LEVELMODE) | (1U << AUX_RACEMODE) | (1U << AUX_HORIZON);
  flags.arm_state = 0;
  flags.arming_disabled_flags = ARMING_DISABLED_NONE;

  uint8_t frame[CRSF_FRAME_SIZE_MAX] = {0};
  const uint8_t frame_size = crsf_tlm_frame_flight_mode(frame);

  crsf_test_assert_frame_crc(frame, frame_size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_FLIGHT_MODE, frame[2]);
  TEST_ASSERT_EQUAL_STRING("RM HORIZON*", (char *)&frame[3]);
}

void test_crsf_msp_request_queues_response_immediately(void) {
  crsf_test_reset(9000000);

  const uint8_t payload[] = {
      CRSF_ADDRESS_FLIGHT_CONTROLLER,
      CRSF_ADDRESS_CRSF_RECEIVER,
      MSP_STATUS_START_MASK | (1 << MSP_STATUS_VERSION_SHIFT),
      0,
      MSP_API_VERSION,
  };
  crsf_test_write_frame(CRSF_FRAMETYPE_MSP_REQ, payload, sizeof(payload));

  TEST_ASSERT_FALSE(rx_serial_check());

  uint8_t frame[CRSF_FRAME_SIZE_MAX] = {0};
  const uint8_t frame_size = crsf_test_read_tx_frame(frame);
  crsf_test_assert_frame_crc(frame, frame_size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_MSP_RESP, frame[2]);
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_CRSF_RECEIVER, frame[3]);
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame[4]);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATUS_START_MASK | (1 << MSP_STATUS_VERSION_SHIFT), frame[5] & (MSP_STATUS_START_MASK | MSP_STATUS_VERSION_MASK | MSP_STATUS_ERROR_MASK));
  TEST_ASSERT_EQUAL_UINT8(3U, frame[6]);
  TEST_ASSERT_EQUAL_UINT8(MSP_API_VERSION, frame[7]);
  TEST_ASSERT_EQUAL_UINT8(0U, frame[8]);
  TEST_ASSERT_EQUAL_UINT8(1U, frame[9]);
  TEST_ASSERT_EQUAL_UINT8(42U, frame[10]);
}

void test_crsf_link_statistics_updates_collected_stats(void) {
  crsf_test_reset(10000000);
  profile.receiver.lqi_source = RX_LQI_SOURCE_DIRECT;
  crsf_stats.uplink_fps = 777;

  const uint8_t link_stats[CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE] = {90, 91, 92, 3, 1, 4, CRSF_TX_POWER_100_MW, 80, 79, -2};
  crsf_test_write_frame(CRSF_FRAMETYPE_LINK_STATISTICS, link_stats, sizeof(link_stats));

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_FLOAT(92.0f, state.rx_rssi);
  TEST_ASSERT_EQUAL_UINT8(90U, crsf_stats.uplink_rssi_2);
  TEST_ASSERT_EQUAL_UINT8(91U, crsf_stats.uplink_rssi_1);
  TEST_ASSERT_EQUAL_UINT8(92U, crsf_stats.uplink_link_quality);
  TEST_ASSERT_EQUAL_INT8(3, crsf_stats.uplink_snr);
  TEST_ASSERT_EQUAL_UINT8(1U, crsf_stats.active_antenna);
  TEST_ASSERT_EQUAL_UINT8(4U, crsf_stats.rf_mode);
  TEST_ASSERT_EQUAL_UINT8(CRSF_TX_POWER_100_MW, crsf_stats.uplink_tx_power);
  TEST_ASSERT_EQUAL_UINT8(80U, crsf_stats.downlink_rssi);
  TEST_ASSERT_EQUAL_UINT8(79U, crsf_stats.downlink_link_quality);
  TEST_ASSERT_EQUAL_INT8(-2, crsf_stats.downlink_snr);
  TEST_ASSERT_EQUAL_UINT16(777U, crsf_stats.uplink_fps);
}

void test_crsf_tx_link_statistics_updates_direct_lqi(void) {
  crsf_test_reset(2000000);
  profile.receiver.lqi_source = RX_LQI_SOURCE_DIRECT;
  crsf_stats.rf_mode = 1;

  const uint8_t tx_link_stats[CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE] = {70, 80, 63, 5, 20, 50};
  crsf_test_write_frame(CRSF_FRAMETYPE_LINK_STATISTICS_TX, tx_link_stats, sizeof(tx_link_stats));

  TEST_ASSERT_FALSE(rx_serial_check());
  TEST_ASSERT_EQUAL_FLOAT(63.0f, state.rx_rssi);
  TEST_ASSERT_EQUAL_UINT8(70U, crsf_stats.uplink_rssi_1);
  TEST_ASSERT_EQUAL_UINT8(70U, crsf_stats.uplink_rssi_2);
  TEST_ASSERT_EQUAL_UINT8(63U, crsf_stats.uplink_link_quality);
  TEST_ASSERT_EQUAL_INT8(5, crsf_stats.uplink_snr);
  TEST_ASSERT_EQUAL_UINT16(500U, crsf_stats.uplink_fps);
  TEST_ASSERT_EQUAL_FLOAT(500.0f, rx_serial_crsf_expected_fps());
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
