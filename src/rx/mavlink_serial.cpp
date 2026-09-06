#include "rx/mavlink_serial.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "control/control.h"
#include "core/profile.h"
#include "core/target.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "rx/rx.h"
#include "util/util.h"

#ifdef USE_RX_UNIFIED

/*
 * Minimal MAVLink implementation for the unified serial receiver.
 *
 * v1 (0xFE) and v2 (0xFD) framing; only the messages needed to receive RC
 * channels and send telemetry. Frames with an unknown crc_extra are dropped.
 * Payload fields follow the MAVLink wire layout (sorted by size, largest first,
 * arrays contiguous), not the order shown in the XML.
 *
 * Outbound is the allowlist in mavlink_telemetry_allowed(), and every field of
 * those messages maps to a value QuickSilver produces: HEARTBEAT.base_mode from
 * flags.arm_state, HEARTBEAT.system_status from flags.failsafe/arm_state,
 * BATTERY_STATUS.current_consumed from state.ibat_drawn (mAh), .current_battery
 * from state.ibat_filtered (mA, sent as 10 mA units) and .voltages from
 * state.vbat_cell_avg for state.lipo_cell_count cells. Fields with no producer -
 * .energy_consumed (-1), .temperature (INT16_MAX), .battery_remaining (-1), or an
 * absent voltage/current sensor (0xFFFF) - use the protocol's own unknown
 * sentinel rather than a fabricated zero. ATTITUDE, VFR_HUD and SYS_STATUS are
 * not sent at all: state.attitude.yaw is never written, the VFR_HUD and
 * SYS_STATUS fields have no producer, and MAVLink cannot mark one field unknown.
 *
 * Received channels are retained, so a message that omits one leaves it alone.
 * RC_CHANNELS marks unused channels UINT16_MAX. RC_CHANNELS_OVERRIDE: 0 = release
 * and UINT16_MAX = ignore on 1-8, both = ignore on 9-18 where release is
 * UINT16_MAX-1. A release drops the whole retained set and does not count as
 * fresh input, so QuickSilver failsafes instead of being handed a centred stick;
 * the same applies to an incomplete primary set or a packet addressed elsewhere.
 * RSSI scaling is documented at mavlink_update_rssi().
 */

#define MAVLINK_STX_V1 0xFE
#define MAVLINK_STX_V2 0xFD

// Our own MAVLink identity
#define MAVLINK_SYSID 1
#define MAVLINK_COMPID 1

// Receive payload cap, matches the unified serial frame buffer size
#define MAVLINK_MAX_RX_PAYLOAD RX_BUFF_SIZE

// Telemetry assembly buffer
#define MAVLINK_TX_BUFFER_SIZE 128

#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_RC_CHANNELS 65
#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE 70
#define MAVLINK_MSG_ID_BATTERY_STATUS 147

// RC_CHANNELS wire layout: time_boot_ms(4), chan1..18(2 each), chancount(1), rssi(1)
#define MAVLINK_RC_CHANNELS_OFFSET 4
#define MAVLINK_RC_CHANNELS_U8_OFFSET 40

// RC_CHANNELS_OVERRIDE wire layout: chan1..8(2 each), target_system(1),
// target_component(1), then chan9..18 as extension fields
#define MAVLINK_RC_OVERRIDE_TARGET_OFFSET 16
#define MAVLINK_RC_OVERRIDE_EXT_OFFSET 18

// Channels we retain, and how many of them carry the primary controls
#define MAVLINK_RC_CHANNEL_COUNT RX_CHANNEL_MAX
#define MAVLINK_RC_PRIMARY 4

// RC_CHANNELS_OVERRIDE can address 18 channels, even though we only retain 16
#define MAVLINK_RC_OVERRIDE_CHANNEL_COUNT 18

// Plausible PWM window. Anything outside is not applied; it is not a stick
// position QuickSilver can act on, so it must not overwrite a retained value.
#define MAVLINK_PWM_MIN 800
#define MAVLINK_PWM_MAX 2200

// Channel sentinels, see the received channel semantics above
#define MAVLINK_CHANNEL_IGNORE 0xFFFF
#define MAVLINK_CHANNEL_RELEASE 0xFFFE
#define MAVLINK_CHANNEL_RELEASE_LOW 0

// MAVLink "unknown" sentinels
#define MAVLINK_UNKNOWN_I8 ((int8_t)-1)
#define MAVLINK_UNKNOWN_I16 ((int16_t)-1)
#define MAVLINK_UNKNOWN_I32 ((int32_t)-1)
#define MAVLINK_UNKNOWN_U16 ((uint16_t)0xFFFF)
#define MAVLINK_UNKNOWN_TEMP ((int16_t)INT16_MAX)

// ---------------------------------------------------------------------------
// Checksum
// ---------------------------------------------------------------------------

static uint16_t mavlink_crc_accumulate(uint8_t data, uint16_t crc) {
  uint8_t tmp = data ^ (uint8_t)(crc & 0xFF);
  tmp ^= (uint8_t)(tmp << 4);
  return (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4));
}

static bool mavlink_crc_extra(uint32_t msgid, uint8_t *extra) {
  switch (msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT:
    *extra = 50;
    return true;
  case MAVLINK_MSG_ID_RC_CHANNELS:
    *extra = 118;
    return true;
  case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    *extra = 124;
    return true;
  case MAVLINK_MSG_ID_BATTERY_STATUS:
    *extra = 154;
    return true;
  default:
    return false;
  }
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------

typedef enum {
  MAVLINK_STATE_MAGIC,
  MAVLINK_STATE_LENGTH,
  MAVLINK_STATE_INCOMPAT,
  MAVLINK_STATE_HEADER,
  MAVLINK_STATE_PAYLOAD,
  MAVLINK_STATE_CRC,
} mavlink_parser_state_t;

typedef struct {
  uint8_t stx;
  uint8_t length;
  uint8_t incompat_flags;
  uint8_t compat_flags;
  uint8_t header[6]; // seq, sys, comp, msgid(1 or 3)
  uint8_t header_length;
  uint8_t header_index;
  uint8_t payload[MAVLINK_MAX_RX_PAYLOAD];
  uint8_t payload_index;
  uint8_t crc[2];
  uint8_t crc_index;
} mavlink_rx_frame_t;

static mavlink_rx_frame_t rx_frame;
static mavlink_parser_state_t rx_state = MAVLINK_STATE_MAGIC;

static uint8_t mavlink_version = 2; // version used when replying, locked to what we receive

// Retained raw channel values and whether that channel has ever been supplied
static uint16_t mavlink_channels[MAVLINK_RC_CHANNEL_COUNT];
static bool mavlink_channels_known[MAVLINK_RC_CHANNEL_COUNT];

static uint32_t mavlink_message_id(const mavlink_rx_frame_t *f) {
  if (f->stx == MAVLINK_STX_V1) {
    return f->header[3];
  }
  return (uint32_t)f->header[3] | ((uint32_t)f->header[4] << 8) | ((uint32_t)f->header[5] << 16);
}

static uint16_t mavlink_read_u16(const uint8_t *data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static bool mavlink_read_channel(const mavlink_rx_frame_t *f, uint32_t offset, uint16_t *value) {
  if (offset + 1 >= f->length) {
    return false;
  }
  *value = mavlink_read_u16(f->payload + offset);
  return true;
}

static bool mavlink_pwm_plausible(uint16_t us) {
  return us >= MAVLINK_PWM_MIN && us <= MAVLINK_PWM_MAX;
}

static void mavlink_clear_channels() {
  for (uint32_t i = 0; i < MAVLINK_RC_CHANNEL_COUNT; i++) {
    mavlink_channels[i] = 0;
    mavlink_channels_known[i] = false;
  }
}

static void mavlink_store_channel(uint32_t channel, uint16_t us) {
  if (channel >= MAVLINK_RC_CHANNEL_COUNT || !mavlink_pwm_plausible(us)) {
    return;
  }
  mavlink_channels[channel] = us;
  mavlink_channels_known[channel] = true;
}

static bool mavlink_primary_valid() {
  for (uint32_t i = 0; i < MAVLINK_RC_PRIMARY; i++) {
    if (!mavlink_channels_known[i]) {
      return false;
    }
  }
  return true;
}

// Stage the retained PWM values as full-range rx_channels. Returns false when
// the four primary controls are incomplete, in which case the retained set must
// not be applied and the frame is not fresh input.
static bool mavlink_apply_channels() {
  if (!mavlink_primary_valid()) {
    return false;
  }

  // Channels are only written when a message supplied a usable value, so a
  // channel that was never provided keeps whatever the previous message left.
  for (uint32_t channel = 0; channel < MAVLINK_RC_CHANNEL_COUNT; channel++) {
    if (!mavlink_channels_known[channel]) {
      continue;
    }
    const int32_t raw = constrain(int32_t(mavlink_channels[channel]), 1000, 2000);
    state.rx_channels[channel] = (uint16_t)(((uint32_t)(raw - 1000) * 65535) / (2000 - 1000));
  }

  return true;
}

// MAVLink defines RC_CHANNELS.rssi as device dependent, 0-254, while
// QuickSilver's link quality is 0-100. 254 maps to 100, and 0 (no signal) or 255
// (unknown) are ignored. Only the direct LQI source consumes a reported RSSI.
static void mavlink_update_rssi(uint8_t rssi) {
  if (rssi == 0 || rssi == 255) {
    return;
  }

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
    rx_lqi_update_direct((float)rssi * (100.0f / 254.0f));
  }
}

typedef enum {
  MAVLINK_CHANNEL_STATE_ABSENT,  // payload trimmed by MAVLink 2, keep the retained value
  MAVLINK_CHANNEL_STATE_IGNORE,  // explicit "do not change", keep the retained value
  MAVLINK_CHANNEL_STATE_RELEASE, // hand control back, we invalidate the control set
  MAVLINK_CHANNEL_STATE_VALUE,
} mavlink_channel_state_t;

static uint32_t mavlink_override_offset(uint32_t channel) {
  if (channel <= 8) {
    return (channel - 1) * 2;
  }
  return MAVLINK_RC_OVERRIDE_EXT_OFFSET + (channel - 9) * 2;
}

static mavlink_channel_state_t mavlink_override_channel(const mavlink_rx_frame_t *f, uint32_t channel, uint16_t *value) {
  uint16_t raw = 0;
  if (!mavlink_read_channel(f, mavlink_override_offset(channel), &raw)) {
    return MAVLINK_CHANNEL_STATE_ABSENT;
  }

  if (channel <= 8) {
    if (raw == MAVLINK_CHANNEL_RELEASE_LOW) {
      return MAVLINK_CHANNEL_STATE_RELEASE;
    }
    if (raw == MAVLINK_CHANNEL_IGNORE) {
      return MAVLINK_CHANNEL_STATE_IGNORE;
    }
  } else {
    if (raw == MAVLINK_CHANNEL_IGNORE || raw == MAVLINK_CHANNEL_RELEASE_LOW) {
      return MAVLINK_CHANNEL_STATE_IGNORE;
    }
    if (raw == MAVLINK_CHANNEL_RELEASE) {
      return MAVLINK_CHANNEL_STATE_RELEASE;
    }
  }

  *value = raw;
  return MAVLINK_CHANNEL_STATE_VALUE;
}

// target_system and target_component are trimmed from short payloads, so a byte
// that is not present counts as its trimmed value: 0, which is broadcast. Each
// byte is validated whenever it exists, so a v2 packet that trims only the
// component still has to address us in the system byte.
static bool mavlink_override_targeted_at_us(const mavlink_rx_frame_t *f) {
  if (f->length > MAVLINK_RC_OVERRIDE_TARGET_OFFSET) {
    const uint8_t target_system = f->payload[MAVLINK_RC_OVERRIDE_TARGET_OFFSET];
    if (target_system != 0 && target_system != MAVLINK_SYSID) {
      return false;
    }
  }

  if (f->length > MAVLINK_RC_OVERRIDE_TARGET_OFFSET + 1) {
    const uint8_t target_component = f->payload[MAVLINK_RC_OVERRIDE_TARGET_OFFSET + 1];
    if (target_component != 0 && target_component != MAVLINK_COMPID) {
      return false;
    }
  }

  return true;
}

static packet_status_t mavlink_process_channels_override(const mavlink_rx_frame_t *f) {
  if (f->length < 2) {
    return PACKET_ERROR;
  }

  // A checksummed override for another system/component is not our input, but it
  // is still valid MAVLink traffic.
  if (!mavlink_override_targeted_at_us(f)) {
    return PACKET_DATA_RECEIVED;
  }

  bool released = false;
  for (uint32_t i = 0; i < MAVLINK_RC_OVERRIDE_CHANNEL_COUNT; i++) {
    uint16_t value = 0;
    switch (mavlink_override_channel(f, i + 1, &value)) {
    case MAVLINK_CHANNEL_STATE_RELEASE:
      released = true;
      break;
    case MAVLINK_CHANNEL_STATE_VALUE:
      mavlink_store_channel(i, value);
      break;
    case MAVLINK_CHANNEL_STATE_ABSENT:
    case MAVLINK_CHANNEL_STATE_IGNORE:
      break;
    }
    if (released) {
      break;
    }
  }

  if (released) {
    // No secondary receiver to fall back to, so a release invalidates the whole
    // control set. The frame is not fresh input, so QuickSilver failsafes.
    mavlink_clear_channels();
    return PACKET_DATA_RECEIVED;
  }

  if (!mavlink_apply_channels()) {
    return PACKET_DATA_RECEIVED;
  }

  rx_lqi_got_packet();
  return PACKET_CHANNELS_RECEIVED;
}

static packet_status_t mavlink_process_channels(const mavlink_rx_frame_t *f) {
  if (f->length < MAVLINK_RC_CHANNELS_OFFSET + 2) {
    return PACKET_ERROR;
  }

  uint32_t available = (uint32_t)(f->length - MAVLINK_RC_CHANNELS_OFFSET) / 2;
  if (available > MAVLINK_RC_CHANNEL_COUNT) {
    available = MAVLINK_RC_CHANNEL_COUNT;
  }

  // chancount reports how many of the trailing channels are meaningful; when it
  // is trimmed, every channel that fits in the payload is present.
  uint32_t used = (f->length > MAVLINK_RC_CHANNELS_U8_OFFSET) ? f->payload[MAVLINK_RC_CHANNELS_U8_OFFSET] : available;
  if (used > available) {
    used = available;
  }

  for (uint32_t i = 0; i < used; i++) {
    uint16_t us = 0;
    if (!mavlink_read_channel(f, MAVLINK_RC_CHANNELS_OFFSET + i * 2, &us)) {
      break;
    }
    // UINT16_MAX marks the channel unused. Both that and an implausible value
    // keep the retained value rather than moving a stick to an extreme.
    if (us == MAVLINK_CHANNEL_IGNORE) {
      continue;
    }
    mavlink_store_channel(i, us);
  }

  if (f->length > MAVLINK_RC_CHANNELS_U8_OFFSET + 1) {
    mavlink_update_rssi(f->payload[MAVLINK_RC_CHANNELS_U8_OFFSET + 1]);
  }

  if (!mavlink_apply_channels()) {
    return PACKET_DATA_RECEIVED;
  }

  rx_lqi_got_packet();
  return PACKET_CHANNELS_RECEIVED;
}

static packet_status_t mavlink_process_message(const mavlink_rx_frame_t *f, uint32_t msgid) {
  switch (msgid) {
  case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    return mavlink_process_channels_override(f);

  case MAVLINK_MSG_ID_RC_CHANNELS:
    return mavlink_process_channels(f);

  default:
    return PACKET_DATA_RECEIVED;
  }
}

packet_status_t rx_serial_process_mavlink() {
  const packet_status_t pending = PACKET_NEEDS_MORE;

mavlink_do_more:
  switch (rx_state) {
  case MAVLINK_STATE_MAGIC: {
    uint8_t magic = 0;
    if (!serial_read_byte(&serial_rx, &magic)) {
      return pending;
    }
    if (magic == MAVLINK_STX_V1) {
      mavlink_version = 1;
    } else if (magic == MAVLINK_STX_V2) {
      mavlink_version = 2;
    } else {
      return PACKET_ERROR;
    }
    rx_frame.stx = magic;
    rx_state = MAVLINK_STATE_LENGTH;
    goto mavlink_do_more;
  }

  case MAVLINK_STATE_LENGTH: {
    if (!serial_read_byte(&serial_rx, &rx_frame.length)) {
      return pending;
    }
    if (rx_frame.length > MAVLINK_MAX_RX_PAYLOAD) {
      rx_state = MAVLINK_STATE_MAGIC;
      return PACKET_ERROR;
    }
    rx_frame.payload_index = 0;
    rx_frame.crc_index = 0;
    rx_frame.header_index = 0;
    if (rx_frame.stx == MAVLINK_STX_V2) {
      rx_state = MAVLINK_STATE_INCOMPAT;
    } else {
      rx_frame.header_length = 4; // seq, sys, comp, msgid
      rx_state = MAVLINK_STATE_HEADER;
    }
    goto mavlink_do_more;
  }

  case MAVLINK_STATE_INCOMPAT: {
    // incompat_flags and compat_flags, both are part of the checksum
    while (rx_frame.header_index < 2) {
      uint8_t flags = 0;
      if (!serial_read_byte(&serial_rx, &flags)) {
        return pending;
      }
      if (rx_frame.header_index == 0) {
        rx_frame.incompat_flags = flags;
        if (flags & 0x01) { // signed frame, not supported
          rx_state = MAVLINK_STATE_MAGIC;
          return PACKET_ERROR;
        }
      } else {
        rx_frame.compat_flags = flags;
      }
      rx_frame.header_index++;
    }
    rx_frame.header_index = 0;
    rx_frame.header_length = 6; // seq, sys, comp, msgid(3)
    rx_state = MAVLINK_STATE_HEADER;
    goto mavlink_do_more;
  }

  case MAVLINK_STATE_HEADER: {
    while (rx_frame.header_index < rx_frame.header_length) {
      if (!serial_read_byte(&serial_rx, &rx_frame.header[rx_frame.header_index])) {
        return pending;
      }
      rx_frame.header_index++;
    }
    rx_state = MAVLINK_STATE_PAYLOAD;
    goto mavlink_do_more;
  }

  case MAVLINK_STATE_PAYLOAD: {
    while (rx_frame.payload_index < rx_frame.length) {
      if (!serial_read_byte(&serial_rx, &rx_frame.payload[rx_frame.payload_index])) {
        return pending;
      }
      rx_frame.payload_index++;
    }
    rx_state = MAVLINK_STATE_CRC;
    goto mavlink_do_more;
  }

  case MAVLINK_STATE_CRC: {
    while (rx_frame.crc_index < 2) {
      if (!serial_read_byte(&serial_rx, &rx_frame.crc[rx_frame.crc_index])) {
        return pending;
      }
      rx_frame.crc_index++;
    }
    rx_state = MAVLINK_STATE_MAGIC;

    const uint32_t msgid = mavlink_message_id(&rx_frame);

    uint8_t extra = 0;
    if (!mavlink_crc_extra(msgid, &extra)) {
      // Unknown message id, cannot validate the checksum
      return PACKET_ERROR;
    }

    uint16_t crc = 0xFFFF;
    crc = mavlink_crc_accumulate(rx_frame.length, crc);
    if (rx_frame.stx == MAVLINK_STX_V2) {
      crc = mavlink_crc_accumulate(rx_frame.incompat_flags, crc);
      crc = mavlink_crc_accumulate(rx_frame.compat_flags, crc);
    }
    for (uint32_t i = 0; i < rx_frame.header_length; i++) {
      crc = mavlink_crc_accumulate(rx_frame.header[i], crc);
    }
    for (uint32_t i = 0; i < rx_frame.length; i++) {
      crc = mavlink_crc_accumulate(rx_frame.payload[i], crc);
    }
    crc = mavlink_crc_accumulate(extra, crc);

    const uint16_t theirs = (uint16_t)rx_frame.crc[0] | ((uint16_t)rx_frame.crc[1] << 8);
    if (crc != theirs) {
      return PACKET_ERROR;
    }

    return mavlink_process_message(&rx_frame, msgid);
  }
  }

  return PACKET_ERROR;
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

// The strict outbound allowlist. Nothing outside this list is transmitted.
static bool mavlink_telemetry_allowed(uint32_t msgid) {
  switch (msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT:
  case MAVLINK_MSG_ID_BATTERY_STATUS:
    return true;
  default:
    return false;
  }
}

static uint8_t tx_buffer[MAVLINK_TX_BUFFER_SIZE];
static uint8_t tx_length = 0;
static uint8_t mavlink_seq = 0;
static uint32_t last_telemetry_ms = 0;

static void payload_u8(uint8_t value) {
  if (tx_length < MAVLINK_TX_BUFFER_SIZE) {
    tx_buffer[tx_length++] = value;
  }
}

static void payload_i8(int8_t value) {
  payload_u8((uint8_t)value);
}

static void payload_u16(uint16_t value) {
  payload_u8((uint8_t)(value & 0xFF));
  payload_u8((uint8_t)((value >> 8) & 0xFF));
}

static void payload_u32(uint32_t value) {
  payload_u16((uint16_t)(value & 0xFFFF));
  payload_u16((uint16_t)((value >> 16) & 0xFFFF));
}

static void payload_i16(int16_t value) {
  payload_u16((uint16_t)value);
}

static void payload_i32(int32_t value) {
  payload_u32((uint32_t)value);
}

// Voltage/current conversions must not overflow their wire type, and a value
// that is not finite is not a measurement.
static float mavlink_sanitize(float value) {
  return isfinite(value) ? value : 0.0f;
}

static uint16_t mavlink_u16_from(float value) {
  value = mavlink_sanitize(value);
  if (!(value > 0.0f)) {
    return 0;
  }
  if (value > 65535.0f) {
    return 65535;
  }
  return (uint16_t)value;
}

static int16_t mavlink_i16_from(float value) {
  value = mavlink_sanitize(value);
  if (value > 32767.0f) {
    return 32767;
  }
  if (value < -32768.0f) {
    return -32768;
  }
  return (int16_t)value;
}

static int32_t mavlink_i32_from(float value) {
  value = mavlink_sanitize(value);
  if (value > 2147483000.0f) {
    return 2147483000;
  }
  if (value < -2147483000.0f) {
    return -2147483000;
  }
  return (int32_t)value;
}

static void mavlink_send_frame(uint32_t msgid, uint8_t extra) {
  if (!mavlink_telemetry_allowed(msgid)) {
    return;
  }

  // MAVLink 2 may trim trailing zero bytes from the payload, the receiver zero
  // fills them again. MAVLink 1 always carries the full payload.
  if (mavlink_version == 2) {
    while (tx_length > 0 && tx_buffer[tx_length - 1] == 0) {
      tx_length--;
    }
  }

  uint8_t frame[MAVLINK_TX_BUFFER_SIZE + 16];
  uint32_t length = 0;

  frame[length++] = (mavlink_version == 1) ? MAVLINK_STX_V1 : MAVLINK_STX_V2;
  frame[length++] = tx_length;

  if (mavlink_version == 2) {
    frame[length++] = 0; // incompat_flags
    frame[length++] = 0; // compat_flags
  }
  frame[length++] = mavlink_seq;
  frame[length++] = MAVLINK_SYSID;
  frame[length++] = MAVLINK_COMPID;
  frame[length++] = (uint8_t)(msgid & 0xFF);
  if (mavlink_version == 2) {
    frame[length++] = (uint8_t)((msgid >> 8) & 0xFF);
    frame[length++] = (uint8_t)((msgid >> 16) & 0xFF);
  }

  uint16_t crc = 0xFFFF;
  for (uint32_t i = 1; i < length; i++) {
    crc = mavlink_crc_accumulate(frame[i], crc);
  }
  for (uint32_t i = 0; i < tx_length; i++) {
    frame[length++] = tx_buffer[i];
    crc = mavlink_crc_accumulate(tx_buffer[i], crc);
  }
  crc = mavlink_crc_accumulate(extra, crc);

  frame[length++] = (uint8_t)(crc & 0xFF);
  frame[length++] = (uint8_t)((crc >> 8) & 0xFF);

  serial_write_bytes(&serial_rx, frame, length);
  mavlink_seq++;
}

static void mavlink_send_heartbeat() {
  tx_length = 0;
  payload_u32(0); // custom_mode
  payload_u8(2);  // type: QUADROTOR
  payload_u8(0);  // autopilot: GENERIC
  payload_u8(flags.arm_state ? 128 : 0); // base_mode: SAFETY_ARMED
  payload_u8(flags.failsafe ? 5 : (flags.arm_state ? 4 : 3)); // system_status: CRITICAL / ACTIVE / STANDBY
  payload_u8(3);  // mavlink_version
  mavlink_send_frame(MAVLINK_MSG_ID_HEARTBEAT, 50);
}

static void mavlink_send_battery_status() {
  const bool voltage_sensed = target.vbat != PIN_NONE;
  const bool current_sensed = target.ibat != PIN_NONE && profile.voltage.ibat_scale != 0;

  uint32_t cells = (uint32_t)state.lipo_cell_count;
  if (cells > 10) {
    cells = 10;
  }
  const uint16_t cell_mv = mavlink_u16_from(state.vbat_cell_avg * 1000.0f);

  tx_length = 0;
  payload_i32(current_sensed ? mavlink_i32_from(state.ibat_drawn) : MAVLINK_UNKNOWN_I32); // current_consumed (mAh)
  payload_i32(MAVLINK_UNKNOWN_I32);                                                      // energy_consumed (hJ)
  payload_i16(MAVLINK_UNKNOWN_TEMP);                                                     // temperature (cdegC)
  for (uint32_t i = 0; i < 10; i++) {
    payload_u16((voltage_sensed && i < cells) ? cell_mv : MAVLINK_UNKNOWN_U16); // voltages (mV)
  }
  // state.ibat_filtered is milliamps (see osd/render.c and msp.c); MAVLink
  // current_battery is in 10 mA units
  payload_i16(current_sensed ? mavlink_i16_from(state.ibat_filtered / 10.0f) : MAVLINK_UNKNOWN_I16); // current_battery (cA)
  payload_u8(0);               // id
  payload_u8(0);               // battery_function: MAV_BATTERY_FUNCTION_UNKNOWN
  payload_u8(0);               // type: MAV_BATTERY_TYPE_UNKNOWN
  payload_i8(MAVLINK_UNKNOWN_I8); // battery_remaining (%)
  mavlink_send_frame(MAVLINK_MSG_ID_BATTERY_STATUS, 154);
}

float rx_serial_mavlink_expected_fps() {
  return 50.0f;
}

void rx_serial_send_mavlink_telemetry() {
  const uint32_t now = time_millis();

  if (now - last_telemetry_ms < 1000) {
    return;
  }
  last_telemetry_ms = now;

  mavlink_send_heartbeat();
  mavlink_send_battery_status();
}

#endif
