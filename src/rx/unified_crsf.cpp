#include "rx/unified_serial.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "control/control.h"
#include "core/debug.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "io/gps.h"
#include "io/msp.h"
#include "rx/crsf.h"
#include "util/crc.h"
#include "util/ring_buffer.h"
#include "util/util.h"

#ifdef USE_RX_UNIFIED

#define MSP_BUFFER_SIZE 128
#define CRSF_CHANNEL_VALUE_MIN 172
#define CRSF_CHANNEL_VALUE_MAX 1811
#define CRSF_CHANNEL_LQI_SCALE (100.0f / AUX_VALUE_MAX)
#define CRSF_BAUDRATE_MAX 2000000
#define CRSF_SUBSET_START_CHANNEL_MASK 0x1F
#define CRSF_SUBSET_RESOLUTION_CONFIG_MASK 0x03
#define CRSF_SUBSET_RESOLUTION_CONFIG_SHIFT 5
#define CRSF_SUBSET_DIGITAL_SWITCH_FLAG 0x80
#define CRSF_SUBSET_DIGITAL_SWITCH_BITS 10
#define CRSF_TELEMETRY_INTERVAL_US 2000
#define CRSF_BAUDRATE_CHANGE_DELAY_US 4000
#define CRSF_BAUDRATE_ERROR_FALLBACK_COUNT 200

typedef enum {
  RATE_LORA_4HZ = 0,
  RATE_LORA_25HZ,
  RATE_LORA_50HZ,
  RATE_LORA_100HZ,
  RATE_LORA_100HZ_8CH,
  RATE_LORA_150HZ,
  RATE_LORA_200HZ,
  RATE_LORA_250HZ,
  RATE_LORA_333HZ_8CH,
  RATE_LORA_500HZ,
  RATE_DVDA_250HZ,
  RATE_DVDA_500HZ,
  RATE_FLRC_500HZ,
  RATE_FLRC_1000HZ,
} crsf_air_rates_t;

typedef enum {
  CRSF_CHECK_MAGIC,
  CRSF_FRAME_LENGTH,
  CRSF_PAYLOAD,
} crsf_parser_state_t;

typedef struct {
  uint32_t pending_baudrate;
  uint32_t response_time_us;
  uint32_t serial_error_count;
  uint16_t error_count;
} crsf_baudrate_state_t;

extern uint8_t rx_data[RX_BUFF_SIZE];

void rx_serial_crsf_msp_send(msp_magic_t magic, uint8_t direction, uint16_t code, const uint8_t *data, uint16_t len);

static uint8_t msp_rx_buffer[MSP_BUFFER_SIZE];
msp_t crsf_msp = {
    .buffer = msp_rx_buffer,
    .buffer_size = MSP_BUFFER_SIZE,
    .buffer_offset = 0,
    .send = rx_serial_crsf_msp_send,
    .device = MSP_DEVICE_RX,
};

static uint8_t msp_origin = 0;
static crsf_baudrate_state_t baudrate_state = {0};
static uint8_t crsf_telemetry_packet[CRSF_FRAME_SIZE_MAX];
static uint8_t crsf_msp_payload[CRSF_MSP_PAYLOAD_SIZE_MAX];

float rx_serial_crsf_expected_fps() {
  if (crsf_stats.uplink_fps != 0)
    return crsf_stats.uplink_fps;

  switch (crsf_stats.rf_mode) {
  case RATE_FLRC_1000HZ:
    return 1000;
  case RATE_FLRC_500HZ:
    return 500;
  case RATE_DVDA_500HZ:
    return 500;
  case RATE_DVDA_250HZ:
    return 250;
  case RATE_LORA_500HZ:
    return 500;
  case RATE_LORA_333HZ_8CH:
    return 333;
  case RATE_LORA_250HZ:
    return 250;
  case RATE_LORA_200HZ:
    return 200;
  case RATE_LORA_150HZ:
    return 150;
  case RATE_LORA_100HZ:
    return 100;
  case RATE_LORA_100HZ_8CH:
    return 100;
  case RATE_LORA_50HZ:
    return 50;
  case RATE_LORA_25HZ:
    return 25;
  case RATE_LORA_4HZ:
    return 4;
  }
  return 1;
}

static uint8_t rx_serial_crsf_payload_length(uint8_t frame_length) {
  if (frame_length < CRSF_FRAME_LENGTH_TYPE_CRC)
    return 0;

  return frame_length - CRSF_FRAME_LENGTH_TYPE_CRC;
}

static bool rx_serial_crsf_queue_frame(const uint8_t *frame, uint32_t size) {
  if (serial_bytes_free(&serial_rx) < size)
    return false;

  return serial_write_bytes(&serial_rx, frame, size);
}

static bool rx_serial_crsf_apply_pending_baudrate() {
  const uint32_t baudrate = baudrate_state.pending_baudrate;
  if (baudrate == 0)
    return true;

  if (!serial_rx.tx_done || ring_buffer_available(serial_rx.tx_buffer) != 0)
    return false;

  if ((time_micros() - baudrate_state.response_time_us) < CRSF_BAUDRATE_CHANGE_DELAY_US)
    return false;

  baudrate_state.pending_baudrate = 0;
  baudrate_state.response_time_us = 0;

  if (serial_set_baudrate(&serial_rx, baudrate)) {
    baudrate_state.error_count = 0;
    baudrate_state.serial_error_count = serial_rx.rx_error_count;
  } else
    quic_debugf("CRSF: baud change failed %u", (unsigned)baudrate);

  return true;
}

static void rx_serial_crsf_update_channel(uint8_t channel, uint32_t raw, uint32_t raw_min, uint32_t raw_max) {
  if (channel >= RX_CHANNEL_MAX || raw_max <= raw_min)
    return;

  const uint32_t value = constrain(raw, raw_min, raw_max) - raw_min;
  state.rx_channels[channel] = (uint16_t)(value * AUX_VALUE_MAX / (raw_max - raw_min));
}

static void rx_serial_crsf_process_subset_channels(const uint8_t *payload, uint8_t payload_length) {
  const crsf_channels_subset_payload_t *frame = (const crsf_channels_subset_payload_t *)payload;
  const uint8_t channel_bits = 10 + ((frame->config >> CRSF_SUBSET_RESOLUTION_CONFIG_SHIFT) & CRSF_SUBSET_RESOLUTION_CONFIG_MASK);
  const uint8_t channel_count = ((payload_length - 1) * 8 - ((frame->config & CRSF_SUBSET_DIGITAL_SWITCH_FLAG) ? CRSF_SUBSET_DIGITAL_SWITCH_BITS : 0)) / channel_bits;
  const uint16_t channel_raw_max = (1U << channel_bits) - 1;
  uint16_t bit_offset = 0;

  for (uint8_t offset = 0; offset < channel_count; offset++, bit_offset += channel_bits) {
    uint32_t raw = 0;

    for (uint8_t bit = 0; bit < channel_bits; bit++) {
      const uint16_t data_bit = bit_offset + bit;
      if (frame->data[data_bit / 8] & (1U << (data_bit % 8)))
        raw |= 1UL << bit;
    }

    rx_serial_crsf_update_channel((frame->config & CRSF_SUBSET_START_CHANNEL_MASK) + offset, raw, 0, channel_raw_max);
  }
}

static void rx_serial_crsf_process_command(uint8_t payload_length) {
  if (payload_length != sizeof(crsf_speed_proposal_payload_t))
    return;

  const crsf_speed_proposal_payload_t *proposal = (const crsf_speed_proposal_payload_t *)&rx_data[1];
  if (crsf_command_crc8(rx_data, sizeof(*proposal)) != proposal->command_crc ||
      (proposal->header.destination != CRSF_ADDRESS_FLIGHT_CONTROLLER && proposal->header.destination != CRSF_ADDRESS_BROADCAST) ||
      proposal->header.subcommand != CRSF_COMMAND_SUBCMD_GENERAL ||
      proposal->header.command != CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL)
    return;

  const uint32_t baudrate = __builtin_bswap32(proposal->baudrate);
  const bool baudrate_supported = baudrate != 0 && baudrate <= CRSF_BAUDRATE_MAX;
  const uint32_t response_size = crsf_frame_speed_response(
      crsf_telemetry_packet,
      proposal->header.origin,
      proposal->port_id,
      baudrate_supported);
  if (!rx_serial_crsf_queue_frame(crsf_telemetry_packet, response_size))
    return;

  if (baudrate_supported && baudrate != serial_rx.config.baudrate) {
    baudrate_state.pending_baudrate = baudrate;
    baudrate_state.response_time_us = time_micros();
  }
}

static bool rx_serial_crsf_update_baudrate(packet_status_t status) {
  const uint32_t rx_errors = serial_rx.rx_error_count;

  if (serial_rx.config.baudrate == CRSF_BAUDRATE_DEFAULT || baudrate_state.pending_baudrate != 0 || status > PACKET_NEEDS_MORE) {
    baudrate_state.error_count = 0;
    baudrate_state.serial_error_count = rx_errors;
    return true;
  }

  if (status == PACKET_NEEDS_MORE && baudrate_state.serial_error_count == rx_errors)
    return true;

  baudrate_state.serial_error_count = rx_errors;

  if (baudrate_state.error_count < CRSF_BAUDRATE_ERROR_FALLBACK_COUNT)
    baudrate_state.error_count++;

  if (baudrate_state.error_count < CRSF_BAUDRATE_ERROR_FALLBACK_COUNT ||
      !serial_rx.tx_done ||
      ring_buffer_available(serial_rx.tx_buffer) != 0)
    return false;

  baudrate_state.error_count = 0;
  if (!serial_set_baudrate(&serial_rx, CRSF_BAUDRATE_DEFAULT)) {
    quic_debugf("CRSF: baud fallback failed %u", (unsigned)CRSF_BAUDRATE_DEFAULT);
    return false;
  }

  baudrate_state.serial_error_count = serial_rx.rx_error_count;
  ring_buffer_clear(serial_rx.rx_buffer);
  return false;
}

static packet_status_t rx_serial_crsf_process_frame(uint8_t frame_length) {
  bool channels_received = false;
  const uint8_t payload_length = rx_serial_crsf_payload_length(frame_length);

  switch (rx_data[0]) {
  case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: {
    if (payload_length < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE)
      break;

    const crsf_channels_t *chan = (crsf_channels_t *)&rx_data[1];
    const uint16_t raw_channels[RX_CHANNEL_MAX] = {
        uint16_t(chan->chan0),
        uint16_t(chan->chan1),
        uint16_t(chan->chan2),
        uint16_t(chan->chan3),
        uint16_t(chan->chan4),
        uint16_t(chan->chan5),
        uint16_t(chan->chan6),
        uint16_t(chan->chan7),
        uint16_t(chan->chan8),
        uint16_t(chan->chan9),
        uint16_t(chan->chan10),
        uint16_t(chan->chan11),
        uint16_t(chan->chan12),
        uint16_t(chan->chan13),
        uint16_t(chan->chan14),
        uint16_t(chan->chan15),
    };

    for (uint8_t channel = 0; channel < RX_CHANNEL_MAX; channel++) {
      rx_serial_crsf_update_channel(channel, raw_channels[channel], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    }

    channels_received = true;
    break;
  }

  case CRSF_FRAMETYPE_RC_CHANNELS_SUBSET_PACKED: {
    if (payload_length < CRSF_FRAME_RC_CHANNELS_SUBSET_MIN_PAYLOAD_SIZE)
      break;

    rx_serial_crsf_process_subset_channels(&rx_data[1], payload_length);
    channels_received = true;
    break;
  }

  case CRSF_FRAMETYPE_LINK_STATISTICS: {
    if (payload_length < CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE)
      break;

    const crsf_link_statistics_payload_t *stats = (const crsf_link_statistics_payload_t *)&rx_data[1];
    crsf_stats.uplink_rssi_2 = stats->uplink_rssi_2;
    crsf_stats.uplink_rssi_1 = stats->uplink_rssi_1;
    crsf_stats.uplink_link_quality = stats->uplink_link_quality;
    crsf_stats.uplink_snr = stats->uplink_snr;
    crsf_stats.active_antenna = stats->active_antenna;
    crsf_stats.rf_mode = stats->rf_mode;
    crsf_stats.uplink_tx_power = stats->uplink_tx_power;
    crsf_stats.downlink_rssi = stats->downlink_rssi;
    crsf_stats.downlink_link_quality = stats->downlink_link_quality;
    crsf_stats.downlink_snr = stats->downlink_snr;

    if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT)
      rx_lqi_update_direct(crsf_stats.uplink_link_quality);

    break;
  }

  case CRSF_FRAMETYPE_LINK_STATISTICS_RX: {
    if (payload_length < CRSF_FRAME_LINK_STATISTICS_RX_PAYLOAD_SIZE)
      break;

    const crsf_link_stats_rx_t *stats = (const crsf_link_stats_rx_t *)&rx_data[1];
    crsf_stats.downlink_rssi = stats->rssi_db;
    crsf_stats.downlink_link_quality = stats->link_quality;
    crsf_stats.downlink_snr = stats->snr;

    if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT)
      rx_lqi_update_direct(stats->link_quality);

    break;
  }

  case CRSF_FRAMETYPE_LINK_STATISTICS_TX: {
    if (payload_length < CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE)
      break;

    const crsf_link_stats_tx_t *stats = (const crsf_link_stats_tx_t *)&rx_data[1];
    crsf_stats.uplink_rssi_1 = stats->rssi_db;
    crsf_stats.uplink_rssi_2 = stats->rssi_db;
    crsf_stats.uplink_link_quality = stats->link_quality;
    crsf_stats.uplink_snr = stats->snr;
    crsf_stats.uplink_fps = (uint16_t)stats->fps * 10;

    if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT)
      rx_lqi_update_direct(stats->link_quality);

    break;
  }

  case CRSF_FRAMETYPE_MSP_WRITE:
  case CRSF_FRAMETYPE_MSP_REQ: {
    msp_origin = rx_data[2];
    crsf_msp.device = profile.serial.smart_audio == profile.serial.rx ? MSP_DEVICE_VTX : MSP_DEVICE_RX;
    msp_process_telemetry(&crsf_msp, rx_data + 3, frame_length - 4);
    break;
  }

  case CRSF_FRAMETYPE_COMMAND: {
    rx_serial_crsf_process_command(payload_length);
    break;
  }

  case CRSF_FRAMETYPE_DEVICE_PING: {
    const crsf_extended_header_t *extended = (const crsf_extended_header_t *)&rx_data[1];
    const uint8_t destination = payload_length >= CRSF_FRAME_ORIGIN_DEST_SIZE ? extended->origin : CRSF_ADDRESS_RADIO_TRANSMITTER;
    const uint32_t telemetry_size = crsf_tlm_frame_device_info(crsf_telemetry_packet, destination);
    return rx_serial_crsf_queue_frame(crsf_telemetry_packet, telemetry_size) ? PACKET_DATA_RECEIVED : PACKET_ERROR;
  }

  default:
    quic_debugf("CRSF: unhandled packet type 0x%x", rx_data[0]);
    break;
  }

  if (channels_received && profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL && profile.receiver.aux[AUX_RSSI].channel < RX_CHANNEL_MAX)
    rx_lqi_update_direct((float)state.rx_channels[profile.receiver.aux[AUX_RSSI].channel] * CRSF_CHANNEL_LQI_SCALE);

  return channels_received ? PACKET_CHANNELS_RECEIVED : PACKET_DATA_RECEIVED;
}

static packet_status_t rx_serial_process_crsf_frame_stream() {
  static crsf_parser_state_t parser_state = CRSF_CHECK_MAGIC;

  static uint8_t frame_length = 0;

crsf_do_more:
  switch (parser_state) {
  case CRSF_CHECK_MAGIC: {
    uint8_t magic = 0;
    if (!serial_read_bytes(&serial_rx, &magic, 1))
      return PACKET_NEEDS_MORE;

    if (magic != CRSF_ADDRESS_FLIGHT_CONTROLLER)
      return PACKET_ERROR;

    parser_state = CRSF_FRAME_LENGTH;
    goto crsf_do_more;
  }

  case CRSF_FRAME_LENGTH: {
    if (!serial_read_bytes(&serial_rx, &frame_length, 1))
      return PACKET_NEEDS_MORE;

    if (frame_length < 2 || frame_length > 62) {
      parser_state = CRSF_CHECK_MAGIC;
      return PACKET_ERROR;
    }
    parser_state = CRSF_PAYLOAD;
    goto crsf_do_more;
  }
  case CRSF_PAYLOAD: {
    if (serial_bytes_available(&serial_rx) < frame_length)
      return PACKET_NEEDS_MORE;

    if (serial_read_bytes(&serial_rx, rx_data, frame_length) != frame_length) {
      parser_state = CRSF_CHECK_MAGIC;
      return PACKET_ERROR;
    }

    const uint8_t crc_ours = crc8_dvb_s2_data(0, rx_data, frame_length - 1);
    const uint8_t crc_theirs = rx_data[frame_length - 1];
    parser_state = CRSF_CHECK_MAGIC;
    if (crc_ours != crc_theirs)
      return PACKET_ERROR;

    return rx_serial_crsf_process_frame(frame_length);
  }
  }

  return PACKET_ERROR;
}

void rx_serial_crsf_msp_send(msp_magic_t magic, uint8_t direction, uint16_t code, const uint8_t *data, uint16_t len) {
  if (len > MSP_BUFFER_SIZE || (len > 0 && data == NULL))
    return;

  static uint8_t msp_seq = 0;
  uint16_t msp_tx_sent = 0;

  do {
    uint8_t *payload = crsf_msp_payload;
    uint8_t header_size = 0;

    if (msp_tx_sent == 0) { // first chunk
      payload[header_size++] = MSP_STATUS_START_MASK | (msp_seq++ & MSP_STATUS_SEQUENCE_MASK);
      if (direction == '!')
        payload[0] |= MSP_STATUS_ERROR_MASK;

      if (magic == MSP1_MAGIC) {
        payload[0] |= (1 << MSP_STATUS_VERSION_SHIFT);
        if (len > 0xFF) {
          payload[header_size++] = 0xFF;
          payload[header_size++] = code;
          payload[header_size++] = (len >> 0) & 0xFF;
          payload[header_size++] = (len >> 8) & 0xFF;
        } else {
          payload[header_size++] = len;
          payload[header_size++] = code;
        }
      } else {
        payload[0] |= (2 << MSP_STATUS_VERSION_SHIFT);
        payload[header_size++] = 0; // flag
        payload[header_size++] = (code >> 0) & 0xFF;
        payload[header_size++] = (code >> 8) & 0xFF;
        payload[header_size++] = (len >> 0) & 0xFF;
        payload[header_size++] = (len >> 8) & 0xFF;
      }
    } else {
      payload[header_size++] = (msp_seq++ & MSP_STATUS_SEQUENCE_MASK) | (1 << MSP_STATUS_VERSION_SHIFT);
    }

    const uint8_t msp_size = MIN(CRSF_MSP_PAYLOAD_SIZE_MAX - header_size, len - msp_tx_sent);
    if (msp_size > 0)
      memcpy(payload + header_size, data + msp_tx_sent, msp_size);
    msp_tx_sent += msp_size;

    const uint32_t telemetry_size = crsf_tlm_frame_msp_resp(crsf_telemetry_packet, msp_origin, payload, msp_size + header_size);
    if (!rx_serial_crsf_queue_frame(crsf_telemetry_packet, telemetry_size))
      return;
  } while (msp_tx_sent < len);
}

static void rx_serial_send_crsf_telemetry() {
  static uint32_t telemetry_time = 0;
  const uint32_t now = time_micros();
  if ((now - telemetry_time) < CRSF_TELEMETRY_INTERVAL_US)
    return;

  uint32_t telemetry_size = 0;
  static uint8_t telemetry_counter = 0;
  const bool gps_enabled = profile.serial.gps != SERIAL_PORT_INVALID;

  switch (telemetry_counter++ % 8) {
  case 0:
    if (gps_enabled)
      telemetry_size = crsf_tlm_frame_gps(crsf_telemetry_packet);
    else
      telemetry_size = crsf_tlm_frame_battery_sensor(crsf_telemetry_packet);
    break;
  case 2:
    if (gps_enabled)
      telemetry_size = crsf_tlm_frame_gps_extended(crsf_telemetry_packet);
    else
      telemetry_size = crsf_tlm_frame_battery_sensor(crsf_telemetry_packet);
    break;
  case 4:
    telemetry_size = crsf_tlm_frame_flight_mode(crsf_telemetry_packet);
    break;
  default:
    telemetry_size = crsf_tlm_frame_battery_sensor(crsf_telemetry_packet);
    break;
  }

  if (rx_serial_crsf_queue_frame(crsf_telemetry_packet, telemetry_size))
    telemetry_time = now;
}

packet_status_t rx_serial_process_crsf() {
  const packet_status_t status = rx_serial_process_crsf_frame_stream();

  if (serial_rx_detected_protcol != RX_SERIAL_PROTOCOL_CRSF)
    return status;

  if (!rx_serial_crsf_update_baudrate(status))
    return status;

  if (rx_serial_crsf_apply_pending_baudrate())
    rx_serial_send_crsf_telemetry();

  return status;
}

bool rx_serial_crsf_bind() {
  if (serial_rx.config.port == SERIAL_PORT_INVALID)
    return false;

  const uint32_t bind_size = crsf_frame_bind(crsf_telemetry_packet);
  return rx_serial_crsf_queue_frame(crsf_telemetry_packet, bind_size);
}

#endif
