#include "rx/unified_serial.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "core/debug.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "flight/control.h"
#include "io/msp.h"
#include "rx/crsf.h"
#include "util/crc.h"
#include "util/ring_buffer.h"
#include "util/util.h"

#define MSP_BUFFER_SIZE 128

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
  CRSF_CHECK_CRC,
} crsf_parser_state_t;

extern int32_t channels[16];
extern uint8_t rx_data[RX_BUFF_SIZE];

static uint8_t telemetry_counter = 0;
static uint8_t crsf_rf_mode = 0;

void rx_serial_crsf_msp_send(msp_magic_t magic, uint8_t direction, uint16_t code, const uint8_t *data, uint16_t len);

static uint8_t msp_rx_buffer[MSP_BUFFER_SIZE];
static msp_t msp = {
    .buffer = msp_rx_buffer,
    .buffer_size = MSP_BUFFER_SIZE,
    .buffer_offset = 0,
    .send = rx_serial_crsf_msp_send,
    .device = MSP_DEVICE_RX,
};

static uint8_t msp_tx_buffer[MSP_BUFFER_SIZE];
static uint32_t msp_tx_len = 0;
static uint8_t msp_last_cmd = 0;
static uint8_t msp_origin = 0;
static bool msp_new_data = false;
static bool msp_is_error = false;

#define USART usart_port_defs[serial_rx_port]

float rx_serial_crsf_expected_fps() {
  switch (crsf_rf_mode) {
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

uint16_t rx_serial_crsf_smoothing_cutoff() {
  switch (crsf_rf_mode) {
  case RATE_LORA_4HZ:
    return 1;
  case RATE_LORA_25HZ:
    return 11;
  case RATE_LORA_50HZ:
    return 22;
  case RATE_LORA_100HZ:
  case RATE_LORA_100HZ_8CH:
    return 45;
  case RATE_LORA_150HZ:
    return 67;
  case RATE_LORA_200HZ:
    return 90;
  case RATE_LORA_250HZ:
  case RATE_DVDA_250HZ:
    return 112;
  case RATE_LORA_333HZ_8CH:
    return 150;
  case RATE_LORA_500HZ:
  case RATE_FLRC_500HZ:
  case RATE_DVDA_500HZ:
    return 225;
  case RATE_FLRC_1000HZ:
    return 450;
  }

  return 67;
}

static uint16_t telemetry_interval() {
  switch (crsf_rf_mode) {
  case RATE_LORA_4HZ:
    return 3;
  case RATE_LORA_25HZ:
    return 3;
  case RATE_LORA_50HZ:
    return 3;
  case RATE_LORA_100HZ:
  case RATE_LORA_100HZ_8CH:
    return 6;
  case RATE_LORA_150HZ:
    return 9;
  case RATE_LORA_200HZ:
    return 12;
  case RATE_LORA_250HZ:
  case RATE_DVDA_250HZ:
  case RATE_LORA_333HZ_8CH:
    return 16;
  case RATE_LORA_500HZ:
  case RATE_FLRC_500HZ:
  case RATE_DVDA_500HZ:
    return 32;
  case RATE_FLRC_1000HZ:
    return 64;
  }

  return 5;
}

static bool tlm_device_info_pending = false;

static packet_status_t rx_serial_crsf_process_frame(uint8_t frame_length) {
  bool channels_received = false;

  switch (rx_data[0]) {
  case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: {
    const crsf_channels_t *chan = (crsf_channels_t *)&rx_data[1];
    channels[0] = chan->chan0;
    channels[1] = chan->chan1;
    channels[2] = chan->chan2;
    channels[3] = chan->chan3;
    channels[4] = chan->chan4;
    channels[5] = chan->chan5;
    channels[6] = chan->chan6;
    channels[7] = chan->chan7;
    channels[8] = chan->chan8;
    channels[9] = chan->chan9;
    channels[10] = chan->chan10;
    channels[11] = chan->chan11;
    channels[12] = chan->chan12;
    channels[13] = chan->chan13;
    channels[14] = chan->chan14;
    channels[15] = chan->chan15;

    // AETR channel order
    const float rc_channels[4] = {
        (channels[0] - 990.5f) * 0.00125707103f,
        (channels[1] - 990.5f) * 0.00125707103f,
        (channels[2] - 990.5f) * 0.00125707103f,
        (channels[3] - 990.5f) * 0.00125707103f,
    };

    rx_map_channels(rc_channels);

    state.aux[AUX_CHANNEL_0] = (channels[4] > 1100) ? 1 : 0; // 1100 cutoff intentionally selected to force aux channels low if
    state.aux[AUX_CHANNEL_1] = (channels[5] > 1100) ? 1 : 0; // being controlled by a transmitter using a 3 pos switch in center state
    state.aux[AUX_CHANNEL_2] = (channels[6] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_3] = (channels[7] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_4] = (channels[8] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_5] = (channels[9] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_6] = (channels[10] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_7] = (channels[11] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_8] = (channels[12] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_9] = (channels[13] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_10] = (channels[14] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_11] = (channels[15] > 1100) ? 1 : 0;

    channels_received = true;

    if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL && profile.receiver.aux[AUX_RSSI] <= AUX_CHANNEL_11) {
      rx_lqi_update_direct(0.00062853551f * (channels[(profile.receiver.aux[AUX_RSSI] + 4)] - 191.0f));
    }
    break;
  }

  case CRSF_FRAMETYPE_LINK_STATISTICS: {
    const crsf_stats_t *stats = (crsf_stats_t *)&rx_data[1];

    crsf_rf_mode = stats->rf_mode;

    if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
      rx_lqi_update_direct(stats->uplink_link_quality);
    }
    break;
  }

  case CRSF_FRAMETYPE_MSP_WRITE:
  case CRSF_FRAMETYPE_MSP_REQ: {
    msp_origin = rx_data[2];
    msp_process_telemetry(&msp, rx_data + 3, frame_length - 4);
    break;
  }

  case CRSF_FRAMETYPE_DEVICE_PING: {
    tlm_device_info_pending = true;
    break;
  }

  default:
    quic_debugf("CRSF: unhandled packet type 0x%x", rx_data[0]);
    break;
  }

  rx_lqi_got_packet();

  return channels_received ? PACKET_CHANNELS_RECEIVED : PACKET_DATA_RECEIVED;
}

packet_status_t rx_serial_process_crsf() {
  static crsf_parser_state_t parser_state = CRSF_CHECK_MAGIC;

  static uint8_t frame_length = 0;

crsf_do_more:
  switch (parser_state) {
  case CRSF_CHECK_MAGIC: {
    uint8_t magic = 0;
    if (!serial_read_bytes(&serial_rx, &magic, 1)) {
      return PACKET_NEEDS_MORE;
    }
    if (magic != CRSF_ADDRESS_FLIGHT_CONTROLLER) {
      return PACKET_ERROR;
    }
    parser_state = CRSF_FRAME_LENGTH;
    goto crsf_do_more;
  }

  case CRSF_FRAME_LENGTH: {
    if (!serial_read_bytes(&serial_rx, &frame_length, 1)) {
      return PACKET_NEEDS_MORE;
    }
    if (frame_length <= 0 || frame_length > 64) {
      parser_state = CRSF_CHECK_MAGIC;
      return PACKET_ERROR;
    }
    parser_state = CRSF_PAYLOAD;
    goto crsf_do_more;
  }
  case CRSF_PAYLOAD: {
    if (serial_bytes_available(&serial_rx) < frame_length) {
      return PACKET_NEEDS_MORE;
    }
    if (serial_read_bytes(&serial_rx, rx_data, frame_length) == frame_length) {
      parser_state = CRSF_CHECK_CRC;
    }
    goto crsf_do_more;
  }
  case CRSF_CHECK_CRC: {
    const uint8_t crc_ours = crc8_dvb_s2_data(0, rx_data, frame_length - 1);
    const uint8_t crc_theirs = rx_data[frame_length - 1];
    parser_state = CRSF_CHECK_MAGIC;
    if (crc_ours != crc_theirs) {
      return PACKET_ERROR;
    }
    telemetry_counter++;
    return rx_serial_crsf_process_frame(frame_length);
  }
  }

  return PACKET_ERROR;
}

void rx_serial_crsf_msp_send(msp_magic_t magic, uint8_t direction, uint16_t code, const uint8_t *data, uint16_t len) {
  msp_last_cmd = code;

  if (len > 0 && data) {
    memcpy(msp_tx_buffer, data, len);
  }
  msp_tx_len = len;

  msp_is_error = direction == '!';
  msp_new_data = true;
}

void rx_serial_send_crsf_telemetry() {
  if (telemetry_counter < telemetry_interval() && !msp_new_data) {
    return;
  }
  telemetry_counter = 0;

  uint8_t telemetry_packet[64];
  crsf_tlm_frame_start(telemetry_packet);

  uint32_t payload_size = 0;
  if (msp_new_data) {
    static uint8_t msp_seq = 0;
    static uint16_t msp_tx_sent = 0;

    uint8_t payload[CRSF_MSP_PAYLOAD_SIZE_MAX];

    uint8_t header_size = 0;
    if (msp_tx_sent == 0) { // first chunk
      payload[header_size++] = MSP_STATUS_START_MASK | (msp_seq++ & MSP_STATUS_SEQUENCE_MASK) | (1 << MSP_STATUS_VERSION_SHIFT);
      if (msp_is_error) {
        payload[0] |= MSP_STATUS_ERROR_MASK;
      }

      if (msp_tx_len > 0xFF) {
        payload[header_size++] = 0xFF;
        payload[header_size++] = msp_last_cmd;
        payload[header_size++] = (msp_tx_len >> 0) & 0xFF;
        payload[header_size++] = (msp_tx_len >> 8) & 0xFF;
      } else {
        payload[header_size++] = msp_tx_len;
        payload[header_size++] = msp_last_cmd;
      }

    } else {
      payload[header_size++] = (msp_seq++ & MSP_STATUS_SEQUENCE_MASK) | (1 << MSP_STATUS_VERSION_SHIFT);
    }

    const uint8_t msp_size = min(CRSF_MSP_PAYLOAD_SIZE_MAX - header_size, msp_tx_len - msp_tx_sent);
    memcpy(payload + header_size, msp_tx_buffer + msp_tx_sent, msp_size);
    msp_tx_sent += msp_size;

    if (msp_tx_sent >= msp_tx_len) {
      msp_tx_len = 0;
      msp_tx_sent = 0;
      msp_new_data = false;
    }

    payload_size = crsf_tlm_frame_msp_resp(telemetry_packet, msp_origin, payload, msp_size + header_size);
  } else {
    if (tlm_device_info_pending) {
      payload_size = crsf_tlm_frame_device_info(telemetry_packet);
      tlm_device_info_pending = false;
    } else {
      payload_size = crsf_tlm_frame_battery_sensor(telemetry_packet);
    }
  }

  const uint32_t telemetry_size = crsf_tlm_frame_finish(telemetry_packet, payload_size);
  serial_write_bytes(&serial_rx, telemetry_packet, telemetry_size);
}