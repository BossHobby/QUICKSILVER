#include "rx/unified_serial.h"

#include <stdbool.h>
#include <stdlib.h>

#include "core/debug.h"
#include "core/profile.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "flight/control.h"
#include "util/util.h"

#ifdef USE_RX_UNIFIED

typedef enum {
  REDPINE_CHECK_MAGIC,
  REDPINE_PAYLOAD,
  REDPINE_CHECK_CRC,
} redpine_parser_state_t;

#define REDPINE_CHANNEL_START 3
#define REDPINE_CRC16_POLY 0x8005

extern int32_t channels[16];
extern uint8_t rx_data[RX_BUFF_SIZE];

static uint16_t redpine_crc16(uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    uint8_t val = data[i];

    for (uint8_t i = 0; i < 8; i++) {
      if (((crc & 0x8000) >> 8) ^ (val & 0x80))
        crc = (crc << 1) ^ REDPINE_CRC16_POLY;
      else
        crc = (crc << 1);
      val <<= 1;
    }
  }
  return crc;
}

static packet_status_t redpine_handle_packet(uint8_t *packet) {
  // packet lost flag
  if ((rx_data[0] & 0xc0) != 0x40) {
    rx_lqi_got_packet();

    const uint16_t channels[4] = {
        (uint16_t)((rx_data[REDPINE_CHANNEL_START + 1] << 8) & 0x700) | rx_data[REDPINE_CHANNEL_START],
        (uint16_t)((rx_data[REDPINE_CHANNEL_START + 2] << 4) & 0x7F0) | ((rx_data[REDPINE_CHANNEL_START + 1] >> 4) & 0xF),
        (uint16_t)((rx_data[REDPINE_CHANNEL_START + 4] << 8) & 0x700) | rx_data[REDPINE_CHANNEL_START + 3],
        (uint16_t)((rx_data[REDPINE_CHANNEL_START + 5] << 4) & 0x7F0) | ((rx_data[REDPINE_CHANNEL_START + 4] >> 4) & 0xF),
    };

    // AETR channel order
    const float rc_channels[4] = {
        (channels[0] - 1020.f) * (1.0f / 820.f),
        (channels[1] - 1020.f) * (1.0f / 820.f),
        (channels[2] - 1020.f) * (1.0f / 820.f),
        (channels[3] - 1020.f) * (1.0f / 820.f),
    };

    rx_map_channels(rc_channels);

    // Here we have the AUX channels Silverware supports
    state.aux[AUX_CHANNEL_0] = (rx_data[REDPINE_CHANNEL_START + 1] & 0x08) ? 1 : 0;
    state.aux[AUX_CHANNEL_1] = (rx_data[REDPINE_CHANNEL_START + 2] & 0x80) ? 1 : 0;
    state.aux[AUX_CHANNEL_2] = (rx_data[REDPINE_CHANNEL_START + 4] & 0x08) ? 1 : 0;
    state.aux[AUX_CHANNEL_3] = (rx_data[REDPINE_CHANNEL_START + 5] & 0x80) ? 1 : 0;
    state.aux[AUX_CHANNEL_4] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x01) ? 1 : 0;
    state.aux[AUX_CHANNEL_5] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x02) ? 1 : 0;
    state.aux[AUX_CHANNEL_6] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x04) ? 1 : 0;
    state.aux[AUX_CHANNEL_7] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x08) ? 1 : 0;
    state.aux[AUX_CHANNEL_8] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x10) ? 1 : 0;
    state.aux[AUX_CHANNEL_9] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x20) ? 1 : 0;
    state.aux[AUX_CHANNEL_10] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x40) ? 1 : 0;
    state.aux[AUX_CHANNEL_11] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x80) ? 1 : 0;
  } else {
    rx_lqi_lost_packet();
  }

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
    rx_lqi_update_direct(rx_data[REDPINE_CHANNEL_START + 7]);
  }
  if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL) {
    rx_lqi_update_direct(0); // aux channels are binary and cannot carry rssi
  }

  return PACKET_CHANNELS_RECEIVED;
}

packet_status_t rx_serial_process_redpine() {
  static redpine_parser_state_t parser_state;

redpine_do_more:
  switch (parser_state) {
  case REDPINE_CHECK_MAGIC: {
    uint8_t magic = 0;
    if (!serial_read_bytes(&serial_rx, &magic, 1)) {
      return PACKET_NEEDS_MORE;
    }
    if ((magic & 0x3F) == 0x2A) {
      parser_state = REDPINE_PAYLOAD;
    }
    rx_data[0] = magic;
    goto redpine_do_more;
  }
  case REDPINE_PAYLOAD: {
    if (serial_bytes_available(&serial_rx) < 11) {
      return PACKET_NEEDS_MORE;
    }
    if (serial_read_bytes(&serial_rx, rx_data + 1, 11) == 11) {
      parser_state = REDPINE_CHECK_CRC;
    }
    goto redpine_do_more;
  }
  case REDPINE_CHECK_CRC: {
    parser_state = REDPINE_CHECK_MAGIC;

    const uint16_t crc_our = redpine_crc16(rx_data + REDPINE_CHANNEL_START, 11 - REDPINE_CHANNEL_START);
    const uint16_t crc_theirs = (uint16_t)(rx_data[1] << 8) | rx_data[2];
    if (crc_our != crc_theirs) {
      return PACKET_ERROR;
    }
    return redpine_handle_packet(rx_data);
  }
  }
  return PACKET_ERROR;
}

#endif