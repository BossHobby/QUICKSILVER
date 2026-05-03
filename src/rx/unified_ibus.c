#include "rx/unified_serial.h"

#include <stdbool.h>
#include <stdlib.h>

#include "control/control.h"
#include "core/profile.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "util/util.h"

#ifdef USE_RX_UNIFIED

#define IBUS_PACKET_SIZE 32

typedef enum {
  IBUS_CHECK_MAGIC0,
  IBUS_CHECK_MAGIC1,
  IBUS_PAYLOAD,
  IBUS_CHECK_CRC,
} ibus_parser_state_t;

extern int32_t channels[16];
extern uint8_t rx_data[RX_BUFF_SIZE];

static packet_status_t ibus_handle_packet(uint8_t *packet) {
  channels[0] = packet[2] + (packet[3] << 8);
  channels[1] = packet[4] + (packet[5] << 8);
  channels[2] = packet[6] + (packet[7] << 8);
  channels[3] = packet[8] + (packet[9] << 8);
  channels[4] = packet[10] + (packet[11] << 8);
  channels[5] = packet[12] + (packet[13] << 8);
  channels[6] = packet[14] + (packet[15] << 8);
  channels[7] = packet[16] + (packet[17] << 8);
  channels[8] = packet[18] + (packet[19] << 8);
  channels[9] = packet[20] + (packet[21] << 8);
  channels[10] = packet[22] + (packet[23] << 8);
  channels[11] = packet[24] + (packet[25] << 8);
  channels[12] = packet[26] + (packet[27] << 8);
  channels[13] = packet[28] + (packet[29] << 8);

  const float rc_channels[4] = {
      (channels[0] - 1500.f) * 0.002f,
      (channels[1] - 1500.f) * 0.002f,
      (channels[2] - 1500.f) * 0.002f,
      (channels[3] - 1500.f) * 0.002f,
  };
  rx_map_channels(rc_channels);

  // Here we have the AUX channels Silverware supports
  for (uint32_t i = 0; i <= AUX_CHANNEL_11; i++) {
    const int32_t raw = constrain(channels[4 + i], 1000, 2000);
    state.aux[i] = (uint16_t)(((uint32_t)(raw - 1000) * 65535) / (2000 - 1000));
  }

  rx_lqi_got_packet();

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL && profile.receiver.aux[AUX_RSSI].channel <= AUX_CHANNEL_11) {
    rx_lqi_update_direct(0.1f * (channels[(profile.receiver.aux[AUX_RSSI].channel + 4)] - 1000));
  }
  if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
    rx_lqi_update_direct(0); // no internal rssi data
  }

  return PACKET_CHANNELS_RECEIVED;
}

packet_status_t rx_serial_process_ibus() {
  static ibus_parser_state_t parser_state = IBUS_CHECK_MAGIC0;

  static uint8_t current_offset = 0;
  static uint16_t crc = 0;

ibus_do_more:
  switch (parser_state) {
  case IBUS_CHECK_MAGIC0: {
    uint8_t magic = 0;
    if (!serial_read_bytes(&serial_rx, &magic, 1)) {
      return PACKET_NEEDS_MORE;
    }
    if (magic != 0x20) {
      return PACKET_ERROR;
    }
    crc = 0xFFFF;
    current_offset = 0;
    rx_data[current_offset++] = magic;
    crc = 0xFFFF - magic;
    parser_state = IBUS_CHECK_MAGIC1;
    goto ibus_do_more;
  }
  case IBUS_CHECK_MAGIC1: {
    uint8_t magic = 0;
    if (!serial_read_bytes(&serial_rx, &magic, 1)) {
      return PACKET_NEEDS_MORE;
    }
    if (magic != 0x40) {
      parser_state = IBUS_CHECK_MAGIC0;
      return PACKET_ERROR;
    }
    rx_data[current_offset++] = magic;
    crc -= magic;
    parser_state = IBUS_PAYLOAD;
    goto ibus_do_more;
  }
  case IBUS_PAYLOAD: {
    uint8_t data = 0;
    if (!serial_read_bytes(&serial_rx, &data, 1)) {
      return PACKET_NEEDS_MORE;
    }
    rx_data[current_offset++] = data;
    crc -= data;

    if (current_offset >= 0x20) {
      parser_state = IBUS_CHECK_CRC;
    }
    goto ibus_do_more;
  }

  case IBUS_CHECK_CRC: {
    parser_state = IBUS_CHECK_MAGIC0;
    if (crc != 0) {
      return PACKET_ERROR;
    }
    return ibus_handle_packet(rx_data);
  }
  }

  return PACKET_ERROR;
}

#endif