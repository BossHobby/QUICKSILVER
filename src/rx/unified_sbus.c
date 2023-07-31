#include "rx/unified_serial.h"

#include <stdbool.h>
#include <stdlib.h>

#include "core/profile.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "flight/control.h"

#ifdef USE_RX_UNIFIED

#define SBUS_PACKET_SIZE 25

typedef enum {
  FPORT_CHECK_MAGIC,
  FPORT_FRAME_LENGTH,
  FPORT_PAYLOAD,
  FPORT_CHECK_CRC,
  FPORT_CHECK_END,
} fport_parser_state_t;

typedef enum {
  SBUS_CHECK_MAGIC,
  SBUS_PAYLOAD,
  SBUS_CHECK_END,
} sbus_parser_state_t;

extern int32_t channels[16];
extern uint8_t failsafe_sbus_failsafe;

extern uint8_t rx_data[RX_BUFF_SIZE];

static bool fport_telemetry_allowed = false;

static void decode_sbus_channels(uint8_t *data) {
  channels[0] = ((data[0] | data[1] << 8) & 0x07FF);
  channels[1] = ((data[1] >> 3 | data[2] << 5) & 0x07FF);
  channels[2] = ((data[2] >> 6 | data[3] << 2 | data[4] << 10) & 0x07FF);
  channels[3] = ((data[4] >> 1 | data[5] << 7) & 0x07FF);
  channels[4] = ((data[5] >> 4 | data[6] << 4) & 0x07FF);
  channels[5] = ((data[6] >> 7 | data[7] << 1 | data[8] << 9) & 0x07FF);
  channels[6] = ((data[8] >> 2 | data[9] << 6) & 0x07FF);
  channels[7] = ((data[9] >> 5 | data[10] << 3) & 0x07FF);
  channels[8] = ((data[11] | data[12] << 8) & 0x07FF);
  channels[9] = ((data[12] >> 3 | data[13] << 5) & 0x07FF); // This is the last channel Silverware previously supported.
  channels[10] = ((data[13] >> 6 | data[14] << 2 | data[15] << 10) & 0x07FF);
  channels[11] = ((data[15] >> 1 | data[16] << 7) & 0x07FF);
  channels[12] = ((data[16] >> 4 | data[17] << 4) & 0x07FF);
  channels[13] = ((data[17] >> 7 | data[18] << 1 | data[19] << 9) & 0x07FF);
  channels[14] = ((data[19] >> 2 | data[20] << 6) & 0x07FF);
  channels[15] = ((data[20] >> 5 | data[21] << 3) & 0x07FF);

  // AETR channel order
  const float rc_channels[4] = {
      (channels[0] - 993.f) * 0.00122026f,
      (channels[1] - 993.f) * 0.00122026f,
      (channels[2] - 993.f) * 0.00122026f,
      (channels[3] - 993.f) * 0.00122026f,
  };

  rx_map_channels(rc_channels);

  // Here we have the AUX channels Silverware supports
  state.aux[AUX_CHANNEL_0] = (channels[4] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_1] = (channels[5] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_2] = (channels[6] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_3] = (channels[7] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_4] = (channels[8] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_5] = (channels[9] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_6] = (channels[10] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_7] = (channels[11] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_8] = (channels[12] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_9] = (channels[13] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_10] = (channels[14] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_11] = (channels[15] > 1600) ? 1 : 0;

  if (data[22] & (1 << 2)) {
    // RX sets this bit when it knows it missed a frame. Presumably this is a timer in the RX.
    rx_lqi_lost_packet();
  } else {
    rx_lqi_got_packet();
  }
  if (data[22] & (1 << 3)) {
    failsafe_sbus_failsafe = 1;              // Sbus packets have a failsafe bit. This is cool. If you forget to trust it you get programs though.
    flags.failsafe = failsafe_sbus_failsafe; // set failsafe rtf-now
  } else {
    failsafe_sbus_failsafe = 0;
  }

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL && profile.receiver.aux[AUX_RSSI] <= AUX_CHANNEL_11) {
    rx_lqi_update_direct(0.0610128f * (channels[(profile.receiver.aux[AUX_RSSI] + 4)] - 173));
  }

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
    rx_lqi_update_direct(data[23]); // no internal rssi data
  }
}

packet_status_t rx_serial_process_sbus() {
  static sbus_parser_state_t parser_state = SBUS_CHECK_MAGIC;

sbus_do_more:
  switch (parser_state) {
  case SBUS_CHECK_MAGIC: {
    if (!serial_read_bytes(&serial_rx, &rx_data[0], 1)) {
      return PACKET_NEEDS_MORE;
    }
    if (rx_data[0] == 0x0F) {
      parser_state = SBUS_PAYLOAD;
    }
    goto sbus_do_more;
  }
  case SBUS_PAYLOAD: {
    if (serial_bytes_available(&serial_rx) < SBUS_PACKET_SIZE - 1) {
      return PACKET_NEEDS_MORE;
    }
    if (serial_read_bytes(&serial_rx, rx_data + 1, SBUS_PACKET_SIZE - 1) != SBUS_PACKET_SIZE - 1) {
      return PACKET_ERROR;
    }
    parser_state = SBUS_CHECK_END;
    goto sbus_do_more;
  }
  case SBUS_CHECK_END: {
    parser_state = SBUS_CHECK_MAGIC;
    if (rx_data[24] != 0x0) {
      return PACKET_ERROR;
    }
    decode_sbus_channels(rx_data + 1);
    return PACKET_CHANNELS_RECEIVED;
  }
  };

  return PACKET_ERROR;
}

static packet_status_t fport_handle_packet(uint8_t *packet) {
  switch (packet[2]) {
  case 0x00:
    decode_sbus_channels(packet + 3);
    return PACKET_CHANNELS_RECEIVED;

  case 0x1:
    fport_telemetry_allowed = true;
    rx_lqi_got_packet();
    return PACKET_DATA_RECEIVED;

  default:
    return PACKET_DATA_RECEIVED;
  }
}

packet_status_t rx_serial_process_fport() {
  static fport_parser_state_t parser_state = FPORT_CHECK_MAGIC;

  static uint8_t current_offset = 0;
  static uint8_t frame_length = 0;
  static uint16_t our_crc = 0;

fport_do_more:
  switch (parser_state) {
  case FPORT_CHECK_MAGIC: {
    if (!serial_read_bytes(&serial_rx, &rx_data[0], 1)) {
      return PACKET_NEEDS_MORE;
    }
    if (rx_data[0] == 0x7E) {
      parser_state = FPORT_FRAME_LENGTH;
      current_offset = 1;
    }
    goto fport_do_more;
  }
  case FPORT_FRAME_LENGTH: {
    if (!serial_read_bytes(&serial_rx, &frame_length, 1)) {
      return PACKET_NEEDS_MORE;
    }
    if (frame_length == 0x7E) {
      rx_data[0] = frame_length;
      goto fport_do_more;
    }
    parser_state = FPORT_PAYLOAD;
    rx_data[current_offset++] = frame_length;
    our_crc = frame_length;
    goto fport_do_more;
  }
  case FPORT_PAYLOAD: {
    uint8_t data = 0;
    if (!serial_read_bytes(&serial_rx, &data, 1)) {
      return PACKET_NEEDS_MORE;
    }

    static bool is_escape = false;
    if (is_escape) {
      // we are in the escape sequence
      // 0x5E + 0x20 == 0x7E
      // 0x5D + 0x20 == 0x7D
      data += 0x20;
      is_escape = false;
    } else if (data == 0x7D) {
      is_escape = true;
      goto fport_do_more;
    }
    rx_data[current_offset++] = data;
    our_crc += data;

    if (current_offset >= frame_length + 2) {
      parser_state = FPORT_CHECK_CRC;
    }
    goto fport_do_more;
  }

  case FPORT_CHECK_CRC: {
    uint8_t crc = 0;
    if (!serial_read_bytes(&serial_rx, &crc, 1)) {
      return PACKET_NEEDS_MORE;
    }

    while (our_crc > 0xFF) {
      our_crc = (our_crc & 0xFF) + (our_crc >> 8);
    }
    our_crc = 0xFF - our_crc;

    if (our_crc == crc) {
      parser_state = FPORT_CHECK_END;
      goto fport_do_more;
    }
    parser_state = FPORT_CHECK_MAGIC;
    return PACKET_ERROR;
  }

  case FPORT_CHECK_END: {
    uint8_t magic = 0;
    if (!serial_read_bytes(&serial_rx, &magic, 1)) {
      return PACKET_NEEDS_MORE;
    }
    parser_state = FPORT_CHECK_MAGIC;
    if (magic != 0x7E) {
      return PACKET_ERROR;
    }
    return fport_handle_packet(rx_data);
  }
  }

  return PACKET_ERROR;
}

void rx_serial_send_fport_telemetry() {
  if (!fport_telemetry_allowed) {
    return;
  }
  fport_telemetry_allowed = false;

  static uint8_t current_id = 0;
  static const uint16_t telemetry_ids[] = {
      0x0210, // VFAS, use for vbat_compensated
      0x0211, // VFAS1, use for vbat_filtered
  };

  uint8_t packet[16];
  packet[0] = 0x08;
  packet[1] = 0x81;
  packet[2] = 0x10;
  if (current_id == 0) {                        // vbat_compensated
    packet[3] = telemetry_ids[current_id];      // 0x10;
    packet[4] = telemetry_ids[current_id] >> 8; // 0x02;
    packet[5] = (int)(state.vbat_compensated * 100);
    packet[6] = (int)(state.vbat_compensated * 100) >> 8;
    packet[7] = 0x00;
    packet[8] = 0x00;
  } else if (current_id == 1) {                 // vbat_filtered
    packet[3] = telemetry_ids[current_id];      // x11;
    packet[4] = telemetry_ids[current_id] >> 8; // 0x02;
    packet[5] = (int)(state.vbat_filtered * 100);
    packet[6] = (int)(state.vbat_filtered * 100) >> 8;
    packet[7] = 0x00;
    packet[8] = 0x00;
  }

  uint16_t crc = 0;
  for (uint32_t i = 0; i < 9; i++) {
    crc += packet[i];
  }
  packet[9] = crc >> 8;

  serial_write_bytes(&serial_rx, packet, 10);
}

#endif