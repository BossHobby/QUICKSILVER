#include "rx_unified_serial.h"

#ifdef RX_UNIFIED_SERIAL

#include <stdlib.h>
#include <string.h>

#include "control.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "profile.h"
#include "usb_configurator.h"

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */

typedef enum {
  CRSF_FRAMETYPE_GPS = 0x02,
  CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
  CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
  CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
  CRSF_FRAMETYPE_ATTITUDE = 0x1E,
  CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
  CRSF_FRAMETYPE_DEVICE_PING = 0x28,
  CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
  CRSF_FRAMETYPE_MSP_REQ = 0x7A,  // response request using msp sequence as command
  CRSF_FRAMETYPE_MSP_RESP = 0x7B, // reply with 58 byte chunked binary
  CRSF_FRAMETYPE_MSP_WRITE = 0x7C // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
} crsf_frame_type_t;

typedef struct {
  // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
  unsigned int chan0 : 11;
  unsigned int chan1 : 11;
  unsigned int chan2 : 11;
  unsigned int chan3 : 11;
  unsigned int chan4 : 11;
  unsigned int chan5 : 11;
  unsigned int chan6 : 11;
  unsigned int chan7 : 11;
  unsigned int chan8 : 11;
  unsigned int chan9 : 11;
  unsigned int chan10 : 11;
  unsigned int chan11 : 11;
  unsigned int chan12 : 11;
  unsigned int chan13 : 11;
  unsigned int chan14 : 11;
  unsigned int chan15 : 11;
} __attribute__((__packed__)) crsf_channels_t;

typedef struct {
  uint8_t uplink_rssi_2;
  uint8_t uplink_rssi_1;
  uint8_t uplink_link_quality;
  int8_t uplink_snr;
  uint8_t active_antenna;
  uint8_t rf_mode;
  uint8_t uplink_tx_power;
  uint8_t downlink_rssi;
  uint8_t downlink_link_quality;
  int8_t downlink_snr;
} crsf_stats_t;

extern uint8_t rx_buffer[RX_BUFF_SIZE];
extern uint8_t rx_data[RX_BUFF_SIZE];

extern volatile uint8_t rx_frame_position;
extern volatile uint8_t expected_frame_length;
extern volatile frame_status_t frame_status;

extern uint16_t link_quality_raw;
extern uint8_t stat_frames_second;
extern uint32_t time_siglost;
extern uint32_t time_lastframe;

extern uint16_t bind_safety;
extern int32_t channels[16];

extern uint8_t failsafe_sbus_failsafe;
extern uint8_t failsafe_siglost;
extern uint8_t failsafe_noframes;

extern profile_t profile;
extern int current_pid_axis;
extern int current_pid_term;

extern uint8_t telemetry_offset;
extern uint8_t telemetry_packet[14];
extern uint8_t ready_for_next_telemetry;

#define USART usart_port_defs[serial_rx_port]

uint8_t crsf_crc8(uint8_t *data, uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc = crc ^ data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

void rx_serial_process_crsf() {

  if (expected_frame_length == 3) {
    //got the header
    memcpy(rx_data, rx_buffer, 3);

    if (rx_data[0] != 0xC8 || rx_data[1] > 64) {
      quic_debugf("CRSF: invalid header");
      frame_status = FRAME_IDLE;
      return;
    }

    // set real frame length
    expected_frame_length = rx_data[1] + 2;
    return;
  } else if (rx_frame_position < expected_frame_length) {
    return;
  }

  // copy rest of the data
  memcpy(rx_data, rx_buffer, expected_frame_length);

  const uint8_t crc_ours = crsf_crc8(&rx_data[2], expected_frame_length - 3);
  const uint8_t crc_theirs = rx_data[expected_frame_length - 1];

  if (crc_ours != crc_theirs) {
    // invalid crc, bail
    quic_debugf("CRSF: invalid crc, bail");
    frame_status = FRAME_IDLE;
    return;
  }

  rx_lqi_update_fps(0);

  switch (rx_data[2]) {
  case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: {
    const crsf_channels_t *chan = (crsf_channels_t *)&rx_data[3];
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
    state.rx.axis[0] = (channels[0] - 990.5f) * 0.00125707103f;
    state.rx.axis[1] = (channels[1] - 990.5f) * 0.00125707103f;
    state.rx.axis[2] = (channels[3] - 990.5f) * 0.00125707103f;
    state.rx.axis[3] = (channels[2] - 191.0f) * 0.00062853551f;

    if (state.rx.axis[3] > 1)
      state.rx.axis[3] = 1;
    if (state.rx.axis[3] < 0)
      state.rx.axis[3] = 0;

    rx_apply_expo();

    state.aux[AUX_CHANNEL_0] = (channels[4] > 1100) ? 1 : 0; //1100 cutoff intentionally selected to force aux channels low if
    state.aux[AUX_CHANNEL_1] = (channels[5] > 1100) ? 1 : 0; //being controlled by a transmitter using a 3 pos switch in center state
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
    break;
  }

  case CRSF_FRAMETYPE_LINK_STATISTICS: {
    const crsf_stats_t *stats = (crsf_stats_t *)&rx_data[3];
    rx_lqi_update_rssi_direct(stats->uplink_link_quality);
    break;
  }

  default:
    // ? handle ?
    break;
  }

  frame_status = FRAME_TX; //We're done with this frame now.

  bind_safety++;
  if (bind_safety > 131) {        //requires 130 good frames to come in before rx_ready safety can be toggled to 1.  About a second of good data
    flags.rx_ready = 1;           // because aux channels initialize low and clear the binding while armed flag before aux updates high
    flags.rx_mode = !RXMODE_BIND; // restores normal led operation
    bind_safety = 131;            // reset counter so it doesnt wrap
  } else {
    flags.rx_mode = RXMODE_BIND; // this is rapid flash during bind safety
  }
}

#endif