#include "rx_unified_serial.h"

#ifdef RX_UNIFIED_SERIAL

#include <stdbool.h>
#include <stdlib.h>

#include "control.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "profile.h"
#include "usb_configurator.h"
#include "util.h"

#define REDPINE_CHANNEL_START 3
#define REDPINE_CRC16_POLY 0x8005
#define LQI_FPS 500

extern uint8_t rx_buffer[RX_BUFF_SIZE];
extern uint8_t rx_data[RX_BUFF_SIZE];

extern volatile uint8_t rx_frame_position;
extern volatile uint8_t expected_frame_length;
extern volatile frame_status_t frame_status;

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

uint16_t redpine_crc16(uint8_t *data, uint16_t len) {
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

void rx_serial_process_redpine() {
  if (rx_frame_position != 11) {
    quic_debugf("REDPINE: unexpected frame length %d vs %d", rx_frame_position, expected_frame_length);
  }

  for (uint8_t i = 0; i < 11; i++) {
    rx_data[i] = rx_buffer[i % RX_BUFF_SIZE];
  }

  const uint16_t crc_our = redpine_crc16(rx_data + REDPINE_CHANNEL_START, 11 - REDPINE_CHANNEL_START);
  const uint16_t crc_theirs = (uint16_t)(rx_data[1] << 8) | rx_data[2];

  if (crc_our != crc_theirs) {
    // invalid crc, bail
    quic_debugf("REDPINE: invalid crc");
    frame_status = FRAME_IDLE;
    return;
  }

  // packet lost flag
  if ((rx_data[0] & 0xc0) != 0x40) {
    rx_lqi_got_packet();

    const uint16_t channels[4] = {
        (uint16_t)((rx_data[REDPINE_CHANNEL_START + 1] << 8) & 0x700) | rx_data[REDPINE_CHANNEL_START],
        (uint16_t)((rx_data[REDPINE_CHANNEL_START + 2] << 4) & 0x7F0) | ((rx_data[REDPINE_CHANNEL_START + 1] >> 4) & 0xF),
        (uint16_t)((rx_data[REDPINE_CHANNEL_START + 4] << 8) & 0x700) | rx_data[REDPINE_CHANNEL_START + 3],
        (uint16_t)((rx_data[REDPINE_CHANNEL_START + 5] << 4) & 0x7F0) | ((rx_data[REDPINE_CHANNEL_START + 4] >> 4) & 0xF),
    };

    // normal rx mode
    bind_safety++;
    if (bind_safety < 130)
      flags.rx_mode = RXMODE_BIND; // this is rapid flash during bind safety

    // AETR channel order
    state.rx.axis[0] = channels[0] - 1020;
    state.rx.axis[1] = channels[1] - 1020;
    state.rx.axis[2] = channels[3] - 1020;
    state.rx.axis[3] = channels[2] - 210;

    for (int i = 0; i < 3; i++) {
      state.rx.axis[i] *= 1.f / 820.f;
    }
    state.rx.axis[3] *= 1.f / 1640.f;
    state.rx.axis[3] = constrainf(state.rx.axis[3], 0, 1);

    rx_apply_expo();

    //Here we have the AUX channels Silverware supports
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

  rx_lqi_update_fps(LQI_FPS);

  if (profile.channel.lqi_source == RX_LQI_SOURCE_PACKET_RATE) { 
    rx_lqi_update_rssi_from_lqi(LQI_FPS);
  }
  if (profile.channel.lqi_source == RX_LQI_SOURCE_DIRECT) {
    rx_lqi_update_rssi_direct(rx_data[REDPINE_CHANNEL_START + 7]);
  }
  if (profile.channel.lqi_source == RX_LQI_SOURCE_CHANNEL) {
    rx_lqi_update_rssi_direct(0);  //aux channels are binary and cannot carry rssi
  }

  frame_status = FRAME_TX; //We're done with this frame now.

  if (bind_safety > 131) {        //requires 130 good frames to come in before rx_ready safety can be toggled to 1.  About a second of good data
    flags.rx_ready = 1;           // because aux channels initialize low and clear the binding while armed flag before aux updates high
    flags.rx_mode = !RXMODE_BIND; // restores normal led operation
    bind_safety = 131;            // reset counter so it doesnt wrap
  }
}

#endif