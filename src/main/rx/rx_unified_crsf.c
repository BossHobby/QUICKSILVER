#include "rx_unified_serial.h"

#ifdef RX_UNIFIED_SERIAL

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "drv_serial.h"
#include "drv_time.h"
#include "flight/control.h"
#include "profile.h"
#include "project.h"
#include "rx_crsf.h"
#include "usb_configurator.h"

typedef enum {
  RATE_LORA_4HZ = 0,
  RATE_LORA_25HZ,
  RATE_LORA_50HZ,
  RATE_LORA_100HZ,
  RATE_LORA_150HZ,
  RATE_LORA_200HZ,
  RATE_LORA_250HZ,
  RATE_LORA_500HZ,
  RATE_FLRC_500HZ,
  RATE_FLRC_1000HZ,
} crsf_air_rates_t;

extern uint8_t rx_buffer[RX_BUFF_SIZE];
extern uint8_t rx_data[RX_BUFF_SIZE];

static uint8_t telemetry_counter = 0;

extern volatile uint8_t rx_frame_position;
extern volatile uint8_t expected_frame_length;
extern volatile frame_status_t frame_status;

extern uint16_t bind_safety;
extern int32_t channels[16];

extern profile_t profile;
extern int current_pid_axis;
extern int current_pid_term;

extern uint8_t telemetry_offset;
extern uint8_t telemetry_packet[14];
extern uint8_t ready_for_next_telemetry;

static uint8_t crsf_rf_mode = 0;
static uint16_t crsf_rf_mode_fps[] = {
    4,    // RATE_LORA_4HZ
    25,   // RATE_LORA_25HZ
    50,   // RATE_LORA_50HZ
    100,  // RATE_LORA_100HZ
    150,  // RATE_LORA_150HZ
    200,  // RATE_LORA_200HZ
    250,  // RATE_LORA_250HZ
    500,  // RATE_LORA_500HZ
    500,  // RATE_FLRC_500HZ
    1000, // RATE_FLRC_1000HZ
};

#define USART usart_port_defs[serial_rx_port]

float rx_serial_crsf_expected_fps() {
  return crsf_rf_mode_fps[crsf_rf_mode];
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
    return 45;
  case RATE_LORA_150HZ:
    return 67;
  case RATE_LORA_200HZ:
    return 90;
  case RATE_LORA_250HZ:
    return 112;
  case RATE_LORA_500HZ:
  case RATE_FLRC_500HZ:
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
    return 6;
  case RATE_LORA_150HZ:
    return 9;
  case RATE_LORA_200HZ:
    return 12;
  case RATE_LORA_250HZ:
    return 16;
  case RATE_LORA_500HZ:
  case RATE_FLRC_500HZ:
    return 32;
  case RATE_FLRC_1000HZ:
    return 64;
  }

  return 5;
}

static bool rx_serial_crsf_process_frame() {
  bool channels_received = false;

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
    const crsf_stats_t *stats = (crsf_stats_t *)&rx_data[3];

    crsf_rf_mode = stats->rf_mode;

    if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
      rx_lqi_update_direct(stats->uplink_link_quality);
    }
    break;
  }

  default:
    quic_debugf("CRSF: unhandled packet type 0x%x", rx_data[2]);
    break;
  }

  rx_lqi_got_packet();

  bind_safety++;
  if (bind_safety > 131) {        // requires 130 good frames to come in before rx_ready safety can be toggled to 1.  About a second of good data
    flags.rx_ready = 1;           // because aux channels initialize low and clear the binding while armed flag before aux updates high
    flags.rx_mode = !RXMODE_BIND; // restores normal led operation
    bind_safety = 131;            // reset counter so it doesnt wrap
  } else {
    flags.rx_mode = RXMODE_BIND; // this is rapid flash during bind safety
  }

  return channels_received;
}

bool rx_serial_process_crsf() {
  bool channels_received = false;

  static int32_t rx_buffer_offset = 0;

  if (rx_frame_position < rx_buffer_offset || rx_buffer[rx_buffer_offset] != 0xC8) {
    // we should have at least one byte by now.
    // fail if its not a magic
    frame_status = FRAME_TX;
    rx_buffer_offset = 0;
    return channels_received;
  }

  if ((rx_frame_position - rx_buffer_offset) < 3) {
    // not enough data
    frame_status = FRAME_IDLE;
    return channels_received;
  }

  // copy the header
  memcpy(rx_data, rx_buffer + rx_buffer_offset, 3);

  if (rx_data[0] != 0xC8 || rx_data[1] > 64) {
    quic_debugf("CRSF: invalid header");
    frame_status = FRAME_TX;
    rx_buffer_offset = 0;
    return channels_received;
  }

  // get real frame length
  const uint32_t frame_length = rx_data[1] + 2;

  if ((rx_frame_position - rx_buffer_offset) < frame_length) {
    // not enough data
    frame_status = FRAME_IDLE;
    return channels_received;
  }

  // copy rest of the data
  memcpy(rx_data, rx_buffer + rx_buffer_offset, frame_length);

  const uint8_t crc_ours = crsf_crc8(&rx_data[2], frame_length - 3);
  const uint8_t crc_theirs = rx_data[frame_length - 1];
  if (crc_ours != crc_theirs) {
    // invalid crc, bail
    quic_debugf("CRSF: invalid crc, bail");
    frame_status = FRAME_TX;
    rx_buffer_offset = 0;
    return channels_received;
  }

  // we got a valid frame, update offset to potentially read another frame
  rx_buffer_offset += frame_length;
  channels_received = rx_serial_crsf_process_frame();

  if ((rx_frame_position - rx_buffer_offset) <= 0) {
    // We're done with this frame now.
    frame_status = FRAME_TX;
    rx_buffer_offset = 0;
    telemetry_counter++; // Telemetry will send data out when this reaches 5
  }

  return channels_received;
}

void rx_serial_send_crsf_telemetry() {
  // Send telemetry back once every 5 packets. This gives the RX time to send ITS telemetry back
  if (telemetry_counter < telemetry_interval() || frame_status != FRAME_TX) {
    frame_status = FRAME_DONE;
    return;
  }

  telemetry_counter = 0;
  frame_status = FRAME_DONE;

  crsf_tlm_frame_start(telemetry_packet);
  const uint32_t payload_size = crsf_tlm_frame_battery_sensor(telemetry_packet);
  crsf_tlm_frame_finish(telemetry_packet, payload_size);

  // QS Telemetry send function assumes 10 bytes of telemetry + the offset for escaped bytes
  // Since we are not escaping anything, and since we know there are 12 bytes (packets) in
  // CRSF telemetry for the battery sensor, Offset is 12-10=2
  telemetry_offset = 2;

  // Shove the packet out the UART.
  while (LL_USART_IsActiveFlag_TXE(USART.channel) == RESET)
    ;
  LL_USART_TransmitData8(USART.channel, telemetry_packet[0]);
  ready_for_next_telemetry = 0;

  // turn on the transmit transfer complete interrupt so that the rest of the telemetry packet gets sent
  // That's it, telemetry has sent the first byte - the rest will be sent by the telemetry tx irq
  LL_USART_EnableIT_TC(USART.channel);
}

#endif