#include "rx_unified_serial.h"

#ifdef RX_UNIFIED_SERIAL

#include <stdbool.h>
#include <stdlib.h>

#include "control.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "profile.h"

#define LQI_FPS (1000.0f / 7.0f)

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

void rx_serial_process_ibus() {
  uint8_t frameLength = 0;
  for (uint8_t counter = 0; counter < 32; counter++) {    //First up, get the data out of the RX buffer and into somewhere safe
    rx_data[counter] = rx_buffer[counter % RX_BUFF_SIZE]; // This can probably go away, as long as the buffer is large enough
    frameLength++;                                        // to accept telemetry requests without overwriting control data
  }

  uint16_t crc_byte = 0xFFFF;
  for (int x = 0; x < 30; x++) {
    crc_byte = crc_byte - rx_data[x];
  }

  if (crc_byte == rx_data[30] + (rx_data[31] << 8)) { //If the CRC is good, shove it into controls

    //Flysky channels are delightfully straightforward
    channels[0] = rx_data[2] + (rx_data[3] << 8);
    channels[1] = rx_data[4] + (rx_data[5] << 8);
    channels[2] = rx_data[6] + (rx_data[7] << 8);
    channels[3] = rx_data[8] + (rx_data[9] << 8);
    channels[4] = rx_data[10] + (rx_data[11] << 8);
    channels[5] = rx_data[12] + (rx_data[13] << 8);
    channels[6] = rx_data[14] + (rx_data[15] << 8);
    channels[7] = rx_data[16] + (rx_data[17] << 8);
    channels[8] = rx_data[18] + (rx_data[19] << 8);
    channels[9] = rx_data[20] + (rx_data[21] << 8);
    channels[10] = rx_data[22] + (rx_data[23] << 8);
    channels[11] = rx_data[24] + (rx_data[25] << 8);
    channels[12] = rx_data[26] + (rx_data[27] << 8);
    channels[13] = rx_data[28] + (rx_data[29] << 8);

    frame_status = FRAME_RX;

  } else {
    // if CRC fails, do this:
    //while(1){} Enable for debugging to lock the FC if CRC fails. In the air we just drop CRC-failed packets
    //Most likely reason for failed CRC is a frame that isn't fully here yet. No need to check again until a new byte comes in.

    frame_status = FRAME_IDLE;
  }

  if (frame_status == FRAME_RX) {
    // normal rx mode
    bind_safety++;
    if (bind_safety < 130)
      flags.rx_mode = RXMODE_BIND; // this is rapid flash during bind safety

    // AETR channel order
    channels[0] -= 1500;
    channels[1] -= 1500;
    channels[2] -= 1000;
    channels[3] -= 1500;

    state.rx.axis[0] = channels[0];
    state.rx.axis[1] = channels[1];
    state.rx.axis[2] = channels[3];
    state.rx.axis[3] = channels[2];

    for (int i = 0; i < 3; i++) {
      state.rx.axis[i] *= 0.002f;
    }
    state.rx.axis[3] *= 0.001f;

    if (state.rx.axis[3] > 1)
      state.rx.axis[3] = 1;
    if (state.rx.axis[3] < 0)
      state.rx.axis[3] = 0;

    //rx_apply_expo()  no longer needed here;

    //Here we have the AUX channels Silverware supports
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

    rx_lqi_update_fps(0);

    if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL) {
      if (profile.receiver.aux[AUX_RSSI] <= AUX_CHANNEL_11) {
        rx_lqi_update_rssi_direct(0.1f * (channels[(profile.receiver.aux[AUX_RSSI] + 4)] - 1000));
      }
    }
    if (profile.receiver.lqi_source == RX_LQI_SOURCE_PACKET_RATE) {
      rx_lqi_update_rssi_from_lqi(LQI_FPS);
    }
    if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
      rx_lqi_update_rssi_direct(0); //no internal rssi data
    }

    frame_status = FRAME_TX; //We're done with this frame now.

    if (bind_safety > 131) {        //requires 130 good frames to come in before rx_ready safety can be toggled to 1.  About a second of good data
      flags.rx_ready = 1;           // because aux channels initialize low and clear the binding while armed flag before aux updates high
      flags.rx_mode = !RXMODE_BIND; // restores normal led operation
      bind_safety = 131;            // reset counter so it doesnt wrap
    }
  }
}

#endif