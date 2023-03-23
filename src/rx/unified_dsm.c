#include "rx/unified_serial.h"

#include <stdbool.h>
#include <stdlib.h>

#include "core/flash.h"
#include "core/profile.h"
#include "driver/fmc.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "flight/control.h"
#include "util/util.h"

typedef enum {
  PROTOCOL_INVALID = 0,
  DSMX_11_2048 = 0xb2,
  DSMX_22_2048 = 0xa2,
  DSM2_11_2048 = 0x12,
  DSM2_22_1024 = 0x01
} dsm_protocol_t;
static dsm_protocol_t dsm_protocol = PROTOCOL_INVALID;

#define DSM_SCALE_PERCENT 147 // this might stay somewhere or be replaced with wizard scaling

extern uint8_t rx_buffer[RX_BUFF_SIZE];
extern uint8_t rx_data[RX_BUFF_SIZE];

extern volatile uint8_t expected_frame_length;
extern volatile frame_status_t frame_status;

extern uint16_t bind_safety;
extern int32_t channels[16];

extern profile_t profile;
extern int current_pid_axis;
extern int current_pid_term;

#define USART usart_port_defs[serial_rx_port]

bool rx_serial_process_dsm() {
  bool channels_received = false;

  for (uint8_t counter = 0; counter < 16; counter++) {    // First up, get the rx_data out of the RX buffer and into somewhere safe
    rx_data[counter] = rx_buffer[counter % RX_BUFF_SIZE]; // This can probably go away, as long as the buffer is large enough
  }

  if (dsm_protocol == PROTOCOL_INVALID) { // dsm variant has not been selected yet
    dsm_protocol = rx_buffer[1];          // detect dsm variant on first contact and run with it
  }

  uint8_t spek_chan_shift;
  uint8_t spek_chan_mask;
  uint8_t dsm_channel_count;
  float dsm_scalefactor;
  float dsm_offset;

  switch (dsm_protocol) {
  case PROTOCOL_INVALID:
    return channels_received;
  case DSM2_22_1024:
    // 10 bit frames
    spek_chan_shift = 2;
    spek_chan_mask = 0x03;
    dsm_channel_count = 7;
    dsm_scalefactor = (0.29354210f / DSM_SCALE_PERCENT);
    dsm_offset = 512.0f;
    break;
  case DSMX_11_2048:
  case DSMX_22_2048:
  case DSM2_11_2048:
  default:
    // 11 bit frames
    spek_chan_shift = 3;
    spek_chan_mask = 0x07;
    dsm_channel_count = 12;
    dsm_scalefactor = (0.14662756f / DSM_SCALE_PERCENT);
    dsm_offset = 1024.0f;
    break;
  }

  for (int b = 3; b < expected_frame_length; b += 2) { // stick data in channels buckets
    const uint8_t spekChannel = 0x0F & (rx_data[b - 1] >> spek_chan_shift);
    if (spekChannel < dsm_channel_count && spekChannel < 12) {
      channels[spekChannel] = ((uint32_t)(rx_data[b - 1] & spek_chan_mask) << 8) + rx_data[b];
      frame_status = FRAME_RX_DONE; // if we can hold 2 here for an entire frame, then we will decode it
    } else {
      // a counter here will flag on 22ms mode which could be used for auto-apply of correct filter cut on rc smoothing
    }
  }

  if (frame_status == FRAME_RX_DONE) {
    bind_safety++;
    if (bind_safety < 120)
      flags.rx_mode = RXMODE_BIND; // this is rapid flash during bind safety

    // AETR channel order
    const float rc_channels[4] = {
        (channels[0] - dsm_offset) * dsm_scalefactor,
        (channels[1] - dsm_offset) * dsm_scalefactor,
        (channels[2] - dsm_offset) * dsm_scalefactor,
        (channels[3] - dsm_offset) * dsm_scalefactor,
    };

    rx_map_channels(rc_channels);

    state.aux[AUX_CHANNEL_0] = (((channels[4] - dsm_offset) * dsm_scalefactor) > 0.11f) ? 1 : 0; // cutoff intentionally selected to force aux channels low if
    state.aux[AUX_CHANNEL_1] = (((channels[5] - dsm_offset) * dsm_scalefactor) > 0.11f) ? 1 : 0; // being controlled by a transmitter using a 3 pos switch in center state
    state.aux[AUX_CHANNEL_2] = (((channels[6] - dsm_offset) * dsm_scalefactor) > 0.11f) ? 1 : 0;
    state.aux[AUX_CHANNEL_3] = (((channels[7] - dsm_offset) * dsm_scalefactor) > 0.11f) ? 1 : 0;
    state.aux[AUX_CHANNEL_4] = (((channels[8] - dsm_offset) * dsm_scalefactor) > 0.11f) ? 1 : 0;
    state.aux[AUX_CHANNEL_5] = (((channels[9] - dsm_offset) * dsm_scalefactor) > 0.11f) ? 1 : 0;
    state.aux[AUX_CHANNEL_6] = (((channels[10] - dsm_offset) * dsm_scalefactor) > 0.11f) ? 1 : 0;
    state.aux[AUX_CHANNEL_7] = (((channels[11] - dsm_offset) * dsm_scalefactor) > 0.11f) ? 1 : 0;

    channels_received = true;

    rx_lqi_got_packet();

    if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL && profile.receiver.aux[AUX_RSSI] <= AUX_CHANNEL_11) {
      rx_lqi_update_direct(100 * (((channels[(profile.receiver.aux[AUX_RSSI] + 4)] - dsm_offset) * dsm_scalefactor * 0.5f) + 0.5f));
    }

    if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
      rx_lqi_update_direct(0); // no internal rssi data
    }

    frame_status = FRAME_DONE; // We're done with this frame now.

    if ((bind_safety > 120) && (rx_buffer[1] == dsm_protocol)) { // requires 120 good frames to come in and one last sanity check the protocol still matches before rx_ready safety can be toggled to 1.  About a second of good data
      flags.rx_ready = 1;                                        // because aux channels initialize low and clear the binding while armed flag before aux updates high
      flags.rx_mode = !RXMODE_BIND;                              // restores normal led operation
      bind_safety = 121;                                         // reset counter so it doesnt wrap
    }
  }

  return channels_received;
}

// Send Spektrum bind pulses to a GPIO e.g. TX1
void rx_spektrum_bind() {
  if (profile.serial.rx == SERIAL_PORT_INVALID) {
    return;
  }

  if (bind_storage.bind_saved == 0) {
    const gpio_pins_t spectrum_bind_pin = target.serial_ports[profile.serial.rx].rx;

    LL_GPIO_InitTypeDef gpio_init;
    gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_NO;
    gpio_pin_init(&gpio_init, spectrum_bind_pin);

    // RX line, set high
    gpio_pin_set(spectrum_bind_pin);
    // Bind window is around 20-140ms after powerup
    time_delay_us(60000);

    for (uint8_t i = 0; i < 9; i++) { // 9 pulses for internal dsmx 11ms, 3 pulses for internal dsm2 22ms
      // RX line, drive low for 120us
      gpio_pin_reset(spectrum_bind_pin);
      time_delay_us(120);

      // RX line, drive high for 120us
      gpio_pin_set(spectrum_bind_pin);
      time_delay_us(120);
    }
  }
}

uint16_t rx_serial_dsm_smoothing_cutoff() {
  switch (dsm_protocol) {
  case PROTOCOL_INVALID:
  case DSMX_11_2048:
  case DSM2_11_2048:
    return 40;
  case DSMX_22_2048:
  case DSM2_22_1024:
    return 20;
  }
  return 40;
}

float rx_serial_dsm_expected_fps() {
  switch (dsm_protocol) {
  case PROTOCOL_INVALID:
  case DSMX_11_2048:
  case DSM2_11_2048:
    return 91;
  case DSMX_22_2048:
  case DSM2_22_1024:
    return 45;
  }
  return 91;
}