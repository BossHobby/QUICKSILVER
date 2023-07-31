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

#ifdef USE_RX_UNIFIED

typedef enum {
  DSM_PROTO_INVALID = 0,
  DSMX_11_2048 = 0xb2,
  DSMX_22_2048 = 0xa2,
  DSM2_11_2048 = 0x12,
  DSM2_22_1024 = 0x01
} dsm_protocol_t;

typedef enum {
  DSM_CHECK_PROTOCOL,
  DSM_PAYLOAD,
} dsm_parser_state_t;

#define DSM_SCALE_PERCENT 147
#define DSM_PACKET_SIZE 16

static dsm_protocol_t dsm_protocol = DSM_PROTO_INVALID;

extern int32_t channels[16];
extern uint8_t rx_data[RX_BUFF_SIZE];

static dsm_protocol_t detect_protocol(uint8_t id) {
  switch (id) {
  case DSM2_22_1024:
    return DSM2_22_1024;
  case DSMX_11_2048:
    return DSMX_11_2048;
  case DSMX_22_2048:
    return DSMX_22_2048;
  case DSM2_11_2048:
    return DSM2_11_2048;
  default:
    return DSM_PROTO_INVALID;
  }
}

static packet_status_t dsm_handle_packet(uint8_t *packet) {
  uint8_t spek_width;
  uint16_t spek_chan_mask;
  uint16_t spek_val_mask;
  float dsm_scalefactor;
  float dsm_offset;

  switch (dsm_protocol) {
  case DSM2_22_1024:
    // 10 bit frames
    spek_width = 10;
    spek_chan_mask = 0xFC00;
    spek_val_mask = 0x03FF;
    dsm_scalefactor = (0.29354210f / DSM_SCALE_PERCENT);
    dsm_offset = 512.0f;
    break;
  case DSMX_11_2048:
  case DSMX_22_2048:
  case DSM2_11_2048:
    // 11 bit frames
    spek_width = 11;
    spek_chan_mask = 0x7800;
    spek_val_mask = 0x07FF;
    dsm_scalefactor = (0.14662756f / DSM_SCALE_PERCENT);
    dsm_offset = 1024.0f;
    break;

  default:
    return PACKET_ERROR;
  }

  const uint16_t *spek_ptr = (uint16_t *)(packet + 2);
  for (uint32_t i = 0; i < 7; i++) {
    const uint16_t spek_data = __builtin_bswap16(spek_ptr[i]);
    const uint16_t spek_val = spek_data & spek_val_mask;
    const uint8_t spek_chan = (spek_data & spek_chan_mask) >> spek_width;
    if (spek_chan < 16) {
      channels[spek_chan] = spek_val;
    } else {
      dsm_protocol = DSM_PROTO_INVALID;
      return PACKET_ERROR;
    }
  }

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

  rx_lqi_got_packet();
  if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL && profile.receiver.aux[AUX_RSSI] <= AUX_CHANNEL_11) {
    rx_lqi_update_direct(100 * (((channels[(profile.receiver.aux[AUX_RSSI] + 4)] - dsm_offset) * dsm_scalefactor * 0.5f) + 0.5f));
  }
  if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
    rx_lqi_update_direct(0); // no internal rssi data
  }

  return PACKET_CHANNELS_RECEIVED;
}

packet_status_t rx_serial_process_dsm() {
  static dsm_parser_state_t parser_state = DSM_CHECK_PROTOCOL;

dsm_do_more:
  switch (parser_state) {
  case DSM_CHECK_PROTOCOL: {
    if (!serial_read_bytes(&serial_rx, rx_data, 2)) {
      return PACKET_NEEDS_MORE;
    }
    dsm_protocol = detect_protocol(rx_data[1]);
    if (dsm_protocol != DSM_PROTO_INVALID && rx_data[0] < 4) {
      parser_state = DSM_PAYLOAD;
    }
    goto dsm_do_more;
  }
  case DSM_PAYLOAD: {
    if (serial_bytes_available(&serial_rx) < DSM_PACKET_SIZE - 2) {
      return PACKET_NEEDS_MORE;
    }
    if (serial_read_bytes(&serial_rx, rx_data + 2, DSM_PACKET_SIZE - 2) != DSM_PACKET_SIZE - 2) {
      return PACKET_ERROR;
    }
    parser_state = DSM_CHECK_PROTOCOL;
    return dsm_handle_packet(rx_data);
  }
  }

  return PACKET_ERROR;
}

// Send Spektrum bind pulses to a GPIO e.g. TX1
void rx_spektrum_bind() {
  if (profile.serial.rx == SERIAL_PORT_INVALID) {
    return;
  }

  if (bind_storage.bind_saved == 0) {
    const gpio_pins_t spectrum_bind_pin = target.serial_ports[profile.serial.rx].rx;

    gpio_config_t gpio_init;
    gpio_init.mode = GPIO_OUTPUT;
    gpio_init.output = GPIO_PUSHPULL;
    gpio_init.pull = GPIO_NO_PULL;
    gpio_pin_init(spectrum_bind_pin, gpio_init);

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

float rx_serial_dsm_expected_fps() {
  switch (dsm_protocol) {
  case DSM_PROTO_INVALID:
  case DSMX_11_2048:
  case DSM2_11_2048:
    return 91;
  case DSMX_22_2048:
  case DSM2_22_1024:
    return 45;
  }
  return 91;
}

#else
void rx_spektrum_bind() {
}
#endif