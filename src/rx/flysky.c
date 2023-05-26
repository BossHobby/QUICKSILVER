#include "rx/flysky.h"

#include <string.h>

#include "core/flash.h"
#include "core/project.h"
#include "driver/spi_a7105.h"
#include "driver/time.h"
#include "flight/control.h" // for state
#include "util/util.h"

#if defined(USE_RX_SPI_FLYSKY)

#define AFHDS_BIND_CHANNEL 0x00
#define AFHDS2A_BIND_CHANNEL 0x0D

rx_flsky_data_t flysky;

// Advances current channel index by the specified step amount and then returns
// the corresponding frequency from our hop table
uint8_t flysky_get_next_channel(uint8_t step) {
  flysky.channel_index = (flysky.channel_index + step) & 0x0F;
  return flysky.rx_channel_map[flysky.channel_index];
}

// Called everytime we've successfully processed an RX packet
void flysky_processed_pkt(uint32_t timestamp) {
  flysky.processed_pkt_count++;
  flysky.last_pkt_time = timestamp;
  flysky.timeout = flysky.timeout_period;
  flysky.num_timeouts = 0;
}

// Returns bind data that was previously stored into flash memory
rx_flysky_bind_data_t *flysky_get_bind_data() {
  return &bind_storage.flysky;
}

// Channel values are in the range of 1000 to 2000 inclusive (but can be as low
// as 988 and high as 2020). So we convert to range -1.0f to +1.0f and clamp
static float rescale_aetr(int v) {
  v -= 1500;
  if (v < -500) {
    v = -500;
  } else if (v > 500) {
    v = 500;
  }
  return v * 0.002f;
}

// Checks if RX or TX has completed. If TX complete then switch to RX mode
// else if RX complete, process the packet.
//
// * If we're bound, and we processed a channel packet, this returns the number of
//   channels provided by the packet (8 for AFHDS, 14 for AFHDS2)
// * If not already bound, and we transition to being bound, returns 1
// * Otherwise this returns 0
//
static uint8_t flysky_check_packet() {
  uint8_t result = 0;

  // Check if RX or TX interrupt occurred, and when it occurred
  uint32_t timestamp;
  if (a7105_rx_tx_irq_time(&timestamp)) {
    const uint8_t mode = a7105_read_reg(A7105_00_MODE);

    // If transceiver was in TX mode and transceiver is disabled (finished)
    // then hop to next channel (if bound) then swtich to RX mode
    if (((mode & A7105_MODE_TRSR) != 0) && ((mode & A7105_MODE_TRER) == 0)) {
      if (flysky.bound) {
        a7105_write_reg(A7105_0F_CHANNEL, flysky_get_next_channel(1));
      }
      a7105_strobe(A7105_RX);
    } else if ((mode & (A7105_MODE_TRER | A7105_MODE_CRCF)) == 0) {
      // ..else if RX is complete and no CRC error
      result = (flysky.protocol == RX_PROTOCOL_FLYSKY_AFHDS2A)
                   ? flysky_afhds2a_process_packet(timestamp)
                   : flysky_afhds_process_packet(timestamp);
    } else {
      a7105_strobe(A7105_RX);
    }
  }

  const uint32_t now = time_micros();

  // If we have a pending transmission in the TX fifo check if it is time to send it
  // (we wait 500 microseconds after last received packet before sending)
  if (flysky.pending_tx && (now - flysky.pending_tx_time) > 500) {
    flysky.pending_tx = false;
    a7105_strobe(A7105_TX);
  }

  if (flysky.bound) {
    // Check if we missed one or more packets
    if ((now - flysky.last_pkt_time) > flysky.timeout) {
      // We'll need to advance an appropriate number of channels in the hop table
      uint32_t num_hops = ((now - flysky.last_pkt_time) / flysky.pkt_period);

      flysky.last_pkt_time = (num_hops > 1) ? now : (flysky.last_pkt_time + flysky.timeout);

      a7105_strobe(A7105_STANDBY);
      a7105_write_reg(A7105_0F_CHANNEL, flysky_get_next_channel(num_hops & 0xF));
      a7105_strobe(A7105_RX);

      // If cycled through hop sequence more than twice without being able to
      // receive a packet then increase the timeout interval to 1.5 times normal
      // packet interval so we hop at a slower pace than TX and hopefully we'll
      // both land on the same frequency and be able to synchronize
      if (flysky.num_timeouts >= 32) {
        flysky.timeout = flysky.pkt_period * 3 / 2;
      } else {
        flysky.timeout = flysky.pkt_period;
        flysky.num_timeouts++;
      }
    }

    // If we send telemetry (AFHDS2A does, AFHDS does not)
    if (flysky.tlm_period) {
      // Check if it is about time to send back a telemetry packet
      if ((now - flysky.last_telemetry_time) >= flysky.tlm_period) {
        flysky.last_telemetry_time = now;
        flysky.send_telemetry = true;
      }

      // Since we send telemetry, see if it is time to compute link quality
      // indicator value (0..100) that we'll send back
      if ((now - flysky.last_tlm_lqi_time) >= (flysky.pkt_period * 100)) {
        flysky.last_tlm_lqi_time = now;
        flysky.tlm_lqi = (flysky.processed_pkt_count >= 100) ? 0 : (100 - flysky.processed_pkt_count);
        flysky.processed_pkt_count = 0;
      }
    }
  } else {
    // If 1/4 of a second has passed since receiving bind packet
    if (((time_micros() - flysky.last_bind_time) > 250000) && flysky.rx_channel_map[0] != 0 && flysky.tx_id != 0) {
      result = 1;
      flysky.bound = true;
      rx_flysky_bind_data_t *bind_data = flysky_get_bind_data();
      bind_data->tx_id = flysky.tx_id; // store tx_id
      memcpy(bind_data->rx_channel_map, flysky.rx_channel_map, sizeof(bind_data->rx_channel_map) / sizeof(bind_data->rx_channel_map[0]));
    }
  }

  return result;
}

// Periodic RX check function. Used for both AFHDS and AFHDS2A protocols
// Called once every main loop iteration
static bool rx_flysky_check(void) {
  const uint32_t now = time_micros();
  bool channels_received = false;

  // Check if we have new packet data
  const uint8_t status = flysky_check_packet();
  if (!status) {
    // nothing
  } else if (status == 1) {
    // We've just bound
    state.rx_status = RX_SPI_STATUS_BOUND;
    flags.rx_mode = RXMODE_NORMAL;
  } else {
    // We've processed a packet containing channel data
    const int num_avail_channels = status;
    channels_received = true;
    flysky.last_rx_time = now;
    state.rx_status = RX_SPI_STATUS_BOUND;
    flags.rx_mode = RXMODE_NORMAL;
    flags.rx_ready = 1;
    flags.failsafe = 0;
    rx_lqi_got_packet();

    // AETR channels
    const float rc_channels[4] = {
        rescale_aetr(flysky.channel_data[0]),
        rescale_aetr(flysky.channel_data[1]),
        rescale_aetr(flysky.channel_data[2]),
        rescale_aetr(flysky.channel_data[3])};
    rx_map_channels(rc_channels);

    // AUX channels
    for (int i = 4; i < num_avail_channels; i++) {
      if (i > AUX_CHANNEL_11) {
        break;
      }
      const int aux_chan = i - 4;
      const uint16_t threshold = 1200; // low is 1000, high is 2000
      state.aux[aux_chan] = (flysky.channel_data[i] > threshold) ? 1 : 0;
    }
  }

  // Check for failsafe
  if ((flags.rx_mode == RXMODE_NORMAL) && !flags.failsafe) {
    if ((now - flysky.last_rx_time) >= FAILSAFETIME) {
      flags.failsafe = true;
    }
  }

  rx_lqi_update();
  if (profile.receiver.lqi_source == RX_LQI_SOURCE_PACKET_RATE) {
    rx_lqi_update_from_fps(flysky.expected_fps);
  }

  return channels_received;
}

static void rx_flysky_init_common(uint8_t start_channel) {
  state.rx_status = RX_SPI_STATUS_BINDING;
  flysky.channel_index = 0;

  // Do we have previously saved bind data?
  const rx_flysky_bind_data_t *bind_data = flysky_get_bind_data();
  flysky.tx_id = bind_data->tx_id;
  if (flysky.tx_id == 0) {
    flysky.bound = false;
    memset(flysky.rx_channel_map, 0, sizeof(flysky.rx_channel_map));
  } else {
    flysky.bound = true;
    memcpy(flysky.rx_channel_map, bind_data->rx_channel_map, sizeof(flysky.rx_channel_map));
    start_channel = flysky_get_next_channel(0);
  }

  a7105_write_reg(A7105_0F_CHANNEL, start_channel);
  a7105_strobe(A7105_RX);

  // We haven't actually processed a packet, but we'll use flysky_processed_pkt() to
  // reset some state variables
  flysky_processed_pkt(time_micros());
}

void rx_flysky_afhds_init() {
  flysky.protocol = RX_PROTOCOL_FLYSKY_AFHDS;
  flysky.expected_fps = 1000000.0f / 1500.0f;
  flysky.pkt_period = 1500;
  flysky.timeout_period = ((uint32_t)(flysky.pkt_period * 1.27) + 0.5f);
  flysky.tlm_period = 0;

  static const uint8_t afhds_reg_table[] = {
      0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff, 0x00,
      0x00, 0x00, 0x00, 0x03, 0x19, 0x05, 0x00, 0x50,
      0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00,
      0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,
      0x13, 0xc3, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00,
      0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
      0x01, 0x0f};
  a7105_init(afhds_reg_table, sizeof(afhds_reg_table));
  rx_flysky_init_common(AFHDS_BIND_CHANNEL);
}

void rx_flysky_afhds2a_init() {
  flysky.protocol = RX_PROTOCOL_FLYSKY_AFHDS2A;
  flysky.expected_fps = 1000000.0f / 3850.0f;
  flysky.pkt_period = 3850;
  flysky.timeout_period = ((uint32_t)(flysky.pkt_period * 1.27) + 0.5f);
  flysky.tlm_period = flysky.pkt_period * 15;
  flysky.rx_id = get_chip_uid();

  static const uint8_t afhds2a_reg_table[] = {
      0xff, 0x62, 0x00, 0x25, 0x00, 0xff, 0xff, 0x00,
      0x00, 0x00, 0x00, 0x03, 0x19, 0x05, 0x00, 0x50,
      0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x4f,
      0x62, 0x80, 0xff, 0xff, 0x2a, 0x32, 0xc3, 0x1f,
      0x1e, 0xff, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00,
      0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
      0x01, 0x0f};
  a7105_init(afhds2a_reg_table, sizeof(afhds2a_reg_table));
  rx_flysky_init_common(AFHDS2A_BIND_CHANNEL);
}

bool rx_flysky_afhds_check() {
  return rx_flysky_check();
}
bool rx_flysky_afhds2a_check() {
  return rx_flysky_check();
}

bool flysky_detect() {
  return a7105_detect();
}

#endif
