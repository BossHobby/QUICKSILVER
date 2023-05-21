#include "rx/flysky.h"

#include <string.h>

#include "core/project.h"
#include "driver/spi_a7105.h"

#if defined(RX_FLYSKY)

enum {
  AFHDS_NUM_CHANS = 8
};

// AFHDS packet type identifiers
enum {
  PACKET_TYPE_CHANNEL_DATA = 0x55,
  PACKET_TYPE_BIND = 0xAA
};

// AFHDS packet structure
typedef struct __attribute__((packed)) {
  uint8_t type; // PACKET_TYPE_CHANNEL_DATA or PACKET_TYPE_BIND
  uint32_t tx_id;
  uint16_t channel_data[AFHDS_NUM_CHANS]; // little-endian, same as the STM32 running this program, but not aligned
} afhds_pkt_t;

static const uint8_t AFHDS_tx_channels[8][4] = {
    {0x12, 0x34, 0x56, 0x78},
    {0x18, 0x27, 0x36, 0x45},
    {0x41, 0x82, 0x36, 0x57},
    {0x84, 0x13, 0x65, 0x72},
    {0x87, 0x64, 0x15, 0x32},
    {0x76, 0x84, 0x13, 0x52},
    {0x71, 0x62, 0x84, 0x35},
    {0x71, 0x86, 0x43, 0x52}};

// Reconstruct the hop table that the TX will be using based on the TX
// identifier it sent us
static void build_hop_table(uint32_t tx_id, uint8_t channel_map[16]) {

  uint8_t chan_row = tx_id & 0x0F;
  uint8_t chan_offset = ((tx_id & 0xF0) >> 4) + 1;

  // limit offset to 9 as higher values may not work with some RX
  if (chan_offset > 9) {
    chan_offset = 9;
  }

  for (uint8_t i = 0; i < 16; i++) {
    uint8_t temp = AFHDS_tx_channels[chan_row >> 1][i >> 2];
    if (i & 0x02)
      temp &= 0x0F;
    else
      temp >>= 4;
    temp *= 0x0A;
    if (i & 0x01)
      temp += 0x50;
    channel_map[((chan_row & 1) ? 15 - i : i)] = temp - chan_offset;
  }
}

// Retrieves packet from RX fifo and attempts to respond to it
// If packet was available and processed, and we're bound, this returns 8
// (the number of channels in an AFHDS packet), otherwise returns 0
uint8_t flysky_afhds_process_packet(const uint32_t timestamp) {
  uint8_t result = 0;

  // If bound then retrieve entire packet, otherwise we only need to retrieve just
  // the beginning part of the packet ('type', 'tx_id' and not 'data').
  afhds_pkt_t pkt;
  const uint8_t num = flysky.bound ? sizeof(pkt) : offsetof(afhds_pkt_t, channel_data);
  a7105_read_fifo((uint8_t *)&pkt, num);

  int hop_step = -1;

  // If bound, and packet contains channel data, and it matches the TX we're bound to...
  if (flysky.bound && pkt.type == PACKET_TYPE_CHANNEL_DATA && pkt.tx_id == flysky.tx_id) {
    // Save the 16bit channel data provided by the packet, AFHDS provides 8 channels
    // Note: We use memcpy() to avoid unaligned memory access of the uint16_t data
    memcpy(flysky.channel_data, pkt.channel_data, sizeof(pkt.channel_data));

    // Return number of channels
    result = AFHDS_NUM_CHANS;

    hop_step = 1;
  }

  // If not bound and this is a BIND packet then generate hop table from the TX id in the bind packet
  if (!flysky.bound && pkt.type == PACKET_TYPE_BIND) {
    flysky.last_bind_time = timestamp;
    flysky.tx_id = pkt.tx_id;
    build_hop_table(flysky.tx_id, flysky.rx_channel_map);

    hop_step = 0;
  }

  // If we handled the packet
  if (-1 != hop_step) {
    flysky_processed_pkt(timestamp);
    a7105_write_reg(A7105_0F_CHANNEL, flysky_get_next_channel(hop_step));
  }

  a7105_strobe(A7105_RX);
  return result;
}

#endif
