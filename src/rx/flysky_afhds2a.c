#include "rx/flysky.h"

#include <string.h>

#include "core/project.h"
#include "driver/spi_a7105.h"
#include "driver/time.h"
#include "flight/control.h" // for state.vbat_filtered

#if defined(USE_RX_SPI_FLYSKY)

enum {
  AFHDS2A_NUM_CHANS = 14
};

enum {
  AFHDS2A_SENSOR_RX_VOLTAGE = 0x00,
  AFHDS2A_SENSOR_RX_ERR_RATE = 0xFE, // This is actually RX LQI
  AFHDS2A_SENSOR_RX_RSSI = 0xFC      // Note: We do not (yet) send this
};

enum {
  AFHDS2A_PACKET_STICKS = 0x58,
  AFHDS2A_PACKET_FAILSAFE = 0x56,
  AFHDS2A_PACKET_SETTINGS = 0xAA,

  AFHDS2A_PACKET_BIND1 = 0xBB,
  AFHDS2A_PACKET_BIND2 = 0xBC,
  AFHDS2A_PACKET_TELEMETRY = 0xAA,
};

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t number;
  uint8_t value_lo;
  uint8_t value_hi;
} sensor_data_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint32_t tx_id;
  uint32_t rx_id;
  sensor_data_t sensor_data[7];
} afhds2a_tlm_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint32_t tx_id;
  uint32_t rx_id;
  uint8_t state;
  uint8_t reserved1;
  uint8_t rx_channel_map[16];
  uint8_t reserved2[10];
} afhds2a_bind_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint32_t tx_id;
  uint32_t rx_id;
  uint16_t channel_data[AFHDS2A_NUM_CHANS]; // little-endian, same as the STM32 running this program, but not aligned
} afhds2a_channel_pkt_t;

// Prepares the provided telemetry packet and writes it into the a7105 fifo
// for later transmission. Note: The tx_id, rx_id members of the packet should be
// pre-initialized by caller; this is easily done by re-using the RX packet that
// was just processed.
//
// Note: We only send back battery voltage and RX LQI. It may be nice to
// extend this to also send RX RSSI
static void prepare_and_write_telemetry(afhds2a_tlm_pkt_t *tlm_pkt) {

  tlm_pkt->type = AFHDS2A_PACKET_TELEMETRY;

  // MPM transmitter code considers AFHDS2A_SENSOR_RX_VOLTAGE to be an
  // 8bit value (value_lo) so we will assume this means the voltage is scaled by 10
  // (4.30 volts is sent as 43). we will also ensure value_hi is set anyways
  // TODO: This still needs verification with a TX that can display the telemetry
  const uint16_t voltage = state.vbat_filtered * 10 + 0.5f;
  tlm_pkt->sensor_data[0].type = AFHDS2A_SENSOR_RX_VOLTAGE;
  tlm_pkt->sensor_data[0].number = 0;
  tlm_pkt->sensor_data[0].value_lo = voltage & 0xFF;
  tlm_pkt->sensor_data[0].value_hi = (voltage >> 8) & 0xFF;

  // MPM transmitter code considers AFHDS2A_SENSOR_RX_ERR_RATE to be RX LQI
  tlm_pkt->sensor_data[1].type = AFHDS2A_SENSOR_RX_ERR_RATE;
  tlm_pkt->sensor_data[1].number = 0;
  tlm_pkt->sensor_data[1].value_lo = flysky.tlm_lqi & 0xFF;
  tlm_pkt->sensor_data[1].value_hi = (flysky.tlm_lqi >> 8) & 0xFF;

  // MPM transmitter code considers a sensor type value of 0xFF to be "end of data"
  // Not sure if OpenTX does the same or just ignores 0xFF types. So we will just
  // set the type value for the rest of the sensors to 0xFF
  for (int i = 2; i < 7; i++) {
    tlm_pkt->sensor_data[i].type = 0xFF;
  }

  // Once bound, the only data we ever send back is telemetry. So we can initialize
  // the write fifo with a full telemetry packet just once, subsequent packets only require
  // us to update the beginning part of the fifo since that's the only part of the
  // packet that changes
  static uint8_t num_to_write = sizeof(afhds2a_tlm_pkt_t);
  a7105_write_fifo((uint8_t *)tlm_pkt, num_to_write);
  num_to_write = offsetof(afhds2a_tlm_pkt_t, sensor_data) + sizeof(sensor_data_t) * 2;
}

// Retrieves packet from RX fifo and attempts to respond to it
// If channel packet was available and processed, and we're bound, this returns 14
// (the number of channels in an AFHDS2A packet), otherwise returns 0
uint8_t flysky_afhds2a_process_packet(const uint32_t timestamp) {
  uint8_t result = 0;

  union {
    afhds2a_bind_pkt_t bind;
    afhds2a_channel_pkt_t channel;
    afhds2a_tlm_pkt_t telemetry;
  } pkt;

  // If bound then read full packet otherwise only need to read as much of the bind
  // packet that we'll actually use
  const uint8_t num = flysky.bound ? sizeof(pkt.channel) : offsetof(afhds2a_bind_pkt_t, reserved2);
  a7105_read_fifo((uint8_t *)&pkt, num);

  // MPM transmitter code sends either AFHDS2A_PACKET_BIND1 or AFHDS2A_PACKET_BIND2 when binding,
  // Otherwise it sends AFHDS2A_PACKET_STICKS, AFHDS2A_PACKET_FAILSAFE or AFHDS2A_PACKET_SETTINGS
  const uint8_t pkt_type = pkt.channel.type;
  if ((pkt_type == AFHDS2A_PACKET_BIND1) || (pkt_type == AFHDS2A_PACKET_BIND2)) {
    if (!flysky.bound) {
      flysky_processed_pkt(timestamp);

      // Save the TX identifier and also the channel map if it was provided
      flysky.tx_id = pkt.bind.tx_id;
      if (pkt.bind.rx_channel_map[0] != 0xFF) {
        memcpy(flysky.rx_channel_map, pkt.bind.rx_channel_map, sizeof(flysky.rx_channel_map));
      }

      // Re-use the bind packet and update it with our RX identifier and write it into a7105
      // fifo so we can send it back to the TX. Re-using it is important as the .state and .tx_id
      // members will be checked by the TX code to determine if it should accept the RX identifier
      pkt.bind.rx_id = flysky.rx_id;
      a7105_write_fifo((uint8_t *)&pkt, sizeof(pkt.bind));
      flysky.pending_tx_time = flysky.last_bind_time = timestamp;
      flysky.pending_tx = true;
    }
  } else if ((pkt_type == AFHDS2A_PACKET_STICKS) || (pkt_type == AFHDS2A_PACKET_FAILSAFE) || (pkt_type == AFHDS2A_PACKET_SETTINGS)) {
    if ((pkt.channel.rx_id == flysky.rx_id && pkt.channel.tx_id == flysky.tx_id)) {
      flysky_processed_pkt(timestamp);

      if (pkt.channel.type == AFHDS2A_PACKET_STICKS) {
        // Save the channel data provided by the packet
        // Note: We use memcpy() to avoid unaligned memory access of the uint16_t data
        memcpy(flysky.channel_data, pkt.channel.channel_data, sizeof(pkt.channel.channel_data));

        if (flysky.send_telemetry) {
          prepare_and_write_telemetry(&pkt.telemetry);
          flysky.send_telemetry = false;
          flysky.pending_tx_time = timestamp;
          flysky.pending_tx = true;
        }

        // Return number of channels
        result = AFHDS2A_NUM_CHANS;
      }

      if (!flysky.pending_tx) {
        a7105_write_reg(A7105_0F_CHANNEL, flysky_get_next_channel(1));
      }
    }
  }

  if (!flysky.pending_tx) {
    a7105_strobe(A7105_RX);
  }
  return result;
}

#endif
