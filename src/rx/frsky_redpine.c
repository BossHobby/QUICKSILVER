#include "rx/frsky.h"

#include "core/debug.h"
#include "core/flash.h"
#include "core/looptime.h"
#include "core/profile.h"
#include "driver/spi_cc2500.h"
#include "driver/time.h"
#include "flight/control.h"
#include "rx/rx_spi.h"
#include "util/util.h"

#if defined(RX_FRSKY)

#define LQI_FPS 500

#define REDPINE_PACKET_SIZE 11
#define REDPINE_PACKET_SIZE_W_ADDONS (REDPINE_PACKET_SIZE + 2)

#define REDPINE_SWITCH_SPEED_US 2000000
#define REDPINE_PACKET_TIME_US 50000

#define REDPINE_CHANNEL_START 3
#define REDPINE_CHANNEL_COUNT 16

extern uint8_t list_length;
extern uint8_t protocol_state;

static uint8_t redpine_fast = 1;

uint8_t frsky_extract_rssi(uint8_t rssi_raw);
void frsky_handle_bind();

void handle_overflows();
void calibrate_channels();
void set_address(uint8_t is_bind);
uint8_t next_channel(uint8_t skip);

static void redpine_set_rc_data() {
  if (rx_spi_packet[REDPINE_CHANNEL_START] == 1 && rx_spi_packet[REDPINE_CHANNEL_START + 1] == 0) {
    // TODO: vtx command frame
    return;
  }

  // if we made it this far, data is ready
  flags.rx_ready = 1;
  flags.failsafe = 0;

  const uint16_t channels[4] = {
      (uint16_t)((rx_spi_packet[REDPINE_CHANNEL_START + 1] << 8) & 0x700) | rx_spi_packet[REDPINE_CHANNEL_START],
      (uint16_t)((rx_spi_packet[REDPINE_CHANNEL_START + 2] << 4) & 0x7F0) | ((rx_spi_packet[REDPINE_CHANNEL_START + 1] >> 4) & 0xF),
      (uint16_t)((rx_spi_packet[REDPINE_CHANNEL_START + 4] << 8) & 0x700) | rx_spi_packet[REDPINE_CHANNEL_START + 3],
      (uint16_t)((rx_spi_packet[REDPINE_CHANNEL_START + 5] << 4) & 0x7F0) | ((rx_spi_packet[REDPINE_CHANNEL_START + 4] >> 4) & 0xF),
  };

  // AETR channel order
  const float rc_channels[4] = {
      (channels[0] - 1020.0f) * (1.0f / 820.f),
      (channels[1] - 1020.0f) * (1.0f / 820.f),
      (channels[2] - 1020.0f) * (1.0f / 820.f),
      (channels[3] - 1020.0f) * (1.0f / 820.f),
  };
  rx_map_channels(rc_channels);

  // Here we have the AUX channels Silverware supports
  state.aux[AUX_CHANNEL_0] = (rx_spi_packet[REDPINE_CHANNEL_START + 1] & 0x08) ? 1 : 0;
  state.aux[AUX_CHANNEL_1] = (rx_spi_packet[REDPINE_CHANNEL_START + 2] & 0x80) ? 1 : 0;
  state.aux[AUX_CHANNEL_2] = (rx_spi_packet[REDPINE_CHANNEL_START + 4] & 0x08) ? 1 : 0;
  state.aux[AUX_CHANNEL_3] = (rx_spi_packet[REDPINE_CHANNEL_START + 5] & 0x80) ? 1 : 0;
  state.aux[AUX_CHANNEL_4] = (rx_spi_packet[REDPINE_CHANNEL_START + 6] & 0x01) ? 1 : 0;
  state.aux[AUX_CHANNEL_5] = (rx_spi_packet[REDPINE_CHANNEL_START + 6] & 0x02) ? 1 : 0;
  state.aux[AUX_CHANNEL_6] = (rx_spi_packet[REDPINE_CHANNEL_START + 6] & 0x04) ? 1 : 0;
  state.aux[AUX_CHANNEL_7] = (rx_spi_packet[REDPINE_CHANNEL_START + 6] & 0x08) ? 1 : 0;
  state.aux[AUX_CHANNEL_8] = (rx_spi_packet[REDPINE_CHANNEL_START + 6] & 0x10) ? 1 : 0;
  state.aux[AUX_CHANNEL_9] = (rx_spi_packet[REDPINE_CHANNEL_START + 6] & 0x20) ? 1 : 0;
  state.aux[AUX_CHANNEL_10] = (rx_spi_packet[REDPINE_CHANNEL_START + 6] & 0x40) ? 1 : 0;
  state.aux[AUX_CHANNEL_11] = (rx_spi_packet[REDPINE_CHANNEL_START + 6] & 0x80) ? 1 : 0;

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL) {
    state.rx_rssi = 0.f;
  }
}

static uint8_t redpine_handle_packet() {
  static uint32_t max_sync_delay = REDPINE_PACKET_TIME_US;

  static uint32_t frames_lost = 0;

  static uint32_t packet_time = 0;
  static uint32_t total_time = 0;
  static uint32_t protocol_time = 0;

  uint8_t ret = 0;
  switch (protocol_state) {
  case FRSKY_STATE_STARTING:
    cc2500_enter_rxmode();
    set_address(0);
    next_channel(1);
    cc2500_strobe(CC2500_SRX);

    protocol_state = FRSKY_STATE_UPDATE;
    protocol_time = time_micros();
    break;

  case FRSKY_STATE_UPDATE:
    packet_time = 0;
    total_time = time_micros();
    protocol_state = FRSKY_STATE_DATA;

    // fallthrough
  case FRSKY_STATE_DATA: {
    handle_overflows();

    uint8_t len = cc2500_packet_size();
    if (len == REDPINE_PACKET_SIZE_W_ADDONS) {
      cc2500_read_fifo((uint8_t *)rx_spi_packet, len);

      if ((rx_spi_packet[0] == REDPINE_PACKET_SIZE - 1) &&
          (rx_spi_packet[1] == bind_storage.frsky.tx_id[0]) &&
          (rx_spi_packet[2] == bind_storage.frsky.tx_id[1])) {

        if (redpine_fast) {
          max_sync_delay = rx_spi_packet[REDPINE_CHANNEL_START + 7] * 100;
        } else {
          max_sync_delay = rx_spi_packet[REDPINE_CHANNEL_START + 7] * 1000;
        }

        max_sync_delay += max_sync_delay / 8;

        packet_time = time_micros();
        total_time = time_micros();
        protocol_time = time_micros();

        next_channel(1);
        cc2500_strobe(CC2500_SRX);

        rx_lqi_got_packet();
        if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
          rx_lqi_update_direct(frsky_extract_rssi(rx_spi_packet[REDPINE_PACKET_SIZE_W_ADDONS - 2]));
        }

        frames_lost = 0;
        ret = 1;
      } else {
        quic_debugf("REDPINE: invalid frame");
        cc2500_strobe(CC2500_SFRX);
      }
    } else if (len > 0) {
      quic_debugf("REDPINE: invalid size %d", len);
      cc2500_strobe(CC2500_SFRX);
    }

    if ((time_micros() - total_time) > 50 * max_sync_delay) {
      // out of sync with packets - do a complete resysnc
      quic_debugf("REDPINE: resync %u", (time_micros() - total_time));
      state.rx_rssi = 0;
      next_channel(1);
      cc2500_strobe(CC2500_SRX);

      protocol_state = FRSKY_STATE_UPDATE;
    } else if (packet_time > 0 && (time_micros() - packet_time) > max_sync_delay) {
      // missed a packet
      quic_debugf("REDPINE: frame lost #%d at %u (%d)", frames_lost, (time_micros() - packet_time), max_sync_delay);
      rx_lqi_lost_packet();
      packet_time = time_micros();
      next_channel(1);
      cc2500_strobe(CC2500_SRX);

      if (frames_lost >= 2) {
        cc2500_switch_antenna();
      }

      frames_lost++;
    } else if ((time_micros() - protocol_time) > REDPINE_SWITCH_SPEED_US) {
      quic_debugf("REDPINE: switching speed");
      redpine_fast = redpine_fast == 1 ? 0 : 1;
      max_sync_delay = REDPINE_PACKET_TIME_US;
      protocol_time = time_micros();
      flags.failsafe = 1;
      protocol_state = FRSKY_STATE_INIT;
      rx_redpine_init();
      looptime_reset();
    }
    break;
  }
  }

  return ret;
}

void rx_redpine_init() {
  if (!frsky_init()) {
    return;
  }

  if (target.rx_spi.exti != PIN_NONE) {
    // enable gdo0 on read
    cc2500_write_reg(CC2500_IOCFG0, 0x01);
  } else {
    // use gdo0 to turn lna on
    cc2500_write_reg(CC2500_IOCFG0, 0x2F);
    cc2500_write_reg(CC2500_IOCFG2, 0x2F | 0x40);
  }

  cc2500_write_reg(CC2500_PKTLEN, REDPINE_PACKET_SIZE); // max packet lenght of 25
  cc2500_write_reg(CC2500_PKTCTRL1, 0x0C);              // only append status
  cc2500_write_reg(CC2500_PKTCTRL0, 0x05);              // variable pkt lenth, enable crc
  cc2500_write_reg(CC2500_PATABLE, 0xFF);               // full power

  if (redpine_fast) {
    cc2500_write_reg(CC2500_FSCTRL1, 0x0A);
    cc2500_write_reg(CC2500_FSCTRL0, 0x00);

    cc2500_write_reg(CC2500_FREQ2, 0x5D);
    cc2500_write_reg(CC2500_FREQ1, 0x93);
    cc2500_write_reg(CC2500_FREQ0, 0xB1);

    cc2500_write_reg(CC2500_MDMCFG4, 0x2D);
    cc2500_write_reg(CC2500_MDMCFG3, 0x3B);
    cc2500_write_reg(CC2500_MDMCFG2, 0x73);
    cc2500_write_reg(CC2500_MDMCFG1, 0x23);
    cc2500_write_reg(CC2500_MDMCFG0, 0x56);

    cc2500_write_reg(CC2500_DEVIATN, 0x00);

    cc2500_write_reg(CC2500_MCSM1, 0x0C);
    cc2500_write_reg(CC2500_MCSM0, 0x08);

    cc2500_write_reg(CC2500_FOCCFG, 0x1D);
    cc2500_write_reg(CC2500_BSCFG, 0x1C);

    cc2500_write_reg(CC2500_AGCCTRL2, 0xC7);
    cc2500_write_reg(CC2500_AGCCTRL1, 0x00);
    cc2500_write_reg(CC2500_AGCCTRL0, 0xB0);

    cc2500_write_reg(CC2500_FREND1, 0xB6);
    cc2500_write_reg(CC2500_FREND0, 0x10);

    cc2500_write_reg(CC2500_FSCAL3, 0xEA);
    cc2500_write_reg(CC2500_FSCAL2, 0x0A);
    cc2500_write_reg(CC2500_FSCAL1, 0x00);
    cc2500_write_reg(CC2500_FSCAL0, 0x11);
  } else {
    cc2500_write_reg(CC2500_FSCTRL1, 0x06);
    cc2500_write_reg(CC2500_FSCTRL0, 0x00);

    cc2500_write_reg(CC2500_FREQ2, 0x5D);
    cc2500_write_reg(CC2500_FREQ1, 0x93);
    cc2500_write_reg(CC2500_FREQ0, 0xB1);

    cc2500_write_reg(CC2500_MDMCFG4, 0x78);
    cc2500_write_reg(CC2500_MDMCFG3, 0x93);
    cc2500_write_reg(CC2500_MDMCFG2, 0x03);
    cc2500_write_reg(CC2500_MDMCFG1, 0x22);
    cc2500_write_reg(CC2500_MDMCFG0, 0xF8);

    cc2500_write_reg(CC2500_DEVIATN, 0x44);

    cc2500_write_reg(CC2500_MCSM1, 0x0C);
    cc2500_write_reg(CC2500_MCSM0, 0x08);

    cc2500_write_reg(CC2500_FOCCFG, 0x16);
    cc2500_write_reg(CC2500_BSCFG, 0x6C);

    cc2500_write_reg(CC2500_AGCCTRL2, 0x43);
    cc2500_write_reg(CC2500_AGCCTRL1, 0x40);
    cc2500_write_reg(CC2500_AGCCTRL0, 0x91);

    cc2500_write_reg(CC2500_FREND1, 0x56);
    cc2500_write_reg(CC2500_FREND0, 0x10);

    cc2500_write_reg(CC2500_FSCAL3, 0xA9);
    cc2500_write_reg(CC2500_FSCAL2, 0x0A);
    cc2500_write_reg(CC2500_FSCAL1, 0x00);
    cc2500_write_reg(CC2500_FSCAL0, 0x11);
  }

  cc2500_write_reg(CC2500_FSTEST, 0x59);

  cc2500_write_reg(CC2500_TEST2, 0x88);
  cc2500_write_reg(CC2500_TEST1, 0x31);
  cc2500_write_reg(CC2500_TEST0, 0x0B);

  cc2500_write_reg(CC2500_FIFOTHR, 0x07);

  cc2500_write_reg(CC2500_ADDR, 0x00);

  calibrate_channels();

  list_length = 49;
}

bool rx_redpine_check() {
  bool channels_received = false;

  if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
    return channels_received;
  }

  if (protocol_state <= FRSKY_STATE_BIND_COMPLETE) {
    frsky_handle_bind();
    return channels_received;
  }

  if (redpine_handle_packet()) {
    redpine_set_rc_data();
    channels_received = true;
  }

  rx_lqi_update();

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_PACKET_RATE) {
    rx_lqi_update_from_fps(LQI_FPS);
  }

  return channels_received;
}

#endif
