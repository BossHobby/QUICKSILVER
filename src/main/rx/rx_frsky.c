#include "rx_frsky.h"

#include "control.h"
#include "drv_spi_cc2500.h"
#include "drv_time.h"
#include "flash.h"
#include "profile.h"
#include "rx_spi.h"
#include "usb_configurator.h"
#include "util.h"

#if defined(RX_FRSKY) && defined(USE_CC2500)

#define FRSKY_HOPTABLE_SIZE 47

uint8_t packet[128];
uint8_t protocol_state = FRSKY_STATE_DETECT;
uint8_t list_length = FRSKY_HOPTABLE_SIZE;

static uint8_t cal_data[255][3];

static uint32_t time_tuned_ms;

uint16_t frsky_extract_rssi(uint8_t rssi_raw) {
  if (rssi_raw >= 128) {
    // adapted to fit better to the original values... FIXME: find real formula
    // return (rssi_raw * 18)/32 - 82;
    return ((((uint16_t)rssi_raw) * 18) >> 5) - 82;
  } else {
    return ((((uint16_t)rssi_raw) * 18) >> 5) + 65;
  }
}

static uint8_t frsky_dectect() {
  const uint8_t chipPartNum = cc2500_read_reg(CC2500_PARTNUM | CC2500_READ_BURST); //CC2500 read registers chip part num
  const uint8_t chipVersion = cc2500_read_reg(CC2500_VERSION | CC2500_READ_BURST); //CC2500 read registers chip version
  if (chipPartNum == 0x80 && chipVersion == 0x03) {
    return 1;
  }
  return 0;
}

void handle_overflows() {
  // fetch marc status
  uint8_t marc_state = cc2500_read_reg(CC2500_MARCSTATE) & 0x1F;
  if (marc_state == 0x11) {
    // flush rx buf
    quic_debugf("FRSKY: RX overflow");
    cc2500_strobe(CC2500_SFRX);
  } else if (marc_state == 0x16) {
    // flush tx buf
    quic_debugf("FRSKY: TX overflow");
    cc2500_strobe(CC2500_SFTX);
  }
}

void set_address(uint8_t is_bind) {
  cc2500_strobe(CC2500_SIDLE);

  // freq offset
  cc2500_write_reg(CC2500_FSCTRL0, (uint8_t)bind_storage.frsky.offset);

  // never automatically calibrate, po_timeout count = 64
  // no autotune as (we use our pll map)
  cc2500_write_reg(CC2500_MCSM0, 0x8);

  cc2500_write_reg(CC2500_ADDR, is_bind ? 0x03 : bind_storage.frsky.tx_id[0]);

  // ADR_CHK, APPEND_STATUS, CRC_AUTOFLUSH
  cc2500_write_reg(CC2500_PKTCTRL1, 0x0D);

  // FOC_LIMIT 10, FOC_POST_K, FOC_PRE_K 10
  cc2500_write_reg(CC2500_FOCCFG, 0x16);

  time_delay_us(10);
}

static void set_channel(uint8_t channel) {
  cc2500_strobe(CC2500_SIDLE);

  cc2500_write_reg(CC2500_FSCAL3, cal_data[channel][0]);
  cc2500_write_reg(CC2500_FSCAL2, cal_data[channel][1]);
  cc2500_write_reg(CC2500_FSCAL1, cal_data[channel][2]);

  cc2500_write_reg(CC2500_CHANNR, channel);
}

uint8_t next_channel(uint8_t skip) {
  static uint8_t channr = 0;

  channr += skip;
  while (channr >= list_length) {
    channr -= list_length;
  }

  set_channel(bind_storage.frsky.hop_data[channr]);

#ifdef RX_FRSKY_D8
  // FRSKY D only
  cc2500_strobe(CC2500_SFRX);
#endif

  return channr;
}

uint8_t packet_size() {
  if (cc2500_read_gdo0() == 0) {
    return 0;
  }

  // there is a bug in the cc2500
  // see p3 http:// www.ti.com/lit/er/swrz002e/swrz002e.pdf
  // workaround: read len register very quickly twice:

  // try this 10 times befor giving up:
  for (uint8_t i = 0; i < 10; i++) {
    uint8_t len1 = cc2500_read_reg(CC2500_RXBYTES | CC2500_READ_BURST) & 0x7F;
    uint8_t len2 = cc2500_read_reg(CC2500_RXBYTES | CC2500_READ_BURST) & 0x7F;

    // valid len found?
    if (len1 == len2) {
      return len1;
    }
  }

  return 0;
}

static uint8_t read_packet() {
  uint8_t len = packet_size();
  if (len == 0) {
    return 0;
  }
  cc2500_read_fifo(packet, len);
  return len;
}

static void init_tune_rx() {
  cc2500_write_reg(CC2500_FOCCFG, 0x14);

  time_tuned_ms = time_millis();
  bind_storage.frsky.offset = -126;

  cc2500_write_reg(CC2500_FSCTRL0, (uint8_t)bind_storage.frsky.offset);
  cc2500_write_reg(CC2500_PKTCTRL1, 0x0C);
  cc2500_write_reg(CC2500_MCSM0, 0x8);

  set_channel(0);
  cc2500_strobe(CC2500_SFRX);
  cc2500_strobe(CC2500_SRX);
}

static void tune_rx_offset(int8_t offset) {
  bind_storage.frsky.offset = offset;
  if (bind_storage.frsky.offset >= 126) {
    bind_storage.frsky.offset = -126;
  }

  cc2500_write_reg(CC2500_FSCTRL0, (uint8_t)bind_storage.frsky.offset);

  time_tuned_ms = time_millis();
}

static uint8_t tune_delay = 50;

static uint8_t tune_rx() {
  if ((time_millis() - time_tuned_ms) > tune_delay) {
    // switch to fine tuning after first hit
    tune_rx_offset(bind_storage.frsky.offset + 5);
    tune_delay = 50;
  }

  uint8_t len = read_packet();
  if (len == 0) {
    return 0;
  }
  if (len > 35) {
    cc2500_strobe(CC2500_SFRX);
    return 0;
  }

  if (packet[len - 1] & 0x80) {
    if (packet[2] == 0x01) {
      uint8_t lqi = packet[len - 1] & 0x7F;
      packet[len - 1] = 0x0;

      // higher lqi represent better link quality
      if (lqi > 50) {
        return 1;
      }
    }
  }
  return 0;
}

static void init_get_bind() {
  set_channel(0);
  cc2500_strobe(CC2500_SFRX);
  time_delay_us(20); // waiting flush FIFO

  cc2500_strobe(CC2500_SRX);
  list_length = 0;
  bind_storage.frsky.idx = 0x05;
}

static uint8_t get_bind1() {
  // len|bind |tx
  // id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
  // Start by getting bind packet 0 and the txid
  uint8_t len = read_packet();
  if (len == 0) {
    return 0;
  }
  if (len > 35) {
    cc2500_strobe(CC2500_SFRX);
    return 0;
  }

  if (packet[len - 1] & 0x80) {
    if (packet[2] == 0x01) {
      if (packet[5] == 0x00) {

        bind_storage.frsky.tx_id[0] = packet[3];
        bind_storage.frsky.tx_id[1] = packet[4];
        for (uint8_t n = 0; n < 5; n++) {
          bind_storage.frsky.hop_data[packet[5] + n] = packet[6 + n];
        }
        bind_storage.frsky.rx_num = packet[12];
        return 1;
      }
    }
  }

  return 0;
}

static uint8_t get_bind2(uint8_t *packet) {
  if (bind_storage.frsky.idx > 120) {
    return 1;
  }

  uint8_t len = read_packet();
  if (len == 0) {
    return 0;
  }
  if (len > 35) {
    cc2500_strobe(CC2500_SFRX);
    return 0;
  }

  if (packet[len - 1] & 0x80) {
    if (packet[2] == 0x01) {
      if ((packet[3] == bind_storage.frsky.tx_id[0]) && (packet[4] == bind_storage.frsky.tx_id[1])) {
        if (packet[5] == bind_storage.frsky.idx) {
          for (uint8_t n = 0; n < 5; n++) {
            if (packet[6 + n] == packet[len - 3] || (packet[6 + n] == 0)) {
              if (bind_storage.frsky.idx >= 0x2D) {
                list_length = packet[5] + n;
                return 1;
              }
            }
            bind_storage.frsky.hop_data[packet[5] + n] = packet[6 + n];
          }

          bind_storage.frsky.idx = bind_storage.frsky.idx + 5;
          return 0;
        }
      }
    }
  }

  return 0;
}

void calibrate_channels() {
  //calibrate all channels
  for (uint32_t c = 0; c < 0xFF; c++) {
    cc2500_strobe(CC2500_SIDLE);
    cc2500_write_reg(CC2500_CHANNR, c);

    cc2500_strobe(CC2500_SCAL);
    while (cc2500_read_reg(CC2500_MARCSTATE) != 0x01)
      ;

    cal_data[c][0] = cc2500_read_reg(CC2500_FSCAL3);
    cal_data[c][1] = cc2500_read_reg(CC2500_FSCAL2);
    cal_data[c][2] = cc2500_read_reg(CC2500_FSCAL1);
  }

  cc2500_strobe(CC2500_SIDLE);
}

void frsky_handle_bind() {
  switch (protocol_state) {
  case FRSKY_STATE_DETECT:
    if (frsky_dectect()) {
      protocol_state = FRSKY_STATE_INIT;
    }
    break;
  case FRSKY_STATE_INIT:
    cc2500_enter_rxmode();
    cc2500_strobe(CC2500_SRX);
    if (bind_storage.frsky.offset == 0x0 &&
        bind_storage.frsky.idx == 0x0 &&
        bind_storage.frsky.tx_id[0] == 0x0 &&
        bind_storage.frsky.hop_data[3] == 0x0) {
      protocol_state = FRSKY_STATE_BIND;
      state.rx_status = RX_SPI_STATUS_BINDING;
    } else {
      protocol_state = FRSKY_STATE_BIND_COMPLETE;
    }
    break;
  case FRSKY_STATE_BIND:
    init_tune_rx();
    handle_overflows();
    protocol_state = FRSKY_STATE_BIND_TUNING;
    break;
  case FRSKY_STATE_BIND_TUNING: {
    static uint8_t tune_samples = 0;

    static int8_t min_offset = 127;
    static int8_t max_offset = -127;

    const int8_t current_offset = bind_storage.frsky.offset;

    handle_overflows();

    if (tune_rx()) {
      if (current_offset < min_offset) {
        min_offset = current_offset;
        tune_delay = 0;
      } else if (current_offset > max_offset) {
        max_offset = current_offset;
        tune_delay = 0;
      } else {
        tune_samples++;
      }
      quic_debugf("FRSKY: tuned %d %d - %d", current_offset, min_offset, max_offset);
    }
    if (tune_samples == 1) {
      tune_rx_offset((max_offset + min_offset) / 2 - 1);

      quic_debugf("FRSKY: tuned offset %d", bind_storage.frsky.offset);
      set_address(1);
      init_get_bind();
      protocol_state = FRSKY_STATE_BIND_BINDING1;
    }
    break;
  }
  case FRSKY_STATE_BIND_BINDING1:
    handle_overflows();
    if (get_bind1(packet)) {
      protocol_state = FRSKY_STATE_BIND_BINDING2;
    }
    break;
  case FRSKY_STATE_BIND_BINDING2:
    handle_overflows();
    if (get_bind2(packet)) {
      protocol_state = FRSKY_STATE_BIND_COMPLETE;
    }
    break;
  case FRSKY_STATE_BIND_COMPLETE:
    state.rx_status = RX_SPI_STATUS_BOUND;
    quic_debugf("FRSKY: bound rx_num %d", bind_storage.frsky.rx_num);
    cc2500_strobe(CC2500_SIDLE);
    flags.rx_mode = RXMODE_NORMAL;
    protocol_state = FRSKY_STATE_STARTING;
    break;
  default:
    break;
  }
}

#endif
