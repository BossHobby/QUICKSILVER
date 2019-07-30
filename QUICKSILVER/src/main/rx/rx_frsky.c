
#include "drv_cc2500.h"

#include "drv_time.h"
#include "drv_usb.h"
#include "project.h"

#ifdef RX_FRSKY

enum {
  STATE_DETECT = 0,
  STATE_INIT,
  STATE_BIND,
  STATE_BIND_TUNING,
  STATE_BIND_BINDING1,
  STATE_BIND_BINDING2,
  STATE_BIND_COMPLETE,
  STATE_STARTING,
  STATE_UPDATE,
  STATE_DATA,
  STATE_TELEMETRY,
  STATE_RESUME,
};

static uint8_t cal_data[255][3];
static uint8_t protocol_state = STATE_DETECT;
static unsigned long timeTunedMs;
static uint8_t packet[128];
static uint8_t listLength;

static int8_t bind_offset;
static uint8_t bindIdx;
static uint8_t bindTxId[2];
static uint8_t bindHopData[50];
static uint8_t rxNum;

int failsafe = 1; // It isn't safe if we haven't checked it!
int rxmode = RXMODE_BIND;
int rx_ready = 0;

static inline unsigned long millis() {
  return gettime() / 1000;
}

void rx_init(void) {
  cc2500_init();

  cc2500_write_reg(CC2500_IOCFG0, 0x01);
  cc2500_write_reg(CC2500_MCSM1, 0x0C);
  cc2500_write_reg(CC2500_MCSM0, 0x18);
  cc2500_write_reg(CC2500_PKTCTRL1, 0x04);
  cc2500_write_reg(CC2500_PATABLE, 0xFF);
  cc2500_write_reg(CC2500_FSCTRL0, 0x00);
  cc2500_write_reg(CC2500_FREQ2, 0x5C);
  cc2500_write_reg(CC2500_FREQ1, 0x76);
  cc2500_write_reg(CC2500_FREQ0, 0x27);
  cc2500_write_reg(CC2500_MDMCFG1, 0x23);
  cc2500_write_reg(CC2500_MDMCFG0, 0x7A);
  cc2500_write_reg(CC2500_FOCCFG, 0x16);
  cc2500_write_reg(CC2500_BSCFG, 0x6C);
  cc2500_write_reg(CC2500_AGCCTRL2, 0x03);
  cc2500_write_reg(CC2500_AGCCTRL1, 0x40);
  cc2500_write_reg(CC2500_AGCCTRL0, 0x91);
  cc2500_write_reg(CC2500_FREND1, 0x56);
  cc2500_write_reg(CC2500_FREND0, 0x10);
  cc2500_write_reg(CC2500_FSCAL3, 0xA9);
  cc2500_write_reg(CC2500_FSCAL2, 0x0A);
  cc2500_write_reg(CC2500_FSCAL1, 0x00);
  cc2500_write_reg(CC2500_FSCAL0, 0x11);
  cc2500_write_reg(CC2500_FSTEST, 0x59);
  cc2500_write_reg(CC2500_TEST2, 0x88);
  cc2500_write_reg(CC2500_TEST1, 0x31);
  cc2500_write_reg(CC2500_TEST0, 0x0B);
  cc2500_write_reg(CC2500_FIFOTHR, 0x07);
  cc2500_write_reg(CC2500_ADDR, 0x00);

  // frsky d
  cc2500_write_reg(CC2500_PKTLEN, 0x19);
  cc2500_write_reg(CC2500_PKTCTRL0, 0x05);
  cc2500_write_reg(CC2500_FSCTRL1, 0x08);
  cc2500_write_reg(CC2500_MDMCFG4, 0xAA);
  cc2500_write_reg(CC2500_MDMCFG3, 0x39);
  cc2500_write_reg(CC2500_MDMCFG2, 0x11);
  cc2500_write_reg(CC2500_DEVIATN, 0x42);

  // frsky x
  /* 
  cc2500_write_reg(CC2500_PKTLEN, 0x1E);
  cc2500_write_reg(CC2500_PKTCTRL0, 0x01);
  cc2500_write_reg(CC2500_FSCTRL1, 0x0A);
  cc2500_write_reg(CC2500_MDMCFG4, 0x7B);
  cc2500_write_reg(CC2500_MDMCFG3, 0x61);
  cc2500_write_reg(CC2500_MDMCFG2, 0x13);
  cc2500_write_reg(CC2500_DEVIATN, 0x51);
  */

  //calibrate all channels
  for (uint32_t c = 0; c < 0xFF; c++) {
    cc2500_strobe(CC2500_SIDLE);
    cc2500_write_reg(CC2500_CHANNR, c);
    cc2500_strobe(CC2500_SCAL);

    delay(900);

    cal_data[c][0] = cc2500_read_reg(CC2500_FSCAL3);
    cal_data[c][1] = cc2500_read_reg(CC2500_FSCAL2);
    cal_data[c][2] = cc2500_read_reg(CC2500_FSCAL1);
  }
}

static void init_tune_rx(void) {
  cc2500_write_reg(CC2500_FOCCFG, 0x14);
  timeTunedMs = millis();
  bind_offset = -126;
  cc2500_write_reg(CC2500_FSCTRL0, (uint8_t)bind_offset);
  cc2500_write_reg(CC2500_PKTCTRL1, 0x0C);
  cc2500_write_reg(CC2500_MCSM0, 0x8);

  cc2500_strobe(CC2500_SIDLE);
  cc2500_write_reg(CC2500_FSCAL3, cal_data[0][0]);
  cc2500_write_reg(CC2500_FSCAL2, cal_data[0][1]);
  cc2500_write_reg(CC2500_FSCAL1, cal_data[0][2]);
  cc2500_write_reg(CC2500_CHANNR, 0);
  cc2500_strobe(CC2500_SFRX);
  cc2500_strobe(CC2500_SRX);
}

static uint8_t tune_rx() {
  if (bind_offset >= 126) {
    bind_offset = -126;
  }
  if ((millis() - timeTunedMs) > 50) {
    timeTunedMs = millis();
    bind_offset += 5;
    cc2500_write_reg(CC2500_FSCTRL0, (uint8_t)bind_offset);
  }

  if (cc2500_read_gdo0() == 0) {
    return 0;
  }
  usb_serial_print("tune_rx got packet\r\n");

  uint8_t len = cc2500_read_reg(CC2500_RXBYTES | CC2500_READ_BURST) & 0x7F;
  if (len == 0) {
    return 0;
  }

  cc2500_read_fifo(packet, len);
  if (packet[len - 1] & 0x80) {
    if (packet[2] == 0x01) {
      uint8_t lqi = packet[len - 1] & 0x7F;
      // higher lqi represent better link quality
      if (lqi > 50) {
        return 1;
      }
    }
  }
  return 0;
}

static void init_get_bind(void) {
  cc2500_strobe(CC2500_SIDLE);
  cc2500_write_reg(CC2500_FSCAL3, cal_data[0][0]);
  cc2500_write_reg(CC2500_FSCAL2, cal_data[0][1]);
  cc2500_write_reg(CC2500_FSCAL1, cal_data[0][2]);
  cc2500_write_reg(CC2500_CHANNR, 0);
  cc2500_strobe(CC2500_SFRX);
  delay(20); // waiting flush FIFO

  cc2500_strobe(CC2500_SRX);
  listLength = 0;
  bindIdx = 0x05;
}

void initialise_data(uint8_t adr) {
  cc2500_write_reg(CC2500_FSCTRL0, (uint8_t)bind_offset);
  cc2500_write_reg(CC2500_MCSM0, 0x8);
  cc2500_write_reg(CC2500_ADDR, adr ? 0x03 : bindTxId[0]);
  cc2500_write_reg(CC2500_PKTCTRL1, 0x0D);
  cc2500_write_reg(CC2500_FOCCFG, 0x16);
  delay(10);
}

static uint8_t get_bind1() {
  // len|bind |tx
  // id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
  // Start by getting bind packet 0 and the txid
  if (cc2500_read_gdo0() == 0) {
    return 0;
  }

  uint8_t len = cc2500_read_reg(CC2500_RXBYTES | CC2500_READ_BURST) & 0x7F;
  if (len == 0) {
    return 0;
  }

  cc2500_read_fifo(packet, len);
  if (packet[len - 1] & 0x80) {
    if (packet[2] == 0x01) {
      if (packet[5] == 0x00) {

        bindTxId[0] = packet[3];
        bindTxId[1] = packet[4];
        for (uint8_t n = 0; n < 5; n++) {
          bindHopData[packet[5] + n] = packet[6 + n];
        }
        rxNum = packet[12];

        return 1;
      }
    }
  }

  return 0;
}

static uint8_t get_bind2(uint8_t *packet) {
  if (bindIdx > 120) {
    return 1;
  }

  if (!cc2500_read_gdo0()) {
    return 0;
  }

  uint8_t len = cc2500_read_reg(CC2500_RXBYTES | CC2500_READ_BURST) & 0x7F;
  if (!len) {
    return 0;
  }

  cc2500_read_fifo(packet, len);
  if (packet[len - 1] & 0x80) {
    if (packet[2] == 0x01) {
      if ((packet[3] == bindTxId[0]) && (packet[4] == bindTxId[1])) {
        if (packet[5] == bindIdx) {
#if defined(DJTS)
          if (packet[5] == 0x2D) {
            for (uint8_t i = 0; i < 2; i++) {
              bindHopData[packet[5] + i] = packet[6 + i];
            }
            listLength = 47;
            return 1;
          }
#endif

          for (uint8_t n = 0; n < 5; n++) {
            if (packet[6 + n] == packet[len - 3] || (packet[6 + n] == 0)) {
              if (bindIdx >= 0x2D) {
                listLength = packet[5] + n;

                return 1;
              }
            }
            bindHopData[packet[5] + n] = packet[6 + n];
          }

          bindIdx = bindIdx + 5;
          return 0;
        }
      }
    }
  }

  return 0;
}

static uint8_t frsky_dectect(void) {
  const uint8_t chipPartNum = cc2500_read_reg(CC2500_PARTNUM | CC2500_READ_BURST); //CC2500 read registers chip part num
  const uint8_t chipVersion = cc2500_read_reg(CC2500_VERSION | CC2500_READ_BURST); //CC2500 read registers chip version
  if (chipPartNum == 0x80 && chipVersion == 0x03) {
    return 1;
  }
  return 0;
}

void checkrx() {
  switch (protocol_state) {
  case STATE_DETECT:
    if (frsky_dectect()) {
      protocol_state = STATE_INIT;
    }
    break;
  case STATE_INIT:
    protocol_state = STATE_BIND;
    break;
  case STATE_BIND:
    init_tune_rx();
    protocol_state = STATE_BIND_TUNING;
    break;
  case STATE_BIND_TUNING:
    if (tune_rx(packet)) {
      init_get_bind();
      initialise_data(1);

      protocol_state = STATE_BIND_BINDING1;
    }
    break;
  case STATE_BIND_BINDING1:
    if (get_bind1(packet)) {
      protocol_state = STATE_BIND_BINDING2;
    }
    break;
  case STATE_BIND_BINDING2:
    if (get_bind2(packet)) {
      cc2500_strobe(CC2500_SIDLE);

      protocol_state = STATE_BIND_COMPLETE;
    }
    break;
  case STATE_BIND_COMPLETE:
    rxmode = RXMODE_NORMAL;
    protocol_state = STATE_STARTING;
    break;
  default:
    usb_serial_printf("STATE_STARTING %d %d %d\r\n", rxNum, bindTxId[0], bindTxId[1]);
    break;
  }
}

#endif