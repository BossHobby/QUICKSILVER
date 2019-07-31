
#include "drv_cc2500.h"

#include "drv_time.h"
#include "drv_usb.h"
#include "project.h"
#include "util.h"

#ifdef RX_FRSKY

#define FRSKY_D_CHANNEL_COUNT 8
#define FRSKY_D_CHANNEL_SCALING (2.0f / 3)

#define FRSKY_RSSI_OFFSET 70
#define FRSKY_HOPTABLE_SIZE 47

#define SYNC_DELAY_MAX 9000
#define MAX_MISSING_PKT 100

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

typedef struct {
  uint8_t length;
  uint8_t tx_id[2];
  uint8_t counter;
  uint8_t channels[14];
  uint8_t crc[2];
} frsky_d_frame;

static uint8_t cal_data[255][3];
static uint8_t packet[128];
static uint8_t list_length;

static uint8_t protocol_state = STATE_DETECT;

static uint32_t missingPackets = 0;
static unsigned long timeoutUs = 50;
static unsigned long time_tuned_ms;

typedef struct {
  int8_t offset;
  uint8_t idx;
  uint8_t tx_id[2];
  uint8_t hop_data[50];
  uint8_t rx_num;
} bind_data;

static bind_data bind = {
    -7,
    45,
    {0xa7, 0x80},
    {
        0x2,
        0xe6,
        0xe1,
        0xdc,
        0xd7,
        0xd2,
        0xcd,
        0xc8,
        0xc3,
        0xbe,
        0xb9,
        0xb4,
        0xaf,
        0xaa,
        0xa5,
        0xa0,
        0x9b,
        0x96,
        0x91,
        0x8c,
        0x87,
        0x84,
        0x7d,
        0x78,
        0x73,
        0x6e,
        0x69,
        0x64,
        0x5f,
        0x5a,
        0x55,
        0x50,
        0x4b,
        0x46,
        0x41,
        0x3c,
        0x37,
        0x32,
        0x2d,
        0x28,
        0x23,
        0x1e,
        0x19,
        0x14,
        0xf,
        0xa,
        0x5,
        0x0,
        0x0,
        0,
    },
};

extern float rx[4];
extern char aux[AUXNUMBER];
extern char lastaux[AUXNUMBER];
extern char auxchange[AUXNUMBER];

int failsafe = 1; // It isn't safe if we haven't checked it!
int rxmode = RXMODE_BIND;
int rx_ready = 0;

void debug_timer_init() {
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}

uint32_t debug_timer_micros() {
  return DWT->CYCCNT / (SystemCoreClock / 1000000L);
}

uint32_t debug_timer_millis() {
  return (debug_timer_micros()) / 1000;
}

uint8_t frsky_extract_rssi(uint8_t rssi_raw) {
  if (rssi_raw >= 128) {
    // adapted to fit better to the original values... FIXME: find real formula
    // return (rssi_raw * 18)/32 - 82;
    return ((((uint16_t)rssi_raw) * 18) >> 5) - 82;
  } else {
    return ((((uint16_t)rssi_raw) * 18) >> 5) + 65;
  }
}

static uint8_t frsky_dectect(void) {
  const uint8_t chipPartNum = cc2500_read_reg(CC2500_PARTNUM | CC2500_READ_BURST); //CC2500 read registers chip part num
  const uint8_t chipVersion = cc2500_read_reg(CC2500_VERSION | CC2500_READ_BURST); //CC2500 read registers chip version
  if (chipPartNum == 0x80 && chipVersion == 0x03) {
    return 1;
  }
  return 0;
}

static void handle_overflows(void) {
  // fetch marc status
  uint8_t marc_state = cc2500_read_reg(CC2500_MARCSTATE) & 0x1F;
  if (marc_state == 0x11) {
    // flush rx buf
    cc2500_strobe(CC2500_SFRX);
  } else if (marc_state == 0x16) {
    // flush tx buf
    cc2500_strobe(CC2500_SFTX);
  }
}

static void set_address(uint8_t is_bind) {
  cc2500_strobe(CC2500_SIDLE);

  // freq offset
  cc2500_write_reg(CC2500_FSCTRL0, (uint8_t)bind.offset);

  // never automatically calibrate, po_timeout count = 64
  // no autotune as (we use our pll map)
  cc2500_write_reg(CC2500_MCSM0, 0x8);

  cc2500_write_reg(CC2500_ADDR, is_bind ? 0x03 : bind.tx_id[0]);

  // ADR_CHK, APPEND_STATUS, CRC_AUTOFLUSH
  cc2500_write_reg(CC2500_PKTCTRL1, 0x0D);

  // FOC_LIMIT 10, FOC_POST_K, FOC_PRE_K 10
  cc2500_write_reg(CC2500_FOCCFG, 0x16);

  delay(10);
}

static void set_channel(uint8_t channel) {
  cc2500_strobe(CC2500_SIDLE);

  cc2500_write_reg(CC2500_FSCAL3, cal_data[channel][0]);
  cc2500_write_reg(CC2500_FSCAL2, cal_data[channel][1]);
  cc2500_write_reg(CC2500_FSCAL1, cal_data[channel][2]);

  cc2500_write_reg(CC2500_CHANNR, channel);
}

static uint8_t next_channel(uint8_t skip) {
  static uint8_t channr = 0;

  channr += skip;
  while (channr >= list_length) {
    channr -= list_length;
  }

  set_channel(bind.hop_data[channr]);

  // FRSKY D only
  cc2500_strobe(CC2500_SFRX);

  return channr;
}

static uint8_t packet_size() {
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

static void init_tune_rx(void) {
  cc2500_write_reg(CC2500_FOCCFG, 0x14);

  time_tuned_ms = debug_timer_millis();
  bind.offset = -126;

  cc2500_write_reg(CC2500_FSCTRL0, (uint8_t)bind.offset);
  cc2500_write_reg(CC2500_PKTCTRL1, 0x0C);
  cc2500_write_reg(CC2500_MCSM0, 0x8);

  set_channel(0);

  cc2500_strobe(CC2500_SRX);
}

static uint8_t tune_rx() {
  if (bind.offset >= 126) {
    bind.offset = -126;
  }
  if ((debug_timer_millis() - time_tuned_ms) > 50) {
    time_tuned_ms = debug_timer_millis();
    bind.offset += 5;
    cc2500_write_reg(CC2500_FSCTRL0, (uint8_t)bind.offset);
  }

  uint8_t len = read_packet();
  if (len == 0) {
    return 0;
  }
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
  set_channel(0);
  cc2500_strobe(CC2500_SFRX);
  delay(20); // waiting flush FIFO

  cc2500_strobe(CC2500_SRX);
  list_length = 0;
  bind.idx = 0x05;
}

static uint8_t get_bind1() {
  // len|bind |tx
  // id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
  // Start by getting bind packet 0 and the txid
  uint8_t len = read_packet();
  if (len == 0) {
    return 0;
  }

  if (packet[len - 1] & 0x80) {
    if (packet[2] == 0x01) {
      if (packet[5] == 0x00) {

        bind.tx_id[0] = packet[3];
        bind.tx_id[1] = packet[4];
        for (uint8_t n = 0; n < 5; n++) {
          bind.hop_data[packet[5] + n] = packet[6 + n];
        }
        bind.rx_num = packet[12];
        return 1;
      }
    }
  }

  return 0;
}

static uint8_t get_bind2(uint8_t *packet) {
  if (bind.idx > 120) {
    return 1;
  }

  uint8_t len = read_packet();
  if (len == 0) {
    return 0;
  }

  if (packet[len - 1] & 0x80) {
    if (packet[2] == 0x01) {
      if ((packet[3] == bind.tx_id[0]) && (packet[4] == bind.tx_id[1])) {
        if (packet[5] == bind.idx) {
          for (uint8_t n = 0; n < 5; n++) {
            if (packet[6 + n] == packet[len - 3] || (packet[6 + n] == 0)) {
              if (bind.idx >= 0x2D) {
                list_length = packet[5] + n;
                return 1;
              }
            }
            bind.hop_data[packet[5] + n] = packet[6 + n];
          }

          bind.idx = bind.idx + 5;
          return 0;
        }
      }
    }
  }

  return 0;
}

static void frsky_d_set_rc_data() {
  uint16_t channels[FRSKY_D_CHANNEL_COUNT];

  channels[0] = (uint16_t)(((packet[10] & 0x0F) << 8 | packet[6]) * FRSKY_D_CHANNEL_SCALING);
  channels[1] = (uint16_t)(((packet[10] & 0xF0) << 4 | packet[7]) * FRSKY_D_CHANNEL_SCALING);
  channels[2] = (uint16_t)(((packet[11] & 0x0F) << 8 | packet[8]) * FRSKY_D_CHANNEL_SCALING);
  channels[3] = (uint16_t)(((packet[11] & 0xF0) << 4 | packet[9]) * FRSKY_D_CHANNEL_SCALING);
  channels[4] = (uint16_t)(((packet[16] & 0x0F) << 8 | packet[12]) * FRSKY_D_CHANNEL_SCALING);
  channels[5] = (uint16_t)(((packet[16] & 0xF0) << 4 | packet[13]) * FRSKY_D_CHANNEL_SCALING);
  channels[6] = (uint16_t)(((packet[17] & 0x0F) << 8 | packet[14]) * FRSKY_D_CHANNEL_SCALING);
  channels[7] = (uint16_t)(((packet[17] & 0xF0) << 4 | packet[15]) * FRSKY_D_CHANNEL_SCALING);

  for (int i = 0; i < FRSKY_D_CHANNEL_COUNT; i++) {
    if ((channels[i] < 800) || (channels[i] > 2200)) {
      failsafe = 1;
      return;
    }
  }

  // if we made it this far, data is ready
  rx_ready = 1;
  failsafe = 0;

  // AETR channel order
  rx[0] = channels[0] - 1499;
  rx[1] = channels[1] - 1499;
  rx[2] = channels[3] - 1499;
  rx[3] = channels[2] - 999;

  for (int i = 0; i < 3; i++) {
    rx[i] *= 1.f / 500.f;
  }
  rx[3] *= 1.f / 1000.f;

  if (rx[3] > 1)
    rx[3] = 1;
  if (rx[3] < 0)
    rx[3] = 0;

  if (aux[LEVELMODE]) {
    if (aux[RACEMODE] && !aux[HORIZON]) {
      if (ANGLE_EXPO_ROLL > 0.01)
        rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
      if (ACRO_EXPO_PITCH > 0.01)
        rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
      if (ANGLE_EXPO_YAW > 0.01)
        rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
    } else if (aux[HORIZON]) {
      if (ANGLE_EXPO_ROLL > 0.01)
        rx[0] = rcexpo(rx[0], ACRO_EXPO_ROLL);
      if (ACRO_EXPO_PITCH > 0.01)
        rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
      if (ANGLE_EXPO_YAW > 0.01)
        rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
    } else {
      if (ANGLE_EXPO_ROLL > 0.01)
        rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
      if (ANGLE_EXPO_PITCH > 0.01)
        rx[1] = rcexpo(rx[1], ANGLE_EXPO_PITCH);
      if (ANGLE_EXPO_YAW > 0.01)
        rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
    }
  } else {
    if (ACRO_EXPO_ROLL > 0.01)
      rx[0] = rcexpo(rx[0], ACRO_EXPO_ROLL);
    if (ACRO_EXPO_PITCH > 0.01)
      rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
    if (ACRO_EXPO_YAW > 0.01)
      rx[2] = rcexpo(rx[2], ACRO_EXPO_YAW);
  }

  //Here we have the AUX channels Silverware supports
  aux[CHAN_5] = (channels[4] > 1600) ? 1 : 0;
  aux[CHAN_6] = (channels[5] > 1600) ? 1 : 0;
  aux[CHAN_7] = (channels[6] > 1600) ? 1 : 0;
  aux[CHAN_8] = (channels[7] > 1600) ? 1 : 0;
  aux[CHAN_9] = 0;
  aux[CHAN_10] = 0;
}

static uint8_t frsky_d_handle_packet() {
  static unsigned long last_packet_received_time = 0;
  static unsigned long telemetryTimeUs;

  const unsigned long current_packet_received_time = debug_timer_micros();

  // make sure we never read the same packet twice by crc flag
  uint8_t has_data = 0;
  packet[19] = 0x00;

  handle_overflows();

  switch (protocol_state) {
  case STATE_STARTING:
    list_length = FRSKY_HOPTABLE_SIZE;

    cc2500_enter_rxmode();
    set_address(0);
    next_channel(1);
    cc2500_strobe(CC2500_SRX);

    protocol_state = STATE_UPDATE;
    break;
  case STATE_UPDATE:
    cc2500_enter_rxmode();
    last_packet_received_time = current_packet_received_time;
    protocol_state = STATE_DATA;
    // fallthrough
  case STATE_DATA: {
    uint8_t len = packet_size();
    if (len >= 20) {
      cc2500_read_fifo(packet, 20);
      if (packet[19] & 0x80) {
        missingPackets = 0;
        timeoutUs = 1;
        if (packet[0] == 0x11 && packet[1] == bind.tx_id[0] && packet[2] == bind.tx_id[1]) {
          const uint8_t channel = next_channel(1);
          frsky_d_frame *frame = (frsky_d_frame *)packet;

          if ((packet[3] % 4) == 2) {
            usb_serial_printf(" T:%d:%d ", frame->counter, channel);
            telemetryTimeUs = debug_timer_micros();
            protocol_state = STATE_TELEMETRY;
          } else {
            cc2500_strobe(CC2500_SRX);
            usb_serial_printf(" P:%d:%d ", frame->counter, channel);

            protocol_state = STATE_UPDATE;
          }

          has_data = 1;
          last_packet_received_time = current_packet_received_time;
        }
      }
    }

    if ((current_packet_received_time - last_packet_received_time) > (timeoutUs * SYNC_DELAY_MAX)) {
      cc2500_enter_rxmode();

      if (timeoutUs == 1) {
        if (missingPackets >= 2) {
          cc2500_switch_antenna();
        }

        if (missingPackets > MAX_MISSING_PKT) {
          usb_serial_printf("E\r\n\r\n");
          timeoutUs = 50;
          failsafe = 1;
        }

        missingPackets++;
        const uint8_t channel = next_channel(1);
        usb_serial_printf(" M:%d:%d ", missingPackets, channel);
      } else {
        next_channel(13);
      }

      cc2500_strobe(CC2500_SRX);
      protocol_state = STATE_UPDATE;
    }
    break;
  }

  case STATE_TELEMETRY:
    // telemetry has to be done 2000us after rx
    if ((debug_timer_micros() - telemetryTimeUs) >= 1380) {
      cc2500_strobe(CC2500_SIDLE);
      cc2500_set_power(6);
      cc2500_strobe(CC2500_SFRX);
      cc2500_enter_txmode();

      static uint8_t frame[20];
      frame[0] = 0x11; // length
      frame[1] = bind.tx_id[0];
      frame[2] = bind.tx_id[1];
      frame[3] = 100;
      frame[4] = 100;
      frame[5] = frsky_extract_rssi(packet[18]);
      frame[6] = 0;
      frame[7] = packet[4];
      cc2500_strobe(CC2500_SIDLE);
      cc2500_write_fifo(frame, frame[0] + 1);

      protocol_state = STATE_UPDATE;
      has_data = 0;
    }
    break;
  }
  return has_data;
}

void rx_init(void) {
  debug_timer_init();
  cc2500_init();

  // enable gdo0 on read
  cc2500_write_reg(CC2500_IOCFG0, 0x01);

  cc2500_write_reg(CC2500_MCSM0, 0x18);
  cc2500_write_reg(CC2500_MCSM1, 0x0C);

  // frsky d
  cc2500_write_reg(CC2500_PKTLEN, 0x19);   // max packet lenght of 25
  cc2500_write_reg(CC2500_PKTCTRL0, 0x05); // variable pkt lenth, enable crc
  cc2500_write_reg(CC2500_PKTCTRL1, 0x04); // only append status
  cc2500_write_reg(CC2500_PATABLE, 0xFF);  // full power

  cc2500_write_reg(CC2500_FSCTRL0, 0x00);
  cc2500_write_reg(CC2500_FSCTRL1, 0x08);

  // set base freq 2404 mhz
  cc2500_write_reg(CC2500_FREQ0, 0x27);
  cc2500_write_reg(CC2500_FREQ1, 0x76);
  cc2500_write_reg(CC2500_FREQ2, 0x5C);

  cc2500_write_reg(CC2500_MDMCFG4, 0xAA);
  cc2500_write_reg(CC2500_MDMCFG3, 0x39);
  cc2500_write_reg(CC2500_MDMCFG2, 0x11);
  cc2500_write_reg(CC2500_MDMCFG1, 0x23);
  cc2500_write_reg(CC2500_MDMCFG0, 0x7A);

  cc2500_write_reg(CC2500_DEVIATN, 0x42);

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

void checkrx() {
  switch (protocol_state) {
  case STATE_DETECT:
    usb_serial_print("FRSKY STATE_DETECT\r\n");
    if (frsky_dectect()) {
      protocol_state = STATE_INIT;
    }
    break;
  case STATE_INIT:
    usb_serial_print("FRSKY STATE_INIT\r\n");
    cc2500_enter_rxmode();
    cc2500_strobe(CC2500_SRX);
    //protocol_state = STATE_BIND_COMPLETE;
    protocol_state = STATE_BIND;
    break;
  case STATE_BIND:
    usb_serial_print("FRSKY STATE_BIND\r\n");
    init_tune_rx();
    protocol_state = STATE_BIND_TUNING;
    break;
  case STATE_BIND_TUNING:
    if (tune_rx(packet)) {
      usb_serial_print("FRSKY STATE_BIND_TUNING DONE\r\n");
      set_address(1);
      init_get_bind();
      protocol_state = STATE_BIND_BINDING1;
    }
    break;
  case STATE_BIND_BINDING1:
    if (get_bind1(packet)) {
      usb_serial_print("FRSKY STATE_BIND_BINDING1 DONE\r\n");
      protocol_state = STATE_BIND_BINDING2;
    }
    break;
  case STATE_BIND_BINDING2:
    if (get_bind2(packet)) {
      cc2500_strobe(CC2500_SIDLE);
      usb_serial_print("FRSKY STATE_BIND_BINDING2 DONE\r\n");
      protocol_state = STATE_BIND_COMPLETE;
    }
    break;
  case STATE_BIND_COMPLETE:
    usb_serial_print("FRSKY BOUND!\r\n");
    usb_serial_printf("idx: %d\r\n", bind.idx);
    usb_serial_printf("offset: %d\r\n", bind.offset);
    usb_serial_printf("tx_id: 0x%x%x\r\n", bind.tx_id[0], bind.tx_id[1]);
    usb_serial_printf("hop_data:");
    for (uint32_t i = 0; i < 50; i++) {
      usb_serial_printf(" %x, ", bind.hop_data[i]);
    }
    usb_serial_print("\r\n");

    rxmode = RXMODE_NORMAL;
    protocol_state = STATE_STARTING;
    break;
  default:
    if (frsky_d_handle_packet()) {
      frsky_d_set_rc_data();
    }
    break;
  }
}

#endif