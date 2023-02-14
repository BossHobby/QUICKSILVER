#include "rx/bayang.h"

#include <stdio.h>

#include "core/failloop.h"
#include "core/flash.h"
#include "core/profile.h"
#include "driver/spi_soft.h"
#include "driver/spi_xn297.h"
#include "driver/time.h"
#include "flight/control.h"
#include "project.h"
#include "util/util.h"

#define RX_MODE_NORMAL RXMODE_NORMAL
#define RX_MODE_BIND RXMODE_BIND
// radio settings

// packet period in uS
#define PACKET_PERIOD 3000
#define PACKET_PERIOD_TELEMETRY 5000

// was 250 ( uS )
#define PACKET_OFFSET 0

// how many times to hop ahead if no reception
#define HOPPING_NUMBER 4

#define RSSI_EXP 0.9f

#ifdef RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND

char lasttrim[4];

int rx_bind_load = 0;

int rf_chan = 0;
uint16_t bind_safety = 0;

uint32_t autobindtime = 0;
int autobind_inhibit = 0;
int packet_period = PACKET_PERIOD;

extern profile_t profile;

void writeregs(uint8_t data[], uint8_t size) {
  spi_cson();
  for (uint8_t i = 0; i < size; i++) {
    spi_sendbyte(data[i]);
  }
  spi_csoff();
}

void rx_protocol_init() {

  // always on (AUX_CHANNEL_ON) channel set 1
  state.aux[AUX_CHANNEL_MAX - 2] = 1;
  // always off (AUX_CHANNEL_OFF) channel set 0
  state.aux[AUX_CHANNEL_MAX - 1] = 0;

#ifdef RADIO_XN297L

#ifndef TX_POWER
#define TX_POWER 7
#endif

  // Gauss filter amplitude - lowest
  static uint8_t demodcal[2] = {0x39, 0b00000001};
  writeregs(demodcal, sizeof(demodcal));

  // powerup defaults
  // static uint8_t rfcal2[7] = { 0x3a , 0x45 , 0x21 , 0xef , 0xac , 0x3a , 0x50};
  // writeregs( rfcal2 , sizeof(rfcal2) );

  static uint8_t rfcal2[7] = {0x3a, 0x45, 0x21, 0xef, 0x2c, 0x5a, 0x50};
  writeregs(rfcal2, sizeof(rfcal2));

  static uint8_t regs_1f[6] = {0x3f, 0x0a, 0x6d, 0x67, 0x9c, 0x46};
  writeregs(regs_1f, sizeof(regs_1f));

  static uint8_t regs_1e[4] = {0x3e, 0xf6, 0x37, 0x5d};
  writeregs(regs_1e, sizeof(regs_1e));

#define XN_TO_RX 0b10001111
#define XN_TO_TX 0b10000010
#define XN_POWER (0b00000001 | ((TX_POWER & 3) << 1))
#endif

#ifdef RADIO_XN297
  static uint8_t bbcal[6] = {0x3f, 0x4c, 0x84, 0x6F, 0x9c, 0x20};
  writeregs(bbcal, sizeof(bbcal));
  // new values
  static uint8_t rfcal[8] =
      {0x3e, 0xc9, 0x9a, 0xA0, 0x61, 0xbb, 0xab, 0x9c};
  writeregs(rfcal, sizeof(rfcal));

  // 0xa7 0x03
  static uint8_t demodcal[6] =
      {0x39, 0x0b, 0xdf, 0xc4, 0b00100111, 0b00000000};
  writeregs(demodcal, sizeof(demodcal));

#ifndef TX_POWER
#define TX_POWER 3
#endif

#define XN_TO_RX 0b00001111
#define XN_TO_TX 0b00000010
#define XN_POWER ((0b00000001 | ((TX_POWER & 3) << 1)) | 0xa0) // | 0x80 rssi  // | 0xa0 filtered rssi
#endif

  time_delay_us(100);

  // write rx address " 0 0 0 0 0 "

  static uint8_t rxaddr[6] = {0x2a, 0, 0, 0, 0, 0};
  writeregs(rxaddr, sizeof(rxaddr));

  xn_writereg(EN_AA, 0);           // aa disabled
  xn_writereg(EN_RXADDR, 1);       // pipe 0 only
  xn_writereg(RF_SETUP, XN_POWER); // power / data rate / lna
  xn_writereg(RX_PW_P0, 15);       // payload size
  xn_writereg(SETUP_RETR, 0);      // no retransmissions ( redundant?)
  xn_writereg(SETUP_AW, 3);        // address size (5 bytes)
  xn_command(FLUSH_RX);
  xn_writereg(RF_CH, 0); // bind on channel 0

#ifdef RADIO_XN297L
  xn_writereg(0x1d, 0b00111000); // 64 bit payload , software ce
  spi_cson();
  spi_sendbyte(0xFD); // internal CE high command
  spi_sendbyte(0);    // required for above
  spi_csoff();
#endif

#ifdef RADIO_XN297
  xn_writereg(0x1d, 0b00011000); // 64 bit payload , software ce
#endif

  xn_writereg(0, XN_TO_RX); // power up, crc enabled, rx mode

#ifdef RADIO_CHECK
  int rxcheck = xn_readreg(0x0f); // rx address pipe 5
  // should be 0xc6
  if (rxcheck != 0xc6)
    failloop(FAILLOOP_RADIO);
#endif

  if (rx_bind_load) {
    uint8_t rxaddr_regs[6] = {
        0x2a,
    };
    for (int i = 1; i < 6; i++) {
      rxaddr_regs[i] = bind_storage.bayang.rxaddress[i - 1];
    }
    // write new rx address
    writeregs(rxaddr_regs, sizeof(rxaddr_regs));
    rxaddr_regs[0] = 0x30; // tx register ( write ) number

    // write new tx address
    writeregs(rxaddr_regs, sizeof(rxaddr_regs));

    xn_writereg(0x25, bind_storage.bayang.rfchannel[rf_chan]); // Set channel frequency
    flags.rx_mode = RX_MODE_NORMAL;

    if (bind_storage.bayang.telemetry_enabled)
      packet_period = PACKET_PERIOD_TELEMETRY;
  } else {
    autobind_inhibit = 1;
  }
}

// #define RXDEBUG

#ifdef RXDEBUG
uint32_t packettime;
int channelcount[4];
int failcount;
int skipstats[12];
int afterskip[12];

#warning "RX debug enabled"
#endif

int packetrx;
int packetpersecond;

void send_telemetry();
void nextchannel();

int loopcounter = 0;
uint32_t send_time;
int telemetry_send = 0;
int oldchan = 0;

#define TELEMETRY_TIMEOUT 10000

void beacon_sequence() {
  static int beacon_seq_state = 0;

  switch (beacon_seq_state) {
  case 0:
    // send data
    telemetry_send = 1;
    send_telemetry();
    beacon_seq_state++;
    break;

  case 1:
    // wait for data to finish transmitting
    if ((xn_readreg(0x17) & 0b00010000)) {
      xn_writereg(0, XN_TO_RX);
      beacon_seq_state = 0;
      telemetry_send = 0;
      nextchannel();
    } else { // if it takes too long we get rid of it
      if (time_micros() - send_time > TELEMETRY_TIMEOUT) {
        xn_command(FLUSH_TX);
        xn_writereg(0, XN_TO_RX);
        beacon_seq_state = 0;
        telemetry_send = 0;
      }
    }
    break;

  default:
    beacon_seq_state = 0;
    break;
  }
}

void send_telemetry() {

  int txdata[15];
  for (int i = 0; i < 15; i++)
    txdata[i] = i;
  txdata[0] = 133;
  txdata[1] = flags.lowbatt;

  int vbatt = state.vbat_filtered * 100;
  // battery volt filtered
  txdata[3] = (vbatt >> 8) & 0xff;
  txdata[4] = vbatt & 0xff;

  vbatt = state.vbat_compensated * 100;
  // battery volt compensated
  txdata[5] = (vbatt >> 8) & 0xff;
  txdata[6] = vbatt & 0xff;

  int temp = packetpersecond / 2;
  if (temp > 255)
    temp = 255;

  txdata[7] = temp; // rx strenght

  if (flags.lowbatt)
    txdata[3] |= (1 << 3);

  int sum = 0;
  for (int i = 0; i < 14; i++) {
    sum += txdata[i];
  }

  txdata[14] = sum;

  xn_command(FLUSH_TX);

  xn_writereg(0, XN_TO_TX);

  xn_writepayload(txdata, 15);

  send_time = time_micros();

  return;
}

static char checkpacket() {
  int status = xn_readreg(7);

#if 1
  if (status & (1 << MASK_RX_DR)) {         // RX packet received
    xn_writereg(STATUS, (1 << MASK_RX_DR)); // rx clear bit
    return 1;
  } else {
#ifdef RADIO_XN297
    if (profile.receiver.aux[AUX_RSSI] <= AUX_CHANNEL_11) { // rssi set to actual rssi register value
      state.rx_rssi = 10.0f * ((xn_readreg(9)) & 0x0f);
      if (state.rx_rssi > 100.0f)
        state.rx_rssi = 100.0f;
      if (state.rx_rssi < 0.0f)
        state.rx_rssi = 0.0f;
    }
#endif
  }
#else
  if ((status & 0b00001110) != 0b00001110) { // rx fifo not empty
    return 2;
  }
#endif

  return 0;
}

int rxdata[15];

float packettodata(int *data) {
  return (((data[0] & 0x0003) * 256 + data[1]) - 512) * 0.001953125;
}

static int decodepacket() {
  if (rxdata[0] == 165) {
    int sum = 0;
    for (int i = 0; i < 14; i++) {
      sum += rxdata[i];
    }
    if ((sum & 0xFF) == rxdata[14]) {
      state.rx.axis[0] = packettodata(&rxdata[4]);
      state.rx.axis[1] = packettodata(&rxdata[6]);
      state.rx.axis[2] = packettodata(&rxdata[10]);
      // throttle
      state.rx.axis[3] =
          ((rxdata[8] & 0x0003) * 256 +
           rxdata[9]) *
          0.000976562f;

      state.aux[CH_INV] = (rxdata[3] & 0x80) ? 1 : 0; // inverted flag

      state.aux[CH_VID] = (rxdata[2] & 0x10) ? 1 : 0;

      state.aux[CH_PIC] = (rxdata[2] & 0x20) ? 1 : 0;

      // state.aux[CH_TO] = (rxdata[3] & 0x20) ? 1 : 0; // take off flag

      state.aux[CH_EMG] = (rxdata[3] & 0x04) ? 1 : 0; // emg stop flag

      state.aux[CH_FLIP] = (rxdata[2] & 0x08) ? 1 : 0;

      // state.aux[CH_EXPERT] = (rxdata[1] == 0xfa) ? 1 : 0;

      state.aux[CH_HEADFREE] = (rxdata[2] & 0x02) ? 1 : 0;

      state.aux[CH_RTH] = (rxdata[2] & 0x01) ? 1 : 0; // rth channel

      return 1; // valid packet
    }
    return 0; // sum fail
  }
  return 0; // first byte different
}

void nextchannel() {
  rf_chan++;
  rf_chan &= 3; // same as %4
  xn_writereg(0x25, bind_storage.bayang.rfchannel[rf_chan]);
}

uint32_t lastrxtime;
uint32_t failsafetime;
uint32_t secondtimer;

uint32_t skipchannel = 0;
int lastrxchan;
int timingfail = 0;

bool rx_bayang_check() {
  bool channels_received = false;

  int packetreceived = checkpacket();
  int pass = 0;
  if (packetreceived) {
    if (flags.rx_mode == RX_MODE_BIND) { // rx startup , bind mode
      xn_readpayload(rxdata, 15);

      if (rxdata[0] == 0xa4 || rxdata[0] == 0xa3) { // bind packet
        if (rxdata[0] == 0xa3) {
          bind_storage.bayang.telemetry_enabled = 1;
          packet_period = PACKET_PERIOD_TELEMETRY;
        }

        bind_storage.bayang.rfchannel[0] = rxdata[6];
        bind_storage.bayang.rfchannel[1] = rxdata[7];
        bind_storage.bayang.rfchannel[2] = rxdata[8];
        bind_storage.bayang.rfchannel[3] = rxdata[9];

        uint8_t rxaddr_regs[6] = {
            0x2a,
        };

        for (int i = 1; i < 6; i++) {
          rxaddr_regs[i] = rxdata[i];
          bind_storage.bayang.rxaddress[i - 1] = rxdata[i];
        }
        // write new rx address
        writeregs(rxaddr_regs, sizeof(rxaddr_regs));
        rxaddr_regs[0] = 0x30; // tx register ( write ) number

        // write new tx address
        writeregs(rxaddr_regs, sizeof(rxaddr_regs));

        xn_writereg(0x25, bind_storage.bayang.rfchannel[rf_chan]); // Set channel frequency

        flags.rx_mode = RX_MODE_NORMAL;

#ifdef SERIAL
        printf(" BIND \n");
#endif
      }
    } else { // normal mode
#ifdef RXDEBUG
      channelcount[rf_chan]++;
      packettime = time_micros() - lastrxtime;

      if (skipchannel && !timingfail)
        afterskip[skipchannel]++;
      if (timingfail)
        afterskip[0]++;

#endif

      uint32_t temptime = time_micros();

      xn_readpayload(rxdata, 15);
      pass = decodepacket();

      if (pass) {
        packetrx++;
        if (bind_storage.bayang.telemetry_enabled)
          beacon_sequence();
        skipchannel = 0;
        timingfail = 0;
        lastrxchan = rf_chan;
        lastrxtime = temptime;
        failsafetime = temptime;
        flags.failsafe = 0;
        channels_received = true;
        if (!telemetry_send)
          nextchannel();
      } else {
#ifdef RXDEBUG
        failcount++;
#endif
      }
      bind_safety++;
      if (bind_safety > 9) { // requires 10 good frames to come in before rx_ready safety can be toggled to 1
        flags.rx_ready = 1;  // because aux channels initialize low and clear the binding while armed flag before aux updates high
        bind_safety = 10;
      }
    } // end normal rx mode

  } // end packet received

  // finish sending if already started
  if (telemetry_send)
    beacon_sequence();

  uint32_t time = time_micros();

  if (time - lastrxtime > (HOPPING_NUMBER * packet_period + 1000) && flags.rx_mode != RX_MODE_BIND) {
    //  channel with no reception
    lastrxtime = time;
    // set channel to last with reception
    if (!timingfail)
      rf_chan = lastrxchan;
    // advance to next channel
    nextchannel();
    // set flag to discard packet timing
    timingfail = 1;
  }

  if (!timingfail && !telemetry_send && skipchannel < HOPPING_NUMBER + 1 && flags.rx_mode != RX_MODE_BIND) {
    uint32_t temp = time - lastrxtime;

    if (temp > 1000 && (temp - (PACKET_OFFSET)) / ((int)packet_period) >=
                           (skipchannel + 1)) {
      nextchannel();
#ifdef RXDEBUG
      skipstats[skipchannel]++;
#endif
      skipchannel++;
    }
  }

  if (time - failsafetime > FAILSAFETIME) { //  failsafe
    flags.failsafe = 1;
    state.rx.axis[0] = 0;
    state.rx.axis[1] = 0;
    state.rx.axis[2] = 0;
    state.rx.axis[3] = 0;
  }

  if (!flags.failsafe)
    autobind_inhibit = 1;
  else if (!autobind_inhibit && time - autobindtime > 15000000) {
    autobind_inhibit = 1;
    flags.rx_mode = RX_MODE_BIND;
    static uint8_t rxaddr[6] = {0x2a, 0, 0, 0, 0, 0};
    writeregs(rxaddr, sizeof(rxaddr));
    xn_writereg(RF_CH, 0); // bind on channel 0
  }

  if (time_micros() - secondtimer > 1000000) {
    packetpersecond = packetrx;
    packetrx = 0;
    secondtimer = time_micros();

#ifdef RADIO_XN297L
    state.rx_rssi = packetpersecond / 200.0f;
    state.rx_rssi = state.rx_rssi * state.rx_rssi * state.rx_rssi * RSSI_EXP + state.rx_rssi * (1 - RSSI_EXP);
    state.rx_rssi *= 100.0f;
    if (state.rx_rssi > 100.0f)
      state.rx_rssi = 100.0f;
    if (state.rx_rssi < 0.0f)
      state.rx_rssi = 0.0f;
#endif
#ifdef RADIO_XN297
    if (profile.receiver.aux[AUX_RSSI] > AUX_CHANNEL_11) { // rssi set to internal link quality
      state.rx_rssi = packetpersecond / 200.0f;
      state.rx_rssi = state.rx_rssi * state.rx_rssi * state.rx_rssi * RSSI_EXP + state.rx_rssi * (1 - RSSI_EXP);
      state.rx_rssi *= 100.0f;
      if (state.rx_rssi > 100.0f)
        state.rx_rssi = 100.0f;
      if (state.rx_rssi < 0.0f)
        state.rx_rssi = 0.0f;
    }
#endif
  }

  return channels_received;
}

#endif
