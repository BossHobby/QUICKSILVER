/*

// original bluetooth LE idea by Dmitry Grinberg
// http://dmitry.gr/index.php?r=05.Projects&proj=11.%20Bluetooth%20LE%20fakery

// some bluetooth LE functions adapted from nrf24 code by Lijun
// http://doc.lijun.li/misc-nrf24-ble.html
// https://github.com/lijunhw/nRF24_BLE/blob/master/Arduino/nRF24_BLE_advertizer_demo/nRF24_BLE_advertizer_demo.ino

*/

#include "rx/bayang.h"

#include <stdio.h>

#include "core/failloop.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/spi_soft.h"
#include "driver/spi_xn297.h"
#include "driver/time.h"
#include "flight/control.h"
#include "util/util.h"

#ifdef RX_BAYANG_BLE_APP

#define RX_MODE_BIND RXMODE_BIND
#define RX_MODE_NORMAL RXMODE_NORMAL

// ble settings
// SilverVISE app coded by SilverAG

// -----------------------------------------------
// Configuration for SilverVISE telemetry  - start
// -----------------------------------------------
// Following set of options will be used only if you enabled #define RX_BAYANG_BLE_APP
//
// For telemetry with SilverVISE application, you must define unique name of each quad you flash.
// Names between quadcopters must not be the same if you plan to fly more than one in the same time.
// Name must be entered inside quotation marks ("") - just replace H8BLUE with desired quad name.
// Be original and keep up to 6 characters (any longer name will be cut to first 6 characters).
// You can use letters A to Z, numbers 0 to 9 and some special characters like # / \ - _ etc.
// Please use only caps lock letters because name is not case sensitive. Avoid using blanks in name!

#define MY_QUAD_NAME "BWHOOP"

// MY_QUAD_ID defines unique ID for MAC address. Leave 127 or replace it with any other value between 0 and 255.
// Use ONLY values between 0 and 255!
// This is actually one byte of MAC address used for bluetooth communication between quad and SilverVISE application.
// Unique MY_QUAD_ID can be the same for all your quads as long as you won't have powered on more than one in the same time (or you are sure that nobody else uses the same number
// for their quad and don't have it powered on in the same time with yours inisde bluetooth range), otherwise TLM data WILL BE MIXED BETWEEN QUADS!
// If this option is not enabled and set, different random number will be assigned to quad's MAC after EACH power-up - BUT THEN BE AWARE THAT ON SOME ANDROID DEVICES (NEXUS 5, 6 etc.)
// THERE IS A BUG THAT, ACCORDING TO SOME INFORMATION AROUND INTERNET, MIGHT CAUSE BLUETOOTH TO STOP WORKING IF TOO MUCH DIFFERENT BLE DEVICES ARE SCANNED!
// CHECK HERE:
//    https://code.google.com/p/android/issues/detail?id=191831
//    http://www.androidpolice.com/2014/05/20/bug-watch-bluetooth-will-begin-crashing-after-encountering-too-many-ble-devices-affects-kitkat-4-4-and-jelly-bean-4-3/
//    https://play.google.com/store/apps/details?id=com.radiusnetworks.bluetoothcrashresolver
// BECAUSE THERE IS NO CONCRETE EXPLANATION WHAT ACTUALLY CAUSES BLUETOOTH PROBLEMS IN SOME CASES, I CAN'T GUARANTEE THAT THIS OPTION MIGHT PREVENT THAT,
// BUT YOU CAN TRY IT (WITH FIXED MAC YOUR QUADCOPTER WILL BE ALWAYS RECOGNIZED AS THE SAME DEVICE WHICH MIGHT PREVENT BLUETOOTH CRASHES)
// 0 - 255
#define MY_QUAD_ID 127

// you can keep the same value for all your quads if you like, but be aware: if you fly them all in the same time or somebody else uses
// the same MAC ID in their quadcopter firmware, SilverVISE application will have TLM data mixed!

// If you want to have different image in SilverVISE assigned to your quadcopter, you can override default value that represents quadcopter model you flash.
// Do it here by uncommenting and setting "#define MY_QUAD_MODEL".
// If disabled, value is set by firmware itself (based on what quad model firmware is made for).
// For this firmware, default image is for BWHOOP with blue canopy. But in case you have orange one, set value to 0x52
// and get propper image in SilverVISE application.
// To resume, fo this firmware, values are:
//
// 0x51 - BWHOOP B-03 - blue canopy (default)
// 0x52 - BWHOOP B-03 - orange canopy
//
// Value 0x00 represents unknown quad (generic image)

// #define MY_QUAD_MODEL 0x52

// *** THE FOLLOWING THREE SETTINGS USE ONLY IF YOU HAVE PROBLEMS WITH VERY OFTEN "TLM DISCONNECTING" ALARMS, ESPECIALLY ON FULL THROTTLE ***
// If you do not experience these problems and have stable telemetry connection, do not enable and set TX_POWER_GENERAL, TX_POWER_ON_TLM nor USE_ALL_BLE_CHANNELS
// BECAUSE ENABLING THESE SETTINGS IN CASE OF STABLE CONNECTION BETWEEN SILVERVISE APPLICATION AND QUADCOPTER MIGHT ACTUALLY CAUSE PROBLEMS
// IN RECEIVING TELEMETRY DATA!
// So, only if you experience problems with constant TLM DISCONNECTED error in SilverVISE application (especially during full throttle),
// enable and set TX_POWER_GENERAL and/or TX_POWER_ON_TLM and experiment with values until you get stable TLM connection and decent bluetooth and controller reception
// WARNING !!!!!!!!!! USE ONLY VALUES FROM 0 TO 7 !!! DO NOT GO OVER 7 AND BELOW 0 !!!

// #define TX_POWER_GENERAL 3 // general value for quadcopter power (use odd numbers, but try with even also - smaller value is better for telemetry signal)

// #define TX_POWER_ON_TLM 0 //quadcopter power during transmitting telemetry data (use even numbers but try odd also - smaller value is better for telemetry signal)
//  usually TX_POWER_ON_TLM need to be lower than TX_POWER_GENERAL, but experiment if that combination does not help...
//  If TX_POWER_GENERAL/TX_POWER_ON_TLM still produces constant TLM DISCONNECTED errors when motors are on and on high throttle, try also enabling USE_ALL_BLE_CHANNELS.
//  You can use all three settings in the same time or experiment with one or two of them until you get better and stable bluetooth signal.

// #define USE_ALL_BLE_CHANNELS

// ----------------------------------------------
// Configuration for SilverVISE telemetry - end
// ----------------------------------------------

// beacon interval
#define BLE_INTERVAL 30000

// this allows different quads to show up at the same time.
// 0 - 255 select a different number for each quad if you need several simultaneous
#define BLE_QUAD_NUMBER 17

// optimized one channel only (bluetooth)
// uses precalculated whitening data
// possible values: 0 / 1
// #define ONE_CHANNEL 1 //commented by silverAG for SilverVISE - not used for now
// SilverVISE - start:
#ifdef USE_ALL_BLE_CHANNELS
#define ONE_CHANNEL 0
#else
#define ONE_CHANNEL 1
#endif
// SilverVISE - end

// radio settings

// packet period in uS
#define PACKET_PERIOD 3000

// was 250 ( uS )
#define PACKET_OFFSET 250

// how many times to hop ahead if no reception
#define HOPPING_NUMBER 4

int current_PID_for_display = 0;
int PID_index_delay = 0;

char lasttrim[4];

char rfchannel[4];
int rxaddress[5];
int rf_chan = 0;
uint16_t bind_safety = 0;

uint32_t total_time_in_air = 0;
uint32_t time_throttle_on = 0;
int bound_for_BLE_packet;
int random_seed;

void bleinit();

void writeregs(uint8_t data[], uint8_t size) {
  spi_cson();
  for (uint8_t i = 0; i < size; i++) {
    spi_sendbyte(data[i]);
  }
  spi_csoff();
  time_delay_us(1000);
}

char quad_name[6] = {'N', 'O', 'N', 'A', 'M', 'E'};

void rx_protocol_init() {
#ifdef RX_BAYANG_BLE_APP
  // for randomising MAC adddress of ble app - this will make the int = raw float value
  random_seed = *(int *)&state.vbat_filtered;
  random_seed = random_seed & 0xff;
#endif

#ifdef AUX4_START_ON
  state.aux[CH_AUX4] = 1;
#endif

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

#define XN_POWER 0b00000001 | ((TX_POWER & 7) << 3)

#define XN_TO_RX 0b10001111
#define XN_TO_TX 0b10000010
  // #define XN_POWER 0b00111111

#endif

#ifdef RADIO_XN297
  static uint8_t bbcal[6] = {0x3f, 0x4c, 0x84, 0x6F, 0x9c, 0x20};
  writeregs(bbcal, sizeof(bbcal));
  // new values
  static uint8_t rfcal[8] = {0x3e, 0xc9, 0x9a, 0xA0, 0x61, 0xbb, 0xab, 0x9c};
  writeregs(rfcal, sizeof(rfcal));

  static uint8_t demodcal[6] = {0x39, 0x0b, 0xdf, 0xc4, 0xa7, 0x03};
  // static uint8_t demodcal[6] = { 0x39 , 0x0b , 0xdf , 0xc4 , 0b00100111 , 0b00000000};
  writeregs(demodcal, sizeof(demodcal));

#define XN_TO_RX 0b00001111
#define XN_TO_TX 0b00000010
// #define XN_POWER 0b00000111 // disabled by silverAG for SilverVISE - value is added from config.h
//  SilverVISE - start:
#ifdef TX_POWER_GENERAL
// use value from config.h
#define XN_POWER TX_POWER_GENERAL
#else
#define XN_POWER 0b00000111
#endif
  // SilverVISE - end

#endif

  bleinit();

  time_delay_us(100);

  int rxaddress[5] = {0, 0, 0, 0, 0};
  xn_writerxaddress(rxaddress);

  xn_writereg(EN_AA, 0);           // aa disabled
  xn_writereg(EN_RXADDR, 1);       // pipe 0 only
  xn_writereg(RF_SETUP, XN_POWER); // lna high current on ( better performance )
  xn_writereg(RX_PW_P0, 15);       // payload size
  xn_writereg(SETUP_RETR, 0);      // no retransmissions ( redundant?)
  xn_writereg(SETUP_AW, 3);        // address size (5 bits)
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

  // fill with characters from MY_QUAD_NAME (just first 6 chars)
  int string_len = 0;
  while (string_len < 6) {
    if (MY_QUAD_NAME[string_len] == '\0')
      break;
    quad_name[string_len] = (char)MY_QUAD_NAME[string_len];
    string_len++;
  }

  // fill the rest (up to 6 bytes) with blanks
  for (int i = string_len; i < 6; i++) {
    quad_name[string_len] = ' '; // blank
  }
}

void btLeCrc(uint8_t *buf, uint8_t len, uint8_t *dst) {

  union {
    uint32_t int32;
    uint8_t u8[4];
  } myint;

  myint.int32 = 0x00aaaaaa;

  while (len--) {
    uint8_t d = *(buf++);
    for (int i = 8; i > 0; i--) {
      uint8_t t = myint.int32 & 1;
      myint.int32 >>= 1;
      if (t != (d & 1)) {
        myint.u8[2] ^= 0xDA;
        myint.u8[1] ^= 0x60;
      }
      d >>= 1;
    }
  }

  for (int i = 0; i < 3; i++)
    dst[i] = (myint.u8[i]);
}

// scrambling sequence for xn297
const uint8_t xn297_scramble[] = {
    0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66,
    0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
    0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc,
    0x1b, 0x5d, 0x19, 0x10, 0x24, 0xd3, 0xdc, 0x3f,
    0x8e, 0xc5, 0x2f};

// bit reversed of above
const uint8_t xn297_scramble_rev[] = {
    0xc7, 0x8d, 0xd2, 0x57, 0xa1, 0x3d, 0xa7, 0x66,
    0xb0, 0x75, 0x31, 0x11, 0x48, 0x96, 0x77, 0xf8,
    0xe3, 0x46, 0xe9, 0xab, 0xd0, 0x9e, 0x53, 0x33,
    0xd8, 0xba, 0x98, 0x8, 0x24, 0xcb, 0x3b, 0xfc,
    0x71, 0xa3, 0xf4, 85, 104, 207, 169, 25,
    108, 93, 76, 4, 146, 229, 29};

// whitening sequence for adv channel 37 (rf chan 2402)
// for speed
const uint8_t ble_whiten_37[] = {
    0x8D, 0xd2, 0x57, 0xa1, 0x3d, 0xa7, 0x66, 0xb0,
    0x75, 0x31, 0x11, 0x48, 0x96, 0x77, 0xf8, 0xe3,
    0x46, 0xe9, 0xab, 0xd0, 0x9e, 0x53, 0x33, 0xd8,
    0xba, 0x98, 0x08, 0x24, 0xcb, 0x3b, 0xfc, 0x71,
    0xa3, 0xf4, 0x55, 0x68, 0xCF, 0xA9, 0x19, 0x6C,
    0x5D, 0x4C}; // whitening sequence channel 37 ( 0 - index ; 2 - rf channel; 37 - ble spec)

uint8_t chRf[3] = {2, 26, 80};
uint8_t chLe[3] = {37, 38, 39};
uint8_t whitenstart[] = {0xa6, 0x66, 0xe6};

/*
uint8_t swapbits_old(uint8_t a){
// reverse the bit order in a single byte
uint8_t v = 0;
if(a & 0x80) v |= 0x01;
if(a & 0x40) v |= 0x02;
if(a & 0x20) v |= 0x04;
if(a & 0x10) v |= 0x08;
if(a & 0x08) v |= 0x10;
if(a & 0x04) v |= 0x20;
if(a & 0x02) v |= 0x40;
if(a & 0x01) v |= 0x80;
return v;
}

// from https://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith32Bits
// reverse the bit order in a single byte
uint8_t swapbits(uint8_t a){
uint32_t b = a;
b = ((b * 0x0802LU & 0x22110LU) | (b * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
return b;
}

// cortex m3 intrinsic bitswap
uint8_t swapbits_m3(uint8_t a)
{
return (uint32_t) __rbit( (uint32_t) a);
}
*/

void btLeWhiten(uint8_t *data, uint8_t len, uint8_t whitenCoeff) {
  // Implementing whitening with LFSR
  uint8_t m;
  while (len--) {
    for (m = 1; m; m <<= 1) {
      if (whitenCoeff & 0x80) {
        whitenCoeff ^= 0x11;
        (*data) ^= m;
      }
      whitenCoeff <<= 1;
    }
    data++;
  }
}
/*
static inline uint8_t btLeWhitenStart(uint8_t chan){
//the value we actually use is what BT'd use left shifted one...makes our life easier
return swapbits(chLe[chan]) | 2;
}
*/

void btLePacketEncode(uint8_t *packet, uint8_t len, uint8_t chan) {
  // Assemble the packet to be transmitted
  // Length is of packet, including crc. pre-populate crc in packet with initial crc value!
  uint8_t i, dataLen = len - 3;

  packet[len - 3] = 0x55; // CRC start value: 0x555555
  packet[len - 2] = 0x55;
  packet[len - 1] = 0x55;

  btLeCrc(packet, dataLen, packet + dataLen);

  // for(i = 0; i < 3; i++, dataLen++)
  //	packet[dataLen] = swapbits(packet[dataLen]);

  if (ONE_CHANNEL) {
    // faster array based whitening
    for (i = 0; i < len; i++)
      packet[i] ^= ble_whiten_37[i];
  } else // lfsr based
    btLeWhiten(packet, len, whitenstart[chan]);
}

#define RXDEBUG

#ifdef RXDEBUG
uint32_t packettime;
int channelcount[4];
int failcount;
int packetrx;
int packetpersecond;

int skipstats[12];
int afterskip[12];
// #warning "RX debug enabled"
#endif

#define MY_MAC_1 0x22
#define MY_MAC_2 0x33
#define MY_MAC_3 0x44
#define MY_MAC_4 0x55
#define MY_MAC_5 0xF6

uint8_t buf[48];
int buffint[48];

uint8_t ch = 0; // RF channel for frequency hopping

int payloadsize;

void bleinit() {
  // Set access addresses (TX address in nRF24L01) to BLE advertising 0x8E89BED6
  // Remember that both bit and byte orders are reversed for BLE packet format

  int txaddr[5];

  /*
  // 4 byte address
txaddr[0] = swapbits(0x8E)^xn297_scramble[3];
txaddr[1] = swapbits(0x89)^xn297_scramble[2];
txaddr[2] = swapbits(0xBE)^xn297_scramble[1];
txaddr[3] = swapbits(0xD6)^xn297_scramble[0];
txaddr[4] = 0;
*/

  /*
    // 4 byte address - optimized
txaddr[0] = 0x71^0xea;
txaddr[1] = 0x91^0x4b;
txaddr[2] = 0x7d^0xb1;
txaddr[3] = 0x6b^0xe3;
txaddr[4] = 0;
*/

  // using 5 byte address because it's shared with rx (rx protocol has 5 bytes)
  // saves changing the address size everytime we send
  txaddr[0] = 0x71 ^ 0x85;
  txaddr[1] = 0x91 ^ 0xea;
  txaddr[2] = 0x7d ^ 0x4b;
  txaddr[3] = 0x6b ^ 0xb1;
  txaddr[4] = 0xaa ^ 0xe3; // preamble

  xn_writetxaddress(txaddr);

  //	xn_writereg( EN_AA , 0 );	// aa disabled -- duplicated
  //	xn_writereg( RF_SETUP , 0b00111011);  // high power xn297L only
  //	xn_writereg( SETUP_RETR , 0 ); // no retransmissions  -- duplicated
  // -- duplicated
  //	xn_writereg( SETUP_AW , XN297_ADDRESS_SIZE_BLE - 2 ); // address size (4 bytes for ble)

  //  xn_writereg( 0x1d, 0b00111000 ); // 64 bit payload , software ce
}

void send_beacon();

int loopcounter = 0;
uint32_t ble_txtime;
int ble_send = 0;
int oldchan = 0;

#define BLE_TX_TIMEOUT 10000

void beacon_sequence() {
  static int beacon_seq_state = 0;

  switch (beacon_seq_state) {
  case 0:
    // send data if enough time passed since last send
    if (time_micros() - ble_txtime > BLE_INTERVAL) {
      ble_send = 1;
      oldchan = rf_chan;

// SilverVISE - start
#ifdef TX_POWER_ON_TLM
      xn_writereg(RF_SETUP, TX_POWER_ON_TLM); // lna low current ( better BLE)
#endif
      // SilverVISE - end

      send_beacon();

      beacon_seq_state++;
    }
    break;

  case 1:
    // wait for data to finish transmitting
    if ((xn_readreg(0x17) & 0b00010000)) {
      xn_writereg(0, XN_TO_RX);
      xn_writereg(0x25, rfchannel[oldchan]);
      beacon_seq_state++;
      goto next;
    } else { // if it takes too long we get rid of it
      if (time_micros() - ble_txtime > BLE_TX_TIMEOUT) {
// SilverVISE - start
#ifdef TX_POWER_ON_TLM
        xn_writereg(RF_SETUP, XN_POWER); // lna high current on ( better performance )
#endif
        // SilverVISE - end

        xn_command(FLUSH_TX);
        xn_writereg(0, XN_TO_RX);
        beacon_seq_state++;
        ble_send = 0;
      }
    }
    break;

  case 2:
  next:
// restore radio settings to protocol compatible
// mainly channel here

// SilverVISE - start
#ifdef TX_POWER_ON_TLM
    xn_writereg(RF_SETUP, XN_POWER); // lna high current on ( better performance )
#endif
    // SilverVISE - end

    ble_send = 0;
    if (flags.rx_mode == 0) {
      xn_writereg(0x25, 0); // Set channel frequency	, bind
    }
    beacon_seq_state++;
    break;

  default:
    beacon_seq_state = 0;
    break;
  }
}

int interleave = 0;

void send_beacon() {

  // Channel hopping
  ch++;
  if (ch > 2) {
    ch = 0;
  }
  // sending on channel 37 only to use whitening array
  if (ONE_CHANNEL)
    ch = 0;

  xn_writereg(RF_CH, chRf[ch]);

  uint8_t L = 0;

  extern int random_seed;

// extern int random_seed; //SilverVISE
// SilverVISE - start
#ifdef MY_QUAD_ID
  random_seed = MY_QUAD_ID;
#else
#warning WARNING!!! USING RANDOM MAC ADDRESS! PLEASE READ COMMENT INSIDE rx_bayang_ble_app.c REGARDING POSSIBLE PROBLEMS WITH BLUETOOTH DUE TO ANDROID BUG!
#endif
  // SilverVISE - end
  int vbatt_comp_int = state.vbat_compensated * 1000.0f;

  uint32_t time = time_micros();

  time = time >> 20; // divide by 1024*1024, no time for accuracy here
  time = time * 10;

  // int acro_or_level_mode;

  // if ( rx_aux_on(AUX_LEVELMODE) ) acro_or_level_mode = 1;
  // else acro_or_level_mode = 0;

  extern uint32_t total_time_in_air;
  extern uint32_t time_throttle_on;
  uint32_t uptime = time_micros();

  int TLMorPID = 0; // 0 = TLM, 1 = PID+TLM

#ifdef PID_GESTURE_TUNING
  TLMorPID = 1; // 0 = TLM, 1 = PID+TLM
#endif

  if (flags.on_ground == 1) {
    time_throttle_on = uptime;
  } else {
    total_time_in_air += (uptime - time_throttle_on);
    time_throttle_on = uptime;
  }
  uint32_t total_time_in_air_time = total_time_in_air >> 20;
  total_time_in_air_time = total_time_in_air_time * 10;

  int rate_and_mode_value = (rx_aux_on(AUX_RATE_PROFILE) << 1) + !!(rx_aux_on(AUX_LEVELMODE));

  extern int bound_for_BLE_packet;
  int onground_and_bind = (flags.failsafe << 2) + (flags.on_ground << 1) + (bound_for_BLE_packet);
  onground_and_bind = 8 + onground_and_bind;

  int packetpersecond_short = packetpersecond / 2;
  if (packetpersecond_short > 0xff)
    packetpersecond = 0xff;

  buf[L++] = 0b00100010; // PDU type, given address is random; 0x42 for Android and 0x40 for iPhone
  // buf[L++] = 0x42; //PDU type, given address is random; 0x42 for Android and 0x40 for iPhone

  // max len 27 with 5 byte address = 37 total payload bytes
  buf[L++] = 10 + 21;     // length of payload
  buf[L++] = random_seed; // SilverVISE
  buf[L++] = MY_MAC_1;
  buf[L++] = MY_MAC_2;
  buf[L++] = MY_MAC_3;
  buf[L++] = MY_MAC_4;
  buf[L++] = MY_MAC_5;

  // packet data unit
  buf[L++] = 2;    // flags lenght(LE-only, limited discovery mode)
  buf[L++] = 0x01; // compulsory flags
  buf[L++] = 0x06; // flag value
  buf[L++] = 0x15; // Length of next block
  buf[L++] = 0x16; // Service Data

  // ------------------------- TLM+PID
  if (TLMorPID == 1) {
    buf[L++] = 0x2F; // PID+TLM datatype_and_packetID;  // xxxxyyyy -> yyyy = 1111 packet type ID (custom BLE type), xxxx = type of data in packet: 0001 -> telemetry, 0002->PID

#ifdef MY_QUAD_MODEL
    buf[L++] = MY_QUAD_MODEL;
#else
    buf[L++] = 0x51;                                           // quad model (00 - unknown, 51 - BWHOOP B-03 blue canopy, 52 - BWHOOP B-03 orange canopy... check comments at start of this file for details)
#endif

    buf[L++] = random_seed; // already custom entry - need to be randomized
    buf[L++] = quad_name[0];
    buf[L++] = quad_name[1];
    buf[L++] = quad_name[2];
    buf[L++] = quad_name[3];
    buf[L++] = quad_name[4];
    buf[L++] = quad_name[5];

    extern int current_pid_term; // 0 = pidkp, 1 = pidki, 2 = pidkd
    extern int current_pid_axis; // 0 = roll, 1 = pitch, 2 = yaw

    // int selectedPID = 0; //inxed of selected PID for changing

    int selectedPID = ((current_pid_term)*3) + (current_pid_axis);

    buf[L++] = (current_PID_for_display << 4) + selectedPID; // xy => x=current PID for display 0 - 14 (cycling...), y = selected PID for changing 0 - 14

    buf[L++] = packetpersecond_short;

    /*
buf[L++] =  onground_and_bind; //binary xxxxabcd - xxxx = error code or warning, a -> 0 = stock TX, 1= other TX, b -> 0 = not failsafe, 1 = failsafe, c = 0 -> not bound, 1 -> bound, d = 0 -> in the air, 1 = on the ground;
*/

#ifdef COMBINE_PITCH_ROLL_PID_TUNING
    buf[L++] = 0b01000000 + ((rate_and_mode_value << 4) + onground_and_bind); // binary xxRMabcd - x = error code or warning, 1 = combined roll+pitch tuning, R = rate (0 - normal, 1 - fast) , M = mode (1 - level, 0 - acro); a -> 0 = stock TX, 1= other TX, b -> 0 = not failsafe, 1 = failsafe, c = 0 -> not bound, 1 -> bound, d = 0 -> in the air, 1 = on the ground;
    int PID_pause = 8;
#else
    buf[L++] = (rate_and_mode_value << 4) + onground_and_bind; // binary x0RMabcd - x = error code or warning, 0 = no combined roll+pitch tuning, R = rate (0 - normal, 1 - fast) , M = mode (1 - level, 0 - acro); a -> 0 = stock TX, 1= other TX, b -> 0 = not failsafe, 1 = failsafe, c = 0 -> not bound, 1 -> bound, d = 0 -> in the air, 1 = on the ground;
    int PID_pause = 12;
#endif

    buf[L++] = vbatt_comp_int >> 8; // Battery voltage compensated
    buf[L++] = vbatt_comp_int;      // Battery voltage compensated

    extern profile_t profile;
    extern float apidkp[]; // current_PID_for_display = 9, 10
    extern float apidki[]; //  current_PID_for_display = 11, 12
    extern float apidkd[]; //  current_PID_for_display = 13, 14

    uint32_t pid_for_display = 0;

    switch (current_PID_for_display) {
    case 0:
      pid_for_display = (uint16_t)(profile_current_pid_rates()->kp.axis[0] * 10000.0f);
      break;
    case 1:
      pid_for_display = (uint16_t)(profile_current_pid_rates()->kp.axis[1] * 10000.0f);
      break;
    case 2:
      pid_for_display = (uint16_t)(profile_current_pid_rates()->kp.axis[2] * 10000.0f);
      break;
    case 3:
      pid_for_display = (uint16_t)(profile_current_pid_rates()->ki.axis[0] * 10000.0f);
      break;
    case 4:
      pid_for_display = (uint16_t)(profile_current_pid_rates()->ki.axis[1] * 10000.0f);
      break;
    case 5:
      pid_for_display = (uint16_t)(profile_current_pid_rates()->ki.axis[2] * 10000.0f);
      break;
    case 6:
      pid_for_display = (uint16_t)(profile_current_pid_rates()->kd.axis[0] * 10000.0f);
      break;
    case 7:
      pid_for_display = (uint16_t)(profile_current_pid_rates()->kd.axis[1] * 10000.0f);
      break;
    case 8:
      pid_for_display = (uint16_t)(profile_current_pid_rates()->kd.axis[2] * 10000.0f);
      break;
      /*
   //level mode PIDs - disabled for now...
   case 9:pid_for_display =(uint16_t)(apidkp[0]*10000.0f);break;
   case 10:pid_for_display =(uint16_t)(apidkp[1]*10000.0f);break;
   case 11:pid_for_display =(uint16_t)(apidki[0]*10000.0f);break;
   case 12:pid_for_display =(uint16_t)(apidki[1]*10000.0f);break;
   case 13:pid_for_display =(uint16_t)(apidkd[0]*10000.0f);break;
   case 14:pid_for_display =(uint16_t)(apidkd[1]*10000.0f);break;
*/
    }

    /*buf[L++] =  total_time_in_air_time>>8;  // total time in air
buf[L++] =  total_time_in_air_time;  // total time in air
buf[L++] =  time>>8;
buf[L++] =  time;
*/

    buf[L++] = total_time_in_air_time >> 8; // total time in air
    buf[L++] = total_time_in_air_time;      // total time in air
    buf[L++] = time >> 8;
    buf[L++] = time;

    buf[L++] = pid_for_display >> 8;
    buf[L++] = pid_for_display;

    L = L + 3; // crc

    PID_index_delay++;
    int PID_index_limit = 8; // number of PIDs to display for acro mode tuning
    // if ((rate_and_mode_value&1) == 1) PID_index_limit = 14; // number of PIDs to display for level mode tuning
    if (PID_index_delay > PID_pause) {
      PID_index_delay = 0;
      current_PID_for_display++;

#ifdef COMBINE_PITCH_ROLL_PID_TUNING
      if (current_PID_for_display == 1)
        current_PID_for_display = 2;
      if (current_PID_for_display == 4)
        current_PID_for_display = 5;
      if (current_PID_for_display == 7)
        current_PID_for_display = 8;
#endif

      if (current_PID_for_display > PID_index_limit)
        current_PID_for_display = 0;
    }
  }

  if (TLMorPID == 0) {
    buf[L++] = 0x1F; // TLM datatype_and_packetID;  // xxxxyyyy -> yyyy = 1111 packet type ID (custom BLE type), xxxx = type of data in packet: 0001 -> telemetry, 0002->PID

#ifdef MY_QUAD_MODEL
    buf[L++] = MY_QUAD_MODEL;
#else
    buf[L++] = 0x11; // quad model (00 - unknown, 11- H8 mini blue board, 20 - H101... check comments at start of this file for details)
#endif

    buf[L++] = random_seed; // already custom entry - need to be randomized
#ifdef MY_QUAD_NAME
    // fill with characters from MY_QUAD_NAME (just first 6 chars)
    int string_len = 0;
    while (string_len < 6) {
      if (MY_QUAD_NAME[string_len] == '\0')
        break;
      buf[L++] = (char)MY_QUAD_NAME[string_len];
      string_len++;
    }

    // fill the rest (up to 6 bytes) with blanks
    for (int i = string_len; i < 6; i++) {
      buf[L++] = ' '; // blank
    }
#else
    buf[L++] = (char)'N';
    buf[L++] = (char)'O';
    buf[L++] = (char)'N';
    buf[L++] = (char)'A';
    buf[L++] = (char)'M';
    buf[L++] = (char)'E';
#endif
    buf[L++] = 0x00; // reserved for future use
    buf[L++] = packetpersecond_short;
    buf[L++] = onground_and_bind;           // binary xxxxabcd - xxxx = error code or warning, a -> 0 = stock TX, 1= other TX, b -> 0 = not failsafe, 1 = failsafe, c = 0 -> not bound, 1 -> bound, d = 0 -> in the air, 1 = on the ground;
    buf[L++] = vbatt_comp_int >> 8;         // Battery voltage compensated
    buf[L++] = vbatt_comp_int;              // Battery voltage compensated
    buf[L++] = total_time_in_air_time >> 8; // total time in air
    buf[L++] = total_time_in_air_time;      // total time in air
    buf[L++] = time >> 8;
    buf[L++] = time;
    buf[L++] = rate_and_mode_value; // xxxxxxRM //rate + mode R = rate (0 - normal, 1 - fast) , M = mode (1 - level, 0 - acro)
    buf[L++] = 0x00;                // reserved for future use

    L = L + 3; // crc
  }

  btLePacketEncode(buf, L, ch);

  // undo xn297 data whitening
  for (uint8_t i = 0; i < L; ++i) {
    buf[i] = buf[i] ^ xn297_scramble_rev[i + 5]; // address size 5
  }

  for (int i = 0; i < L; i++)
    buffint[i] = buf[i];

  xn_command(FLUSH_TX);

  xn_writereg(0, XN_TO_TX);

  payloadsize = L;
  xn_writepayload(buffint, L);

  ble_txtime = time_micros();

  return;
}

static char checkpacket() {
  int status = xn_readreg(7);

  if (status & (1 << MASK_RX_DR)) { // rx clear bit
                                    // this is not working well
                                    // xn_writereg( STATUS , (1<<MASK_RX_DR) );
                                    // RX packet received
                                    // return 1;
  }
  if ((status & 0b00001110) != 0b00001110) {
    // rx fifo not empty
    return 2;
  }

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
      state.rx.axis[3] = ((rxdata[8] & 0x0003) * 256 + rxdata[9]) * 0.000976562f;

      // trims are 50% of controls at max
      // trims are not used as trims because they interfere
      // with dynamic trims feature of devo firmware

      //                      state.rx.axis[0] = state.rx.axis[0] + 0.03225 * 0.5 * (float)(((rxdata[4])>>2) - 31);
      //                      state.rx.axis[1] = state.rx.axis[1] + 0.03225 * 0.5 * (float)(((rxdata[6])>>2) - 31);
      //                      state.rx.axis[2] = state.rx.axis[2] + 0.03225 * 0.5 * (float)(((rxdata[10])>>2) - 31);

      // this share the same numbers to the above CH_PIT_TRIM etc
      state.aux[CH_VID] = (rxdata[2] & 0x10) ? 1 : 0;

      state.aux[CH_PIC] = (rxdata[2] & 0x20) ? 1 : 0;

      state.aux[CH_INV] = (rxdata[3] & 0x80) ? 1 : 0; // inverted flag

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
  rf_chan %= 4;
  xn_writereg(0x25, rfchannel[rf_chan]);
}

uint32_t lastrxtime;
uint32_t failsafetime;
uint32_t secondtimer;

uint32_t skipchannel = 0;
int lastrxchan;
int timingfail = 0;
extern int bound_for_BLE_packet; // SilverVISE

bool rx_bayang_check() {
  bool channels_received = false;

  int packetreceived = checkpacket();
  int pass = 0;
  if (packetreceived) {
    if (flags.rx_mode == RX_MODE_BIND) { // rx startup , bind mode
      xn_readpayload(rxdata, 15);

      if (rxdata[0] == 164) { // bind packet
        rfchannel[0] = rxdata[6];
        rfchannel[1] = rxdata[7];
        rfchannel[2] = rxdata[8];
        rfchannel[3] = rxdata[9];

        int rxaddress[5];
        rxaddress[0] = rxdata[1];
        rxaddress[1] = rxdata[2];
        rxaddress[2] = rxdata[3];
        rxaddress[3] = rxdata[4];
        rxaddress[4] = rxdata[5];

        xn_writerxaddress(rxaddress);
        xn_writereg(0x25, rfchannel[rf_chan]); // Set channel frequency
        flags.rx_mode = RX_MODE_NORMAL;
        bound_for_BLE_packet = 1; // SilverVISE

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

      nextchannel();

      xn_readpayload(rxdata, 15);
      pass = decodepacket();

      if (pass) {
#ifdef RXDEBUG
        packetrx++;
#endif
        skipchannel = 0;
        timingfail = 0;
        lastrxchan = rf_chan;
        lastrxtime = temptime;
        failsafetime = temptime;
        channels_received = true;
        flags.failsafe = 0;
      } else {
#ifdef RXDEBUG
        failcount++;
#endif
      }

    } // end normal rx mode
    bind_safety++;
    if (bind_safety > 9) { // requires 10 good frames to come in before rx_ready safety can be toggled to 1
      flags.rx_ready = 1;  // because aux channels initialize low and clear the binding while armed flag before aux updates high
      bind_safety = 10;
    }
  } // end packet received

  beacon_sequence();

  uint32_t time = time_micros();

  // sequence period 12000
  if (time - lastrxtime > (HOPPING_NUMBER * PACKET_PERIOD + 1000) && flags.rx_mode != RX_MODE_BIND) {
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

  if (!timingfail && !ble_send && skipchannel < HOPPING_NUMBER + 1 && flags.rx_mode != RX_MODE_BIND) {
    uint32_t temp = time - lastrxtime;

    if (temp > 1000 && (temp - (PACKET_OFFSET)) / ((int)PACKET_PERIOD) >= (skipchannel + 1)) {
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
#ifdef RXDEBUG
  if (time_micros() - secondtimer > 1000000) {
    packetpersecond = packetrx;
    packetrx = 0;
    secondtimer = time_micros();
  }
#endif

  return channels_received;
}

#endif
