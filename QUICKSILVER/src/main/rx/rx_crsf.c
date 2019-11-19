#include <stdio.h>

#include "defines.h"
#include "drv_fmc.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "profile.h"
#include "project.h"
#include "util.h"

#ifdef RX_CRSF

// global use rx variables
extern float rx[4];
extern char aux[AUX_CHANNEL_MAX];
extern char lastaux[AUX_CHANNEL_MAX];
extern char auxchange[AUX_CHANNEL_MAX];
int failsafe = 0;
int rxmode = 0;
int rx_ready = 0;
int bind_safety = 0;
int rx_bind_enable = 0;

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */

// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US (1100 * (TICK_CLOCK_FREQ_HZ / 1000000)) // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US (6667 * (TICK_CLOCK_FREQ_HZ / 1000000))   // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
//#define SERIAL_BAUDRATE 420000
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PAYLOAD_SIZE_MAX 60

#define USART usart_port_defs[serial_rx_port]

int crsfFrameDone = 0;
uint32_t crsfChannelData[CRSF_MAX_CHANNEL];
int rx_frame_pending;
int rx_frame_pending_last;
uint32_t flagged_time;
int framestarted = -1;

typedef struct crsfFrameDef_s {
  uint8_t deviceAddress;
  uint8_t frameLength;
  uint8_t type;
  uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; // +1 for CRC at end of payload
} crsfFrameDef_t;

typedef union crsfFrame_u {
  uint8_t bytes[CRSF_FRAME_SIZE_MAX];
  crsfFrameDef_t frame;
} crsfFrame_t;

crsfFrame_t crsfFrame;

typedef enum {
  CRSF_FRAMETYPE_GPS = 0x02,
  CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
  CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
  CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
  CRSF_FRAMETYPE_ATTITUDE = 0x1E,
  CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
  CRSF_FRAMETYPE_DEVICE_PING = 0x28,
  CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
  CRSF_FRAMETYPE_MSP_REQ = 0x7A,  // response request using msp sequence as command
  CRSF_FRAMETYPE_MSP_RESP = 0x7B, // reply with 58 byte chunked binary
  CRSF_FRAMETYPE_MSP_WRITE = 0x7C // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
} crsfFrameType_e;

enum {
  CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
  CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
  CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
  CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22, // 11 bits per channel * 16 channels = 22 bytes.
  CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
  CRSF_FRAME_TX_MSP_FRAME_SIZE = 58,
  CRSF_FRAME_RX_MSP_FRAME_SIZE = 8,
  CRSF_FRAME_ORIGIN_DEST_SIZE = 2,
  CRSF_FRAME_LENGTH_ADDRESS = 1,     // length of ADDRESS field
  CRSF_FRAME_LENGTH_FRAMELENGTH = 1, // length of FRAMELENGTH field
  CRSF_FRAME_LENGTH_TYPE = 1,        // length of TYPE field
  CRSF_FRAME_LENGTH_CRC = 1,         // length of CRC field
  CRSF_FRAME_LENGTH_TYPE_CRC = 2,    // length of TYPE and CRC fields combined
  CRSF_FRAME_LENGTH_EXT_TYPE_CRC = 4 // length of Extended Dest/Origin, TYPE and CRC fields combined
};

struct crsfPayloadRcChannelsPacked_s {
  // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
  unsigned int chan0 : 11;
  unsigned int chan1 : 11;
  unsigned int chan2 : 11;
  unsigned int chan3 : 11;
  unsigned int chan4 : 11;
  unsigned int chan5 : 11;
  unsigned int chan6 : 11;
  unsigned int chan7 : 11;
  unsigned int chan8 : 11;
  unsigned int chan9 : 11;
  unsigned int chan10 : 11;
  unsigned int chan11 : 11;
  unsigned int chan12 : 11;
  unsigned int chan13 : 11;
  unsigned int chan14 : 11;
  unsigned int chan15 : 11;
} __attribute__((__packed__));

typedef struct crsfPayloadRcChannelsPacked_s crsfPayloadRcChannelsPacked_t;

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
  crc ^= a;
  for (int ii = 0; ii < 8; ++ii) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0xD5;
    } else {
      crc = crc << 1;
    }
  }
  return crc;
}

uint8_t crsfFrameCRC(void) {
  // CRC includes type and payload
  uint8_t crc = crc8_dvb_s2(0, crsfFrame.frame.type);
  for (int ii = 0; ii < crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
    crc = crc8_dvb_s2(crc, crsfFrame.frame.payload[ii]);
  }
  return crc;
}

// Receive ISR callback, called back from serial port
void RX_USART_ISR(void) {
  static uint8_t crsfFramePosition = 0;
  unsigned long maxticks = SysTick->LOAD;
  unsigned long ticks = SysTick->VAL;
  unsigned long crsfTimeInterval;
  static unsigned long lastticks;
  if (ticks < lastticks)
    crsfTimeInterval = lastticks - ticks;
  else { // overflow ( underflow really)
    crsfTimeInterval = lastticks + (maxticks - ticks);
  }
  lastticks = ticks;

  if (USART_GetFlagStatus(USART.channel, USART_FLAG_ORE)) {
    // overflow means something was lost
    USART_ClearFlag(USART.channel, USART_FLAG_ORE);
    crsfFramePosition = 0;
  }

  if (crsfTimeInterval > CRSF_TIME_NEEDED_PER_FRAME_US) {
    // We've received a character after max time needed to complete a frame,
    // so this must be the start of a new frame.
    crsfFramePosition = 0;
  }

  // assume frame is 5 bytes long until we have received the frame length
  // full frame length includes the length of the address and framelength fields
  const uint8_t fullFrameLength = crsfFramePosition < 3 ? 5 : crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;
  if (crsfFramePosition < fullFrameLength) {
    crsfFrame.bytes[crsfFramePosition++] = USART_ReceiveData(USART.channel);
    if (crsfFramePosition < fullFrameLength) {
      crsfFrameDone = 0;
    } else {
      crsfFrameDone = 1;
    }
    if (crsfFrameDone) {
      crsfFramePosition = 0;
    }
  }
}

void crsfFrameStatus(void) {
  if (crsfFrameDone == 0) {
    rx_frame_pending = 1; //flags when last time through we had a frame and this time we dont
  } else {
    crsfFrameDone = 0;
    if (crsfFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
      // CRC includes type and payload of each frame
      const uint8_t crc = crsfFrameCRC();
      if (crc != crsfFrame.frame.payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE]) {
        //              toss out bad frame
      } else {
        // unpack the RC channels
        const crsfPayloadRcChannelsPacked_t *const rcChannels = (crsfPayloadRcChannelsPacked_t *)&crsfFrame.frame.payload;
        crsfChannelData[0] = rcChannels->chan0;
        crsfChannelData[1] = rcChannels->chan1;
        crsfChannelData[2] = rcChannels->chan2;
        crsfChannelData[3] = rcChannels->chan3;
        crsfChannelData[4] = rcChannels->chan4;
        crsfChannelData[5] = rcChannels->chan5;
        crsfChannelData[6] = rcChannels->chan6;
        crsfChannelData[7] = rcChannels->chan7;
        crsfChannelData[8] = rcChannels->chan8;
        crsfChannelData[9] = rcChannels->chan9;
        crsfChannelData[10] = rcChannels->chan10;
        crsfChannelData[11] = rcChannels->chan11;
        crsfChannelData[12] = rcChannels->chan12;
        crsfChannelData[13] = rcChannels->chan13;
        crsfChannelData[14] = rcChannels->chan14;
        crsfChannelData[15] = rcChannels->chan15;
        framestarted = 1;
        rx_frame_pending = 0; //flags when last time through we didn't have a frame and this time we do
        bind_safety++;
      } // incriments up as good frames come in till we pass a safe point where aux channels are updated
    }
  }
}

void crsf_init(void) {
  failsafe = 1; //kill motors while initializing usart (maybe not necessary)
  serial_rx_init(RX_PROTOCOL_CRSF);
  // set setup complete flag
  framestarted = 0;
  rxmode = !RXMODE_BIND; // put LEDS in normal signal status
}

void rx_init(void) {
  crsf_init();
}

void checkrx() {
  if (framestarted < 0) {
    crsf_init(); // toggles "framestarted = 0;" after initializing
  }

  if (framestarted == 0) { // this is the failsafe condition
    failsafe = 1;          //keeps motors off while waiting for first frame and if no new frame for more than 1s
  }

  rx_frame_pending_last = rx_frame_pending;
  crsfFrameStatus();
  if (rx_frame_pending != rx_frame_pending_last)
    flagged_time = gettime(); //updates flag to current time only on changes of losing a frame or getting one back
  if (gettime() - flagged_time > FAILSAFETIME)
    framestarted = 0; //watchdog if more than 1 sec passes without a frame causes failsafe

  if (framestarted == 1) {
    if ((bind_safety < 900) && (bind_safety > 0))
      rxmode = RXMODE_BIND; // normal rx mode - removes waiting for bind led leaving failsafe flashes as data starts to come in

    // AETR channel order
    rx[0] = (crsfChannelData[0] - 990.5f) * 0.00125707103f;
    rx[1] = (crsfChannelData[1] - 990.5f) * 0.00125707103f;
    rx[2] = (crsfChannelData[3] - 990.5f) * 0.00125707103f;
    rx[3] = (crsfChannelData[2] - 191.0f) * 0.00062853551f;

    if (rx[3] > 1)
      rx[3] = 1;
    if (rx[3] < 0)
      rx[3] = 0;

    rx_apply_expo();

    aux[AUX_CHANNEL_0] = (crsfChannelData[4] > 1100) ? 1 : 0; //1100 cutoff intentionally selected to force aux channels low if
    aux[AUX_CHANNEL_1] = (crsfChannelData[5] > 1100) ? 1 : 0; //being controlled by a transmitter using a 3 pos switch in center state
    aux[AUX_CHANNEL_2] = (crsfChannelData[6] > 1100) ? 1 : 0;
    aux[AUX_CHANNEL_3] = (crsfChannelData[7] > 1100) ? 1 : 0;
    aux[AUX_CHANNEL_4] = (crsfChannelData[8] > 1100) ? 1 : 0;
    aux[AUX_CHANNEL_5] = (crsfChannelData[9] > 1100) ? 1 : 0;
    aux[AUX_CHANNEL_6] = (crsfChannelData[10] > 1100) ? 1 : 0;
    aux[AUX_CHANNEL_7] = (crsfChannelData[11] > 1100) ? 1 : 0;
    aux[AUX_CHANNEL_8] = (crsfChannelData[12] > 1100) ? 1 : 0;
    aux[AUX_CHANNEL_9] = (crsfChannelData[13] > 1100) ? 1 : 0;
    aux[AUX_CHANNEL_10] = (crsfChannelData[14] > 1100) ? 1 : 0;
    aux[AUX_CHANNEL_11] = (crsfChannelData[15] > 1100) ? 1 : 0;

    if (bind_safety > 100) { //requires 10 good frames to come in before rx_ready safety can be toggled to 1.  900 is about 2 seconds of good data
      rx_ready = 1;          // because aux channels initialize low and clear the binding while armed flag before aux updates high
      failsafe = 0;          // turn off failsafe delayed a bit to emmulate led behavior of sbus protocol - optional either here or just above here
      rxmode = !RXMODE_BIND; // restores normal led operation
      bind_safety = 101;     // reset counter so it doesnt wrap
    }
  }
}
#endif
