//Yes, this started life as a copy/paste of rx_fport.c, which started as a copy/paste/gut/refill of rx_sbus.c

//The content of this file, other than what came from rx_sbus.c was written by Bobnova in March/April 2019.

// serial for stm32 not used yet
#include <stdio.h>

#include "defines.h"
#include "drv_fmc.h"
#include "drv_rx_serial.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "drv_uart.h"
#include "project.h"
#include "util.h"

// iBus input ( pin SWCLK after calibration)
// WILL DISABLE PROGRAMMING AFTER GYRO CALIBRATION - 2 - 3 seconds after powerup)

#ifdef RX_IBUS

#define SERIAL_BAUDRATE 115200

// global use rx variables
extern float rx[4];
extern char aux[AUXNUMBER];
extern char lastaux[AUXNUMBER];
extern char auxchange[AUXNUMBER];
int failsafe = 1337; //It ain't safe if it ain't there
int rxmode = 0;
int rx_ready = 0;

// internal iBus variables
#define RX_BUFF_SIZE 40
uint8_t rx_buffer[RX_BUFF_SIZE];
uint8_t rx_end = 0; //Replaced by the struct above
uint16_t rx_time[RX_BUFF_SIZE];
uint8_t frameLengthReceived = 0;
int frameStatus = -1;
uint8_t frameStart = 0;
uint8_t frameEnd = 0;
uint8_t telemetryCounter = 0;
uint8_t frameLength = 0;
uint8_t escapedChars = 0;

unsigned long time_lastrx;
unsigned long time_siglost;
uint8_t last_rx_end = 0;
int last_byte = 0;
unsigned long time_lastframe;
int frame_received = 0;
int rx_state = 0;
int bind_safety = 0;
uint8_t data[33];
int channels[16];
uint16_t CRCByte = 0; //Defined here to allow Debug to see it.

int failsafe_ibus_failsafe = 0;
int failsafe_siglost = 0;
int failsafe_noframes = 0;

// enable statistics
const int ibus_stats = 0;

// statistics
int stat_framestartcount;
int stat_timing_fail;
int stat_garbage;
//int stat_timing[25];
int stat_frames_accepted = 0;
int stat_frames_second;
int stat_overflow;
uint32 ticksStart = 0;
uint32 ticksEnd = 0;
uint32 ticksLongest = 0;

//Telemetry variables

//Global values to send as telemetry
extern float vbattfilt;
extern float vbatt_comp;
extern unsigned int lastlooptime;
uint8_t telemetryPacket[10];

uint16_t telemetryIDs[] = {
    0x0210, //VFAS, use for vbat_comp
    0x0211, //VFAS1, use for vbattfilt
};
uint8_t telemetryPosition = 0; //This iterates through the above, you can only send one sensor per frame.
uint8_t teleCounter = 0;

//void SERIAL_RX_USART _IRQHandler(void)
void RX_USART_ISR(void) {
  rx_buffer[rx_end] = USART_ReceiveData(SERIAL_RX_USART);
  // calculate timing since last rx
  unsigned long maxticks = SysTick->LOAD;
  unsigned long ticks = SysTick->VAL;
  unsigned long elapsedticks;
  static unsigned long lastticks;
  if (ticks < lastticks)
    elapsedticks = lastticks - ticks;
  else { // overflow ( underflow really)
    elapsedticks = lastticks + (maxticks - ticks);
  }

  if (elapsedticks < 65536)
    rx_time[rx_end] = elapsedticks; //
  else
    rx_time[rx_end] = 65535; //ffff

  lastticks = ticks;

  if (USART_GetFlagStatus(SERIAL_RX_USART, USART_FLAG_ORE)) {
    // overflow means something was lost
    rx_time[rx_end] = 0xFFFe;
    USART_ClearFlag(SERIAL_RX_USART, USART_FLAG_ORE);
    if (ibus_stats)
      stat_overflow++;
  }

  frameLengthReceived++;
  if (rx_buffer[rx_end] == 0x20 && rx_time[rx_end] > 21000) {
    frameStart = rx_end;
    frameLengthReceived = 1;
    frameStatus = 0;                                          //0 is "frame in progress or first frame not arrived", 1 is "frame ready to be read",
                                                              //3 is "frame already complete and processed (this ignores the telemetry packet)", 2 is unused.
  } else if (frameLengthReceived == 32 && frameStatus == 0) { //iBus frames are always 32 bytes. Convenient!
    frameStatus = 1;
  }
  rx_end++;
  rx_end %= (RX_BUFF_SIZE);
}

void ibus_init(void) {
  //Serial init bits are all in drv_rx_serial.c now
  usart_rx_init(); //Which is called here.
  rxmode = !RXMODE_BIND;

  // set setup complete flag
  frameStatus = 0;
}

void rx_init(void) {
}

void checkrx() {
  if (frameStatus < 0) //can't read a packet before you set up the uart.
  {
    // initialize ibus
    ibus_init();
    // set in routine above "frameStatus = 0;"
  }

  if (frameStatus == 1) //UART says there's something to look at. Let's look at it.
  {
    frameLength = 0;
    for (uint8_t counter = 0; counter < 32; counter++) {                //First up, get the data out of the RX buffer and into somewhere safe
      data[counter] = rx_buffer[(counter + frameStart) % RX_BUFF_SIZE]; // This can probably go away, as long as the buffer is large enough
      frameLength++;                                                    // to accept telemetry requests without overwriting control data
    }

    CRCByte = 0xFFFF;
    for (int x = 0; x < 30; x++) {
      CRCByte = CRCByte - data[x];
    }

    if (CRCByte == data[30] + (data[31] << 8)) { //If the CRC is good, shove it into controls

      //Flysky channels are delightfully straightforward
      channels[0] = data[2] + (data[3] << 8);
      channels[1] = data[4] + (data[5] << 8);
      channels[2] = data[6] + (data[7] << 8);
      channels[3] = data[8] + (data[9] << 8);
      channels[4] = data[10] + (data[11] << 8);
      channels[5] = data[12] + (data[13] << 8);
      channels[6] = data[14] + (data[15] << 8);
      channels[7] = data[16] + (data[17] << 8);
      channels[8] = data[18] + (data[19] << 8);
      channels[9] = data[20] + (data[21] << 8);
      channels[10] = data[22] + (data[23] << 8);
      channels[11] = data[24] + (data[25] << 8);
      channels[12] = data[26] + (data[27] << 8);
      channels[13] = data[28] + (data[29] << 8);

      if (rx_state == 0) //Stay in failsafe until we've received a stack of frames AND throttle is under 10% or so
      {
        // wait for valid ibus signal
        static int frame_count = 0;
        failsafe = 1;
        rxmode = RXMODE_BIND;
        // if throttle < 10%
        if (channels[2] < 1100)
          frame_count++; //AETR!
        if (frame_count > 130) {
          if (stat_frames_second > 30) {
            rx_state++;
            rxmode = !RXMODE_BIND;
          } else {
            frame_count = 0;
          }
        }
      }

      if (rx_state == 1) {
        // normal rx mode

        // AETR channel order
        channels[0] -= 1500;
        channels[1] -= 1500;
        channels[2] -= 1000;
        channels[3] -= 1500;

        rx[0] = channels[0];
        rx[1] = channels[1];
        rx[2] = channels[3];
        rx[3] = channels[2];

        for (int i = 0; i < 3; i++) {
          rx[i] *= 0.002f;
        }
        rx[3] *= 0.001f;

        if (rx[3] > 1)
          rx[3] = 1;
        if (rx[3] < 0)
          rx[3] = 0;

        rx_apply_expo();

        //Here we have the AUX channels Silverware supports
        aux[AUX_CHANNEL_0] = (channels[4] > 1600) ? 1 : 0;
        aux[AUX_CHANNEL_1] = (channels[5] > 1600) ? 1 : 0;
        aux[AUX_CHANNEL_2] = (channels[6] > 1600) ? 1 : 0;
        aux[AUX_CHANNEL_3] = (channels[7] > 1600) ? 1 : 0;
        aux[AUX_CHANNEL_4] = (channels[8] > 1600) ? 1 : 0;
        aux[AUX_CHANNEL_5] = (channels[9] > 1600) ? 1 : 0;
        //and here we have the rest of the iBus AUX channels/
        /* Currently Silverware only has six AUX channels.
					aux[AUX_CHANNEL_6] = (channels[10] > 1600) ? 1 : 0;
					aux[AUX_CHANNEL_7] = (channels[11] > 1600) ? 1 : 0;
					aux[AUX_CHANNEL_8] = (channels[12] > 1600) ? 1 : 0;
					aux[AUX_CHANNEL_9] = (channels[13] > 1600) ? 1 : 0;
					*/

        time_lastframe = gettime();
        if (ibus_stats)
          stat_frames_accepted++;
        if (bind_safety > 141) { //requires one second worth of good frames to come in before rx_ready safety can be toggled to 1
          rx_ready = 1;          // because aux channels initialize low and clear the binding while armed flag before aux updates high
          bind_safety = 142;
        }
      }

      // stats
      static int fps = 0;
      static unsigned long secondtime = 0;

      if (gettime() - secondtime > 1000000) {
        stat_frames_second = fps;
        fps = 0;
        secondtime = gettime();
      }
      fps++;
      frameStatus = 3; //We're done with this frame now.
      bind_safety++;   // It was a good frame, increment the good frame counter.

    } else { // if CRC fails, do this:
      //while(1){} Enable for debugging to lock the FC if CRC fails. In the air we just drop CRC-failed packets
    }
  } // end frame received

  if (gettime() - time_lastframe > 1000000) //Failsafe, it's important!
  {
    failsafe_noframes = 1;
  } else
    failsafe_noframes = 0;

  // add the 3 failsafes together
  failsafe = failsafe_noframes || failsafe_siglost || failsafe_ibus_failsafe;
}

#endif
