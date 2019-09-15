#include <stdbool.h>
#include <stdio.h>

#include "defines.h"
#include "drv_rx_serial.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "drv_uart.h"
#include "profile.h"
#include "project.h"
#include "util.h"

// sbus input ( pin SWCLK after calibration)
// WILL DISABLE PROGRAMMING AFTER GYRO CALIBRATION - 2 - 3 seconds after powerup)

#ifdef RX_UNIFIED_SERIAL

#define SERIAL_BAUDRATE 115200

// FPort is normally inverted
//#define INVERT_UART    //now handled by a function in rx_serial driver and a config define

// global use rx variables
extern float rx[4];
extern char aux[AUX_CHANNEL_MAX];
extern char lastaux[AUX_CHANNEL_MAX];
extern char auxchange[AUX_CHANNEL_MAX];
int failsafe = 9001; //OVER 9000
int rxmode = 0;
int rx_ready = 0;

// internal variables
#define RX_BUFF_SIZE 64
uint8_t rx_buffer[RX_BUFF_SIZE];
uint8_t rx_end = 0;
uint16_t rx_time[RX_BUFF_SIZE];
uint8_t bytesSinceStart = 0;
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
uint8_t data[64]; //Significantly larger than any known packets
int channels[16];
uint16_t CRCByte = 0; //Defined here to allow Debug to see it.

int failsafe_sbus_failsafe = 0;
int failsafe_siglost = 0;
int failsafe_noframes = 0;

// enable statistics
const int sbus_stats = 0;

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
bool FPORTDebugTelemetry = false;
extern float vbattfilt;
extern float vbatt_comp;
extern unsigned int lastlooptime;
uint8_t telemetryPacket[10];
extern int current_pid_axis;
extern int current_pid_term;
extern profile_t profile;
extern float lipo_cell_count;

uint16_t SbusTelemetryIDs[] = {
    0x0210, //VFAS, use for vbat_comp
    0x0211, //VFAS1, use for vbattfilt
    //Everything past here is only active in FPORT-Debug-Telemetry mode
    0x0900, //A3_FIRST_ID, used for cell count
    0x0400, //T1, used for Axis Identifier
    0x0700, //ACC-X, misused for PID-P
    0x0710, //ACC-X, misused for PID-I
    0x0720, //ACC-X, misused for PID-D
};
uint8_t telemetryPosition = 0; //This iterates through the above, you can only send one sensor per frame.
uint8_t teleCounter = 0;

void RX_USART_ISR(void) {
  rx_buffer[rx_end] = USART_ReceiveData(USART1);
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

  if (USART_GetFlagStatus(USART1, USART_FLAG_ORE)) {
    // overflow means something was lost
    rx_time[rx_end] = 0xFFFe;
    USART_ClearFlag(USART1, USART_FLAG_ORE);
    if (sbus_stats)
      stat_overflow++;
  }

  
  bytesSinceStart++;
  if (rx_buffer[rx_end] == 0x0f && rx_time[rx_end] > 21000) { //0x7e is both start and end for frames, as well as start/end for telemetry frames.
    frameStart = rx_end;                                      //a 0x7e after a "long" pause is a new frame starting
    bytesSinceStart = 0;
    frameStatus = 0; //0 is "frame in progress or first frame not arrived", 1 is "frame ready(ish) to be read",
    //2 is "Frame looks complete, CRC it and read controls", 3 is "frame already complete and processed (this ignores the telemetry packet)"
  } else if (bytesSinceStart > 39 && frameStatus == 0) { //It's long enough to be a full frame, and we're waiting for a full frame. Check it!
    frameStatus = 1;
  }
  
  rx_end++;
  rx_end %= (RX_BUFF_SIZE);
}

void rx_init(void) {
    
}

void checkrx() {

if(frameStatus < 50){
usart_rx_init(2);
frameStatus = 100;
}

}


#endif