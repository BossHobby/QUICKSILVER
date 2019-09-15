#include <stdbool.h>
#include <stdio.h>
#include "defines.h"
#include "drv_fmc.h"
#include "drv_rx_serial.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "drv_uart.h"
#include "profile.h"
#include "project.h"
#include "util.h"
#include "led.h"
#include "rx.h"

// sbus input ( pin SWCLK after calibration)
// WILL DISABLE PROGRAMMING AFTER GYRO CALIBRATION - 2 - 3 seconds after powerup)

#ifdef RX_UNIFIED_SERIAL

#define SERIAL_BAUDRATE 115200


// global use rx variables
extern float rx[4];
extern char aux[AUX_CHANNEL_MAX];
extern char lastaux[AUX_CHANNEL_MAX];
extern char auxchange[AUX_CHANNEL_MAX];
uint8_t rxusart = 2;
int failsafe = 9001; //OVER 9000
int rxmode = 0;
int rx_ready = 0;

// internal variables
uint8_t RXProtocol = 0;
#define RX_BUFF_SIZE 64
uint8_t rx_buffer[RX_BUFF_SIZE];
uint8_t rx_end = 0;
uint16_t rx_time[RX_BUFF_SIZE];
uint8_t bytesSinceStart = 0;
int frameStatus = -1;
uint8_t frameStart = 0;
uint8_t frameEnd = 0;
uint8_t telemetryCounter = 0;
uint8_t frameLength = 10;
uint8_t escapedChars = 0;
uint8_t rx_bind_enable = 0;

unsigned long time_lastrx;
unsigned long time_siglost;
uint8_t last_rx_end = 0;
int last_byte = 0;
unsigned long time_lastframe;
int frame_received = 0;
int rx_state = 0;
int bind_safety = 0;
uint8_t data[64]; //Significantly larger than any known frames
int channels[16];
uint16_t CRCByte = 0; //Defined here to allow Debug to see it.
uint8_t protocolToCheck = 1;//Defined here to allow Debug to see it.
uint8_t protocolDetectTimer = 0;//Defined here to allow Debug to see it.



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
int bobnovas = 0;

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
  if (rx_time[rx_end] > 9000) { //Long delay since last byte. Probably a new frame, or we missed some bytes. Either way, start over.
    frameStart = rx_end;                                      
    bytesSinceStart = 0;
    frameStatus = 0; //0 is "frame in progress or first frame not arrived", 
                     //1 is "frame ready(ish) to be read, or at least checked",
                     //2 is "Frame looks complete, CRC it and read controls", 
                     //3 is "frame already complete and processed, do nothing (or send telemetry if enabled)"
  } 
  else if (bytesSinceStart >= frameLength && frameStatus == 0) { //It's long enough to be a full frame, and we're waiting for a full frame. Check it!
    frameStatus = 1;
  }
  
  rx_end++;
  rx_end %= (RX_BUFF_SIZE);
}

void rx_init(void) {
    
}





void checkrx() {
  frameStatus = 0; // We're ready for radio packets
  if(RXProtocol == 0){
    findprotocol();
  }
  else{
    usart_rx_init(RXProtocol);
  }
  
  

}


//NOTE TO SELF: Put in some double-check code on the detections somehow.



void findprotocol(void){
  bobnovas++;
  //uint8_t protocolToCheck = 1; Moved to global for visibility
  while(RXProtocol == 0){
    bobnovas++;
    //uint8_t protocolDetectTimer = 0;Moved to global for visibility
    //protocolToCheck = 3;
    usart_rx_init(protocolToCheck); //Configure a protocol!
    delay(500000);
    protocolDetectTimer = 0;
    
    while((frameStatus == 0 || frameStatus == 3) && protocolDetectTimer < 5){ // Wait 5 seconds to see if something turns up. 
    bobnovas = bobnovas + 10;
      for(int i = 0; i < protocolToCheck; i++){
        ledon(255);
        delay(20000);
        ledoff(255);
        delay(50000);
        }
      ledoff(255);
      delay(1000000 - (protocolToCheck * 70000));
      protocolDetectTimer++;
    }
    if(frameStatus == 1){//We got something! What is it?
      switch (protocolToCheck)
      {
      case 1: // FPORT
        if(rx_buffer[frameStart] == 0x7E){
          RXProtocol = protocolToCheck;
        }
        break;
      case 2: // SBUS
        if(rx_buffer[frameStart] == 0x0F){
          RXProtocol = protocolToCheck;
        }
        break;
      case 3: // IBUS
        if(rx_buffer[frameStart] == 0x20){  
          RXProtocol = protocolToCheck;
        }
        break;
      case 4: // DSM
        if(rx_buffer[frameStart] == 0x00 && rx_buffer[frameStart+1] == 0x00 && rx_buffer[frameStart] !=0x00){
          RXProtocol = protocolToCheck;
        }
        break;
      case 5: // CRSF
        if(rx_buffer[frameStart] != 0xFF && 1 == 2){  //Need to look up the expected start value.
          RXProtocol = protocolToCheck;
        }
        break;
      default:
        frameStatus = 3; //Whatever we got, it didn't make sense. Mark the frame as Checked and start over.
        break;
      }
      //if(rx_buffer[frameStart])
    }
    
    protocolToCheck++;
    if(protocolToCheck > 5){
      protocolToCheck = 1;
      rxusart = 1;
    }
  }
}



// Send Spektrum bind pulses to a GPIO e.g. TX1
void rx_spektrum_bind(void) {
//
  rx_bind_enable = fmc_read_float(56);
  if (rx_bind_enable == 0) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = SERIAL_RX_SPEKBIND_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SERIAL_RX_PORT, &GPIO_InitStructure);

    // RX line, set high
    GPIO_SetBits(SERIAL_RX_PORT, SERIAL_RX_SPEKBIND_RX_PIN);
    // Bind window is around 20-140ms after powerup
    delay(60000);

    for (uint8_t i = 0; i < 9; i++) { // 9 pulses for internal dsmx 11ms, 3 pulses for internal dsm2 22ms
      // RX line, drive low for 120us
      GPIO_ResetBits(SERIAL_RX_PORT, SERIAL_RX_SPEKBIND_RX_PIN);
      delay(120);

      // RX line, drive high for 120us
      GPIO_SetBits(SERIAL_RX_PORT, SERIAL_RX_SPEKBIND_RX_PIN);
      delay(120);
    }
  }
//#endif
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = SERIAL_RX_SPEKBIND_BINDTOOL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SERIAL_RX_PORT, &GPIO_InitStructure);

  // RX line, set high
  GPIO_SetBits(SERIAL_RX_PORT, SERIAL_RX_SPEKBIND_BINDTOOL_PIN);
  // Bind window is around 20-140ms after powerup
  delay(60000);

  for (uint8_t i = 0; i < 9; i++) { // 9 pulses for internal dsmx 11ms, 3 pulses for internal dsm2 22ms
    // RX line, drive low for 120us
    GPIO_ResetBits(SERIAL_RX_PORT, SERIAL_RX_SPEKBIND_BINDTOOL_PIN);
    delay(120);

    // RX line, drive high for 120us
    GPIO_SetBits(SERIAL_RX_PORT, SERIAL_RX_SPEKBIND_BINDTOOL_PIN);
    delay(120);
  }
}



#endif