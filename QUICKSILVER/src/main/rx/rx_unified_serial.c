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
#include "debug.h"

// sbus input ( pin SWCLK after calibration)
// WILL DISABLE PROGRAMMING AFTER GYRO CALIBRATION - 2 - 3 seconds after powerup)

#ifdef RX_UNIFIED_SERIAL

extern debug_type debug;






// global use rx variables
extern float rx[4];
extern char aux[AUX_CHANNEL_MAX];
extern char lastaux[AUX_CHANNEL_MAX];
extern char auxchange[AUX_CHANNEL_MAX];
uint8_t rxusart = 1;
int failsafe = 9001; //OVER 9000
int rxmode = 0;
int rx_ready = 0;

// internal variables
uint8_t RXProtocol = 0;
#define RX_BUFF_SIZE 68
uint8_t rx_buffer[RX_BUFF_SIZE];
uint8_t rx_end = 0;
uint16_t rx_time[RX_BUFF_SIZE];
uint8_t rx_data[RX_BUFF_SIZE]; //A place to put the RX frame so nothing can get overwritten during processing.
uint8_t bytesSinceStart = 0;
int frameStatus = -1;
uint8_t frameStart = 0;
uint8_t frameEnd = 0;
uint8_t telemetryCounter = 0;
uint8_t expectedFrameLength = 10;
uint8_t escapedChars = 0;
uint8_t rx_bind_enable = 0;

unsigned long time_lastrx;
unsigned long time_siglost;
uint8_t last_rx_end = 0;
unsigned long time_lastframe;
int rx_state = 0;
int bind_safety = 0;
int channels[16];
uint16_t CRCByte = 0; //Defined here to allow Debug to see it.
uint8_t protocolToCheck = 1;//Defined here to allow Debug to see it.
uint8_t protocolDetectTimer = 0;//Defined here to allow Debug to see it.


int failsafe_sbus_failsafe = 0;
int failsafe_siglost = 0;
int failsafe_noframes = 0;

uint8_t stat_frames_second;
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

  if (elapsedticks < 65536){
    rx_time[rx_end] = elapsedticks; //
  }
  else{
    rx_time[rx_end] = 65535; //ffff
  }

  lastticks = ticks;

  if (USART_GetFlagStatus(USART1, USART_FLAG_ORE)) {
    // overflow means something was lost
    rx_time[rx_end] = 0xFFFe;
    USART_ClearFlag(USART1, USART_FLAG_ORE);
  }

  bytesSinceStart++;
  if (rx_time[rx_end] > 9000) { //Long delay since last byte. Probably a new frame, or we missed some bytes. Either way, start over.
    frameStart = rx_end;                                      
    bytesSinceStart = 0;
    frameStatus = 0; //0 is "frame in progress or first frame not arrived", 
                     //1 is "frame ready(ish) to be read, or at least checked",
                     //2 is "Frame looks complete, CRC it and read controls", 
  }                  //3 is "frame already complete and processed, do nothing (or send telemetry if enabled)"
  else if (bytesSinceStart >= expectedFrameLength && frameStatus == 0) { //It's long enough to be a full frame, and we're waiting for a full frame. Check it!
    frameStatus = 1;
  }
  
  rx_end++;
  rx_end %= (RX_BUFF_SIZE);
  bobnovas+=100;
}

void rx_init(void) {
    
}

void rx_serial_init(void){
  frameStatus = 0; //Let the uart ISR do its stuff.
    if(RXProtocol == 0){ //No known protocol? Autodetect!
      findprotocol();
    }
    else{
      usart_rx_init(RXProtocol); //There's already a known protocol, we're good.
    }
    switch (RXProtocol)
    {
    case 1: // DSM
      expectedFrameLength = 16;
      break;
    case 2: // SBUS
      expectedFrameLength = 24;
      break;
    case 3: // IBUS
      expectedFrameLength = 32;
      break;
    case 4: // FPORT
      expectedFrameLength = 40; //Minimum. Can be up to 74 or so in theory. >95% are 40 though so we start there.
      break;
    case 5: // CRSF
      expectedFrameLength = 64; //Maybe 65? Not sure where the Sync Byte comes in
      break;
    
    default:
      break;
    }
}



void checkrx() {
  if(frameStatus == -1){ //RX/USART not set up.
    rx_serial_init(); //Set it up. This includes autodetecting protocol if necesary
    rxmode = !RXMODE_BIND;
  }

  if(frameStatus == 1){//USART ISR says there's enough frame to look at. Look at it.
    switch (RXProtocol)
    {
    case 1: // DSM
      
      break;
    case 2: // SBUS
    bobnovas++;
     processSBUS();
     
      break;
    case 3: // IBUS
      
      break;
    case 4: // FPORT
      
      break;
    case 5: // CRSF
      
      break;
    
    default:
      break;
    }
  }
 //FAILSAFE! It gets checked every time!     FAILSAFE! It gets checked every time!     FAILSAFE! It gets checked every time!     
if (gettime() - time_lastframe > 1000000) //Failsafe, it's important!
  {
    failsafe_noframes = 1;
  } else
    failsafe_noframes = 0;

  // add the 3 failsafes together
  failsafe = failsafe_noframes || failsafe_siglost || failsafe_sbus_failsafe;
}



void processDSMX(void){
  
}


void processSBUS(void){
    for (uint8_t counter = 0; counter < 25; counter++) {                //First up, get therx_data out of the RX buffer and into somewhere safe
     rx_data[counter] = rx_buffer[(counter + frameStart) % RX_BUFF_SIZE]; // This can probably go away, as long as the buffer is large enough
    }

    if (rx_data[23] & (1 << 2)) //RX sets this bit when it knows it missed a frame. Presumably this is a timer in the RX.
    {
      if (!time_siglost)
        time_siglost = gettime();
      if (gettime() - time_siglost > TICK_CLOCK_FREQ_HZ) //8,000,000 ticks on F0, 21M on F4. One second.
      {
        failsafe_siglost = 1;
      }
    } else {
      time_siglost = 0;
      failsafe_siglost = 0;
    }
    if (rx_data[23] & (1 << 3)) {
      failsafe_sbus_failsafe = 1; // Sbus packets have a failsafe bit. This is cool. If you forget to trust it you get programs though.
    } else {
      failsafe_sbus_failsafe = 0;
    }

    //Sbus channels are weird.
    channels[0] = ((rx_data[1] |rx_data[2] << 8) & 0x07FF);
    channels[1] = ((rx_data[2] >> 3 |rx_data[3] << 5) & 0x07FF);
    channels[2] = ((rx_data[3] >> 6 |rx_data[4] << 2 |rx_data[5] << 10) & 0x07FF);
    channels[3] = ((rx_data[5] >> 1 |rx_data[6] << 7) & 0x07FF);
    channels[4] = ((rx_data[6] >> 4 |rx_data[7] << 4) & 0x07FF);
    channels[5] = ((rx_data[7] >> 7 |rx_data[8] << 1 |rx_data[9] << 9) & 0x07FF);
    channels[6] = ((rx_data[9] >> 2 |rx_data[10] << 6) & 0x07FF);
    channels[7] = ((rx_data[10] >> 5 |rx_data[11] << 3) & 0x07FF);
    channels[8] = ((rx_data[12] |rx_data[13] << 8) & 0x07FF);

    if (rx_state == 0) //Stay in failsafe until we've received a stack of frames AND throttle is under 10% or so
    {
      // wait for valid sbus signal
      static int frame_count = 0;
      failsafe = 1;
      rxmode = RXMODE_BIND;
      // if throttle < 10%
      if (channels[2] < 300)
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
      channels[0] -= 993;
      channels[1] -= 993;
      channels[3] -= 993;

      rx[0] = channels[0];
      rx[1] = channels[1];
      rx[2] = channels[3];

      for (int i = 0; i < 3; i++) {
        rx[i] *= 0.00122026f;
      }

      channels[2] -= 173;
      rx[3] = 0.000610128f * channels[2];

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
      //and here we have the rest of the sbus AUX channels/
      /* Currently Silverware only has six AUX channels.
					aux[AUX_CHANNEL_6] = (channels[10] > 1600) ? 1 : 0;
					aux[AUX_CHANNEL_7] = (channels[11] > 1600) ? 1 : 0;
					aux[AUX_CHANNEL_8] = (channels[12] > 1600) ? 1 : 0;
					aux[AUX_CHANNEL_9] = (channels[13] > 1600) ? 1 : 0;
					*/

      time_lastframe = gettime();
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

  } // end frame received



void processIBUS(void){
  
}


void processFPORT(void){
  
}


void processCRSF(void){
  
}















//NOTE TO SELF: Put in some double-check code on the detections somehow.



void findprotocol(void){

rx_spektrum_bind(); // Light off DSMX binding, it *must* come first.
    protocolToCheck = 1; //Start with DSMX
  //uint8_t protocolToCheck = 1; Moved to global for visibility
  
  while(RXProtocol == 0){
    //uint8_t protocolDetectTimer = 0;Moved to global for visibility
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
        case 1: // DSM
          if(rx_buffer[frameStart] == 0x00 && rx_buffer[frameStart+1] == 0x00 && rx_buffer[frameStart] !=0x00){
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
      case 4: // FPORT
        if(rx_buffer[frameStart] == 0x7E){
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
  frameStatus = 3;//All done!
  debug.max_cpu_loop_number = gettime();
  lastlooptime = gettime();
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