//Yes, this started life as a copy/paste of rx_sbus.c
// F.port just stuffs sbus and smartport into one lump and one pin/uart.

//NOTE: This implementation of FPORT uses the RX pin, NOT the more typical TX pin!

//This file, other than the bits copied from rx_sbus.c, was created by Bobnova in March of 2019.

// serial for stm32 not used yet
#include <stdbool.h>
#include <stdio.h>

#include "defines.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "profile.h"
#include "project.h"
#include "util.h"

// sbus input ( pin SWCLK after calibration)
// WILL DISABLE PROGRAMMING AFTER GYRO CALIBRATION - 2 - 3 seconds after powerup)

#ifdef RX_FPORT

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

// internal FPORT variables
#define RX_BUFF_SIZE 86
uint8_t rx_buffer[RX_BUFF_SIZE];
uint8_t rx_end = 0; //Replaced by the struct above
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
uint8_t data[48]; //Normal frame is 40 bytes, this allows for up to 8 escaped characters.
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
float rx_rssi;

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

uint16_t telemetryIDs[] = {
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

#define USART usart_port_defs[serial_rx_port]

void RX_USART_ISR(void) {
  rx_buffer[rx_end] = USART_ReceiveData(USART.channel);
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

  if (USART_GetFlagStatus(USART.channel, USART_FLAG_ORE)) {
    // overflow means something was lost
    rx_time[rx_end] = 0xFFFe;
    USART_ClearFlag(USART.channel, USART_FLAG_ORE);
    if (sbus_stats)
      stat_overflow++;
  }

  bytesSinceStart++;
  if (rx_buffer[rx_end] == 0x7e && rx_time[rx_end] > 21000) { //0x7e is both start and end for frames, as well as start/end for telemetry frames.
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

void fport_init() {
  // initialize USART for FPORT
  serial_rx_init(RX_PROTOCOL_FPORT); //initialize usart in drv_rx_serial
  rxmode = !RXMODE_BIND;
  // set setup complete flag
  frameStatus = 0;
}

void rx_init(void) {
  fport_init();
}

void rx_check() {
  if (frameStatus < 0) {
    //can't read a packet before you set up the uart.
    fport_init();
  }

  if (frameStatus == 1) //UART says there's something to look at. Let's look at it.
  {
    frameLength = 0;
    uint8_t tempEscapedChars = 0;
    for (uint8_t counter = 0; counter <= bytesSinceStart; counter++) {                     //First up, get the data out of the RX buffer and into somewhere safe
      data[counter] = rx_buffer[(counter + tempEscapedChars + frameStart) % RX_BUFF_SIZE]; //The RX may well receive additional bytes while we process!
      frameLength++;

      if (data[counter - 1] == 0x7D) { //0x7D and 0x7E are reserved, 0x7D is the marker for a reserved / escaped character
        if (data[counter] == 0x5E) {   //0x5E in the byte following 0x7D means it was a 0x7E data byte.
          data[counter - 1] = 0x7E;    //So, make it 0x7E.
          escapedChars++;              //Now we have to get rid of the "current" byte, and adjust the incoming frame length.
          tempEscapedChars++;
          counter--;
          frameLength--;
        } else if (data[counter] == 0x5D) {
          //data[counter - 1] = 0x7D;  It already is, this line is a reminder
          escapedChars++;
          tempEscapedChars++;
          counter--;
          frameLength--;
        }
      }

      if (data[counter] == 0x7e && data[counter - 1] == 0x7e && frameLength > 29) { //Looks like a complete frame, check CRC and process controls if it's good.
        frameStatus = 2;
        counter = 200; //Breaks out of the for loop processing the data array.
        //The telemetry request packet is not read, as it never seems to change. Less control lag if we ignore it.
      }
    }

    if (frameStatus == 2) { //If it looks complete, process it further.

      CRCByte = 0;
      for (int x = 1; x < frameLength - 2; x++) {
        CRCByte = CRCByte + data[x];
      }
      CRCByte = CRCByte + (CRCByte >> 8);
      CRCByte = CRCByte << 8;
      CRCByte = CRCByte >> 8;
      if (CRCByte == 0x00FF) { //CRC is good, check Failsafe bit(s) and shove it into controls

        //FPORT uses SBUS style data, but starts further in the packet

        if (data[25] & (1 << 2)) //RX appears to set this bit when it knows it missed a frame. How does it know?
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
        if (data[25] & (1 << 3)) {
          failsafe_sbus_failsafe = 1; // Sbus packets have a failsafe bit. This is cool.
        } else {
          failsafe_sbus_failsafe = 0;
        }

        channels[0] = ((data[3] | data[4] << 8) & 0x07FF);
        channels[1] = ((data[4] >> 3 | data[5] << 5) & 0x07FF);
        channels[2] = ((data[5] >> 6 | data[6] << 2 | data[7] << 10) & 0x07FF);
        channels[3] = ((data[7] >> 1 | data[8] << 7) & 0x07FF);
        channels[4] = ((data[8] >> 4 | data[9] << 4) & 0x07FF);
        channels[5] = ((data[9] >> 7 | data[10] << 1 | data[11] << 9) & 0x07FF);
        channels[6] = ((data[11] >> 2 | data[12] << 6) & 0x07FF);
        channels[7] = ((data[12] >> 5 | data[13] << 3) & 0x07FF);
        channels[8] = ((data[14] | data[15] << 8) & 0x07FF);
        channels[9] = ((data[15] >> 3 | data[16] << 5) & 0x07FF); //This is the last channel Silverware previously supported.
        channels[10] = ((data[16] >> 6 | data[17] << 2 | data[18] << 10) & 0x07FF);
        channels[11] = ((data[18] >> 1 | data[19] << 7) & 0x07FF);
        channels[12] = ((data[19] >> 4 | data[20] << 4) & 0x07FF);
        channels[13] = ((data[20] >> 7 | data[21] << 1 | data[22] << 9) & 0x07FF);
        channels[14] = ((data[22] >> 2 | data[23] << 6) & 0x07FF);
        channels[15] = ((data[23] >> 5 | data[24] << 3) & 0x07FF);

        if (rx_state == 0) //Stay in failsafe until we've received a stack of frames AND throttle is under 10% or so
        {
          // wait for valid sbus signal
          static int frame_count = 0;
          failsafe = 1;
          rxmode = RXMODE_BIND;
          // if throttle < 10%
          if (channels[2] < 336)
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
          //I don't know what any of this really does.
          for (int i = 0; i < 3; i++) {
            rx[i] *= 0.00122026f;
          }

          channels[2] -= 173;
          rx[3] = 0.000610128f * channels[2];

          if (rx[3] > 1)
            rx[3] = 1;
          if (rx[3] < 0)
            rx[3] = 0;

          //I also don't know why expo is being applied in RX code.
          rx_apply_expo();

          //Here we have the AUX channels Silverware supports
          aux[AUX_CHANNEL_0] = (channels[4] > 993) ? 1 : 0;
          aux[AUX_CHANNEL_1] = (channels[5] > 993) ? 1 : 0;
          aux[AUX_CHANNEL_2] = (channels[6] > 993) ? 1 : 0;
          aux[AUX_CHANNEL_3] = (channels[7] > 993) ? 1 : 0;
          aux[AUX_CHANNEL_4] = (channels[8] > 993) ? 1 : 0;
          aux[AUX_CHANNEL_5] = (channels[9] > 993) ? 1 : 0;
          aux[AUX_CHANNEL_6] = (channels[10] > 993) ? 1 : 0;
          aux[AUX_CHANNEL_7] = (channels[11] > 993) ? 1 : 0;
          aux[AUX_CHANNEL_8] = (channels[12] > 993) ? 1 : 0;
          aux[AUX_CHANNEL_9] = (channels[13] > 993) ? 1 : 0;
          aux[AUX_CHANNEL_10] = (channels[14] > 993) ? 1 : 0;
          aux[AUX_CHANNEL_11] = (channels[15] > 993) ? 1 : 0;

          if (channels[12] > 993) { // Channel 13 is now FPORT Debug Telemetry switch. Integrate this better sometime
            FPORTDebugTelemetry = true;
          } else {
            FPORTDebugTelemetry = false;
          }

          time_lastframe = gettime();
          if (sbus_stats)
            stat_frames_accepted++;
          if (bind_safety > 111) { //requires one second worth of good frames to come in before rx_ready safety can be toggled to 1
            rx_ready = 1;          // because aux channels initialize low and clear the binding while armed flag before aux updates high
            bind_safety = 112;
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

        if (telemetryCounter > 0) { // Send telemetry back after every other packet. This gives the RX time to send ITS telemetry back
          telemetryCounter = 0;

          //Telemetry time! Let's have some variables
          //uint8_t telemetryPacket[10]; //Redefining this every pass is (probably) faster than zeroing it.
          uint16_t teleCRC = 0; //Really, really want to start with a zerod CRC!

          telemetryPacket[0] = 0x08; //Bytes 0 through 2 are static in this implementation
          telemetryPacket[1] = 0x81;
          telemetryPacket[2] = 0x10;
          //Note: This does not properly escape 0x7E and 0x7D characters. Some telemetry packets will fail CRC on the other end. This will be fixed.
          if (telemetryPosition == 0) {                                //vbat_comp
            telemetryPacket[3] = telemetryIDs[telemetryPosition];      //0x10;
            telemetryPacket[4] = telemetryIDs[telemetryPosition] >> 8; //0x02;
            telemetryPacket[5] = (int)(vbatt_comp * 100);
            telemetryPacket[6] = (int)(vbatt_comp * 100) >> 8;
            telemetryPacket[7] = 0x00;
            telemetryPacket[8] = 0x00;
          } else if (telemetryPosition == 1) {                         //vbattfilt
            telemetryPacket[3] = telemetryIDs[telemetryPosition];      //x11;
            telemetryPacket[4] = telemetryIDs[telemetryPosition] >> 8; //0x02;
            telemetryPacket[5] = (int)(vbattfilt * 100);
            telemetryPacket[6] = (int)(vbattfilt * 100) >> 8;
            telemetryPacket[7] = 0x00;
            telemetryPacket[8] = 0x00;
          }

          else if (telemetryPosition == 2) { //Cell count
            telemetryPacket[3] = telemetryIDs[telemetryPosition];
            telemetryPacket[4] = telemetryIDs[telemetryPosition] >> 8;
            telemetryPacket[5] = (int)(lipo_cell_count * 100);
            telemetryPacket[6] = (int)(lipo_cell_count * 100) >> 8;
            telemetryPacket[7] = 0x00;
            telemetryPacket[8] = 0x00;
          } else if (telemetryPosition == 3) {                         //PID axis(hundreds column) and P/I/D (ones column) being adjusted currently
            uint16_t axisAndPidID = (current_pid_axis + 1) * 100;      //Adding one so there's always a value. 1 for Pitch (or Pitch/roll), 2 for Roll, 3 for Yaw
            axisAndPidID += current_pid_term + 1;                      //Adding one here too, humans don't deal well with counting starting at zero for this sort of thing
            telemetryPacket[4] = telemetryIDs[telemetryPosition] >> 8; // Adding one to the above makes it match the LED flash codes too
            telemetryPacket[5] = (int)(axisAndPidID);
            telemetryPacket[6] = (int)(axisAndPidID) >> 8;
            telemetryPacket[7] = 0x00;
            telemetryPacket[8] = 0x00;
          } else if (telemetryPosition == 4) { //PID-P
            telemetryPacket[3] = telemetryIDs[telemetryPosition];
            telemetryPacket[4] = telemetryIDs[telemetryPosition] >> 8;
            telemetryPacket[5] = (int)(profile_current_pid_rates()->kp.axis[current_pid_axis] * 10000);
            telemetryPacket[6] = (int)(profile_current_pid_rates()->kp.axis[current_pid_axis] * 10000) >> 8;
            telemetryPacket[7] = 0x00;
            telemetryPacket[8] = 0x00;
          } else if (telemetryPosition == 5) { //PID-I
            telemetryPacket[3] = telemetryIDs[telemetryPosition];
            telemetryPacket[4] = telemetryIDs[telemetryPosition] >> 8;
            telemetryPacket[5] = (int)(profile_current_pid_rates()->ki.axis[current_pid_axis] * 1000);
            telemetryPacket[6] = (int)(profile_current_pid_rates()->ki.axis[current_pid_axis] * 1000) >> 8;
            telemetryPacket[7] = 0x00;
            telemetryPacket[8] = 0x00;
          } else if (telemetryPosition == 6) { //PID-D
            telemetryPacket[3] = telemetryIDs[telemetryPosition];
            telemetryPacket[4] = telemetryIDs[telemetryPosition] >> 8;
            telemetryPacket[5] = (int)(profile_current_pid_rates()->kd.axis[current_pid_axis] * 1000);
            telemetryPacket[6] = (int)(profile_current_pid_rates()->kd.axis[current_pid_axis] * 1000) >> 8;
            telemetryPacket[7] = 0x00;
            telemetryPacket[8] = 0x00;
          }

          //Calculate CRC for packet. This function does not support escaped characters.

          for (int x = 0; x < 9; x++) {
            teleCRC = teleCRC + telemetryPacket[x];
          }
          teleCRC = teleCRC + (teleCRC >> 8);
          teleCRC = 0xff - teleCRC;
          teleCRC = teleCRC << 8;
          teleCRC = teleCRC >> 8;
          telemetryPacket[9] = teleCRC;      //0x34;
          for (uint8_t x = 0; x < 10; x++) { //Shove the packet out the UART. This also doesn't support escaped characters
            while (USART_GetFlagStatus(USART.channel, USART_FLAG_TXE) == RESET)
              ; // Wait for Empty
            USART_SendData(USART.channel, telemetryPacket[x]);
          } //That's it, telemetry sent
          telemetryPosition++;
          if (FPORTDebugTelemetry) {
            if (telemetryPosition >= sizeof(telemetryIDs) / 2) // 2 byte ints, so this should give the number of entries. It just incremented, which takes care of the count with 0 or 1
            {
              telemetryPosition = 0;
            }
          } else {
            if (telemetryPosition == 2) {
              telemetryPosition = 0;
            }
          }

        } else {
          telemetryCounter++;
        }
      } else { // if CRC fails, do this:
        //while(1){} Enable for debugging to lock the FC if CRC fails. In the air we just drop CRC-failed packets
      }
    } // end frame received
  }
  if (gettime() - time_lastframe > 1000000) //Failsafe, it's important!
  {
    failsafe_noframes = 1;
  } else
    failsafe_noframes = 0;

  // add the 3 failsafes together
  failsafe = failsafe_noframes || failsafe_siglost || failsafe_sbus_failsafe;
}

#endif
