// serial for stm32 not used yet
#include <stdio.h>

#include "defines.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "profile.h"
#include "project.h"
#include "util.h"

// sbus input ( pin SWCLK after calibration)
// WILL DISABLE PROGRAMMING AFTER GYRO CALIBRATION - 2 - 3 seconds after powerup)

#ifdef RX_SBUS

//#define SERIAL_BAUDRATE 100000   //moved to rx_serial driver

// sbus is normally inverted
//#define SBUS_INVERT 1            //now handled by a function in rx_serial driver and a config define

// global use rx variables
extern float rx[4];
extern char aux[AUX_CHANNEL_MAX];
extern char lastaux[AUX_CHANNEL_MAX];
extern char auxchange[AUX_CHANNEL_MAX];
int failsafe = 1; // It isn't safe if we haven't checked it!
int rxmode = 0;
int rx_ready = 0;

// internal sbus variables
#define RX_BUFF_SIZE 32          //25 byte frames, plus some spare
uint8_t rx_buffer[RX_BUFF_SIZE]; //Create the buffer
uint8_t rx_start = 0;
uint8_t rx_end = 0;
uint8_t rx_time[RX_BUFF_SIZE]; //For timing the byte arrivals and finding the beginning of a packet

int framestarted = -1;
uint8_t framestart = 0;

unsigned long time_lastrx;
unsigned long time_siglost;
uint8_t last_rx_end = 0;
int last_byte = 0;
unsigned long time_lastframe;
int frame_received = 0;
int rx_state = 0;
int bind_safety = 0;
uint8_t data[RX_BUFF_SIZE];
int channels[16];

int failsafe_sbus_failsafe = 0;
int failsafe_siglost = 0;
int failsafe_noframes = 0;

uint8_t frameLengthReceived = 0;
int frameStatus = -1;
uint8_t frameStart = 0;
uint8_t frameEnd = 0;
uint8_t frameLength = 0;

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

extern profile_t profile;
float rx_rssi;

#define USART usart_port_defs[serial_rx_port]

//void SERIAL_RX_USART_IRQHandler(void)
void RX_USART_ISR(void) {
  rx_buffer[rx_end] = USART_ReceiveData(USART.channel);
  // calculate timing since last rx
  uint16_t maxticks = (SysTick->LOAD) / 260; // Hack the 24bit counters down to 16bit
  uint16_t ticks = (SysTick->VAL) / 260;
  uint16_t elapsedticks;
  static uint16_t lastticks;
  if (ticks < lastticks)
    elapsedticks = lastticks - ticks;
  else { // overflow ( underflow really)
    elapsedticks = lastticks + (maxticks - ticks);
  }

  if (elapsedticks < 255) {         //
    rx_time[rx_end] = elapsedticks; // Now that the counters are 16bit the byte timing can be 8bit. RAM savings!
  } else {
    rx_time[rx_end] = 255; //ff
  }

  lastticks = ticks;

  if (USART_GetFlagStatus(USART.channel, USART_FLAG_ORE)) {
    // overflow means something was lost
    rx_time[rx_end] = 0xFe;
    USART_ClearFlag(USART.channel, USART_FLAG_ORE);
    if (sbus_stats)
      stat_overflow++;
  }

  frameLengthReceived++;
  if (rx_buffer[rx_end] == 0x0F && rx_time[rx_end] > 80) {
    frameStart = rx_end;
    frameLengthReceived = 1;
    frameStatus = 0;                                          //0 is "frame in progress or first frame not arrived", 1 is "frame ready to be read",
                                                              //3 is "frame already complete and processed (this ignores the telemetry packet)", 2 is unused.
  } else if (frameLengthReceived == 25 && frameStatus == 0) { //sbus frames are always 24 bytes. Convenient!
    frameStatus = 1;
  }
  rx_end++;
  rx_end %= (RX_BUFF_SIZE);
}

// initialize sbus
void sbus_init(void) {
  serial_rx_init(RX_PROTOCOL_SBUS); //initialize usart in drv_rx_serial
  rxmode = !RXMODE_BIND;
  // set setup complete flag
  framestarted = 0;
}

void rx_init(void) {
  sbus_init();
}

void rx_check() {
  if (frameStatus < 0) {
    //can't read a packet before you set up the uart.
    sbus_init();
    // set in routine above "frameStatus = 0;"
  }

  if (frameStatus == 1) //UART says there's something to look at. Let's look at it.
  {
    frameLength = 0;
    for (uint8_t counter = 0; counter < 32; counter++) {                //First up, get the data out of the RX buffer and into somewhere safe
      data[counter] = rx_buffer[(counter + frameStart) % RX_BUFF_SIZE]; // This can probably go away, as long as the buffer is large enough
      frameLength++;
    }

    if (data[23] & (1 << 2)) //RX appears to set this bit when it knows it missed a frame. How does it know?
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
    if (data[23] & (1 << 3)) {
      failsafe_sbus_failsafe = 1; // Sbus packets have a failsafe bit. This is cool.
    } else {
      failsafe_sbus_failsafe = 0;
    }

    //Sbus channels are weird.
    channels[0] = ((data[1] | data[2] << 8) & 0x07FF);
    channels[1] = ((data[2] >> 3 | data[3] << 5) & 0x07FF);
    channels[2] = ((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF);
    channels[3] = ((data[5] >> 1 | data[6] << 7) & 0x07FF);
    channels[4] = ((data[6] >> 4 | data[7] << 4) & 0x07FF);
    channels[5] = ((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF);
    channels[6] = ((data[9] >> 2 | data[10] << 6) & 0x07FF);
    channels[7] = ((data[10] >> 5 | data[11] << 3) & 0x07FF);
    channels[8] = ((data[12] | data[13] << 8) & 0x07FF);
    channels[9] = ((data[13] >> 3 | data[14] << 5) & 0x07FF); //This is the last channel Silverware previously supported.
    channels[10] = ((data[14] >> 6 | data[15] << 2 | data[16] << 10) & 0x07FF);
    channels[11] = ((data[16] >> 1 | data[17] << 7) & 0x07FF);
    channels[12] = ((data[17] >> 4 | data[18] << 4) & 0x07FF);
    channels[13] = ((data[18] >> 7 | data[19] << 1 | data[20] << 9) & 0x07FF);
    channels[14] = ((data[20] >> 2 | data[21] << 6) & 0x07FF);
    channels[15] = ((data[21] >> 5 | data[22] << 3) & 0x07FF);

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

      aux[AUX_CHANNEL_0] = (channels[4] > 1600) ? 1 : 0;
      aux[AUX_CHANNEL_1] = (channels[5] > 1600) ? 1 : 0;
      aux[AUX_CHANNEL_2] = (channels[6] > 1600) ? 1 : 0;
      aux[AUX_CHANNEL_3] = (channels[7] > 1600) ? 1 : 0;
      aux[AUX_CHANNEL_4] = (channels[8] > 1600) ? 1 : 0;
      aux[AUX_CHANNEL_5] = (channels[9] > 1600) ? 1 : 0;
      aux[AUX_CHANNEL_6] = (channels[10] > 1600) ? 1 : 0;
      aux[AUX_CHANNEL_7] = (channels[11] > 1600) ? 1 : 0;
      aux[AUX_CHANNEL_8] = (channels[12] > 1600) ? 1 : 0;
      aux[AUX_CHANNEL_9] = (channels[13] > 1600) ? 1 : 0;
      aux[AUX_CHANNEL_10] = (channels[14] > 1600) ? 1 : 0;
      aux[AUX_CHANNEL_11] = (channels[15] > 1600) ? 1 : 0;

      rx_rssi = 0.0610128f * (channels[(profile.channel.aux[AUX_RSSI] + 4)] - 173);
      if (rx_rssi > 100.0f)
        rx_rssi = 100.0f;
      if (rx_rssi < 0.0f)
        rx_rssi = 0.0f;

      time_lastframe = gettime();
      if (sbus_stats)
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

  } // end frame received

  if (gettime() - time_lastframe > 1000000) //Failsafe, it's important!
  {
    failsafe_noframes = 1;
  } else
    failsafe_noframes = 0;

  // add the 3 failsafes together
  failsafe = failsafe_noframes || failsafe_siglost || failsafe_sbus_failsafe;
}
#endif

/*
  if ( framestarted == 0)
  {
    while (  rx_end != rx_start )
    {
      if ( rx_buffer[rx_start] == 0x0f )
      {
        // start detected
        framestart = rx_start;
        framestarted = 1;
        stat_framestartcount++;
        break;
      }
      rx_start++;
      rx_start %= (RX_BUFF_SIZE);

      stat_garbage++;
    }

  }
  else if ( framestarted == 1)
  {
    // frame already has begun
    int size = 0;
    if (rx_end > framestart ) size = rx_end - framestart;
    else size = RX_BUFF_SIZE - framestart + rx_end;
    if ( size >= 24 )
    {
      int timing_fail = 0;

      for ( int i = framestart ; i < framestart + 25; i++  )
      {
        data[ i - framestart] = rx_buffer[i % (RX_BUFF_SIZE)];
        int symboltime = rx_time[i % (RX_BUFF_SIZE)];
        //stat_timing[ i - framestart] = symboltime;
        if ( symboltime > 1024 &&  i - framestart > 0 ) timing_fail = 1;
      }

      if (!timing_fail)
      {
        frame_received = 1;

        if (data[23] & (1 << 2))
        {
          // frame lost bit
          if ( !time_siglost ) time_siglost = gettime();
          if ( gettime() - time_siglost > 1000000 )
          {
            failsafe_siglost = 1;
          }
        }
        else
        {
          time_siglost = 0;
          failsafe_siglost = 0;
        }


        if (data[23] & (1 << 3))
        {
          // failsafe bit
          failsafe_sbus_failsafe = 1;
        }
        else {
          failsafe_sbus_failsafe = 0;
        }


      } else if (sbus_stats) stat_timing_fail++;

      last_byte = data[24];


      rx_start = rx_end;
      framestarted = 0;
      bind_safety++;
    } // end frame complete

  }// end frame pending
  else if ( framestarted < 0)
  {
    // initialize sbus
    sbus_init();
    // set in routine above "framestarted = 0;"
  }

  if ( frame_received )
  {
    int channels[9];
    //decode frame
    channels[0]  = ((data[1] | data[2] << 8) & 0x07FF);
    channels[1]  = ((data[2] >> 3 | data[3] << 5) & 0x07FF);
    channels[2]  = ((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF);
    channels[3]  = ((data[5] >> 1 | data[6] << 7) & 0x07FF);
    channels[4]  = ((data[6] >> 4 | data[7] << 4) & 0x07FF);
    channels[5]  = ((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF);
    channels[6]  = ((data[9] >> 2 | data[10] << 6) & 0x07FF);
    channels[7]  = ((data[10] >> 5 | data[11] << 3) & 0x07FF);
    channels[8]  = ((data[12] | data[13] << 8) & 0x07FF);

    if ( rx_state == 0)
    {
      // wait for valid sbus signal
      static int frame_count = 0;
      failsafe = 1;
      rxmode = RXMODE_BIND;
      // if throttle < 10%
      if (  channels[2] < 336 ) frame_count++;
      if (frame_count  > 130 )
      {
        if ( stat_frames_second > 30 )
        {
          rx_state++;
          rxmode = !RXMODE_BIND;
        }
        else
        {
          frame_count = 0;
        }
      }

    }

    if ( rx_state == 1)
    {
      // normal rx mode

      // AETR channel order
      channels[0] -= 993;
      channels[1] -= 993;
      channels[3] -= 993;

      rx[0] = channels[0];
      rx[1] = channels[1];
      rx[2] = channels[3];

      for ( int i = 0 ; i < 3 ; i++)
      {
        rx[i] *= 0.00122026f;
      }

      channels[2] -= 173;
      rx[3] = 0.000610128f * channels[2];

      if ( rx[3] > 1 ) rx[3] = 1;

      if (aux[LEVELMODE]) {
        if (aux[RACEMODE] && !aux[HORIZON]) {
          if ( ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
          if ( ACRO_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
          if ( ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
        } else if (aux[HORIZON]) {
          if ( ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ACRO_EXPO_ROLL);
          if ( ACRO_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
          if ( ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
        } else {
          if ( ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
          if ( ANGLE_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ANGLE_EXPO_PITCH);
          if ( ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
        }
      } else {
        if ( ACRO_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ACRO_EXPO_ROLL);
        if ( ACRO_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
        if ( ACRO_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ACRO_EXPO_YAW);
      }

      aux[AUX_CHANNEL_0] = (channels[4] > 993) ? 1 : 0;
      aux[AUX_CHANNEL_1] = (channels[5] > 993) ? 1 : 0;
      aux[AUX_CHANNEL_2] = (channels[6] > 993) ? 1 : 0;
      aux[AUX_CHANNEL_3] = (channels[7] > 993) ? 1 : 0;
      aux[AUX_CHANNEL_4] = (channels[8] > 993) ? 1 : 0;

      time_lastframe = gettime();
      if (sbus_stats) stat_frames_accepted++;
      if (bind_safety > 9) {              //requires 10 good frames to come in before rx_ready safety can be toggled to 1
        rx_ready = 1;                     // because aux channels initialize low and clear the binding while armed flag before aux updates high
        bind_safety = 10;
      }
    }


    // stats
    static int fps = 0;
    static unsigned long secondtime = 0;

    if ( gettime() - secondtime > 1000000 )
    {
      stat_frames_second = fps;
      fps = 0;
      secondtime = gettime();
    }
    fps++;

    frame_received = 0;
  } // end frame received


  if ( gettime() - time_lastframe > 1000000 )
  {
    failsafe_noframes = 1;
  } else failsafe_noframes = 0;

  // add the 3 failsafes together
  failsafe = failsafe_noframes || failsafe_siglost || failsafe_sbus_failsafe;

}

#endif
*/
