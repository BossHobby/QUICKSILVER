#include "rx.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "control.h"
#include "debug.h"
#include "drv_fmc.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "led.h"
#include "profile.h"
#include "project.h"
#include "usb_configurator.h"
#include "util.h"

// sbus input ( pin SWCLK after calibration)
// WILL DISABLE PROGRAMMING AFTER GYRO CALIBRATION - 2 - 3 seconds after powerup)

#ifdef RX_UNIFIED_SERIAL

#define RX_BUFF_SIZE 68

//*****************************************
//*****************************************
#define RX_FRAME_INTERVAL_TRIGGER_TICKS (1500 * (TICK_CLOCK_FREQ_HZ / 1000000)) //This is the microsecond threshold for triggering a new frame to re-index to position 0 in the ISR
//*****************************************
//*****************************************
#define DSM_SCALE_PERCENT 147 //this might stay somewhere or be replaced with wizard scaling
//*****************************************
//*****************************************
//#define RX_DSM2_1024_TEMP     //for legacy override to dsm2 in place of dsmx
//*****************************************
//*****************************************
//#define DSM_RSSI_FADES		//for internal dsm link quality rssi based on satellite fades instead of packets per second
//*****************************************
//*****************************************

rx_serial_protocol_t rx_serial_protocol = RX_SERIAL_PROTOCOL_INVALID;

static uint8_t rx_buffer[RX_BUFF_SIZE];
static uint8_t rx_data[RX_BUFF_SIZE]; //A place to put the RX frame so nothing can get overwritten during processing.  //reduce size?
static uint8_t rx_frame_position = 0;

typedef enum {
  FRAME_INVALID,
  FRAME_IDLE,
  FRAME_RX,
  FRAME_RX_DONE,
  FRAME_TX,
  FRAME_DONE
} frame_status_t;

static frame_status_t frame_status = FRAME_INVALID;

static uint8_t telemetry_counter = 0;
static uint8_t expected_frame_length = 10;

int rx_bind_enable = 0;

//uint32_t rx_framerate[3] = {0, 0, 1}; //new from NFE - do we wanna keep this?
//uint32_t rx_framerate_ticks = 0;      //new from NFE - do we wanna keep this?

static uint8_t stat_frames_second;
static uint32_t time_siglost;
static uint32_t time_lastframe;

int bind_safety = 0;
int channels[16];

static uint16_t crc_byte = 0;              //Defined here to allow Debug to see it.
static uint8_t protocol_to_check = 1;      //Defined here to allow Debug to see it.
static uint16_t protocol_detect_timer = 0; //Defined here to allow Debug to see it.

extern profile_t profile;
uint16_t link_quality_raw;
#define LQ_EXPO 0.9f
float rx_rssi;

int failsafe_sbus_failsafe = 0;
int failsafe_siglost = 0;
int failsafe_noframes = 0;

//Telemetry variables
//***********************************
//Global values to send as telemetry
static uint8_t ready_for_next_telemetry = 1;
bool fport_debug_telemetry = false;

uint8_t telemetry_offset = 0;
uint8_t telemetry_packet[14];

extern int current_pid_axis;
extern int current_pid_term;

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
uint8_t telemetry_position = 0; //This iterates through the above, you can only send one sensor per frame.

#define USART usart_port_defs[serial_rx_port]

void TX_USART_ISR(void) {                       //USART_ClearITPendingBit() for TC handled in drv_serial.c
  static uint8_t increment_transmit_buffer = 1; // buffer position 0 has already been called by the telemetry process so we start at 1
  uint8_t bytes_to_send = 0;                    // reset this to 0 so that a protocol switch will not create a tx isr that does stuff without need
  if (rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT_INVERTED) {
    bytes_to_send = 10 + telemetry_offset; //upload total telemetry bytes to send so telemetry transmit triggers action appropriate to protocol
  }
  if (increment_transmit_buffer < bytes_to_send) {                      // check the index to see if we have drained the buffer yet
    while (USART_GetFlagStatus(USART.channel, USART_FLAG_TXE) == RESET) // just in case - but this should do nothing since irq was called based on TXE
      ;
    USART_SendData(USART.channel, telemetry_packet[increment_transmit_buffer]); // send a byte out of the buffer indexed by the counter
    increment_transmit_buffer++;                                                // increment the counter
  } else {                                                                      // this interrupt ran because the last byte was sent
    increment_transmit_buffer = 1;                                              // reset the counter to the right index for the next telemetry irq event
    ready_for_next_telemetry = 1;                                               // set the flag to allow the telemetry process to run again
    USART_ITConfig(USART.channel, USART_IT_TC, DISABLE);
  }
}

void RX_USART_ISR(void) {
  uint32_t rx_byte_interval = 0;
  uint32_t maxticks = SysTick->LOAD;
  uint32_t ticks = SysTick->VAL;

  static uint32_t lastticks;
  if (ticks < lastticks)
    rx_byte_interval = lastticks - ticks;
  else { // overflow ( underflow really)
    rx_byte_interval = lastticks + (maxticks - ticks);
  }
  lastticks = ticks;

  if (USART_GetFlagStatus(USART.channel, USART_FLAG_ORE)) {
    // overflow means something was lost
    USART_ClearFlag(USART.channel, USART_FLAG_ORE);
    rx_frame_position = 0;
  }

  if (rx_byte_interval > RX_FRAME_INTERVAL_TRIGGER_TICKS || frame_status == FRAME_DONE) {
    rx_frame_position = 0;
    frame_status = FRAME_IDLE;
  }

  rx_buffer[rx_frame_position++] = USART_ReceiveData(USART.channel);
  if (rx_frame_position >= expected_frame_length && frame_status == FRAME_IDLE) {
    frame_status = FRAME_RX;
  }

  rx_frame_position %= (RX_BUFF_SIZE);
}

void rx_init(void) {
  flags.rx_mode = !RXMODE_BIND; // put LEDS in normal signal status
  rx_serial_init();
}

void rx_serial_init(void) {
  //Let the uart ISR do its stuff.
  frame_status = FRAME_IDLE;

  if (rx_serial_protocol == RX_SERIAL_PROTOCOL_INVALID) { //No known protocol? Can't really set the radio up yet then can we?
    rx_serial_find_protocol();
  } else {
    serial_rx_init(rx_serial_protocol); //There's already a known protocol, we're good.
  }

  switch (rx_serial_protocol) {
  case RX_SERIAL_PROTOCOL_DSM:
    expected_frame_length = 16;
    break;
  case RX_SERIAL_PROTOCOL_SBUS:
  case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
    expected_frame_length = 24;
    break;
  case RX_SERIAL_PROTOCOL_IBUS:
    expected_frame_length = 32;
    break;
  case RX_SERIAL_PROTOCOL_FPORT:
  case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
    expected_frame_length = 28; //Minimum.
    break;
  case RX_SERIAL_PROTOCOL_CRSF:
    expected_frame_length = 64; //Maybe 65? Not sure where the Sync Byte comes in
    break;
  case RX_SERIAL_PROTOCOL_REDPINE:
  case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
    expected_frame_length = 11;
    break;
  default:
    break;
  }
}

void rx_check() {
  if (rx_serial_protocol == RX_SERIAL_PROTOCOL_INVALID) { //If there's no protocol, there's no reason to check failsafe.
    rx_serial_find_protocol();
    return;
  }

  //FAILSAFE! It gets checked every time!
  if (timer_micros() - time_lastframe > 1000000) {
    failsafe_noframes = 1;
  } else {
    failsafe_noframes = 0;
  }

  // add the 3 failsafes together
  if (flags.rx_ready)
    flags.failsafe = failsafe_noframes || failsafe_siglost || failsafe_sbus_failsafe;

  if (frame_status == FRAME_INVALID) {
    //RX/USART not set up.

    //Set it up. This includes autodetecting protocol if necesary
    rx_serial_init();
    flags.rx_mode = !RXMODE_BIND;

    //bail
    return;
  }

  if (frame_status == FRAME_RX) {
    //USART ISR says there's enough frame to look at. Look at it.
    switch (rx_serial_protocol) {
    case RX_SERIAL_PROTOCOL_DSM:
      rx_serial_process_dsmx();
      break;
    case RX_SERIAL_PROTOCOL_SBUS:
    case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
      rx_serial_process_sbus();
      break;
    case RX_SERIAL_PROTOCOL_IBUS:
      rx_serial_process_ibus();
      break;
    case RX_SERIAL_PROTOCOL_FPORT:
    case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
      rx_serial_process_fport();
      break;
    case RX_SERIAL_PROTOCOL_CRSF:
      rx_serial_process_crsf();
      break;
    case RX_SERIAL_PROTOCOL_REDPINE:
    case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
      rx_serial_process_redpine();
    default:
      break;
    }
  } else if (frame_status == FRAME_TX) {
    switch (rx_serial_protocol) {
    case RX_SERIAL_PROTOCOL_DSM:
      // Run DSM Telemetry
      frame_status = FRAME_DONE;
      break;
    case RX_SERIAL_PROTOCOL_SBUS:
    case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
      //Run smartport telemetry?
      frame_status = FRAME_DONE;
      break;
    case RX_SERIAL_PROTOCOL_IBUS:
      //IBUS Telemetry function call goes here
      frame_status = FRAME_DONE;
      break;
    case RX_SERIAL_PROTOCOL_FPORT:
    case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
      if (ready_for_next_telemetry)
        rx_serial_send_fport_telemetry();
      else
        frame_status = FRAME_DONE;
      break;
    case RX_SERIAL_PROTOCOL_CRSF:
      //CRSF telemetry function call yo
      frame_status = FRAME_DONE;
      break;

    default:
      frame_status = FRAME_DONE;
      break;
    }
  }
}

void rx_serial_process_dsmx(void) {

  for (uint8_t counter = 0; counter < 16; counter++) {    //First up, get the rx_data out of the RX buffer and into somewhere safe
    rx_data[counter] = rx_buffer[counter % RX_BUFF_SIZE]; // This can probably go away, as long as the buffer is large enough
  }

#ifdef RX_DSM2_1024_TEMP
  float dsm2_scalefactor = (0.29354210f / DSM_SCALE_PERCENT);
  // 10 bit frames
  static uint8_t spek_chan_shift = 2;
  static uint8_t spek_chan_mask = 0x03;
  static uint8_t dsm_channel_count = 7;
#else //DSMX_2048
#define RX_DSMX_2048_UNIFIED
  float dsmx_scalefactor = (0.14662756f / DSM_SCALE_PERCENT);
  // 11 bit frames
  static uint8_t spek_chan_shift = 3;
  static uint8_t spek_chan_mask = 0x07;
  static uint8_t dsm_channel_count = 12;
#endif
#define SPEKTRUM_MAX_FADE_PER_SEC 40
#define SPEKTRUM_FADE_REPORTS_PER_SEC 2

  // Fade to rssi hack
#ifdef DSM_RSSI_FADES
  uint16_t fade_count = (rx_data[0] << 8) + rx_data[1];
  uint32_t timestamp = timer_micros() / 1000 / (1000 / SPEKTRUM_FADE_REPORTS_PER_SEC);
  static uint32_t last_fade_timestamp = 0; // Stores the timestamp of the last fade read.
  static uint16_t last_fade_count = 0;     // Stores the fade count at the last fade read.
  if (last_fade_timestamp == 0) {          //first frame received
    last_fade_count = fade_count;
    last_fade_timestamp = timestamp;
  } else if ((timestamp - last_fade_timestamp) >= 1) {
#ifdef RX_DSMX_2048_UNIFIED
    link_quality_raw = 2048 - ((fade_count - last_fade_count) * 2048 / (SPEKTRUM_MAX_FADE_PER_SEC / SPEKTRUM_FADE_REPORTS_PER_SEC));
#else
    link_quality_raw = 1024 - ((fade_count - last_fade_count) * 1024 / (SPEKTRUM_MAX_FADE_PER_SEC / SPEKTRUM_FADE_REPORTS_PER_SEC));
#endif
    last_fade_count = fade_count;
    last_fade_timestamp = timestamp;
  }
#endif

  for (int b = 3; b < expected_frame_length; b += 2) { //stick data in channels buckets
    const uint8_t spekChannel = 0x0F & (rx_data[b - 1] >> spek_chan_shift);
    if (spekChannel < dsm_channel_count && spekChannel < 12) {
      channels[spekChannel] = ((uint32_t)(rx_data[b - 1] & spek_chan_mask) << 8) + rx_data[b];
      frame_status = FRAME_RX_DONE; // if we can hold 2 here for an entire frame, then we will decode it
    } else {
      //a counter here will flag on 22ms mode which could be used for auto-apply of correct filter cut on rc smoothing
    }
  }

  if (frame_status == FRAME_RX_DONE) {
    bind_safety++;
    if (bind_safety < 120)
      flags.rx_mode = RXMODE_BIND; // this is rapid flash during bind safety
                                   // TAER channel order
#ifdef RX_DSMX_2048_UNIFIED
    state.rx.axis[0] = (channels[1] - 1024.0f) * dsmx_scalefactor;
    state.rx.axis[1] = (channels[2] - 1024.0f) * dsmx_scalefactor;
    state.rx.axis[2] = (channels[3] - 1024.0f) * dsmx_scalefactor;
    state.rx.axis[3] = ((channels[0] - 1024.0f) * dsmx_scalefactor * 0.5f) + 0.5f;

    if (state.rx.axis[3] > 1)
      state.rx.axis[3] = 1;
    if (state.rx.axis[3] < 0)
      state.rx.axis[3] = 0;
#endif

#ifdef RX_DSM2_1024_TEMP
    state.rx.axis[0] = (channels[1] - 512.0f) * dsm2_scalefactor;
    state.rx.axis[1] = (channels[2] - 512.0f) * dsm2_scalefactor;
    state.rx.axis[2] = (channels[3] - 512.0f) * dsm2_scalefactor;
    state.rx.axis[3] = ((channels[0] - 512.0f) * dsm2_scalefactor * 0.5f) + 0.5f;

    if (state.rx.axis[3] > 1)
      state.rx.axis[3] = 1;
    if (state.rx.axis[3] < 0)
      state.rx.axis[3] = 0;
#endif

    rx_apply_expo();

#ifdef RX_DSMX_2048_UNIFIED
    state.aux[AUX_CHANNEL_0] = (channels[4] > 1100) ? 1 : 0; //1100 cutoff intentionally selected to force aux channels low if
    state.aux[AUX_CHANNEL_1] = (channels[5] > 1100) ? 1 : 0; //being controlled by a transmitter using a 3 pos switch in center state
    state.aux[AUX_CHANNEL_2] = (channels[6] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_3] = (channels[7] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_4] = (channels[8] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_5] = (channels[9] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_6] = (channels[10] > 1100) ? 1 : 0;
    state.aux[AUX_CHANNEL_7] = (channels[11] > 1100) ? 1 : 0;
#endif

#ifdef RX_DSM2_1024_TEMP
    state.aux[AUX_CHANNEL_0] = (channels[4] > 550) ? 1 : 0; //550 cutoff intentionally selected to force aux channels low if
    state.aux[AUX_CHANNEL_1] = (channels[5] > 550) ? 1 : 0; //being controlled by a transmitter using a 3 pos switch in center state
    state.aux[AUX_CHANNEL_2] = (channels[6] > 550) ? 1 : 0;
#endif

    //for failsafe_noframes
    time_lastframe = timer_micros();

    /*    //for framerate calculation - totally unnecessary but cool to see
    rx_framerate[2] = !(rx_framerate[2]);
    rx_framerate[rx_framerate[2]] = time_lastframe;
    rx_framerate_ticks = abs(rx_framerate[0] - rx_framerate[1]);*/

    // link quality & rssi
    static int fps_counter = 0;
    static unsigned long secondtime = 0;
    if (time_lastframe - secondtime > 1000000) {
      stat_frames_second = fps_counter;
      fps_counter = 0;
      secondtime = time_lastframe;
    }
    fps_counter++;

    if (profile.channel.aux[AUX_RSSI] > AUX_CHANNEL_11) { //rssi set to internal link quality
#ifdef DSM_RSSI_FADES
#ifdef RX_DSMX_2048_UNIFIED
      rx_rssi = 0.000488281 * link_quality_raw;
#else
      rx_rssi = 0.000976563 * link_quality_raw;
#endif
#else
      rx_rssi = stat_frames_second / 91.0f;
#endif
      rx_rssi = rx_rssi * rx_rssi * rx_rssi * LQ_EXPO + rx_rssi * (1 - LQ_EXPO);
      rx_rssi *= 100.0f;
    } else { //rssi set to value decoded from aux channel input from receiver
#ifdef RX_DSMX_2048_UNIFIED
      rx_rssi = ((channels[(profile.channel.aux[AUX_RSSI] + 4)] - 1024.0f) * dsmx_scalefactor * 0.5f) + 0.5f;
#else
      rx_rssi = ((channels[(profile.channel.aux[AUX_RSSI] + 4)] - 512.0f) * dsm2_scalefactor * 0.5f) + 0.5f;
#endif
    }
    if (rx_rssi > 100.0f)
      rx_rssi = 100.0f;
    if (rx_rssi < 0.0f)
      rx_rssi = 0.0f;

    frame_status = FRAME_TX; //We're done with this frame now.

    if (bind_safety > 120) {        //requires 120 good frames to come in before rx_ready safety can be toggled to 1.  About a second of good data
      flags.rx_ready = 1;           // because aux channels initialize low and clear the binding while armed flag before aux updates high
      flags.rx_mode = !RXMODE_BIND; // restores normal led operation
      bind_safety = 121;            // reset counter so it doesnt wrap
    }
  }
}

void rx_serial_process_sbus(void) {
  for (uint8_t counter = 0; counter < 25; counter++) {    //First up, get therx_data out of the RX buffer and into somewhere safe
    rx_data[counter] = rx_buffer[counter % RX_BUFF_SIZE]; // This can probably go away, as long as the buffer is large enough
  }

  if (rx_data[23] & (1 << 2)) //RX sets this bit when it knows it missed a frame. Presumably this is a timer in the RX.
  {
    link_quality_raw++;
    if (!time_siglost)
      time_siglost = timer_micros();
    if (timer_micros() - time_siglost > TICK_CLOCK_FREQ_HZ) //8,000,000 ticks on F0, 21M on F4. One second.
    {
      failsafe_siglost = 1;
    }
  } else {
    time_siglost = 0;
    failsafe_siglost = 0;
  }
  if (rx_data[23] & (1 << 3)) {
    failsafe_sbus_failsafe = 1;              // Sbus packets have a failsafe bit. This is cool. If you forget to trust it you get programs though.
    flags.failsafe = failsafe_sbus_failsafe; //set failsafe rtf-now
  } else {
    failsafe_sbus_failsafe = 0;
  }

  //Sbus channels, this could be a struct and a memcpy but eh.
  //All 16 are decoded, even if Quicksilver doesn't want to use them.
  channels[0] = ((rx_data[1] | rx_data[2] << 8) & 0x07FF);
  channels[1] = ((rx_data[2] >> 3 | rx_data[3] << 5) & 0x07FF);
  channels[2] = ((rx_data[3] >> 6 | rx_data[4] << 2 | rx_data[5] << 10) & 0x07FF);
  channels[3] = ((rx_data[5] >> 1 | rx_data[6] << 7) & 0x07FF);
  channels[4] = ((rx_data[6] >> 4 | rx_data[7] << 4) & 0x07FF);
  channels[5] = ((rx_data[7] >> 7 | rx_data[8] << 1 | rx_data[9] << 9) & 0x07FF);
  channels[6] = ((rx_data[9] >> 2 | rx_data[10] << 6) & 0x07FF);
  channels[7] = ((rx_data[10] >> 5 | rx_data[11] << 3) & 0x07FF);
  channels[8] = ((rx_data[12] | rx_data[13] << 8) & 0x07FF);
  channels[9] = ((rx_data[13] >> 3 | rx_data[14] << 5) & 0x07FF); //This is the last channel Silverware previously supported.
  channels[10] = ((rx_data[14] >> 6 | rx_data[15] << 2 | rx_data[16] << 10) & 0x07FF);
  channels[11] = ((rx_data[16] >> 1 | rx_data[17] << 7) & 0x07FF);
  channels[12] = ((rx_data[17] >> 4 | rx_data[18] << 4) & 0x07FF);
  channels[13] = ((rx_data[18] >> 7 | rx_data[19] << 1 | rx_data[20] << 9) & 0x07FF);
  channels[14] = ((rx_data[20] >> 2 | rx_data[21] << 6) & 0x07FF);
  channels[15] = ((rx_data[21] >> 5 | rx_data[22] << 3) & 0x07FF);

  // normal rx mode
  bind_safety++;
  if (bind_safety < 130)
    flags.rx_mode = RXMODE_BIND; // this is rapid flash during bind safety

  // AETR channel order
  channels[0] -= 993;
  channels[1] -= 993;
  channels[3] -= 993;

  state.rx.axis[0] = channels[0];
  state.rx.axis[1] = channels[1];
  state.rx.axis[2] = channels[3];

  for (int i = 0; i < 3; i++) {
    state.rx.axis[i] *= 0.00122026f;
  }

  channels[2] -= 173;
  state.rx.axis[3] = 0.000610128f * channels[2];

  if (state.rx.axis[3] > 1)
    state.rx.axis[3] = 1;
  if (state.rx.axis[3] < 0)
    state.rx.axis[3] = 0;

  rx_apply_expo();

  //Here we have the AUX channels Silverware supports
  state.aux[AUX_CHANNEL_0] = (channels[4] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_1] = (channels[5] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_2] = (channels[6] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_3] = (channels[7] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_4] = (channels[8] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_5] = (channels[9] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_6] = (channels[10] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_7] = (channels[11] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_8] = (channels[12] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_9] = (channels[13] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_10] = (channels[14] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_11] = (channels[15] > 1600) ? 1 : 0;

  time_lastframe = timer_micros();

  // link quality & rssi
  static unsigned long secondtime = 0;
  if (time_lastframe - secondtime > 1000000) {
    stat_frames_second = 112 - link_quality_raw;
    link_quality_raw = 0;
    secondtime = time_lastframe;
  }

  if (profile.channel.aux[AUX_RSSI] > AUX_CHANNEL_11) { //rssi set to internal link quality
    rx_rssi = stat_frames_second / 112.0f;
    rx_rssi = rx_rssi * rx_rssi * rx_rssi * LQ_EXPO + rx_rssi * (1 - LQ_EXPO);
    rx_rssi *= 100.0f;
  } else { //rssi set to value decoded from aux channel input from receiver
    rx_rssi = 0.0610128f * (channels[(profile.channel.aux[AUX_RSSI] + 4)] - 173);
  }
  if (rx_rssi > 100.0f)
    rx_rssi = 100.0f;
  if (rx_rssi < 0.0f)
    rx_rssi = 0.0f;

  frame_status = FRAME_TX; //We're done with this frame now.

  if (bind_safety > 131) {        //requires 130 good frames to come in before rx_ready safety can be toggled to 1.  About a second of good data
    flags.rx_ready = 1;           // because aux channels initialize low and clear the binding while armed flag before aux updates high
    flags.rx_mode = !RXMODE_BIND; // restores normal led operation
    bind_safety = 131;            // reset counter so it doesnt wrap
  }
}

void rx_serial_process_ibus(void) {
  uint8_t frameLength = 0;
  for (uint8_t counter = 0; counter < 32; counter++) {    //First up, get the data out of the RX buffer and into somewhere safe
    rx_data[counter] = rx_buffer[counter % RX_BUFF_SIZE]; // This can probably go away, as long as the buffer is large enough
    frameLength++;                                        // to accept telemetry requests without overwriting control data
  }

  crc_byte = 0xFFFF;
  for (int x = 0; x < 30; x++) {
    crc_byte = crc_byte - rx_data[x];
  }

  if (crc_byte == rx_data[30] + (rx_data[31] << 8)) { //If the CRC is good, shove it into controls

    //Flysky channels are delightfully straightforward
    channels[0] = rx_data[2] + (rx_data[3] << 8);
    channels[1] = rx_data[4] + (rx_data[5] << 8);
    channels[2] = rx_data[6] + (rx_data[7] << 8);
    channels[3] = rx_data[8] + (rx_data[9] << 8);
    channels[4] = rx_data[10] + (rx_data[11] << 8);
    channels[5] = rx_data[12] + (rx_data[13] << 8);
    channels[6] = rx_data[14] + (rx_data[15] << 8);
    channels[7] = rx_data[16] + (rx_data[17] << 8);
    channels[8] = rx_data[18] + (rx_data[19] << 8);
    channels[9] = rx_data[20] + (rx_data[21] << 8);
    channels[10] = rx_data[22] + (rx_data[23] << 8);
    channels[11] = rx_data[24] + (rx_data[25] << 8);
    channels[12] = rx_data[26] + (rx_data[27] << 8);
    channels[13] = rx_data[28] + (rx_data[29] << 8);

    frame_status = FRAME_RX;

  } else {
    // if CRC fails, do this:
    //while(1){} Enable for debugging to lock the FC if CRC fails. In the air we just drop CRC-failed packets
    //Most likely reason for failed CRC is a frame that isn't fully here yet. No need to check again until a new byte comes in.

    frame_status = FRAME_IDLE;
  }

  if (frame_status == FRAME_RX) {
    // normal rx mode
    bind_safety++;
    if (bind_safety < 130)
      flags.rx_mode = RXMODE_BIND; // this is rapid flash during bind safety

    // AETR channel order
    channels[0] -= 1500;
    channels[1] -= 1500;
    channels[2] -= 1000;
    channels[3] -= 1500;

    state.rx.axis[0] = channels[0];
    state.rx.axis[1] = channels[1];
    state.rx.axis[2] = channels[3];
    state.rx.axis[3] = channels[2];

    for (int i = 0; i < 3; i++) {
      state.rx.axis[i] *= 0.002f;
    }
    state.rx.axis[3] *= 0.001f;

    if (state.rx.axis[3] > 1)
      state.rx.axis[3] = 1;
    if (state.rx.axis[3] < 0)
      state.rx.axis[3] = 0;

    rx_apply_expo();

    //Here we have the AUX channels Silverware supports
    state.aux[AUX_CHANNEL_0] = (channels[4] > 1600) ? 1 : 0;
    state.aux[AUX_CHANNEL_1] = (channels[5] > 1600) ? 1 : 0;
    state.aux[AUX_CHANNEL_2] = (channels[6] > 1600) ? 1 : 0;
    state.aux[AUX_CHANNEL_3] = (channels[7] > 1600) ? 1 : 0;
    state.aux[AUX_CHANNEL_4] = (channels[8] > 1600) ? 1 : 0;
    state.aux[AUX_CHANNEL_5] = (channels[9] > 1600) ? 1 : 0;
    state.aux[AUX_CHANNEL_6] = (channels[10] > 1600) ? 1 : 0;
    state.aux[AUX_CHANNEL_7] = (channels[11] > 1600) ? 1 : 0;
    state.aux[AUX_CHANNEL_8] = (channels[12] > 1600) ? 1 : 0;
    state.aux[AUX_CHANNEL_9] = (channels[13] > 1600) ? 1 : 0;
    state.aux[AUX_CHANNEL_10] = (channels[14] > 1600) ? 1 : 0;
    state.aux[AUX_CHANNEL_11] = (channels[15] > 1600) ? 1 : 0;

    time_lastframe = timer_micros();

    // stats & rssi
    static int fps_counter = 0;
    static unsigned long secondtime = 0;
    if (time_lastframe - secondtime > 1000000) {
      stat_frames_second = fps_counter;
      fps_counter = 0;
      secondtime = time_lastframe;
    }
    fps_counter++;

    if (profile.channel.aux[AUX_RSSI] > AUX_CHANNEL_11) { //rssi set to internal link quality
      rx_rssi = stat_frames_second / 111.0f;              //**this needs adjusting to actual ibus expected packets per second
      rx_rssi = rx_rssi * rx_rssi * rx_rssi * LQ_EXPO + rx_rssi * (1 - LQ_EXPO);
      rx_rssi *= 100.0f;
    } else { //rssi set to value decoded from aux channel input from receiver
      rx_rssi = 0.1f * (channels[(profile.channel.aux[AUX_RSSI] + 4)] - 1000);
    }
    if (rx_rssi > 100.0f)
      rx_rssi = 100.0f;
    if (rx_rssi < 0.0f)
      rx_rssi = 0.0f;

    frame_status = FRAME_TX; //We're done with this frame now.

    if (bind_safety > 131) {        //requires 130 good frames to come in before rx_ready safety can be toggled to 1.  About a second of good data
      flags.rx_ready = 1;           // because aux channels initialize low and clear the binding while armed flag before aux updates high
      flags.rx_mode = !RXMODE_BIND; // restores normal led operation
      bind_safety = 131;            // reset counter so it doesnt wrap
    }
  }
}

void rx_serial_process_fport(void) {
  uint8_t frameLength = 0;
  static uint8_t escapedChars = 0;
  uint8_t tempEscapedChars = 0;
  for (uint8_t counter = 0; counter <= rx_frame_position; counter++) {         //First up, get the data out of the RX buffer and into somewhere safe
    rx_data[counter] = rx_buffer[(counter + tempEscapedChars) % RX_BUFF_SIZE]; //We're going to be messing with the data and it wouldn't do to destroy a packet
    frameLength++;

    if (rx_data[counter - 1] == 0x7D) { //0x7D and 0x7E are reserved, 0x7D is the marker for a reserved / escaped character
      if (rx_data[counter] == 0x5E) {   //0x5E in the byte following 0x7D means it was a 0x7E data byte.
        rx_data[counter - 1] = 0x7E;    //So, make it 0x7E.
        escapedChars++;                 //Now we have to get rid of the "current" byte, and adjust the incoming frame length.
        tempEscapedChars++;
        counter--;
        frameLength--;
      } else if (rx_data[counter] == 0x5D) {
        escapedChars++;
        tempEscapedChars++;
        counter--;
        frameLength--;
      }
    }

    if (rx_data[counter] == 0x7e && rx_data[counter - 1] == 0x7e && frameLength > 29) { //Looks like a complete frame, check CRC and process controls if it's good.
      frame_status = FRAME_RX_DONE;
      //counter = 200; //Breaks out of the for loop processing the data array. - NFE-IS THIS STILL NEEDED?
      //The telemetry request packet is not read, as it never seems to change. Less control lag if we ignore it.
      crc_byte = 0;
      for (int x = 1; x < frameLength - 2; x++) {
        crc_byte = crc_byte + rx_data[x];
      }
      crc_byte = crc_byte + (crc_byte >> 8);
      crc_byte = crc_byte << 8;
      crc_byte = crc_byte >> 8;
      if (crc_byte == 0x00FF) {     //CRC is good, check Failsafe bit(s) and shove it into controls
                                    //FPORT uses SBUS style data, but starts further in the packet
        if (rx_data[25] & (1 << 2)) //RX appears to set this bit when it knows it missed a frame.
        {
          link_quality_raw++;
          if (!time_siglost)
            time_siglost = timer_micros();
          if (timer_micros() - time_siglost > TICK_CLOCK_FREQ_HZ) //8,000,000 ticks on F0, 21M on F4. One second.
          {
            failsafe_siglost = 1;
          }
        } else {
          time_siglost = 0;
          failsafe_siglost = 0;
        }
        if (rx_data[25] & (1 << 3)) {
          failsafe_sbus_failsafe = 1;              // Sbus packets have a failsafe bit. This is cool.
          flags.failsafe = failsafe_sbus_failsafe; //set failsafe rtf-now
        } else {
          failsafe_sbus_failsafe = 0;
        }

        channels[0] = ((rx_data[3] | rx_data[4] << 8) & 0x07FF);
        channels[1] = ((rx_data[4] >> 3 | rx_data[5] << 5) & 0x07FF);
        channels[2] = ((rx_data[5] >> 6 | rx_data[6] << 2 | rx_data[7] << 10) & 0x07FF);
        channels[3] = ((rx_data[7] >> 1 | rx_data[8] << 7) & 0x07FF);
        channels[4] = ((rx_data[8] >> 4 | rx_data[9] << 4) & 0x07FF);
        channels[5] = ((rx_data[9] >> 7 | rx_data[10] << 1 | rx_data[11] << 9) & 0x07FF);
        channels[6] = ((rx_data[11] >> 2 | rx_data[12] << 6) & 0x07FF);
        channels[7] = ((rx_data[12] >> 5 | rx_data[13] << 3) & 0x07FF);
        channels[8] = ((rx_data[14] | rx_data[15] << 8) & 0x07FF);
        channels[9] = ((rx_data[15] >> 3 | rx_data[16] << 5) & 0x07FF); //This is the last channel Silverware previously supported.
        channels[10] = ((rx_data[16] >> 6 | rx_data[17] << 2 | rx_data[18] << 10) & 0x07FF);
        channels[11] = ((rx_data[18] >> 1 | rx_data[19] << 7) & 0x07FF);
        channels[12] = ((rx_data[19] >> 4 | rx_data[20] << 4) & 0x07FF);
        channels[13] = ((rx_data[20] >> 7 | rx_data[21] << 1 | rx_data[22] << 9) & 0x07FF);
        channels[14] = ((rx_data[22] >> 2 | rx_data[23] << 6) & 0x07FF);
        channels[15] = ((rx_data[23] >> 5 | rx_data[24] << 3) & 0x07FF);

        //frame_status = 2;

      } else { // if CRC fails, do this:
        //while(1){} Enable for debugging to lock the FC if CRC fails.
        frame_status = FRAME_IDLE; //Most likely reason for failed CRC is a frame that isn't fully here yet. No need to check again until a new byte comes in.
      }
    }

    if (frame_status == FRAME_RX_DONE) {
      // normal rx mode
      bind_safety++;
      if (bind_safety < 130)
        flags.rx_mode = RXMODE_BIND; // this is rapid flash during bind safety

      // AETR channel order
      channels[0] -= 993;
      channels[1] -= 993;
      channels[3] -= 993;

      state.rx.axis[0] = channels[0];
      state.rx.axis[1] = channels[1];
      state.rx.axis[2] = channels[3];
      for (int i = 0; i < 3; i++) {
        state.rx.axis[i] *= 0.00122026f;
      }

      channels[2] -= 173;
      state.rx.axis[3] = 0.000610128f * channels[2];

      if (state.rx.axis[3] > 1)
        state.rx.axis[3] = 1;
      if (state.rx.axis[3] < 0)
        state.rx.axis[3] = 0;

      rx_apply_expo();

      //Here we have the AUX channels Silverware supports
      state.aux[AUX_CHANNEL_0] = (channels[4] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_1] = (channels[5] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_2] = (channels[6] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_3] = (channels[7] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_4] = (channels[8] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_5] = (channels[9] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_6] = (channels[10] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_7] = (channels[11] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_8] = (channels[12] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_9] = (channels[13] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_10] = (channels[14] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_11] = (channels[15] > 993) ? 1 : 0;

      if (channels[12] > 993) { // Channel 13 is now FPORT Debug Telemetry switch. Integrate this better sometime
        fport_debug_telemetry = true;
      } else {
        fport_debug_telemetry = false;
      }

      time_lastframe = timer_micros();

      // link quality & rssi
      static unsigned long secondtime = 0;
      if (time_lastframe - secondtime > 1000000) {
        stat_frames_second = 112 - link_quality_raw;
        link_quality_raw = 0;
        secondtime = time_lastframe;
      }

      if (profile.channel.aux[AUX_RSSI] > AUX_CHANNEL_11) { //rssi set to internal link quality
        rx_rssi = stat_frames_second / 112.0f;
        rx_rssi = rx_rssi * rx_rssi * rx_rssi * LQ_EXPO + rx_rssi * (1 - LQ_EXPO);
        rx_rssi *= 100.0f;
      } else { //rssi set to value decoded from aux channel input from receiver
        rx_rssi = 0.0610128f * (channels[(profile.channel.aux[AUX_RSSI] + 4)] - 173);
      }
      if (rx_rssi > 100.0f)
        rx_rssi = 100.0f;
      if (rx_rssi < 0.0f)
        rx_rssi = 0.0f;

      frame_status = FRAME_TX; //We're done with this frame now.
      telemetry_counter++;     // Let the telemetry section know it's time to send.

      if (bind_safety > 131) {        //requires 130 good frames to come in before rx_ready safety can be toggled to 1.  About a second of good data
        flags.rx_ready = 1;           // because aux channels initialize low and clear the binding while armed flag before aux updates high
        flags.rx_mode = !RXMODE_BIND; // restores normal led operation
        bind_safety = 131;            // reset counter so it doesnt wrap
      }
    }
  } // end frame received
}

vec3_t *get_pid_value(uint8_t term) {
  switch (term) {
  case 0:
    return &profile.pid.pid_rates[profile.pid.pid_profile].kp;
  case 1:
    return &profile.pid.pid_rates[profile.pid.pid_profile].ki;
  case 2:
    return &profile.pid.pid_rates[profile.pid.pid_profile].kd;
  }
  return NULL;
}

void rx_serial_send_fport_telemetry() {
  if (telemetry_counter > 1 && rx_frame_position >= 41 && frame_status == FRAME_TX) { // Send telemetry back every other packet. This gives the RX time to send ITS telemetry back
    static uint8_t skip_a_loop;
    skip_a_loop++;
    if (skip_a_loop < 3) {
      return;
    }
    skip_a_loop = 0;
    telemetry_counter = 0;
    frame_status = FRAME_DONE;

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

    //Telemetry time! Let's have some variables
    telemetry_packet[0] = 0x08; //Bytes 0 through 2 are static in this implementation
    telemetry_packet[1] = 0x81;
    telemetry_packet[2] = 0x10;
    if (telemetry_position == 0) {                                 //vbat_comp
      telemetry_packet[3] = telemetryIDs[telemetry_position];      //0x10;
      telemetry_packet[4] = telemetryIDs[telemetry_position] >> 8; //0x02;
      telemetry_packet[5] = (int)(state.vbatt_comp * 100);
      telemetry_packet[6] = (int)(state.vbatt_comp * 100) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    } else if (telemetry_position == 1) {                          //vbattfilt
      telemetry_packet[3] = telemetryIDs[telemetry_position];      //x11;
      telemetry_packet[4] = telemetryIDs[telemetry_position] >> 8; //0x02;
      telemetry_packet[5] = (int)(state.vbattfilt * 100);
      telemetry_packet[6] = (int)(state.vbattfilt * 100) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    } else if (telemetry_position == 2) { //Cell count
      telemetry_packet[3] = telemetryIDs[telemetry_position];
      telemetry_packet[4] = telemetryIDs[telemetry_position] >> 8;
      telemetry_packet[5] = (int)(state.lipo_cell_count * 100);
      telemetry_packet[6] = (int)(state.lipo_cell_count * 100) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    } else if (telemetry_position == 3) {                          //PID axis(hundreds column) and P/I/D (ones column) being adjusted currently
      uint16_t axisAndPidID = (current_pid_axis + 1) * 100;        //Adding one so there's always a value. 1 for Pitch (or Pitch/roll), 2 for Roll, 3 for Yaw
      axisAndPidID += current_pid_term + 1;                        //Adding one here too, humans don't deal well with counting starting at zero for this sort of thing
      telemetry_packet[4] = telemetryIDs[telemetry_position] >> 8; // Adding one to the above makes it match the LED flash codes too
      telemetry_packet[5] = (int)(axisAndPidID);
      telemetry_packet[6] = (int)(axisAndPidID) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    } else if (telemetry_position == 4) { //PID-P
      telemetry_packet[3] = telemetryIDs[telemetry_position];
      telemetry_packet[4] = telemetryIDs[telemetry_position] >> 8;
      telemetry_packet[5] = (int)(get_pid_value(0)->axis[current_pid_axis] * 100);
      telemetry_packet[6] = (int)(get_pid_value(0)->axis[current_pid_axis] * 100) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    } else if (telemetry_position == 5) { //PID-I
      telemetry_packet[3] = telemetryIDs[telemetry_position];
      telemetry_packet[4] = telemetryIDs[telemetry_position] >> 8;
      telemetry_packet[5] = (int)(get_pid_value(1)->axis[current_pid_axis] * 100);
      telemetry_packet[6] = (int)(get_pid_value(1)->axis[current_pid_axis] * 100) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    } else if (telemetry_position == 6) { //PID-D
      telemetry_packet[3] = telemetryIDs[telemetry_position];
      telemetry_packet[4] = telemetryIDs[telemetry_position] >> 8;
      telemetry_packet[5] = (int)(get_pid_value(2)->axis[current_pid_axis] * 100);
      telemetry_packet[6] = (int)(get_pid_value(2)->axis[current_pid_axis] * 100) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    }

    // This *should* properly escape 0x7D and 0x7E characters. It doesn't.
    telemetry_offset = 0;
    for (uint8_t i = 8; i > 4; i--) {
      if (telemetry_packet[i] == 0x7D || telemetry_packet[i] == 0x7E) {
        for (uint8_t x = 8; x >= i; x--) {
          telemetry_packet[x + 1] = telemetry_packet[x];
        }
        telemetry_packet[i] = 0x7D;
        if (telemetry_packet[i + 1] == 0x7D) {
          telemetry_packet[i + 1] = 0x5D;
        } else {
          telemetry_packet[i + 1] = 0x5E;
        }
        telemetry_offset++;
      }
    }
    telemetry_packet[0] += telemetry_offset;

    uint16_t teleCRC = 0;
    //Calculate CRC for packet. This function does not support escaped characters.
    for (int x = 0; x < 9 + telemetry_offset; x++) {
      teleCRC = teleCRC + telemetry_packet[x];
    }
    teleCRC = teleCRC + (teleCRC >> 8);
    teleCRC = 0xff - teleCRC;
    teleCRC = teleCRC << 8;
    teleCRC = teleCRC >> 8;
    telemetry_packet[9 + telemetry_offset] = teleCRC; //0x34;
    //Shove the packet out the UART. This *should* support escaped characters, but it doesn't work.
    while (USART_GetFlagStatus(USART.channel, USART_FLAG_TXE) == RESET) //just in case - but this should do nothing if ready_for_next_telemetry flag is properly cleared by irq
      ;
    USART_SendData(USART.channel, telemetry_packet[0]);
    ready_for_next_telemetry = 0;
    USART_ITConfig(USART.channel, USART_IT_TC, ENABLE); //turn on the transmit transfer complete interrupt so that the rest of the telemetry packet gets sent
    //That's it, telemetry has sent the first byte - the rest will be sent by the telemetry tx irq
    telemetry_position++;
    if (fport_debug_telemetry) {
      if (telemetry_position >= sizeof(telemetryIDs) / 2) // 2 byte ints, so this should give the number of entries. It just incremented, which takes care of the count with 0 or 1
      {
        telemetry_position = 0;
      }
    } else {
      if (telemetry_position == 2) {
        telemetry_position = 0;
      }
    }
  }
}

void rx_serial_process_crsf(void) {
  //We should probably put something here.
}

void rx_serial_process_redpine(void) {
#define REDPINE_CHANNEL_START 3
  for (uint8_t i = 0; i < 11; i++) {
    rx_data[i] = rx_buffer[i % RX_BUFF_SIZE];
  }

  const uint16_t channels[4] = {
      (uint16_t)((rx_data[REDPINE_CHANNEL_START + 1] << 8) & 0x700) | rx_data[REDPINE_CHANNEL_START],
      (uint16_t)((rx_data[REDPINE_CHANNEL_START + 2] << 4) & 0x7F0) | ((rx_data[REDPINE_CHANNEL_START + 1] >> 4) & 0xF),
      (uint16_t)((rx_data[REDPINE_CHANNEL_START + 4] << 8) & 0x700) | rx_data[REDPINE_CHANNEL_START + 3],
      (uint16_t)((rx_data[REDPINE_CHANNEL_START + 5] << 4) & 0x7F0) | ((rx_data[REDPINE_CHANNEL_START + 4] >> 4) & 0xF),
  };

  // normal rx mode
  bind_safety++;
  if (bind_safety < 130)
    flags.rx_mode = RXMODE_BIND; // this is rapid flash during bind safety

  // AETR channel order
  state.rx.axis[0] = channels[0] - 1020;
  state.rx.axis[1] = channels[1] - 1020;
  state.rx.axis[2] = channels[3] - 1020;
  state.rx.axis[3] = channels[2] - 210;

  for (int i = 0; i < 3; i++) {
    state.rx.axis[i] *= 1.f / 820.f;
  }
  state.rx.axis[3] *= 1.f / 1640.f;
  state.rx.axis[3] = constrainf(state.rx.axis[3], 0, 1);

  rx_apply_expo();

  //Here we have the AUX channels Silverware supports
  state.aux[AUX_CHANNEL_0] = (rx_data[REDPINE_CHANNEL_START + 1] & 0x08) ? 1 : 0;
  state.aux[AUX_CHANNEL_1] = (rx_data[REDPINE_CHANNEL_START + 2] & 0x80) ? 1 : 0;
  state.aux[AUX_CHANNEL_2] = (rx_data[REDPINE_CHANNEL_START + 4] & 0x08) ? 1 : 0;
  state.aux[AUX_CHANNEL_3] = (rx_data[REDPINE_CHANNEL_START + 5] & 0x80) ? 1 : 0;
  state.aux[AUX_CHANNEL_4] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x01) ? 1 : 0;
  state.aux[AUX_CHANNEL_5] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x02) ? 1 : 0;
  state.aux[AUX_CHANNEL_6] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x04) ? 1 : 0;
  state.aux[AUX_CHANNEL_7] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x08) ? 1 : 0;
  state.aux[AUX_CHANNEL_8] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x10) ? 1 : 0;
  state.aux[AUX_CHANNEL_9] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x20) ? 1 : 0;
  state.aux[AUX_CHANNEL_10] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x40) ? 1 : 0;
  state.aux[AUX_CHANNEL_11] = (rx_data[REDPINE_CHANNEL_START + 6] & 0x80) ? 1 : 0;

  time_lastframe = timer_micros();

  // link quality & rssi
  static unsigned long secondtime = 0;
  if (time_lastframe - secondtime > 1000000) {
    stat_frames_second = 112 - link_quality_raw;
    link_quality_raw = 0;
    secondtime = time_lastframe;
  }

  int16_t rssi = rx_data[REDPINE_CHANNEL_START + 7];
  if (rssi >= 128) {
    rssi = ((rssi - 256) / 2) - 71;
  } else {
    rssi = (rssi / 2) - 71;
  }

  rx_rssi = rssi;

  frame_status = FRAME_TX; //We're done with this frame now.

  if (bind_safety > 131) {        //requires 130 good frames to come in before rx_ready safety can be toggled to 1.  About a second of good data
    flags.rx_ready = 1;           // because aux channels initialize low and clear the binding while armed flag before aux updates high
    flags.rx_mode = !RXMODE_BIND; // restores normal led operation
    bind_safety = 131;            // reset counter so it doesnt wrap
  }
}

//NOTE TO SELF: Put in some double-check code on the detections somehow.
//NFE note:  how about we force hold failsafe until protocol is saved.  This acts like kind of a check on proper mapping/decoding as stick gesture must be used as a test
// also, we ought to be able to clear noframes_failsafe in addition to satisfying the start byte check in order to hard select a radio protocol
void rx_serial_find_protocol(void) {
  if (protocol_detect_timer == 0) {
    protocol_to_check++; //Check the next protocol down the list.
    if (protocol_to_check > RX_SERIAL_PROTOCOL_MAX) {
      protocol_to_check = RX_SERIAL_PROTOCOL_DSM;
    }
    serial_rx_init(protocol_to_check); //Configure a protocol!
    quic_debugf("UNIFIED: trying protocol %d", protocol_to_check);
  }

  protocol_detect_timer++; //Should increment once per main loop

  if (frame_status == FRAME_RX) { //We got something! What is it?
    switch (protocol_to_check) {
    case RX_SERIAL_PROTOCOL_DSM:
      if (rx_buffer[0] == 0x00 && rx_buffer[1] <= 0x04 && rx_buffer[2] != 0x00) {
        // allow up to 4 fades or detection will fail.  Some dsm rx will log a fade or two during binding
        rx_serial_process_dsmx();
        if (bind_safety > 0)
          rx_serial_protocol = protocol_to_check;
      }
    case RX_SERIAL_PROTOCOL_SBUS:
    case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
      if (rx_buffer[0] == 0x0F) {
        rx_serial_protocol = protocol_to_check;
      }
      break;
    case RX_SERIAL_PROTOCOL_IBUS:
      if (rx_buffer[0] == 0x20) {
        rx_serial_protocol = protocol_to_check;
      }
      break;
    case RX_SERIAL_PROTOCOL_FPORT:
    case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
      if (rx_buffer[0] == 0x7E) {
        rx_serial_process_fport();
        if (bind_safety > 5) //FPORT INVERTED will trigger a frame on FPORT HALF DUPLEX - require >5 frames for good measure
          rx_serial_protocol = protocol_to_check;
      }
      break;
    case RX_SERIAL_PROTOCOL_CRSF:
      if (rx_buffer[0] != 0xFF && 1 == 2) { //Need to look up the expected start value.
        rx_serial_protocol = protocol_to_check;
      }
      break;
    case RX_SERIAL_PROTOCOL_REDPINE:
    case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
      if (rx_buffer[0] == 11) {
        rx_serial_protocol = protocol_to_check;
      }
      break;
    default:
      frame_status = FRAME_TX; //Whatever we got, it didn't make sense. Mark the frame as Checked and start over.
      break;
    }
  }

  if (protocol_detect_timer > 4000) { //4000 loops, half a second
    protocol_detect_timer = 0;        // Reset timer, triggering a shift to detecting the next protocol
  }
}

// Send Spektrum bind pulses to a GPIO e.g. TX1
void rx_spektrum_bind(void) {
#define SPECTRUM_BIND_PIN usart_port_defs[profile.serial.rx].rx_pin

  if (profile.serial.rx == USART_PORT_INVALID) {
    return;
  }

  rx_bind_enable = fmc_read_float(56);
  if (rx_bind_enable == 0) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = SPECTRUM_BIND_PIN.pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SPECTRUM_BIND_PIN.port, &GPIO_InitStructure);

    // RX line, set high
    GPIO_SetBits(SPECTRUM_BIND_PIN.port, SPECTRUM_BIND_PIN.pin);
    // Bind window is around 20-140ms after powerup
    delay(60000);

    for (uint8_t i = 0; i < 9; i++) { // 9 pulses for internal dsmx 11ms, 3 pulses for internal dsm2 22ms
      // RX line, drive low for 120us
      GPIO_ResetBits(SPECTRUM_BIND_PIN.port, SPECTRUM_BIND_PIN.pin);
      delay(120);

      // RX line, drive high for 120us
      GPIO_SetBits(SPECTRUM_BIND_PIN.port, SPECTRUM_BIND_PIN.pin);
      delay(120);
    }
  }
}

#endif
