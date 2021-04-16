#include "rx_unified_serial.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "control.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "flash.h"
#include "led.h"
#include "profile.h"
#include "project.h"
#include "usb_configurator.h"
#include "util.h"

#ifdef RX_UNIFIED_SERIAL

//This is the microsecond threshold for triggering a new frame to re-index to position 0 in the ISR
#define RX_FRAME_INTERVAL_TRIGGER_TICKS (1500 * (SYS_CLOCK_FREQ_HZ / 1000000L))

uint8_t rx_buffer[RX_BUFF_SIZE];
uint8_t rx_data[RX_BUFF_SIZE]; //A place to put the RX frame so nothing can get overwritten during processing.  //reduce size?

volatile uint8_t rx_frame_position = 0;
volatile uint8_t expected_frame_length = 10;
volatile frame_status_t frame_status = FRAME_INVALID;

uint32_t link_quality_raw;
uint32_t stat_frames_second;
uint32_t time_siglost;
uint32_t time_lastframe;

uint16_t bind_safety = 0;
int32_t channels[16];

uint8_t failsafe_sbus_failsafe = 0;
uint8_t failsafe_siglost = 0;
uint8_t failsafe_noframes = 0;

uint8_t telemetry_offset = 0;
uint8_t telemetry_packet[14];
uint8_t ready_for_next_telemetry = 1;

static rx_serial_protocol_t protocol_to_check = 1;
static uint16_t protocol_detect_timer = 0;

extern profile_t profile;

#define USART usart_port_defs[serial_rx_port]

void TX_USART_ISR(void) {
  //USART_ClearITPendingBit() for TC handled in drv_serial.c

  // buffer position 0 has already been called by the telemetry process so we start at 1
  static uint8_t increment_transmit_buffer = 1;

  // reset this to 0 so that a protocol switch will not create a tx isr that does stuff without need
  uint8_t bytes_to_send = 0;
  if (bind_storage.unified.protocol == RX_SERIAL_PROTOCOL_FPORT ||
      bind_storage.unified.protocol == RX_SERIAL_PROTOCOL_FPORT_INVERTED) {
    //upload total telemetry bytes to send so telemetry transmit triggers action appropriate to protocol
    bytes_to_send = 10 + telemetry_offset;
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
  volatile uint32_t ticks = DWT->CYCCNT;
  static volatile uint32_t last_ticks = 0;

  volatile uint32_t rx_byte_interval = 0;
  if (ticks >= last_ticks) {
    rx_byte_interval = ticks - last_ticks;
  } else {
    rx_byte_interval = (UINT32_MAX + ticks) - last_ticks;
  }
  last_ticks = ticks;

  if ((rx_byte_interval > RX_FRAME_INTERVAL_TRIGGER_TICKS) || frame_status == FRAME_DONE) {
    rx_frame_position = 0;
    frame_status = FRAME_IDLE;
  }

  if (USART_GetFlagStatus(USART.channel, USART_FLAG_ORE)) {
    // overflow means something was lost
    USART_ClearFlag(USART.channel, USART_FLAG_ORE);
    rx_frame_position = 0;
  }

  if (USART_GetITStatus(USART.channel, USART_IT_RXNE)) {
    USART_ClearITPendingBit(USART.channel, USART_IT_RXNE);

    rx_buffer[rx_frame_position++] = USART_ReceiveData(USART.channel);
    if (rx_frame_position >= expected_frame_length && frame_status == FRAME_IDLE) {
      frame_status = FRAME_RX;
    }

    rx_frame_position %= (RX_BUFF_SIZE);
  }
}

void rx_lqi_lost_packet() {
  link_quality_raw++;
  if (!time_siglost) {
    time_siglost = timer_micros();
  }

  // was TICK_CLOCK_FREQ_HZ 8,000,000 ticks on F0, 21M on F4. One second.
  // however gettime and timer_micros are in us.
  if (timer_micros() - time_siglost > FAILSAFETIME) {
    failsafe_siglost = 1;
  }
}

void rx_lqi_got_packet() {
  time_siglost = 0;
  failsafe_siglost = 0;
}

void rx_lqi_update_fps(uint16_t fixed_fps) {
  time_lastframe = timer_micros();

  // link quality & rssi
  static uint32_t fps_counter = 0;
  static uint32_t time_last_fps_update = 0;
  if (time_lastframe - time_last_fps_update > 1000000) {
    // two cases here: we have a fixed fps (fixed_fps > 0)
    // or we calculate fps on the fly
    if (fixed_fps > 0) {
      stat_frames_second = fixed_fps - link_quality_raw;
    } else {
      stat_frames_second = fps_counter;
      fps_counter = 0;
    }

    link_quality_raw = 0;
    time_last_fps_update = time_lastframe;
  }

  fps_counter++;
}

void rx_lqi_update_rssi_from_lqi(float expected_fps) {
  state.rx_rssi = stat_frames_second / expected_fps;
  state.rx_rssi = state.rx_rssi * state.rx_rssi * state.rx_rssi * LQ_EXPO + state.rx_rssi * (1 - LQ_EXPO);
  state.rx_rssi *= 100.0f;

  state.rx_rssi = constrainf(state.rx_rssi, 0.f, 100.f);
}

void rx_lqi_update_rssi_direct(float rssi) {
  state.rx_rssi = constrainf(rssi, 0.f, 100.f);
}

void rx_init(void) {
  flags.rx_mode = !RXMODE_BIND; // put LEDS in normal signal status
  rx_serial_init();
}

void rx_serial_update_frame_length(rx_serial_protocol_t proto) {
  switch (proto) {
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
    // crsf has variable frame length, assume 3 (header size) until we receive it
    expected_frame_length = 3;
    break;
  case RX_SERIAL_PROTOCOL_REDPINE:
  case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
    expected_frame_length = 11;
    break;
  default:
    break;
  }
}

void rx_serial_init(void) {
  //Let the uart ISR do its stuff.
  frame_status = FRAME_IDLE;

#ifdef RX_SBUS
  bind_storage.unified.protocol = RX_SERIAL_PROTOCOL_SBUS;
#endif
#ifdef RX_CRSF
  bind_storage.unified.protocol = RX_SERIAL_PROTOCOL_CRSF;
#endif
#ifdef RX_IBUS
  bind_storage.unified.protocol = RX_SERIAL_PROTOCOL_IBUS;
#endif
#ifdef RX_FPORT
  bind_storage.unified.protocol = RX_SERIAL_PROTOCOL_FPORT;
#endif
#ifdef RX_DSMX_2048
  bind_storage.unified.protocol = RX_SERIAL_PROTOCOL_DSM;
#endif
#ifdef RX_DSM2_1024
  bind_storage.unified.protocol = RX_SERIAL_PROTOCOL_DSM;
#endif

  if (bind_storage.unified.protocol == RX_SERIAL_PROTOCOL_INVALID) {
    //No known protocol? Can't really set the radio up yet then can we?
    state.rx_status = RX_STATUS_DETECTING;
    rx_serial_find_protocol();
  } else {
    state.rx_status = RX_STATUS_DETECTED + bind_storage.unified.protocol;
    serial_rx_init(bind_storage.unified.protocol); //There's already a known protocol, we're good.
    rx_serial_update_frame_length(bind_storage.unified.protocol);
  }
}

void rx_check() {
  if (bind_storage.unified.protocol == RX_SERIAL_PROTOCOL_INVALID) { //If there's no protocol, there's no reason to check failsafe.
    rx_serial_find_protocol();
    return;
  }

  state.rx_status = RX_STATUS_DETECTED + bind_storage.unified.protocol;

  //FAILSAFE! It gets checked every time!
  if (timer_micros() - time_lastframe > FAILSAFETIME) {
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
    switch (bind_storage.unified.protocol) {
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

    return;
  }

  if (frame_status == FRAME_TX) {
    switch (bind_storage.unified.protocol) {
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

      // reset frame length for next run
      expected_frame_length = 3;

      frame_status = FRAME_DONE;
      break;

    default:
      frame_status = FRAME_DONE;
      break;
    }

    return;
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
    quic_debugf("UNIFIED: trying protocol %d", protocol_to_check);

    state.rx_status = RX_STATUS_DETECTING + protocol_to_check;
    serial_rx_init(protocol_to_check); //Configure a protocol!
    rx_serial_update_frame_length(protocol_to_check);
  }

  protocol_detect_timer++; //Should increment once per main loop

  if (frame_status == FRAME_RX) { //We got something! What is it?
    switch (protocol_to_check) {
    case RX_SERIAL_PROTOCOL_DSM:
      if (rx_buffer[0] == 0x00 && rx_buffer[1] <= 0x04 && rx_buffer[2] != 0x00) {
        // allow up to 4 fades or detection will fail.  Some dsm rx will log a fade or two during binding
        rx_serial_process_dsmx();
        if (bind_safety > 0)
          bind_storage.unified.protocol = protocol_to_check;
      }
    case RX_SERIAL_PROTOCOL_SBUS:
    case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
      if (rx_buffer[0] == 0x0F) {
        bind_storage.unified.protocol = protocol_to_check;
      }
      break;
    case RX_SERIAL_PROTOCOL_IBUS:
      if (rx_buffer[0] == 0x20) {
        bind_storage.unified.protocol = protocol_to_check;
      }
      break;
    case RX_SERIAL_PROTOCOL_FPORT:
    case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
      if (rx_buffer[0] == 0x7E) {
        rx_serial_process_fport();
        if (bind_safety > 5) //FPORT INVERTED will trigger a frame on FPORT HALF DUPLEX - require >5 frames for good measure
          bind_storage.unified.protocol = protocol_to_check;
      }
      break;
    case RX_SERIAL_PROTOCOL_CRSF:
      if (rx_buffer[0] == 0xC8 &&
          rx_buffer[1] <= 64 &&
          (rx_buffer[2] == 0x16 || rx_buffer[2] == 0x14)) {
        bind_storage.unified.protocol = protocol_to_check;
      }
      break;
    case RX_SERIAL_PROTOCOL_REDPINE:
    case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
      if ((rx_buffer[0] & 0x3F) == 0x2A) {
        bind_storage.unified.protocol = protocol_to_check;
      }
      break;
    default:
      frame_status = FRAME_TX; //Whatever we got, it didn't make sense. Mark the frame as Checked and start over.
      break;
    }
  }

  if (bind_storage.unified.protocol == protocol_to_check) {
    quic_debugf("UNIFIED: protocol %d found", protocol_to_check);
  }

  if (protocol_detect_timer > 4000) { //4000 loops, half a second
    protocol_detect_timer = 0;        // Reset timer, triggering a shift to detecting the next protocol
  }
}

#endif
