#include "rx/unified_serial.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "core/debug.h"
#include "core/flash.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "flight/control.h"
#include "io/led.h"
#include "util/ring_buffer.h"
#include "util/util.h"

#define USART usart_port_defs[serial_rx_port]

// This is the microsecond threshold for triggering a new frame to re-index to position 0 in the ISR
#define RX_FRAME_INTERVAL_TRIGGER_TICKS (250 * (SYS_CLOCK_FREQ_HZ / 1000000L))

uint8_t rx_buffer[RX_BUFF_SIZE];
uint8_t rx_data[RX_BUFF_SIZE]; // A place to put the RX frame so nothing can get overwritten during processing.

static uint8_t _rx_data[RX_BUFF_SIZE];
ring_buffer_t rx_ring = {
    .buffer = _rx_data,
    .head = 0,
    .tail = 0,
    .size = RX_BUFF_SIZE,
};

volatile uint8_t rx_frame_position = 0;
volatile uint8_t expected_frame_length = 10;
volatile frame_status_t frame_status = FRAME_INVALID;

uint16_t bind_safety = 0;
int32_t channels[16];

uint8_t failsafe_sbus_failsafe = 0;
extern uint8_t failsafe_siglost;
uint8_t failsafe_noframes = 0;

uint8_t telemetry_packet[64];

static bool telemetry_done = true;
static uint32_t telemetry_size = 0;
static uint32_t telemetry_offset = 0;

static rx_serial_protocol_t protocol_to_check = 1;
static uint16_t protocol_detect_timer = 0;

extern profile_t profile;
extern uint32_t last_frame_time_us;

void rx_serial_isr() {
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

  if (LL_USART_IsActiveFlag_ORE(USART.channel)) {
    // overflow means something was lost
    LL_USART_ClearFlag_ORE(USART.channel);
    rx_frame_position = 0;
  }

  if (LL_USART_IsEnabledIT_RXNE(USART.channel) && LL_USART_IsActiveFlag_RXNE(USART.channel)) {
    const uint8_t data = LL_USART_ReceiveData8(USART.channel);
    rx_buffer[rx_frame_position] = data;
    rx_frame_position = (rx_frame_position + 1) % RX_BUFF_SIZE;

    ring_buffer_write(&rx_ring, data);

    if (rx_frame_position >= expected_frame_length && frame_status == FRAME_IDLE) {
      frame_status = FRAME_RX;
    }
  }

  if (LL_USART_IsEnabledIT_TC(USART.channel) && LL_USART_IsActiveFlag_TC(USART.channel)) {
    LL_USART_ClearFlag_TC(USART.channel);
    if (telemetry_offset == telemetry_size && !telemetry_done) {
      telemetry_done = true;
      LL_USART_DisableIT_TC(USART.channel);
    }
  }

  if (LL_USART_IsEnabledIT_TXE(USART.channel) && LL_USART_IsActiveFlag_TXE(USART.channel)) {
    if (telemetry_offset < telemetry_size) {
      LL_USART_TransmitData8(USART.channel, telemetry_packet[telemetry_offset]);
      telemetry_offset++;
    }
    if (telemetry_offset == telemetry_size) {
      LL_USART_DisableIT_TXE(USART.channel);
    }
  }
}

void rx_serial_send_telemetry(uint32_t size) {
  telemetry_size = size;
  telemetry_offset = 0;

  telemetry_done = false;

  LL_USART_ClearFlag_TC(USART.channel);

  LL_USART_EnableIT_TXE(USART.channel);
  LL_USART_EnableIT_TC(USART.channel);
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
    expected_frame_length = 28; // Minimum.
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

float rx_serial_expected_fps() {
  switch (bind_storage.unified.protocol) {
  case RX_SERIAL_PROTOCOL_INVALID:
    return 0;

  case RX_SERIAL_PROTOCOL_DSM:
    return rx_serial_dsm_expected_fps();

  case RX_SERIAL_PROTOCOL_FPORT:
  case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
  case RX_SERIAL_PROTOCOL_SBUS:
  case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
    return 112;

  case RX_SERIAL_PROTOCOL_IBUS:
    return (1000.0f / 7.0f);

  case RX_SERIAL_PROTOCOL_CRSF:
    return rx_serial_crsf_expected_fps();

  case RX_SERIAL_PROTOCOL_REDPINE:
  case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
    return 500;
  }

  return 0;
}

void rx_serial_init() {
  if (profile.serial.rx == USART_PORT_INVALID) {
    return;
  }

  // Let the uart ISR do its stuff.
  flags.rx_mode = !RXMODE_BIND; // put LEDS in normal signal status
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
#ifdef RX_DSMX
  bind_storage.unified.protocol = RX_SERIAL_PROTOCOL_DSM;
#endif
#ifdef RX_DSM2
  bind_storage.unified.protocol = RX_SERIAL_PROTOCOL_DSM;
#endif

  if (bind_storage.unified.protocol == RX_SERIAL_PROTOCOL_INVALID) {
    // No known protocol? Can't really set the radio up yet then can we?
    state.rx_status = RX_STATUS_DETECTING;
    rx_serial_find_protocol();
  } else {
    state.rx_status = RX_STATUS_DETECTED + bind_storage.unified.protocol;
    serial_rx_init(bind_storage.unified.protocol); // There's already a known protocol, we're good.
    rx_serial_update_frame_length(bind_storage.unified.protocol);
  }
}

bool rx_serial_check() {
  if (profile.serial.rx == USART_PORT_INVALID) {
    return false;
  }

  if (serial_rx_port != USART_PORT_INVALID && serial_rx_port != profile.serial.rx) {
    return false;
  }

  if (bind_storage.unified.protocol == RX_SERIAL_PROTOCOL_INVALID) { // If there's no protocol, there's no reason to check failsafe.
    rx_serial_find_protocol();
    return false;
  }

  bool channels_received = false;

  state.rx_status = RX_STATUS_DETECTED + bind_storage.unified.protocol;

  // FAILSAFE! It gets checked every time!
  if (time_micros() - last_frame_time_us > FAILSAFETIME) {
    failsafe_noframes = 1;
  } else {
    failsafe_noframes = 0;
  }

  // add the 3 failsafes together
  if (flags.rx_ready) {
    flags.failsafe = failsafe_noframes || failsafe_siglost || failsafe_sbus_failsafe;
  }

  if (frame_status == FRAME_INVALID) {
    // RX/USART not set up.
    // Set it up. This includes autodetecting protocol if necesary
    rx_serial_init();
  } else if (frame_status == FRAME_RX) {
    // USART ISR says there's enough frame to look at. Look at it.
    switch (bind_storage.unified.protocol) {
    case RX_SERIAL_PROTOCOL_DSM:
      channels_received = rx_serial_process_dsm();
      break;
    case RX_SERIAL_PROTOCOL_SBUS:
    case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
      channels_received = rx_serial_process_sbus();
      break;
    case RX_SERIAL_PROTOCOL_IBUS:
      channels_received = rx_serial_process_ibus();
      break;
    case RX_SERIAL_PROTOCOL_FPORT:
    case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
      channels_received = rx_serial_process_fport();
      break;
    case RX_SERIAL_PROTOCOL_CRSF:
      channels_received = rx_serial_process_crsf();
      break;
    case RX_SERIAL_PROTOCOL_REDPINE:
    case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
      channels_received = rx_serial_process_redpine();
    default:
      break;
    }
  }

  if (!channels_received) {
    switch (bind_storage.unified.protocol) {
    case RX_SERIAL_PROTOCOL_FPORT:
    case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
      if (telemetry_done) {
        rx_serial_send_fport_telemetry();
      }
      break;
    case RX_SERIAL_PROTOCOL_CRSF:
      if (telemetry_done) {
        rx_serial_send_crsf_telemetry();
      }
      break;

    default:
      break;
    }
  }

  rx_lqi_update();

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_PACKET_RATE) {
    rx_lqi_update_from_fps(rx_serial_expected_fps());
  }

  return channels_received;
}

// NOTE TO SELF: Put in some double-check code on the detections somehow.
// NFE note:  how about we force hold failsafe until protocol is saved.  This acts like kind of a check on proper mapping/decoding as stick gesture must be used as a test
//  also, we ought to be able to clear noframes_failsafe in addition to satisfying the start byte check in order to hard select a radio protocol
void rx_serial_find_protocol() {
  if (protocol_detect_timer == 0) {
    protocol_to_check++; // Check the next protocol down the list.
    if (protocol_to_check > RX_SERIAL_PROTOCOL_MAX) {
      protocol_to_check = RX_SERIAL_PROTOCOL_DSM;
    }
    quic_debugf("UNIFIED: trying protocol %d", protocol_to_check);

    state.rx_status = RX_STATUS_DETECTING + protocol_to_check;
    serial_rx_init(protocol_to_check); // Configure a protocol!
    rx_serial_update_frame_length(protocol_to_check);
  }

  protocol_detect_timer++; // Should increment once per main loop

  if (frame_status == FRAME_RX) { // We got something! What is it?
    switch (protocol_to_check) {
    case RX_SERIAL_PROTOCOL_DSM:
      if (rx_buffer[0] < 4 && (rx_buffer[1] == 0x12 || rx_buffer[1] == 0x01 || rx_buffer[1] == 0xB2 || rx_buffer[1] == 0xA2)) {
        // allow up to 4 fades or detection will fail.  Some dsm rx will log a fade or two during binding
        rx_serial_process_dsm();
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
        if (bind_safety > 5) // FPORT INVERTED will trigger a frame on FPORT HALF DUPLEX - require >5 frames for good measure
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
      frame_status = FRAME_DONE; // Whatever we got, it didn't make sense. Mark the frame as Checked and start over.
      break;
    }
  }

  if (bind_storage.unified.protocol == protocol_to_check) {
    quic_debugf("UNIFIED: protocol %d found", protocol_to_check);
  }

  if (protocol_detect_timer > 4000) { // 4000 loops, half a second
    protocol_detect_timer = 0;        // Reset timer, triggering a shift to detecting the next protocol
  }
}
