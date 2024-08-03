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

#define BIND_SAFETY_COUNTER 120

rx_serial_protocol_t serial_rx_detected_protcol = RX_SERIAL_PROTOCOL_INVALID;

#ifdef USE_RX_UNIFIED

static uint8_t tx_buffer_data[512];
static ring_buffer_t tx_buffer = {
    .buffer = tx_buffer_data,
    .head = 0,
    .tail = 0,
    .size = 512,
};

static uint8_t rx_buffer_data[512];
static ring_buffer_t rx_buffer = {
    .buffer = rx_buffer_data,
    .head = 0,
    .tail = 0,
    .size = 512,
};

serial_port_t serial_rx = {
    .rx_buffer = &rx_buffer,
    .tx_buffer = &tx_buffer,

    .tx_done = true,
};

int32_t channels[16];

uint8_t failsafe_noframes = 0;
uint8_t failsafe_sbus_failsafe = 0;
extern uint8_t failsafe_siglost;

// A place to put the RX frame so nothing can get overwritten during processing.
uint8_t rx_data[RX_BUFF_SIZE];

extern uint32_t last_frame_time_us;

static uint8_t bind_safety = 0;

static float rx_serial_expected_fps() {
  switch (serial_rx_detected_protcol) {
  case RX_SERIAL_PROTOCOL_INVALID:
    return 0;

  case RX_SERIAL_PROTOCOL_DSM:
    return rx_serial_dsm_expected_fps();

  case RX_SERIAL_PROTOCOL_FPORT:
  case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
  case RX_SERIAL_PROTOCOL_SBUS:
  case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
    return 111;

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

static packet_status_t rx_serial_process(rx_serial_protocol_t proto) {
  switch (proto) {
  case RX_SERIAL_PROTOCOL_DSM:
    return rx_serial_process_dsm();
  case RX_SERIAL_PROTOCOL_SBUS:
  case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
    return rx_serial_process_sbus();
  case RX_SERIAL_PROTOCOL_IBUS:
    return rx_serial_process_ibus();
  case RX_SERIAL_PROTOCOL_FPORT:
  case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
    return rx_serial_process_fport();
  case RX_SERIAL_PROTOCOL_CRSF:
    return rx_serial_process_crsf();
  case RX_SERIAL_PROTOCOL_REDPINE:
  case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
    return rx_serial_process_redpine();
  default:
    return PACKET_ERROR;
  }
}

static void serial_rx_init(rx_serial_protocol_t proto) {
  if (proto == RX_SERIAL_PROTOCOL_INVALID || profile.serial.rx == SERIAL_PORT_INVALID) {
    return;
  }

  if (serial_is_soft(profile.serial.rx)) {
    return;
  }

  const target_serial_port_t *dev = &target.serial_ports[profile.serial.rx];
  if (!target_serial_port_valid(dev)) {
    return;
  }

  serial_port_config_t config;
  config.port = profile.serial.rx;

  switch (proto) {
  case RX_SERIAL_PROTOCOL_DSM:
  case RX_SERIAL_PROTOCOL_IBUS:
    config.baudrate = 115200;
    config.stop_bits = SERIAL_STOP_BITS_1;
    config.direction = SERIAL_DIR_RX;
    break;

  case RX_SERIAL_PROTOCOL_FPORT:
  case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
    config.baudrate = 115200;
    config.stop_bits = SERIAL_STOP_BITS_1;
    config.direction = SERIAL_DIR_TX_RX;
    break;

  case RX_SERIAL_PROTOCOL_SBUS:
  case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
    config.baudrate = 100000;
    config.stop_bits = SERIAL_STOP_BITS_2;
    config.direction = SERIAL_DIR_RX;
    break;

  case RX_SERIAL_PROTOCOL_CRSF:
    config.baudrate = 420000;
    config.stop_bits = SERIAL_STOP_BITS_1;
    config.direction = SERIAL_DIR_TX_RX;
    break;

  case RX_SERIAL_PROTOCOL_REDPINE:
  case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
    config.baudrate = 230400;
    config.stop_bits = SERIAL_STOP_BITS_1;
    config.direction = SERIAL_DIR_RX;
    break;

  default:
    break;
  }

#if defined(INVERT_UART)
  // inversion is hard defined, always invert
  config.invert = true;
#else
  // invert according to protocol
  config.invert = proto == RX_SERIAL_PROTOCOL_SBUS_INVERTED || proto == RX_SERIAL_PROTOCOL_FPORT_INVERTED || proto == RX_SERIAL_PROTOCOL_REDPINE_INVERTED;
#endif

  // RX_SERIAL_PROTOCOL_FPORT_INVERTED requires half duplex off
  config.half_duplex = proto == RX_SERIAL_PROTOCOL_FPORT;
  config.half_duplex_pp = false;

  serial_init(&serial_rx, config);
}

static void rx_serial_find_protocol() {
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
#ifdef RX_DSM
  bind_storage.unified.protocol = RX_SERIAL_PROTOCOL_DSM;
#endif

  if (bind_storage.unified.protocol != RX_SERIAL_PROTOCOL_INVALID) {
    state.rx_status = RX_STATUS_DETECTED + bind_storage.unified.protocol;
    serial_rx_init(bind_storage.unified.protocol);
    serial_rx_detected_protcol = bind_storage.unified.protocol;
    return;
  }

  static uint32_t protocol_detect_time = 0;
  static rx_serial_protocol_t protocol_to_check = RX_SERIAL_PROTOCOL_INVALID;

  if ((time_millis() - protocol_detect_time) > 1000 || protocol_to_check == RX_SERIAL_PROTOCOL_INVALID) {
    protocol_to_check++; // Check the next protocol down the list.
    if (protocol_to_check > RX_SERIAL_PROTOCOL_MAX) {
      protocol_to_check = RX_SERIAL_PROTOCOL_DSM;
    }
    quic_debugf("UNIFIED: trying protocol %d", protocol_to_check);

    state.rx_status = RX_STATUS_DETECTING + protocol_to_check;
    serial_rx_init(protocol_to_check); // Configure a protocol!

    protocol_detect_time = time_millis();
    bind_safety = 0;
    return;
  }

  const packet_status_t status = rx_serial_process(protocol_to_check);
  if (status <= PACKET_NEEDS_MORE) {
    // no channels received, we are done here.
    return;
  }

  // dsm does only 45 packets / second worst case
  // lets hope that ~40 is enough
  if (bind_safety < 40) {
    flags.rx_mode = RXMODE_BIND;
    bind_safety++;
  } else {
    flags.rx_mode = RXMODE_NORMAL;
    flags.rx_ready = 1;
    bind_storage.unified.protocol = protocol_to_check;
    serial_rx_detected_protcol = protocol_to_check;
    quic_debugf("UNIFIED: protocol %d found", protocol_to_check);
  }
}

void rx_serial_init() {
  if (profile.serial.rx == SERIAL_PORT_INVALID) {
    return;
  }

  state.rx_status = RX_STATUS_DETECTING;
  rx_serial_find_protocol();
}

bool rx_serial_check() {
  if (profile.serial.rx == SERIAL_PORT_INVALID) {
    return false;
  }

  if (serial_rx.config.port != SERIAL_PORT_INVALID && serial_rx.config.port != profile.serial.rx) {
    return false;
  }

  if (serial_rx_detected_protcol == RX_SERIAL_PROTOCOL_INVALID) {
    rx_serial_find_protocol();
    return false;
  }

  // FAILSAFE! It gets checked every time!
  if (time_micros() - last_frame_time_us > FAILSAFE_TIME_US) {
    failsafe_noframes = 1;
  } else {
    failsafe_noframes = 0;
  }

  // add the 3 failsafes together
  if (flags.rx_ready) {
    flags.failsafe = failsafe_noframes || failsafe_siglost || failsafe_sbus_failsafe;
  }
  state.rx_status = RX_STATUS_DETECTED + serial_rx_detected_protcol;

  const packet_status_t status = rx_serial_process(serial_rx_detected_protcol);
  if (status == PACKET_NEEDS_MORE) {
    switch (serial_rx_detected_protcol) {
    case RX_SERIAL_PROTOCOL_FPORT:
    case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
      rx_serial_send_fport_telemetry();
      break;
    case RX_SERIAL_PROTOCOL_CRSF:
      rx_serial_send_crsf_telemetry();
      break;

    default:
      break;
    }
  }

  rx_lqi_update();

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_PACKET_RATE) {
    rx_lqi_update_from_fps(rx_serial_expected_fps());
  }

  if (status <= PACKET_NEEDS_MORE) {
    // no channels received, we are done here.
    return false;
  }

  if (bind_safety < BIND_SAFETY_COUNTER) {
    flags.rx_mode = RXMODE_BIND;
    bind_safety++;
  } else {
    flags.rx_mode = RXMODE_NORMAL;
    flags.rx_ready = 1;
  }

  return status == PACKET_CHANNELS_RECEIVED;
}

#endif