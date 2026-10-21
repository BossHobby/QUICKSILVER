#include "rx/rx_serial.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control/control.h"
#include "core/debug.h"
#include "core/flash.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "io/led.h"
#include "rx/crsf.h"
#include "rx/serial_mavlink.h"
#include "util/ring_buffer.h"
#include "util/util.h"

#define BIND_SAFETY_COUNTER 120

#ifdef USE_RX_SERIAL

typedef struct {
  rx_protocol_t protocol;
  bool invert;
} rx_serial_candidate_t;

// Autodetect order: MAVLink last so the RC protocols lock first.
static constexpr rx_serial_candidate_t detect_candidates[] = {
    {RX_PROTOCOL_DSM, false},
    {RX_PROTOCOL_SBUS, false},
    {RX_PROTOCOL_IBUS, false},
    {RX_PROTOCOL_FPORT, false},
    {RX_PROTOCOL_CRSF, false},
    {RX_PROTOCOL_REDPINE_SERIAL, false},
    {RX_PROTOCOL_SBUS, true},
    {RX_PROTOCOL_FPORT, true},
    {RX_PROTOCOL_REDPINE_SERIAL, true},
    {RX_PROTOCOL_MAVLINK, false},
};
static constexpr uint32_t DETECT_CANDIDATE_COUNT = sizeof(detect_candidates) / sizeof(detect_candidates[0]);

static DMA_RAM uint8_t tx_buffer_data[512];
static ring_buffer_t tx_buffer = {
    .buffer = tx_buffer_data,
    .head = 0,
    .tail = 0,
    .size = 512,
};

static DMA_RAM uint8_t rx_buffer_data[512];
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

// A place to put the RX frame so nothing can get overwritten during processing.
uint8_t rx_data[RX_BUFF_SIZE];

static uint8_t bind_safety = 0;

static float rx_serial_expected_fps(rx_protocol_t proto) {
  switch (proto) {
  case RX_PROTOCOL_DSM:
    return rx_serial_dsm_expected_fps();

  case RX_PROTOCOL_FPORT:
  case RX_PROTOCOL_SBUS:
    return 111;

  case RX_PROTOCOL_IBUS:
    return (1000.0f / 7.0f);

  case RX_PROTOCOL_CRSF:
    return rx_serial_crsf_expected_fps();

  case RX_PROTOCOL_REDPINE_SERIAL:
    return 500;

  case RX_PROTOCOL_MAVLINK:
    return rx_serial_mavlink_expected_fps();

  default:
    return 0;
  }
}

static void rx_serial_update_lqi(rx_protocol_t proto, packet_status_t status) {
  if (status == PACKET_CHANNELS_RECEIVED)
    rx_lqi_got_packet();

  rx_lqi_update(rx_serial_expected_fps(proto));
}

static packet_status_t rx_serial_process(rx_protocol_t proto) {
  switch (proto) {
  case RX_PROTOCOL_DSM:
    return rx_serial_process_dsm();
  case RX_PROTOCOL_SBUS:
    return rx_serial_process_sbus();
  case RX_PROTOCOL_IBUS:
    return rx_serial_process_ibus();
  case RX_PROTOCOL_FPORT:
    return rx_serial_process_fport();
  case RX_PROTOCOL_CRSF:
    return rx_serial_process_crsf();
  case RX_PROTOCOL_REDPINE_SERIAL:
    return rx_serial_process_redpine();
  case RX_PROTOCOL_MAVLINK:
    return rx_serial_process_mavlink();
  default:
    return PACKET_ERROR;
  }
}

static void serial_rx_init(rx_protocol_t proto, bool invert) {
  mutex_guard_t configuration(profile_mutex);
  if (profile.serial.rx == SERIAL_PORT_INVALID) {
    return;
  }

  if (serial_is_soft(profile.serial.rx)) {
    return;
  }

  const target_serial_port_t *dev = &target.serial_ports[profile.serial.rx];
  if (!target_serial_port_valid(dev)) {
    return;
  }

  serial_port_config_t config = {};
  config.port = profile.serial.rx;

  switch (proto) {
  case RX_PROTOCOL_DSM:
  case RX_PROTOCOL_IBUS:
    config.baudrate = 115200;
    config.stop_bits = SERIAL_STOP_BITS_1;
    config.direction = SERIAL_DIR_RX;
    config.dma_mode = SERIAL_DMA_RX;
    break;

  case RX_PROTOCOL_FPORT:
    config.baudrate = 115200;
    config.stop_bits = SERIAL_STOP_BITS_1;
    config.direction = SERIAL_DIR_TX_RX;
    config.dma_mode = SERIAL_DMA_BOTH;
    break;

  case RX_PROTOCOL_SBUS:
    config.baudrate = 100000;
    config.stop_bits = SERIAL_STOP_BITS_2;
    config.direction = SERIAL_DIR_RX;
    config.dma_mode = SERIAL_DMA_RX;
    break;

  case RX_PROTOCOL_CRSF:
    config.baudrate = CRSF_BAUDRATE_DEFAULT;
    config.stop_bits = SERIAL_STOP_BITS_1;
    config.direction = SERIAL_DIR_TX_RX;
    config.dma_mode = SERIAL_DMA_BOTH;
    break;

  case RX_PROTOCOL_REDPINE_SERIAL:
    config.baudrate = 230400;
    config.stop_bits = SERIAL_STOP_BITS_1;
    config.direction = SERIAL_DIR_RX;
    config.dma_mode = SERIAL_DMA_RX;
    break;

  case RX_PROTOCOL_MAVLINK:
    config.baudrate = 460800;
    config.stop_bits = SERIAL_STOP_BITS_1;
    config.direction = SERIAL_DIR_TX_RX;
    break;

  default:
    return;
  }

  config.invert = invert;
  // inverted FPort runs full duplex
  config.half_duplex = proto == RX_PROTOCOL_FPORT && !invert;
  config.half_duplex_pp = false;

  serial_init(&serial_rx, config);
}

// Cycles the candidates once a second until one delivers enough frames, then
// makes it the active protocol and the profile selection, so a save keeps it.
static void rx_serial_find_protocol() {
  static uint32_t protocol_detect_time = 0;
  static uint32_t candidate_index = DETECT_CANDIDATE_COUNT;

  if ((time_millis() - protocol_detect_time) > 1000 || candidate_index == DETECT_CANDIDATE_COUNT) {
    candidate_index = candidate_index + 1 >= DETECT_CANDIDATE_COUNT ? 0 : candidate_index + 1;
    const rx_serial_candidate_t *next = &detect_candidates[candidate_index];
    quic_debugf("SERIAL RX: trying protocol %d invert %d", next->protocol, next->invert);

    serial_rx_init(next->protocol, next->invert);

    protocol_detect_time = time_millis();
    bind_safety = 0;
    return;
  }

  const rx_serial_candidate_t *candidate = &detect_candidates[candidate_index];
  const packet_status_t status = rx_serial_process(candidate->protocol);
  if (status > PACKET_NEEDS_MORE)
    rx_serial_update_lqi(candidate->protocol, status);

  if (status <= PACKET_NEEDS_MORE) {
    // no channels received, we are done here.
    return;
  }

  // dsm does only 45 packets / second worst case, so ~40 is enough for the rc protocols.
  // mavlink frames are validated by a message specific crc_extra and its rc uplink is
  // much slower, so a few frames are already conclusive
  const uint8_t detect_frames = candidate->protocol == RX_PROTOCOL_MAVLINK ? 3 : 40;
  if (bind_safety < detect_frames) {
    flags.rx_mode = RXMODE_BIND;
    bind_safety++;
  } else {
    mutex_guard_t configuration(profile_mutex);
    flags.rx_mode = RXMODE_NORMAL;
    flags.rx_ready = 1;
    profile.receiver.protocol = candidate->protocol;
    profile.receiver.invert = candidate->invert;
    state.rx_protocol = candidate->protocol;
    quic_debugf("SERIAL RX: protocol %d invert %d found", candidate->protocol, candidate->invert);
  }
}

void rx_serial_init() {
  if (profile.serial.rx == SERIAL_PORT_INVALID) {
    return;
  }

  if (state.rx_protocol == RX_PROTOCOL_SERIAL_AUTO) {
    rx_serial_find_protocol();
    return;
  }

  serial_rx_init(state.rx_protocol, profile.receiver.invert);
}

bool rx_serial_check() {
  if (profile.serial.rx == SERIAL_PORT_INVALID) {
    return false;
  }

  if (serial_rx.config.port != SERIAL_PORT_INVALID && serial_rx.config.port != profile.serial.rx) {
    return false;
  }

  const rx_protocol_t protocol = state.rx_protocol;
  if (protocol == RX_PROTOCOL_SERIAL_AUTO) {
    rx_serial_find_protocol();
    return false;
  }

  bool received = false;
  uint16_t valid_channels[RX_CHANNEL_MAX];
  memcpy(valid_channels, rx_channels, sizeof(valid_channels));
  // Each complete frame or rejected byte advances the parser. Bound attempts
  // by the queued bytes so continuous arrivals cannot monopolize the worker.
  const uint32_t attempts = serial_bytes_available(&serial_rx) + 1;
  for (uint32_t i = 0; i < attempts; i++) {
    const packet_status_t status = rx_serial_process(protocol);
    // A lost-frame packet may decode channels without making them valid input.
    if (status == PACKET_CHANNELS_RECEIVED)
      memcpy(valid_channels, rx_channels, sizeof(valid_channels));
    else
      memcpy(rx_channels, valid_channels, sizeof(valid_channels));
    rx_serial_update_lqi(protocol, status);
    if (status == PACKET_NEEDS_MORE)
      break;
    if (status == PACKET_ERROR)
      continue;

    if (bind_safety < BIND_SAFETY_COUNTER) {
      flags.rx_mode = RXMODE_BIND;
      bind_safety++;
    } else {
      flags.rx_mode = RXMODE_NORMAL;
      flags.rx_ready = 1;
    }
    received |= status == PACKET_CHANNELS_RECEIVED;
  }

  if (protocol == RX_PROTOCOL_MAVLINK)
    rx_serial_send_mavlink_telemetry();

  return received;
}

#endif
