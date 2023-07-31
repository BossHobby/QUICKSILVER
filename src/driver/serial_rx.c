#include "driver/serial.h"

#include "core/profile.h"
#include "driver/time.h"

#ifdef USE_RX_UNIFIED

static uint8_t tx_data[512];
static ring_buffer_t tx_buffer = {
    .buffer = tx_data,
    .head = 0,
    .tail = 0,
    .size = 512,
};

static uint8_t rx_data[512];
static ring_buffer_t rx_buffer = {
    .buffer = rx_data,
    .head = 0,
    .tail = 0,
    .size = 512,
};

serial_port_t serial_rx = {
    .rx_buffer = &rx_buffer,
    .tx_buffer = &tx_buffer,

    .tx_done = true,
};

void serial_rx_init(rx_serial_protocol_t proto) {
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

#endif