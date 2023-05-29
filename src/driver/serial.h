#pragma once

#include <stdbool.h>

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/rcc.h"
#include "util/ring_buffer.h"

typedef enum {
  RX_SERIAL_PROTOCOL_INVALID,
  RX_SERIAL_PROTOCOL_DSM,
  RX_SERIAL_PROTOCOL_SBUS,
  RX_SERIAL_PROTOCOL_IBUS,
  RX_SERIAL_PROTOCOL_FPORT,
  RX_SERIAL_PROTOCOL_CRSF,
  RX_SERIAL_PROTOCOL_REDPINE,
  RX_SERIAL_PROTOCOL_SBUS_INVERTED,
  RX_SERIAL_PROTOCOL_FPORT_INVERTED,
  RX_SERIAL_PROTOCOL_REDPINE_INVERTED,
  RX_SERIAL_PROTOCOL_MAX = RX_SERIAL_PROTOCOL_REDPINE_INVERTED
} rx_serial_protocol_t;

typedef enum {
  SERIAL_STOP_BITS_0_5,
  SERIAL_STOP_BITS_1,
  SERIAL_STOP_BITS_1_5,
  SERIAL_STOP_BITS_2,
} serial_stop_bits_t;

typedef enum {
  SERIAL_DIR_NONE,
  SERIAL_DIR_RX,
  SERIAL_DIR_TX,
  SERIAL_DIR_TX_RX,
} serial_direction_t;

typedef struct {
  serial_ports_t port;
  uint32_t baudrate;
  serial_direction_t direction;
  serial_stop_bits_t stop_bits;
  bool invert;
  bool half_duplex;
} serial_port_config_t;

typedef struct {
  serial_port_config_t config;

  ring_buffer_t *rx_buffer;
  ring_buffer_t *tx_buffer;

  bool tx_done;
} serial_port_t;

typedef struct {
  uint8_t channel_index;
  usart_dev_t *channel;
  IRQn_Type irq;
  rcc_reg_t rcc;
} usart_port_def_t;

extern const usart_port_def_t usart_port_defs[SERIAL_PORT_MAX];

extern serial_port_t serial_rx;
extern serial_port_t serial_hdzero;
extern serial_port_t serial_vtx;

void serial_rx_init(rx_serial_protocol_t rx_serial_protocol);

void serial_init(serial_port_t *serial, serial_port_config_t config);
uint32_t serial_bytes_available(serial_port_t *serial);
uint32_t serial_read_bytes(serial_port_t *serial, uint8_t *data, const uint32_t size);
bool serial_write_bytes(serial_port_t *serial, const uint8_t *data, const uint32_t size);

bool serial_is_soft(serial_ports_t port);