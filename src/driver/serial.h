#pragma once

#include <stdbool.h>

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/rcc.h"
#include "driver/serial_soft.h"
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
} rx_serial_protocol_t;

// potentially a debug tool to limit the detection sequence of universal serial
// todo:  purge this if deemed unnecessary
// #define RX_SERIAL_PROTOCOL_MAX RX_SERIAL_PROTOCOL_CRSF
#define RX_SERIAL_PROTOCOL_MAX RX_SERIAL_PROTOCOL_REDPINE_INVERTED

typedef struct {
  serial_ports_t port;

  ring_buffer_t *rx_buffer;
  ring_buffer_t *tx_buffer;
} serial_port_t;

typedef struct {
  uint8_t channel_index;
  USART_TypeDef *channel;
  IRQn_Type irq;
  rcc_reg_t rcc;
} usart_port_def_t;

extern const usart_port_def_t usart_port_defs[SERIAL_PORT_MAX];

extern serial_ports_t serial_rx_port;
extern serial_ports_t serial_smart_audio_port;
extern serial_ports_t serial_hdzero_port;

void serial_rx_init(rx_serial_protocol_t rx_serial_protocol);

void serial_enable_rcc(serial_ports_t port);
void serial_port_init(serial_ports_t port, LL_USART_InitTypeDef *usart_init, bool half_duplex, bool invert);

void serial_enable_isr(serial_ports_t port);
void serial_disable_isr(serial_ports_t port);

void serial_init(serial_port_t *serial, serial_ports_t port, uint32_t baudrate, uint8_t stop_bits, bool half_duplex);
uint32_t serial_read_bytes(serial_port_t *serial, uint8_t *data, const uint32_t size);
bool serial_write_bytes(serial_port_t *serial, const uint8_t *data, const uint32_t size);

bool serial_is_soft(serial_ports_t port);