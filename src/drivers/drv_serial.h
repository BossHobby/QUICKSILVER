#pragma once

#include "defines.h"
#include "drv_gpio.h"

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
//#define RX_SERIAL_PROTOCOL_MAX RX_SERIAL_PROTOCOL_CRSF
#define RX_SERIAL_PROTOCOL_MAX RX_SERIAL_PROTOCOL_REDPINE_INVERTED

typedef struct {
  uint8_t channel_index;
  USART_TypeDef *channel;

  uint32_t gpio_af;

  gpio_pins_t rx_pin;
  gpio_pins_t tx_pin;
} usart_port_def_t;

extern usart_port_def_t usart_port_defs[USART_PORTS_MAX];

extern usart_ports_t serial_rx_port;
extern usart_ports_t serial_smart_audio_port;

void serial_enable_rcc(usart_ports_t port);
void serial_enable_isr(usart_ports_t port);

void serial_debug_init(void);
void serial_rx_init(rx_serial_protocol_t rx_serial_protocol);
void serial_smart_audio_init(void);
