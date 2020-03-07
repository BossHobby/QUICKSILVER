#pragma once

#include "defines.h"
#include "drv_gpio.h"

#ifdef F4
#include "stm32f4xx_usart.h"
#endif
#ifdef F0
#include "stm32f0xx_usart.h"
#endif

typedef enum {
  RX_SERIAL_PROTOCOL_INVALID = 0,
  RX_SERIAL_PROTOCOL_DSM = 1,
  RX_SERIAL_PROTOCOL_SBUS = 2,
  RX_SERIAL_PROTOCOL_IBUS = 3,
  RX_SERIAL_PROTOCOL_FPORT = 4,
  RX_SERIAL_PROTOCOL_CRSF = 5,
} rx_serial_protocol_t;

typedef struct {
  uint8_t channel_index;
  USART_TypeDef *channel;

  uint32_t gpio_af;

  gpio_pin_def_t rx_pin;
  gpio_pin_def_t tx_pin;
} usart_port_def_t;

extern usart_port_def_t usart_port_defs[USART_PORTS_MAX];

extern usart_ports_t serial_rx_port;
extern usart_ports_t serial_smart_audio_port;

void serial_enable_rcc(usart_ports_t port);
void serial_enable_isr(usart_ports_t port);

void serial_debug_init(void);
void serial_rx_init(uint8_t rx_serial_protocol);
void serial_smart_audio_init(void);