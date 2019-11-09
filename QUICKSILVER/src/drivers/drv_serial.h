#include "defines.h"

#define USART_IDENT(channel) USART_PORT##channel
#define USART_PORT(channel, port, rx_pin, tx_pin) USART_IDENT(channel),

typedef enum {
  USART_PORTS USART_PORTS_MAX,
} usart_ports_t;

#undef USART_PORT

typedef struct {
  uint8_t channel_index;
  USART_TypeDef *channel;

  GPIO_TypeDef *gpio_port;
  uint32_t gpio_af;

  uint8_t rx_pin_index;
  uint32_t rx_pin;
  uint32_t rx_pin_source;
  uint8_t tx_pin_index;
  uint32_t tx_pin;
  uint32_t tx_pin_source;
} usart_port_def_t;

extern usart_port_def_t usart_port_defs[USART_PORTS_MAX];

void serial_init(void);
void serial_enable_rcc(usart_ports_t port);
void serial_enable_interrupt(usart_ports_t port);

void usart_rx_init(void);
