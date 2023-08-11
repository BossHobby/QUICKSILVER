#include "driver/serial.h"

#include "driver/serial_soft.h"

const usart_port_def_t usart_port_defs[SERIAL_PORT_MAX] = {
    {},
    {
        .channel_index = 1,
        .channel = USART1,
        .irq = USART1_IRQn,
        .rcc = RCC_ENCODE(USART1),
    },
    {
        .channel_index = 2,
        .channel = USART2,
        .irq = USART2_IRQn,
        .rcc = RCC_ENCODE(USART2),
    },
    {
        .channel_index = 3,
        .channel = USART3,
        .irq = USART3_IRQn,
        .rcc = RCC_ENCODE(USART3),
    },
    {
        .channel_index = 4,
        .channel = UART4,
        .irq = UART4_IRQn,
        .rcc = RCC_ENCODE(UART4),
    },
    {
        .channel_index = 5,
        .channel = UART5,
        .irq = UART5_IRQn,
        .rcc = RCC_ENCODE(UART5),
    },
    {
        .channel_index = 6,
        .channel = USART6,
        .irq = USART6_IRQn,
        .rcc = RCC_ENCODE(USART6),
    },
    {
        .channel_index = 7,
        .channel = UART7,
        .irq = UART7_IRQn,
        .rcc = RCC_ENCODE(UART7),
    },
    {
        .channel_index = 8,
        .channel = UART8,
        .irq = UART8_IRQn,
        .rcc = RCC_ENCODE(UART8),
    },
};

static const uint32_t stop_bits_map[] = {
    [SERIAL_STOP_BITS_0_5] = USART_STOP_0_5_BIT,
    [SERIAL_STOP_BITS_1] = USART_STOP_1_BIT,
    [SERIAL_STOP_BITS_1_5] = USART_STOP_1_5_BIT,
    [SERIAL_STOP_BITS_2] = USART_STOP_2_BIT,
};

extern serial_port_t *serial_ports[SERIAL_PORT_MAX];

#define USART usart_port_defs[port]

void handle_usart_invert(serial_ports_t port, bool invert) {
  // no inversion right now
}

void serial_hard_init(serial_port_t *serial, serial_port_config_t config, bool swap) {
  const serial_ports_t port = config.port;

  serial_enable_rcc(port);
  serial_disable_isr(port);

  usart_reset(USART.channel);
  usart_init(USART.channel, config.baudrate, USART_DATA_8BITS, stop_bits_map[config.stop_bits]);
  usart_parity_selection_config(USART.channel, USART_PARITY_NONE);

  switch (config.direction) {
  case SERIAL_DIR_NONE:
    break;
  case SERIAL_DIR_RX:
    usart_receiver_enable(USART.channel, TRUE);
    break;
  case SERIAL_DIR_TX:
    usart_transmitter_enable(USART.channel, TRUE);
    break;
  case SERIAL_DIR_TX_RX:
    usart_transmitter_enable(USART.channel, TRUE);
    usart_receiver_enable(USART.channel, TRUE);
    break;
  }

  handle_usart_invert(port, config.invert);

  if (swap) {
    usart_transmit_receive_pin_swap(USART.channel, TRUE);
  }
  if (config.half_duplex) {
    usart_single_line_halfduplex_select(USART.channel, TRUE);
  }

  usart_enable(USART.channel, TRUE);

  usart_interrupt_enable(USART.channel, USART_TDC_INT, TRUE);
  usart_interrupt_enable(USART.channel, USART_RDBF_INT, TRUE);

  serial_enable_isr(serial->config.port);
}

bool serial_write_bytes(serial_port_t *serial, const uint8_t *data, const uint32_t size) {
  if (size == 0) {
    return true;
  }

  if (serial->config.half_duplex) {
    if (!serial_is_soft(serial->config.port)) {
      const usart_port_def_t *port = &usart_port_defs[serial->config.port];
      usart_receiver_enable(port->channel, FALSE);
      usart_transmitter_enable(port->channel, TRUE);
    } else {
      soft_serial_enable_write(serial->config.port);
    }
  }

  uint32_t written = 0;
  while (written < size) {
    written += ring_buffer_write_multi(serial->tx_buffer, data + written, size - written);
    serial->tx_done = false;

    if (!serial_is_soft(serial->config.port)) {
      const usart_port_def_t *port = &usart_port_defs[serial->config.port];
      usart_interrupt_enable(port->channel, USART_TDBE_INT, TRUE);
    }
  }

  return true;
}

RAM_FUNC static void handle_serial_isr(serial_port_t *serial) {
  const usart_port_def_t *port = &usart_port_defs[serial->config.port];

  if (usart_flag_get(port->channel, USART_TDC_FLAG)) {
    usart_flag_clear(port->channel, USART_TDC_FLAG);
    if (serial->tx_done && serial->config.half_duplex) {
      usart_receiver_enable(port->channel, TRUE);
      usart_transmitter_enable(port->channel, FALSE);
    }
  }

  if (usart_flag_get(port->channel, USART_TDBE_FLAG)) {
    uint8_t data = 0;
    if (ring_buffer_read(serial->tx_buffer, &data)) {
      usart_data_transmit(port->channel, data);
    } else {
      usart_interrupt_enable(port->channel, USART_TDBE_INT, FALSE);
      serial->tx_done = true;
    }
  }

  if (usart_flag_get(port->channel, USART_RDBF_FLAG)) {
    const volatile uint8_t data = usart_data_receive(port->channel);
    ring_buffer_write(serial->rx_buffer, data);
  }

  if (usart_flag_get(port->channel, USART_ROERR_FLAG) == SET) {
    usart_flag_clear(port->channel, USART_ROERR_FLAG);
  }
}

RAM_FUNC static void handle_usart_isr(serial_ports_t index) {
  if (serial_ports[index]) {
    handle_serial_isr(serial_ports[index]);
    return;
  }

  // stray serial port. disable
  const usart_port_def_t *port = &usart_port_defs[index];
  usart_interrupt_enable(port->channel, USART_TDBE_INT, FALSE);
  usart_interrupt_enable(port->channel, USART_RDBF_INT, FALSE);
  usart_flag_clear(port->channel, USART_ROERR_FLAG);
  usart_enable(port->channel, FALSE);
}

// we need handlers for both U_S_ART and UART.
// simply define both for every enabled port.
#define USART_IRQ_HANDLER(channel)              \
  RAM_FUNC void USART##channel##_IRQHandler() { \
    handle_usart_isr(SERIAL_PORT##channel);     \
  }                                             \
  RAM_FUNC void UART##channel##_IRQHandler() {  \
    handle_usart_isr(SERIAL_PORT##channel);     \
  }

USART_IRQ_HANDLER(1)
USART_IRQ_HANDLER(2)
USART_IRQ_HANDLER(3)
USART_IRQ_HANDLER(4)
USART_IRQ_HANDLER(5)
USART_IRQ_HANDLER(6)
USART_IRQ_HANDLER(7)
USART_IRQ_HANDLER(8)

#undef USART_IRQ_HANDLER