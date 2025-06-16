#include "driver/serial.h"

#include "driver/dma.h"
#include "driver/interrupt.h"
#include "driver/resource.h"
#include "driver/serial_soft.h"

const usart_port_def_t usart_port_defs[SERIAL_PORT_MAX] = {
    {},
    {
        .channel_index = 1,
        .channel = USART1,
        .irq = USART1_IRQn,
        .rcc = RCC_ENCODE(USART1),
        .dma_rx = DMA_DEVICE_SERIAL1_RX,
        .dma_tx = DMA_DEVICE_SERIAL1_TX,
        .dma_request_rx = 24,
        .dma_request_tx = 25,
    },
    {
        .channel_index = 2,
        .channel = USART2,
        .irq = USART2_IRQn,
        .rcc = RCC_ENCODE(USART2),
        .dma_rx = DMA_DEVICE_SERIAL2_RX,
        .dma_tx = DMA_DEVICE_SERIAL2_TX,
        .dma_request_rx = 26,
        .dma_request_tx = 27,
    },
    {
        .channel_index = 3,
        .channel = USART3,
        .irq = USART3_IRQn,
        .rcc = RCC_ENCODE(USART3),
        .dma_rx = DMA_DEVICE_SERIAL3_RX,
        .dma_tx = DMA_DEVICE_SERIAL3_TX,
        .dma_request_rx = 28,
        .dma_request_tx = 29,
    },
    {
        .channel_index = 4,
        .channel = UART4,
        .irq = UART4_IRQn,
        .rcc = RCC_ENCODE(UART4),
        .dma_rx = DMA_DEVICE_SERIAL4_RX,
        .dma_tx = DMA_DEVICE_SERIAL4_TX,
        .dma_request_rx = 30,
        .dma_request_tx = 31,
    },
    {
        .channel_index = 5,
        .channel = UART5,
        .irq = UART5_IRQn,
        .rcc = RCC_ENCODE(UART5),
        .dma_rx = DMA_DEVICE_SERIAL5_RX,
        .dma_tx = DMA_DEVICE_SERIAL5_TX,
        .dma_request_rx = 32,
        .dma_request_tx = 33,
    },
    {
        .channel_index = 6,
        .channel = USART6,
        .irq = USART6_IRQn,
        .rcc = RCC_ENCODE(USART6),
        .dma_rx = DMA_DEVICE_SERIAL6_RX,
        .dma_tx = DMA_DEVICE_SERIAL6_TX,
        .dma_request_rx = 34,
        .dma_request_tx = 35,
    },
    {
        .channel_index = 7,
        .channel = UART7,
        .irq = UART7_IRQn,
        .rcc = RCC_ENCODE(UART7),
        .dma_rx = DMA_DEVICE_SERIAL7_RX,
        .dma_tx = DMA_DEVICE_SERIAL7_TX,
        .dma_request_rx = 36,
        .dma_request_tx = 37,
    },
    {
        .channel_index = 8,
        .channel = UART8,
        .irq = UART8_IRQn,
        .rcc = RCC_ENCODE(UART8),
        .dma_rx = DMA_DEVICE_SERIAL8_RX,
        .dma_tx = DMA_DEVICE_SERIAL8_TX,
        .dma_request_rx = 38,
        .dma_request_tx = 39,
    },
};

static const uint32_t stop_bits_map[] = {
    [SERIAL_STOP_BITS_0_5] = USART_STOP_0_5_BIT,
    [SERIAL_STOP_BITS_1] = USART_STOP_1_BIT,
    [SERIAL_STOP_BITS_1_5] = USART_STOP_1_5_BIT,
    [SERIAL_STOP_BITS_2] = USART_STOP_2_BIT,
};

extern serial_port_t *serial_ports[SERIAL_PORT_MAX];

static void serial_init_dma_rx(serial_port_t *serial, const dma_stream_def_t *dma_rx) {
  const serial_ports_t port = serial->config.port;

  dma_enable_rcc(dma_rx);

  dma_reset(dma_rx->stream);
  dmamux_init(dma_rx->mux, target.dma[usart_port_defs[port].dma_rx].request);

  dma_init_type init;
  init.peripheral_base_addr = (uint32_t)(&usart_port_defs[port].channel->dt);
  init.memory_base_addr = (uint32_t)serial->rx_buffer->buffer;
  init.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  init.buffer_size = serial->rx_buffer->size;
  init.peripheral_inc_enable = FALSE;
  init.memory_inc_enable = TRUE;
  init.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  init.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  init.loop_mode_enable = TRUE;
  init.priority = DMA_PRIORITY_HIGH;
  dma_init(dma_rx->stream, &init);

  usart_dma_receiver_enable(usart_port_defs[port].channel, TRUE);
  usart_interrupt_enable(usart_port_defs[port].channel, USART_IDLE_INT, TRUE);
  dma_channel_enable(dma_rx->stream, TRUE);

  serial->rx_buffer->tail = 0;
  serial->rx_buffer->head = 0;
}

static void serial_init_dma_tx(serial_port_t *serial, const dma_stream_def_t *dma_tx) {
  const serial_ports_t port = serial->config.port;

  dma_enable_rcc(dma_tx);

  dma_reset(dma_tx->stream);
  dmamux_init(dma_tx->mux, target.dma[usart_port_defs[port].dma_tx].request);

  dma_init_type init;
  init.peripheral_base_addr = (uint32_t)(&usart_port_defs[port].channel->dt);
  init.memory_base_addr = 0;
  init.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  init.buffer_size = 0;
  init.peripheral_inc_enable = FALSE;
  init.memory_inc_enable = TRUE;
  init.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  init.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  init.loop_mode_enable = FALSE;
  init.priority = DMA_PRIORITY_HIGH;
  dma_init(dma_tx->stream, &init);

  usart_dma_transmitter_enable(usart_port_defs[port].channel, TRUE);
  interrupt_enable(dma_tx->irq, DMA_PRIORITY);
  dma_interrupt_enable(dma_tx->stream, DMA_FDT_INT, TRUE);
}

void handle_usart_invert(serial_ports_t port, bool invert) {
  // no inversion right now
}

void serial_hard_init(serial_port_t *serial, serial_port_config_t config, bool swap) {
  if (!serial || !serial->rx_buffer || !serial->tx_buffer) {
    return;
  }

  const serial_ports_t port = config.port;
  if (port <= SERIAL_PORT_INVALID || port >= SERIAL_PORT_MAX) {
    return;
  }

  if (config.stop_bits >= (sizeof(stop_bits_map) / sizeof(stop_bits_map[0]))) {
    return;
  }

  if (!usart_port_defs[port].channel) {
    return;
  }

  serial_enable_rcc(port);
  serial_disable_isr(port);

  usart_reset(usart_port_defs[port].channel);
  usart_init(usart_port_defs[port].channel, config.baudrate, USART_DATA_8BITS, stop_bits_map[config.stop_bits]);
  usart_parity_selection_config(usart_port_defs[port].channel, USART_PARITY_NONE);

  switch (config.direction) {
  case SERIAL_DIR_NONE:
    break;
  case SERIAL_DIR_RX:
    usart_receiver_enable(usart_port_defs[port].channel, TRUE);
    break;
  case SERIAL_DIR_TX:
    usart_transmitter_enable(usart_port_defs[port].channel, TRUE);
    break;
  case SERIAL_DIR_TX_RX:
    usart_transmitter_enable(usart_port_defs[port].channel, TRUE);
    usart_receiver_enable(usart_port_defs[port].channel, TRUE);
    break;
  }

  handle_usart_invert(port, config.invert);

  if (swap) {
    usart_transmit_receive_pin_swap(usart_port_defs[port].channel, TRUE);
  }
  if (config.half_duplex) {
    usart_single_line_halfduplex_select(usart_port_defs[port].channel, TRUE);
  }

  usart_enable(usart_port_defs[port].channel, TRUE);

  if ((config.direction & SERIAL_DIR_RX)) {
    const dma_stream_def_t *dma_rx = serial_get_dma_rx(serial->config.port);
    if (dma_rx)
      serial_init_dma_rx(serial, dma_rx);
    else
      usart_interrupt_enable(usart_port_defs[port].channel, USART_RDBF_INT, TRUE);
  }

  if ((config.direction & SERIAL_DIR_TX)) {
    const dma_stream_def_t *dma_tx = serial_get_dma_tx(serial->config.port);
    if (dma_tx)
      serial_init_dma_tx(serial, dma_tx);
    else
      usart_interrupt_enable(usart_port_defs[port].channel, USART_TDC_INT, TRUE);
  }

  serial_enable_isr(serial->config.port);
}

static bool serial_start_dma_tx(serial_port_t *serial) {
  const dma_stream_def_t *dma_tx = serial_get_dma_tx(serial->config.port);
  if (!dma_tx || serial_is_soft(serial->config.port)) {
    return false;
  }

  while (dma_tx->stream->ctrl_bit.chen)
    ;

  uint32_t tail;

  ATOMIC_BLOCK_ALL {
    if (!serial->tx_done)
      return false;

    const uint32_t head = serial->tx_buffer->head;
    tail = serial->tx_buffer->tail;

    if (head == tail)
      return false;

    serial->tx_dma_transfer_size = (head >= tail) ? (head - tail) : (serial->tx_buffer->size - tail);
    serial->tx_done = false;
  }

  dma_prepare_tx_memory((void *)(serial->tx_buffer->buffer + tail), serial->tx_dma_transfer_size);
  dma_tx->stream->maddr = (uint32_t)(serial->tx_buffer->buffer + tail);
  dma_data_number_set(dma_tx->stream, serial->tx_dma_transfer_size);
  dma_channel_enable(dma_tx->stream, TRUE);
  return true;
}

bool serial_write_bytes(serial_port_t *serial, const uint8_t *data, const uint32_t size) {
  if (!serial || !data || !serial->tx_buffer) {
    return false;
  }

  if (size == 0) {
    return true;
  }

  if (serial->config.port <= SERIAL_PORT_INVALID || serial->config.port >= SERIAL_PORT_MAX) {
    return false;
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

    if (serial_is_soft(serial->config.port) || !serial->tx_done) {
      continue;
    }

    if (serial_get_dma_tx(serial->config.port)) {
      serial_start_dma_tx(serial);
    } else {
      serial->tx_done = false;
      const usart_port_def_t *usart_def = &usart_port_defs[serial->config.port];
      usart_interrupt_enable(usart_def->channel, USART_TDBE_INT, TRUE);
    }
  }

  return true;
}

static void handle_serial_isr(serial_port_t *serial) {
  const usart_port_def_t *port = &usart_port_defs[serial->config.port];
  const dma_stream_def_t *dma_rx = serial_get_dma_rx(serial->config.port);

  if (usart_flag_get(port->channel, USART_ROERR_FLAG) == SET) {
    usart_flag_clear(port->channel, USART_ROERR_FLAG);
    ring_buffer_clear(serial->rx_buffer);
  }
  if (usart_flag_get(port->channel, USART_PERR_FLAG) == SET) {
    usart_flag_clear(port->channel, USART_PERR_FLAG);
    usart_data_receive(port->channel);
  }
  if (usart_flag_get(port->channel, USART_FERR_FLAG) == SET) {
    usart_flag_clear(port->channel, USART_FERR_FLAG);
    usart_data_receive(port->channel);
  }
  if (usart_flag_get(port->channel, USART_NERR_FLAG) == SET) {
    usart_flag_clear(port->channel, USART_NERR_FLAG);
    usart_data_receive(port->channel);
  }
  if (usart_flag_get(port->channel, USART_BFF_FLAG) == SET) {
    usart_flag_clear(port->channel, USART_BFF_FLAG);
    usart_data_receive(port->channel);
  }

  if (usart_flag_get(port->channel, USART_TDC_FLAG)) {
    usart_flag_clear(port->channel, USART_TDC_FLAG);
    if (serial->tx_done && serial->config.half_duplex && port->channel) {
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

  if (usart_flag_get(port->channel, USART_IDLEF_FLAG)) {
    usart_flag_clear(port->channel, USART_IDLEF_FLAG);
  }

  if (usart_flag_get(port->channel, USART_RDBF_FLAG)) {
    const volatile uint8_t data = usart_data_receive(port->channel);
    ring_buffer_write(serial->rx_buffer, data);
  }

  if (dma_rx) {
    const uint32_t dma_head = serial->rx_buffer->size - dma_data_number_get(dma_rx->stream);
    serial->rx_buffer->head = dma_head;
  }
}

static void handle_usart_isr(serial_ports_t index) {
  if (index <= SERIAL_PORT_INVALID || index >= SERIAL_PORT_MAX) {
    return;
  }

  if (serial_ports[index]) {
    handle_serial_isr(serial_ports[index]);
    return;
  }

  // stray serial port. disable all interrupts and clear error flags
  const usart_port_def_t *port = &usart_port_defs[index];
  usart_interrupt_enable(port->channel, USART_TDBE_INT, FALSE);
  usart_interrupt_enable(port->channel, USART_RDBF_INT, FALSE);
  usart_interrupt_enable(port->channel, USART_TDC_INT, FALSE);
  usart_flag_clear(port->channel, USART_ROERR_FLAG | USART_PERR_FLAG | USART_FERR_FLAG |
                                      USART_NERR_FLAG | USART_BFF_FLAG | USART_TDC_FLAG | USART_TDBE_FLAG | USART_RDBF_FLAG);
  usart_enable(port->channel, FALSE);
}

// we need handlers for both U_S_ART and UART.
// simply define both for every enabled port.
#define USART_IRQ_HANDLER(channel)          \
  void USART##channel##_IRQHandler() {      \
    handle_usart_isr(SERIAL_PORT##channel); \
  }                                         \
  void UART##channel##_IRQHandler() {       \
    handle_usart_isr(SERIAL_PORT##channel); \
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

static void handle_dma_tx_isr(serial_ports_t port) {
  const dma_stream_def_t *dma_tx = serial_get_dma_tx(port);
  if (!dma_tx || !serial_ports[port]) {
    return;
  }

  serial_port_t *serial = serial_ports[port];

  // Check for any error conditions first
  if (dma_is_flag_active(dma_tx, DMA_DTERR_FLAG)) {
    dma_clear_flag_tc(dma_tx);
    dma_channel_enable(dma_tx->stream, FALSE);
    serial->tx_done = true;
    return;
  }

  if (dma_is_flag_active(dma_tx, DMA_FDT_FLAG)) {
    dma_channel_enable(dma_tx->stream, FALSE);

    // Wait for stream to be fully disabled before proceeding
    while (dma_tx->stream->ctrl_bit.chen)
      ;

    const uint32_t remaining = dma_data_number_get(dma_tx->stream);
    if (remaining == 0)
      serial->tx_buffer->tail = (serial->tx_buffer->tail + serial->tx_dma_transfer_size) % serial->tx_buffer->size;

    if (!serial_start_dma_tx(serial)) {
      serial->tx_done = true;
    }
  }

  dma_clear_flag_tc(dma_tx);
}

void serial_dma_isr(const dma_device_t dev) {
  switch (dev) {
  case DMA_DEVICE_SERIAL1_TX:
    handle_dma_tx_isr(SERIAL_PORT1);
    break;
  case DMA_DEVICE_SERIAL2_TX:
    handle_dma_tx_isr(SERIAL_PORT2);
    break;
  case DMA_DEVICE_SERIAL3_TX:
    handle_dma_tx_isr(SERIAL_PORT3);
    break;
  case DMA_DEVICE_SERIAL4_TX:
    handle_dma_tx_isr(SERIAL_PORT4);
    break;
  case DMA_DEVICE_SERIAL5_TX:
    handle_dma_tx_isr(SERIAL_PORT5);
    break;
  case DMA_DEVICE_SERIAL6_TX:
    handle_dma_tx_isr(SERIAL_PORT6);
    break;
  case DMA_DEVICE_SERIAL7_TX:
    handle_dma_tx_isr(SERIAL_PORT7);
    break;
  case DMA_DEVICE_SERIAL8_TX:
    handle_dma_tx_isr(SERIAL_PORT8);
    break;
  default:
    break;
  }
}