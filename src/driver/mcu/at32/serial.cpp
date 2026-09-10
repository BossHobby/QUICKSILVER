#include "driver/serial.h"

#include "driver/dma.h"
#include "driver/interrupt.h"
#include "driver/serial_soft.h"

extern const usart_port_def_t usart_port_defs[SERIAL_PORT_MAX] = {
    {},
    {
        .channel_index = 1,
        .channel = USART1,
        .irq = USART1_IRQn,
        .rcc = RCC_ENCODE(USART1),
        .dma_request = {.rx = DMAMUX_DMAREQ_ID_USART1_RX, .tx = DMAMUX_DMAREQ_ID_USART1_TX},
    },
    {
        .channel_index = 2,
        .channel = USART2,
        .irq = USART2_IRQn,
        .rcc = RCC_ENCODE(USART2),
        .dma_request = {.rx = DMAMUX_DMAREQ_ID_USART2_RX, .tx = DMAMUX_DMAREQ_ID_USART2_TX},
    },
    {
        .channel_index = 3,
        .channel = USART3,
        .irq = USART3_IRQn,
        .rcc = RCC_ENCODE(USART3),
        .dma_request = {.rx = DMAMUX_DMAREQ_ID_USART3_RX, .tx = DMAMUX_DMAREQ_ID_USART3_TX},
    },
    {
        .channel_index = 4,
        .channel = UART4,
        .irq = UART4_IRQn,
        .rcc = RCC_ENCODE(UART4),
        .dma_request = {.rx = DMAMUX_DMAREQ_ID_UART4_RX, .tx = DMAMUX_DMAREQ_ID_UART4_TX},
    },
    {
        .channel_index = 5,
        .channel = UART5,
        .irq = UART5_IRQn,
        .rcc = RCC_ENCODE(UART5),
        .dma_request = {.rx = DMAMUX_DMAREQ_ID_UART5_RX, .tx = DMAMUX_DMAREQ_ID_UART5_TX},
    },
    {
        .channel_index = 6,
        .channel = USART6,
        .irq = USART6_IRQn,
        .rcc = RCC_ENCODE(USART6),
        .dma_request = {.rx = DMAMUX_DMAREQ_ID_USART6_RX, .tx = DMAMUX_DMAREQ_ID_USART6_TX},
    },
    {
        .channel_index = 7,
        .channel = UART7,
        .irq = UART7_IRQn,
        .rcc = RCC_ENCODE(UART7),
        .dma_request = {.rx = DMAMUX_DMAREQ_ID_UART7_RX, .tx = DMAMUX_DMAREQ_ID_UART7_TX},
    },
    {
        .channel_index = 8,
        .channel = UART8,
        .irq = UART8_IRQn,
        .rcc = RCC_ENCODE(UART8),
        .dma_request = {.rx = DMAMUX_DMAREQ_ID_UART8_RX, .tx = DMAMUX_DMAREQ_ID_UART8_TX},
    },
};

static const usart_stop_bit_num_type stop_bits_map[] = {
    USART_STOP_0_5_BIT, // SERIAL_STOP_BITS_0_5
    USART_STOP_1_BIT,   // SERIAL_STOP_BITS_1
    USART_STOP_1_5_BIT, // SERIAL_STOP_BITS_1_5
    USART_STOP_2_BIT,   // SERIAL_STOP_BITS_2
};

extern serial_port_t *serial_ports[SERIAL_PORT_MAX];

typedef struct {
  dma_stream_t rx_stream;
  dma_stream_t tx_stream;
  uint32_t rx_pos;
  uint32_t tx_len;
} serial_dma_state_t;

static_assert(DMA_STREAM_INVALID == 0 && SERIAL_PORT_INVALID == 0);
static serial_dma_state_t serial_dma_state[SERIAL_PORT_MAX] = {};
static serial_ports_t serial_dma_tx_stream_ports[DMA_STREAM_MAX] = {};

static uint32_t serial_dma_tx_chunk_size(ring_buffer_t *tx_buffer) {
  if (!tx_buffer || !tx_buffer->buffer || tx_buffer->size < 2) {
    return 0;
  }

  const uint32_t head = tx_buffer->head;
  const uint32_t tail = tx_buffer->tail;
  MEMORY_BARRIER();

  if (head == tail) {
    return 0;
  }
  if (tail < head) {
    return head - tail;
  }
  return tx_buffer->size - tail;
}

static void serial_dma_release_port(serial_ports_t port) {
  if (port <= SERIAL_PORT_INVALID || port >= SERIAL_PORT_MAX) {
    return;
  }

  serial_dma_state_t *state = &serial_dma_state[port];

  if (state->rx_stream != DMA_STREAM_INVALID) {
    const dma_stream_def_t *dma = &dma_stream_defs[state->rx_stream];
    usart_dma_receiver_enable(usart_port_defs[port].channel, FALSE);
    dma_channel_enable(dma->stream, FALSE);
    dma_release(state->rx_stream, DMA_DEVICE_SERIAL_RX);
  }

  if (state->tx_stream != DMA_STREAM_INVALID) {
    const dma_stream_def_t *dma = &dma_stream_defs[state->tx_stream];
    usart_dma_transmitter_enable(usart_port_defs[port].channel, FALSE);
    dma_channel_enable(dma->stream, FALSE);
    serial_dma_tx_stream_ports[state->tx_stream] = SERIAL_PORT_INVALID;
    dma_release(state->tx_stream, DMA_DEVICE_SERIAL_TX);
  }

  *state = (serial_dma_state_t){
      .rx_stream = DMA_STREAM_INVALID,
      .tx_stream = DMA_STREAM_INVALID,
  };
}

static bool serial_dma_setup_rx(serial_port_t *serial) {
  const serial_ports_t port = serial->config.port;
  const uint32_t request = usart_port_defs[port].dma_request.rx;
  if (request == 0) {
    return false;
  }
  if (!serial->rx_buffer || !serial->rx_buffer->buffer || serial->rx_buffer->size < 2) {
    return false;
  }

  const dma_stream_t stream = dma_claim_unused(DMA_DEVICE_SERIAL_RX);
  if (stream == DMA_STREAM_INVALID) {
    return false;
  }

  serial_dma_state_t *state = &serial_dma_state[port];
  const dma_stream_def_t *dma = &dma_stream_defs[stream];
  dma_enable_rcc(dma);

  dma_reset(dma->stream);
  dmamux_init(dma->mux, (dmamux_requst_id_sel_type)request);

  dma_init_type init;
  init.peripheral_base_addr = (uint32_t)&usart_port_defs[port].channel->dt;
  init.memory_base_addr = (uint32_t)serial->rx_buffer->buffer;
  init.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  init.buffer_size = serial->rx_buffer->size;
  init.peripheral_inc_enable = FALSE;
  init.memory_inc_enable = TRUE;
  init.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  init.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  init.loop_mode_enable = TRUE;
  init.priority = DMA_PRIORITY_HIGH;
  dma_init(dma->stream, &init);

  dma_prepare_rx_memory(serial->rx_buffer->buffer, serial->rx_buffer->size);
  dma_clear_flag_tc(dma);
  dma_channel_enable(dma->stream, TRUE);
  usart_dma_receiver_enable(usart_port_defs[port].channel, TRUE);

  state->rx_stream = stream;
  state->rx_pos = 0;
  return true;
}

static bool serial_dma_setup_tx(serial_ports_t port) {
  const uint32_t request = usart_port_defs[port].dma_request.tx;
  if (request == 0) {
    return false;
  }

  const dma_stream_t stream = dma_claim_unused(DMA_DEVICE_SERIAL_TX);
  if (stream == DMA_STREAM_INVALID) {
    return false;
  }

  serial_dma_state_t *state = &serial_dma_state[port];
  const dma_stream_def_t *dma = &dma_stream_defs[stream];
  dma_enable_rcc(dma);

  dma_reset(dma->stream);
  dmamux_init(dma->mux, (dmamux_requst_id_sel_type)request);

  dma_init_type init;
  init.peripheral_base_addr = (uint32_t)&usart_port_defs[port].channel->dt;
  init.memory_base_addr = 0;
  init.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  init.buffer_size = 0;
  init.peripheral_inc_enable = FALSE;
  init.memory_inc_enable = TRUE;
  init.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  init.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  init.loop_mode_enable = FALSE;
  init.priority = DMA_PRIORITY_HIGH;
  dma_init(dma->stream, &init);

  dma_clear_flag_tc(dma);
  interrupt_enable(dma->irq, DMA_PRIORITY);
  dma_interrupt_enable(dma->stream, DMA_FDT_INT, TRUE);
  dma_interrupt_enable(dma->stream, DMA_DTERR_INT, TRUE);

  state->tx_stream = stream;
  state->tx_len = 0;
  serial_dma_tx_stream_ports[stream] = port;
  return true;
}

static bool serial_dma_start_tx(serial_port_t *serial) {
  const serial_ports_t port = serial->config.port;
  serial_dma_state_t *state = &serial_dma_state[port];
  if (state->tx_stream == DMA_STREAM_INVALID) {
    return false;
  }
  if (state->tx_len > 0) {
    return true;
  }

  ring_buffer_t *tx_buffer = serial->tx_buffer;
  const uint32_t size = serial_dma_tx_chunk_size(tx_buffer);
  if (size == 0) {
    return false;
  }
  const uint32_t tail = tx_buffer->tail;

  const dma_stream_def_t *dma = &dma_stream_defs[state->tx_stream];
  while (dma->stream->ctrl_bit.chen)
    ;

  dma_clear_flag_tc(dma);
  dma_prepare_tx_memory(&tx_buffer->buffer[tail], size);
  dma->stream->maddr = (uint32_t)&tx_buffer->buffer[tail];
  dma_data_number_set(dma->stream, size);
  usart_flag_clear(usart_port_defs[port].channel, USART_TDC_FLAG);
  usart_interrupt_enable(usart_port_defs[port].channel, USART_TDBE_INT, FALSE);
  usart_interrupt_enable(usart_port_defs[port].channel, USART_TDC_INT, FALSE);
  dma_channel_enable(dma->stream, TRUE);
  usart_dma_transmitter_enable(usart_port_defs[port].channel, TRUE);

  state->tx_len = size;
  serial->tx_done = false;
  return true;
}

static serial_dma_mode_t serial_dma_init(serial_port_t *serial) {
  const serial_ports_t port = serial->config.port;
  serial_dma_mode_t active = SERIAL_DMA_NONE;

  if ((serial->config.direction & SERIAL_DIR_RX) &&
      (serial->config.dma_mode & SERIAL_DMA_RX) &&
      serial_dma_setup_rx(serial)) {
    active = static_cast<serial_dma_mode_t>(active | SERIAL_DMA_RX);
  }
  if ((serial->config.direction & SERIAL_DIR_TX) &&
      (serial->config.dma_mode & SERIAL_DMA_TX) &&
      serial_dma_setup_tx(port)) {
    active = static_cast<serial_dma_mode_t>(active | SERIAL_DMA_TX);
  }
  return active;
}

void serial_hard_sync_rx(serial_port_t *serial) {
  if (!serial || serial->config.port <= SERIAL_PORT_INVALID || serial->config.port >= SERIAL_PORT_MAX) {
    return;
  }

  const serial_ports_t port = serial->config.port;
  serial_dma_state_t *state = &serial_dma_state[port];
  if (state->rx_stream == DMA_STREAM_INVALID) {
    return;
  }

  ring_buffer_t *rx_buffer = serial->rx_buffer;
  if (!rx_buffer || rx_buffer->size < 2) {
    return;
  }

  const dma_stream_def_t *dma = &dma_stream_defs[state->rx_stream];
  const bool transfer_complete = dma_is_flag_active(dma, DMA_FDT_FLAG);
  const bool transfer_error = dma_is_flag_active(dma, DMA_DTERR_FLAG);
  if (transfer_complete || transfer_error || dma_is_flag_active(dma, DMA_HDT_FLAG)) {
    dma_clear_flag_tc(dma);
  }
  if (transfer_error) {
    serial->rx_error_count = serial->rx_error_count + 1;
  }

  const uint32_t pos = (rx_buffer->size - dma_data_number_get(dma->stream)) % rx_buffer->size;
  const uint32_t old_pos = state->rx_pos;
  uint32_t received = (pos >= old_pos) ? (pos - old_pos) : (rx_buffer->size - old_pos + pos);
  if (received == 0) {
    if (!transfer_complete) {
      return;
    }
    received = rx_buffer->size;
  }

  if (received > ring_buffer_free(rx_buffer)) {
    serial->rx_error_count = serial->rx_error_count + 1;
    rx_buffer->tail = (pos + 1) % rx_buffer->size;
  }

  state->rx_pos = pos;
  MEMORY_BARRIER();
  rx_buffer->head = pos;
}

void serial_dma_tx_isr(dma_stream_t stream) {
  if (stream <= DMA_STREAM_INVALID || stream >= DMA_STREAM_MAX) {
    return;
  }

  const serial_ports_t port = serial_dma_tx_stream_ports[stream];
  if (port <= SERIAL_PORT_INVALID || port >= SERIAL_PORT_MAX) {
    return;
  }

  serial_dma_state_t *state = &serial_dma_state[port];
  if (state->tx_stream != stream) {
    return;
  }

  const dma_stream_def_t *dma = &dma_stream_defs[stream];
  const bool transfer_error = dma_is_flag_active(dma, DMA_DTERR_FLAG);
  const bool transfer_complete = dma_is_flag_active(dma, DMA_FDT_FLAG);
  dma_clear_flag_tc(dma);

  if (!serial_ports[port]) {
    return;
  }

  serial_port_t *serial = serial_ports[port];
  usart_dma_transmitter_enable(usart_port_defs[port].channel, FALSE);
  dma_channel_enable(dma->stream, FALSE);

  ring_buffer_t *tx_buffer = serial->tx_buffer;
  if (tx_buffer && tx_buffer->size > 0) {
    tx_buffer->tail = (tx_buffer->tail + state->tx_len) % tx_buffer->size;
  }
  state->tx_len = 0;

  if (transfer_error) {
    serial->tx_done = true;
    return;
  }

  if (transfer_complete && !serial_dma_start_tx(serial)) {
    usart_flag_clear(usart_port_defs[port].channel, USART_TDC_FLAG);
    usart_interrupt_enable(usart_port_defs[port].channel, USART_TDC_INT, TRUE);
  }
}

#define USART usart_port_defs[port]

void handle_usart_invert(serial_ports_t port, bool invert) {
  // no inversion right now
}

void serial_hard_init(serial_port_t *serial, serial_port_config_t config, bool swap) {
  const serial_ports_t port = config.port;

  serial_enable_rcc(port);
  serial_disable_isr(port);
  serial_dma_release_port(port);

  usart_reset(USART.channel);
  usart_init(USART.channel, config.baudrate, USART_DATA_8BITS, stop_bits_map[config.stop_bits]);
  usart_parity_selection_config(USART.channel, USART_PARITY_NONE);

  if (config.direction & SERIAL_DIR_RX) {
    usart_receiver_enable(USART.channel, TRUE);
  }
  if (config.direction & SERIAL_DIR_TX) {
    usart_transmitter_enable(USART.channel, TRUE);
  }

  handle_usart_invert(port, config.invert);

  if (swap) {
    usart_transmit_receive_pin_swap(USART.channel, TRUE);
  }
  if (config.half_duplex) {
    usart_single_line_halfduplex_select(USART.channel, TRUE);
  }

  usart_enable(USART.channel, TRUE);

  const serial_dma_mode_t dma = serial_dma_init(serial);

  if (config.direction & SERIAL_DIR_RX) {
    usart_interrupt_enable(USART.channel, USART_RDBF_INT, (dma & SERIAL_DMA_RX) ? FALSE : TRUE);
  }

  serial_enable_isr(serial->config.port);
}

bool serial_hard_set_baudrate(serial_port_t *serial, uint32_t baudrate) {
  if (!serial || baudrate == 0 || serial->config.port <= SERIAL_PORT_INVALID || serial->config.port >= SERIAL_PORT_MAX) {
    return false;
  }

  const serial_ports_t port = serial->config.port;
  crm_clocks_freq_type clocks_freq;
  crm_clocks_freq_get(&clocks_freq);

  const uint32_t apb_clock = (USART.channel == USART1 || USART.channel == USART6) ? clocks_freq.apb2_freq : clocks_freq.apb1_freq;
  uint32_t div = apb_clock * 10 / baudrate;
  div = (div % 10) < 5 ? (div / 10) : (div / 10) + 1;
  USART.channel->baudr_bit.div = div;
  return true;
}

bool serial_write_bytes(serial_port_t *serial, const uint8_t *data, const uint32_t size) {
  if (!serial || !data || !serial->tx_buffer || serial->config.port <= SERIAL_PORT_INVALID || serial->config.port >= SERIAL_PORT_MAX) {
    return false;
  }

  if (size == 0) {
    return true;
  }

  const bool is_soft = serial_is_soft(serial->config.port);
  const usart_port_def_t *port = is_soft ? NULL : &usart_port_defs[serial->config.port];

  uint32_t written = 0;
  while (written < size) {
    written += ring_buffer_write_multi(serial->tx_buffer, data + written, size - written);

    if (is_soft) {
      if (serial->config.half_duplex) {
        soft_serial_enable_write(serial->config.port);
      }
      serial->tx_done = false;
    } else {
      // If TDC handler completed while we were writing, switch back to TX
      if (serial->config.half_duplex && serial->tx_done) {
        usart_receiver_enable(port->channel, FALSE);
        usart_transmitter_enable(port->channel, TRUE);
      }
      ATOMIC_BLOCK(DMA_PRIORITY) {
        if (!serial_dma_start_tx(serial)) {
          usart_interrupt_enable(port->channel, USART_TDBE_INT, TRUE);
          serial->tx_done = false;
        }
      }
    }
  }

  return true;
}

static void handle_serial_isr(serial_port_t *serial) {
  const usart_port_def_t *port = &usart_port_defs[serial->config.port];

  const bool overrun_error = usart_flag_get(port->channel, USART_ROERR_FLAG) == SET;
  const bool parity_error = usart_flag_get(port->channel, USART_PERR_FLAG) == SET;
  const bool framing_error = usart_flag_get(port->channel, USART_FERR_FLAG) == SET;
  const bool noise_error = usart_flag_get(port->channel, USART_NERR_FLAG) == SET;
  const bool break_error = usart_flag_get(port->channel, USART_BFF_FLAG) == SET;

  if (overrun_error)
    usart_flag_clear(port->channel, USART_ROERR_FLAG);
  if (parity_error)
    usart_flag_clear(port->channel, USART_PERR_FLAG);
  if (framing_error)
    usart_flag_clear(port->channel, USART_FERR_FLAG);
  if (noise_error)
    usart_flag_clear(port->channel, USART_NERR_FLAG);
  if (break_error)
    usart_flag_clear(port->channel, USART_BFF_FLAG);
  serial->rx_error_count = serial->rx_error_count + overrun_error + parity_error + framing_error + noise_error + break_error;

  if (usart_interrupt_flag_get(port->channel, USART_RDBF_FLAG)) {
    const volatile uint8_t data = usart_data_receive(port->channel);
    ring_buffer_write(serial->rx_buffer, data);
  }

  if (usart_interrupt_flag_get(port->channel, USART_TDBE_FLAG)) {
    uint8_t data = 0;
    if (ring_buffer_read(serial->tx_buffer, &data)) {
      usart_data_transmit(port->channel, data);
    }
    if (ring_buffer_available(serial->tx_buffer) == 0) {
      usart_interrupt_enable(port->channel, USART_TDBE_INT, FALSE);
      usart_flag_clear(port->channel, USART_TDC_FLAG);
      usart_interrupt_enable(port->channel, USART_TDC_INT, TRUE);
    }
  }

  if (usart_interrupt_flag_get(port->channel, USART_TDC_FLAG)) {
    usart_flag_clear(port->channel, USART_TDC_FLAG);
    usart_interrupt_enable(port->channel, USART_TDC_INT, FALSE);

    if (ring_buffer_available(serial->tx_buffer)) {
      if (!serial_dma_start_tx(serial)) {
        usart_interrupt_enable(port->channel, USART_TDBE_INT, TRUE);
      }
    } else {
      if (serial->config.half_duplex) {
        usart_transmitter_enable(port->channel, FALSE);
        usart_receiver_enable(port->channel, TRUE);
      }
      serial->tx_done = true;
    }
  }
}

static void handle_usart_isr(serial_ports_t index) {
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
#define USART_IRQ_HANDLER(channel)          \
  void USART##channel##_IRQHandler() {      \
    handle_usart_isr(SERIAL_PORT##channel); \
  }                                         \
  void UART##channel##_IRQHandler() {       \
    handle_usart_isr(SERIAL_PORT##channel); \
  }

extern "C" {
USART_IRQ_HANDLER(1)
USART_IRQ_HANDLER(2)
USART_IRQ_HANDLER(3)
USART_IRQ_HANDLER(4)
USART_IRQ_HANDLER(5)
USART_IRQ_HANDLER(6)
USART_IRQ_HANDLER(7)
USART_IRQ_HANDLER(8)
}

#undef USART_IRQ_HANDLER
