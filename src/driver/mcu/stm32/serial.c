#include "driver/serial.h"

#include "driver/dma.h"
#include "driver/interrupt.h"
#include "driver/serial_soft.h"

const usart_port_def_t usart_port_defs[SERIAL_PORT_MAX] = {
    {},
    {
        .channel_index = 1,
        .channel = USART1,
        .irq = USART1_IRQn,
        .rcc = RCC_APB2_GRP1(USART1),
#if defined(STM32H7)
        .dma_request = {.rx = LL_DMAMUX1_REQ_USART1_RX, .tx = LL_DMAMUX1_REQ_USART1_TX},
#elif defined(STM32G4)
        .dma_request = {.rx = LL_DMAMUX_REQ_USART1_RX, .tx = LL_DMAMUX_REQ_USART1_TX},
#endif
    },
    {
        .channel_index = 2,
        .channel = USART2,
        .irq = USART2_IRQn,
        .rcc = RCC_APB1_GRP1(USART2),
#if defined(STM32H7)
        .dma_request = {.rx = LL_DMAMUX1_REQ_USART2_RX, .tx = LL_DMAMUX1_REQ_USART2_TX},
#elif defined(STM32G4)
        .dma_request = {.rx = LL_DMAMUX_REQ_USART2_RX, .tx = LL_DMAMUX_REQ_USART2_TX},
#endif
    },
#if !defined(STM32F411)
    {
        .channel_index = 3,
        .channel = USART3,
        .irq = USART3_IRQn,
        .rcc = RCC_APB1_GRP1(USART3),
#if defined(STM32H7)
        .dma_request = {.rx = LL_DMAMUX1_REQ_USART3_RX, .tx = LL_DMAMUX1_REQ_USART3_TX},
#elif defined(STM32G4)
        .dma_request = {.rx = LL_DMAMUX_REQ_USART3_RX, .tx = LL_DMAMUX_REQ_USART3_TX},
#endif
    },
    {
        .channel_index = 4,
        .channel = UART4,
        .irq = UART4_IRQn,
        .rcc = RCC_APB1_GRP1(UART4),
#if defined(STM32H7)
        .dma_request = {.rx = LL_DMAMUX1_REQ_UART4_RX, .tx = LL_DMAMUX1_REQ_UART4_TX},
#elif defined(STM32G4)
        .dma_request = {.rx = LL_DMAMUX_REQ_UART4_RX, .tx = LL_DMAMUX_REQ_UART4_TX},
#endif
    },
    {
        .channel_index = 5,
        .channel = UART5,
        .irq = UART5_IRQn,
        .rcc = RCC_APB1_GRP1(UART5),
#if defined(STM32H7)
        .dma_request = {.rx = LL_DMAMUX1_REQ_UART5_RX, .tx = LL_DMAMUX1_REQ_UART5_TX},
#elif defined(STM32G4)
        .dma_request = {.rx = LL_DMAMUX_REQ_UART5_RX, .tx = LL_DMAMUX_REQ_UART5_TX},
#endif
    },
#endif
#ifndef STM32G4
    {
        .channel_index = 6,
        .channel = USART6,
        .irq = USART6_IRQn,
        .rcc = RCC_APB2_GRP1(USART6),
#if defined(STM32H7)
        .dma_request = {.rx = LL_DMAMUX1_REQ_USART6_RX, .tx = LL_DMAMUX1_REQ_USART6_TX},
#endif
    },
#endif
#if defined(STM32F7) || defined(STM32H7)
    {
        .channel_index = 7,
        .channel = UART7,
        .irq = UART7_IRQn,
        .rcc = RCC_APB1_GRP1(UART7),
#if defined(STM32H7)
        .dma_request = {.rx = LL_DMAMUX1_REQ_UART7_RX, .tx = LL_DMAMUX1_REQ_UART7_TX},
#endif
    },
    {
        .channel_index = 8,
        .channel = UART8,
        .irq = UART8_IRQn,
        .rcc = RCC_APB1_GRP1(UART8),
#if defined(STM32H7)
        .dma_request = {.rx = LL_DMAMUX1_REQ_UART8_RX, .tx = LL_DMAMUX1_REQ_UART8_TX},
#endif
    },
#endif
};

static const uint32_t stop_bits_map[] = {
    [SERIAL_STOP_BITS_0_5] = LL_USART_STOPBITS_0_5,
    [SERIAL_STOP_BITS_1] = LL_USART_STOPBITS_1,
    [SERIAL_STOP_BITS_1_5] = LL_USART_STOPBITS_1_5,
    [SERIAL_STOP_BITS_2] = LL_USART_STOPBITS_2,
};

static uint32_t serial_direction_map(serial_direction_t direction) {
  uint32_t ll_direction = LL_USART_DIRECTION_NONE;
  if (direction & SERIAL_DIR_RX) {
    ll_direction |= LL_USART_DIRECTION_RX;
  }
  if (direction & SERIAL_DIR_TX) {
    ll_direction |= LL_USART_DIRECTION_TX;
  }
  return ll_direction;
}

extern serial_port_t *serial_ports[SERIAL_PORT_MAX];

#if defined(STM32H7) || defined(STM32G4)
typedef struct {
  dma_stream_t rx_stream;
  dma_stream_t tx_stream;
  uint32_t rx_pos;
  uint32_t tx_len;
} serial_dma_state_t;

static serial_dma_state_t serial_dma_state[SERIAL_PORT_MAX] = {
    [RANGE_INIT(0, SERIAL_PORT_MAX)] = {
        .rx_stream = DMA_STREAM_INVALID,
        .tx_stream = DMA_STREAM_INVALID,
    },
};

static serial_ports_t serial_dma_tx_stream_ports[DMA_STREAM_MAX] = {
    [RANGE_INIT(0, DMA_STREAM_MAX)] = SERIAL_PORT_INVALID,
};

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
    LL_USART_DisableDMAReq_RX(usart_port_defs[port].channel);
    LL_DMA_DisableStream(dma->port, dma->stream_index);
    dma_release(state->rx_stream, DMA_DEVICE_SERIAL_RX);
  }

  if (state->tx_stream != DMA_STREAM_INVALID) {
    const dma_stream_def_t *dma = &dma_stream_defs[state->tx_stream];
    LL_USART_DisableDMAReq_TX(usart_port_defs[port].channel);
    LL_DMA_DisableStream(dma->port, dma->stream_index);
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

  LL_DMA_DeInit(dma->port, dma->stream_index);

  LL_DMA_InitTypeDef dma_init;
  LL_DMA_StructInit(&dma_init);
  dma_init.PeriphRequest = request;
  dma_init.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(usart_port_defs[port].channel, LL_USART_DMA_REG_DATA_RECEIVE);
  dma_init.MemoryOrM2MDstAddress = (uint32_t)serial->rx_buffer->buffer;
  dma_init.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  dma_init.NbData = serial->rx_buffer->size;
  dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  dma_init.Mode = LL_DMA_MODE_CIRCULAR;
  dma_init.Priority = LL_DMA_PRIORITY_HIGH;
#ifndef STM32G4
  dma_init.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  dma_init.MemBurst = LL_DMA_MBURST_SINGLE;
  dma_init.PeriphBurst = LL_DMA_PBURST_SINGLE;
#endif
  LL_DMA_Init(dma->port, dma->stream_index, &dma_init);
  LL_DMA_DisableStream(dma->port, dma->stream_index);

  dma_prepare_rx_memory(serial->rx_buffer->buffer, serial->rx_buffer->size);
  dma_clear_flag_tc(dma);
  LL_DMA_EnableStream(dma->port, dma->stream_index);
  LL_USART_EnableDMAReq_RX(usart_port_defs[port].channel);

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

  LL_DMA_DeInit(dma->port, dma->stream_index);

  LL_DMA_InitTypeDef dma_init;
  LL_DMA_StructInit(&dma_init);
  dma_init.PeriphRequest = request;
  dma_init.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(usart_port_defs[port].channel, LL_USART_DMA_REG_DATA_TRANSMIT);
  dma_init.MemoryOrM2MDstAddress = 0;
  dma_init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  dma_init.NbData = 0;
  dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  dma_init.Mode = LL_DMA_MODE_NORMAL;
  dma_init.Priority = LL_DMA_PRIORITY_HIGH;
#ifndef STM32G4
  dma_init.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  dma_init.MemBurst = LL_DMA_MBURST_SINGLE;
  dma_init.PeriphBurst = LL_DMA_PBURST_SINGLE;
#endif
  LL_DMA_Init(dma->port, dma->stream_index, &dma_init);
  LL_DMA_DisableStream(dma->port, dma->stream_index);

  dma_clear_flag_tc(dma);
  interrupt_enable(dma->irq, DMA_PRIORITY);
  LL_DMA_EnableIT_TC(dma->port, dma->stream_index);
  LL_DMA_EnableIT_TE(dma->port, dma->stream_index);

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
  while (LL_DMA_IsEnabledStream(dma->port, dma->stream_index))
    ;

  dma_clear_flag_tc(dma);
  dma_prepare_tx_memory(&tx_buffer->buffer[tail], size);
  LL_DMA_SetMemoryAddress(dma->port, dma->stream_index, (uint32_t)&tx_buffer->buffer[tail]);
  LL_DMA_SetDataLength(dma->port, dma->stream_index, size);
  LL_USART_ClearFlag_TC(usart_port_defs[port].channel);
  LL_USART_DisableIT_TXE(usart_port_defs[port].channel);
  LL_USART_DisableIT_TC(usart_port_defs[port].channel);
  LL_DMA_EnableStream(dma->port, dma->stream_index);
  LL_USART_EnableDMAReq_TX(usart_port_defs[port].channel);

  state->tx_len = size;
  serial->tx_done = false;
  return true;
}

static serial_dma_mode_t serial_dma_init(serial_port_t *serial) {
  const serial_ports_t port = serial->config.port;
  serial_dma_mode_t active = SERIAL_DMA_NONE;

  if ((serial->config.direction & SERIAL_DIR_RX) && (serial->config.dma_mode & SERIAL_DMA_RX) && serial_dma_setup_rx(serial)) {
    active |= SERIAL_DMA_RX;
  }
  if ((serial->config.direction & SERIAL_DIR_TX) && (serial->config.dma_mode & SERIAL_DMA_TX) && serial_dma_setup_tx(port)) {
    active |= SERIAL_DMA_TX;
  }
  return active;
}

static void serial_dma_sync_rx(serial_port_t *serial) {
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
  const bool transfer_complete = dma_is_flag_active(dma, DMA_FLAG_TC);
  const bool transfer_error = dma_is_flag_active(dma, DMA_FLAG_TE);
  if (transfer_complete || transfer_error || dma_is_flag_active(dma, DMA_FLAG_HT)) {
    dma_clear_flag_tc(dma);
  }
  if (transfer_error) {
    serial->rx_error_count++;
  }

  const uint32_t pos = (rx_buffer->size - LL_DMA_GetDataLength(dma->port, dma->stream_index)) % rx_buffer->size;
  const uint32_t old_pos = state->rx_pos;
  uint32_t received = (pos >= old_pos) ? (pos - old_pos) : (rx_buffer->size - old_pos + pos);
  if (received == 0) {
    if (!transfer_complete) {
      return;
    }
    received = rx_buffer->size;
  }

  if (received > ring_buffer_free(rx_buffer)) {
    serial->rx_error_count++;
    rx_buffer->tail = (pos + 1) % rx_buffer->size;
  }

  state->rx_pos = pos;
  MEMORY_BARRIER();
  rx_buffer->head = pos;
}

void serial_hard_sync_rx(serial_port_t *serial) {
  serial_dma_sync_rx(serial);
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
  const bool transfer_error = dma_is_flag_active(dma, DMA_FLAG_TE);
  const bool transfer_complete = dma_is_flag_active(dma, DMA_FLAG_TC);
  dma_clear_flag_tc(dma);

  if (!serial_ports[port]) {
    return;
  }

  serial_port_t *serial = serial_ports[port];
  LL_USART_DisableDMAReq_TX(usart_port_defs[port].channel);
  LL_DMA_DisableStream(dma->port, dma->stream_index);

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
    LL_USART_ClearFlag_TC(usart_port_defs[port].channel);
    LL_USART_EnableIT_TC(usart_port_defs[port].channel);
  }
}
#else
void serial_hard_sync_rx(serial_port_t *serial) {}
void serial_dma_tx_isr(dma_stream_t stream) {}

static void serial_dma_release_port(serial_ports_t port) {}
static serial_dma_mode_t serial_dma_init(serial_port_t *serial) {
  (void)serial;
  return SERIAL_DMA_NONE;
}
static bool serial_dma_start_tx(serial_port_t *serial) { return false; }
#endif

#define USART usart_port_defs[port]

void handle_usart_invert(serial_ports_t port, bool invert) {
#if defined(STM32F4)
  const target_serial_port_t *dev = &target.serial_ports[port];
  if (!target_serial_port_valid(dev)) {
    return;
  }
  if (dev->inverter == PIN_NONE) {
    return;
  }

  // Inverter control line, set high
  gpio_config_t gpio_init = gpio_config_default();
  gpio_init.mode = GPIO_OUTPUT;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;
  gpio_pin_init(dev->inverter, gpio_init);
  if (invert) {
    gpio_pin_set(dev->inverter);
  } else {
    gpio_pin_reset(dev->inverter);
  }
#endif
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
  if (invert) {
    LL_USART_SetRXPinLevel(USART.channel, LL_USART_RXPIN_LEVEL_INVERTED);
    LL_USART_SetTXPinLevel(USART.channel, LL_USART_TXPIN_LEVEL_INVERTED);
  } else {
    LL_USART_SetRXPinLevel(USART.channel, LL_USART_RXPIN_LEVEL_STANDARD);
    LL_USART_SetTXPinLevel(USART.channel, LL_USART_TXPIN_LEVEL_STANDARD);
  }
#endif
}

void serial_hard_init(serial_port_t *serial, serial_port_config_t config, bool swap) {
  const serial_ports_t port = config.port;

  serial_enable_rcc(port);
  serial_disable_isr(port);
  serial_dma_release_port(port);

  LL_USART_Disable(USART.channel);
  LL_USART_DeInit(USART.channel);

  LL_USART_InitTypeDef usart_init;
  LL_USART_StructInit(&usart_init);
  usart_init.BaudRate = config.baudrate;
  usart_init.DataWidth = LL_USART_DATAWIDTH_8B;
  usart_init.StopBits = stop_bits_map[config.stop_bits];
  usart_init.Parity = LL_USART_PARITY_NONE;
  usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  usart_init.TransferDirection = serial_direction_map(config.direction);
  usart_init.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART.channel, &usart_init);

  handle_usart_invert(port, config.invert);

#ifndef STM32F4
  if (swap) {
    LL_USART_SetTXRXSwap(USART.channel, LL_USART_TXRX_SWAPPED);
  }
#endif

#if !defined(STM32F7) && !defined(STM32H7) && !defined(STM32G4)
  LL_USART_ClearFlag_RXNE(USART.channel);
#endif
  LL_USART_ClearFlag_TC(USART.channel);

  LL_USART_DisableIT_RXNE(USART.channel);
  LL_USART_DisableIT_TC(USART.channel);

#if defined(STM32H7) || defined(STM32G4)
  LL_USART_SetTXFIFOThreshold(USART.channel, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART.channel, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART.channel);
#endif

  if (config.half_duplex) {
    LL_USART_SetTransferDirection(USART.channel, LL_USART_DIRECTION_RX);
    LL_USART_ConfigHalfDuplexMode(USART.channel);
  }

  LL_USART_Enable(USART.channel);

#if defined(STM32H7) || defined(STM32G4)
  if (LL_USART_GetTransferDirection(USART.channel) & LL_USART_DIRECTION_RX)
    while (!LL_USART_IsActiveFlag_REACK(USART.channel))
      ;
#endif
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
  if (LL_USART_GetTransferDirection(USART.channel) & LL_USART_DIRECTION_TX)
    while (!LL_USART_IsActiveFlag_TEACK(USART.channel))
      ;
#endif

  const serial_dma_mode_t dma = serial_dma_init(serial);

  if (LL_USART_GetTransferDirection(USART.channel) & LL_USART_DIRECTION_RX) {
    if (dma & SERIAL_DMA_RX) {
      LL_USART_DisableIT_RXNE(usart_port_defs[serial->config.port].channel);
    } else {
      LL_USART_EnableIT_RXNE(usart_port_defs[serial->config.port].channel);
    }
  }

  serial_enable_isr(serial->config.port);
}

static uint32_t serial_hard_get_clock(serial_ports_t port) {
  if (USART.channel == USART1
#if defined(USART6)
      || USART.channel == USART6
#endif
  ) {
    return HAL_RCC_GetPCLK2Freq();
  }
  return HAL_RCC_GetPCLK1Freq();
}

bool serial_hard_set_baudrate(serial_port_t *serial, uint32_t baudrate) {
  if (!serial || baudrate == 0 || serial->config.port <= SERIAL_PORT_INVALID || serial->config.port >= SERIAL_PORT_MAX) {
    return false;
  }

  const serial_ports_t port = serial->config.port;
  const uint32_t periphclk = serial_hard_get_clock(port);
  if (periphclk == 0) {
    return false;
  }

  const uint32_t was_enabled = LL_USART_IsEnabled(USART.channel);
  if (was_enabled) {
    LL_USART_Disable(USART.channel);
  }

#if defined(STM32H7) || defined(STM32G4)
  LL_USART_SetBaudRate(USART.channel, periphclk, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, baudrate);
#else
  LL_USART_SetBaudRate(USART.channel, periphclk, LL_USART_OVERSAMPLING_16, baudrate);
#endif

  if (was_enabled) {
    LL_USART_Enable(USART.channel);

#if defined(STM32H7) || defined(STM32G4)
    if (LL_USART_GetTransferDirection(USART.channel) & LL_USART_DIRECTION_RX)
      while (!LL_USART_IsActiveFlag_REACK(USART.channel))
        ;
#endif
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    if (LL_USART_GetTransferDirection(USART.channel) & LL_USART_DIRECTION_TX)
      while (!LL_USART_IsActiveFlag_TEACK(USART.channel))
        ;
#endif
  }

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
      if (serial->config.half_duplex && serial->tx_done) {
        soft_serial_enable_write(serial->config.port);
      }
      serial->tx_done = false;
    } else {
      // If TC handler completed while we were writing, switch back to TX
      if (serial->config.half_duplex && serial->tx_done) {
        LL_USART_SetTransferDirection(port->channel, LL_USART_DIRECTION_TX);
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
        while (!LL_USART_IsActiveFlag_TEACK(port->channel))
          ;
#endif
      }
      ATOMIC_BLOCK(DMA_PRIORITY) {
        if (!serial_dma_start_tx(serial)) {
          LL_USART_EnableIT_TXE(port->channel);
          serial->tx_done = false;
        }
      }
    }
  }

  return true;
}

static void handle_serial_isr(serial_port_t *serial) {
  const usart_port_def_t *port = &usart_port_defs[serial->config.port];

  const bool overrun_error = LL_USART_IsActiveFlag_ORE(port->channel);
  const bool parity_error = LL_USART_IsActiveFlag_PE(port->channel);
  const bool framing_error = LL_USART_IsActiveFlag_FE(port->channel);
  const bool noise_error = LL_USART_IsActiveFlag_NE(port->channel);

#if defined(STM32F4)
  if (overrun_error || parity_error || framing_error || noise_error)
    LL_USART_ClearFlag_ORE(port->channel);
#else
  if (overrun_error)
    LL_USART_ClearFlag_ORE(port->channel);
  if (parity_error)
    LL_USART_ClearFlag_PE(port->channel);
  if (framing_error)
    LL_USART_ClearFlag_FE(port->channel);
  if (noise_error)
    LL_USART_ClearFlag_NE(port->channel);
#endif
  serial->rx_error_count += overrun_error + parity_error + framing_error + noise_error;

  if (LL_USART_IsEnabledIT_RXNE(port->channel) && LL_USART_IsActiveFlag_RXNE(port->channel)) {
    const volatile uint8_t data = LL_USART_ReceiveData8(port->channel);
    ring_buffer_write(serial->rx_buffer, data);
#if defined(STM32F4)
    LL_USART_ClearFlag_RXNE(port->channel);
#endif
  }

  if (LL_USART_IsEnabledIT_TXE(port->channel) && LL_USART_IsActiveFlag_TXE(port->channel)) {
    uint8_t data = 0;
    if (ring_buffer_read(serial->tx_buffer, &data)) {
      LL_USART_TransmitData8(port->channel, data);
    }
    if (ring_buffer_available(serial->tx_buffer) == 0) {
      LL_USART_DisableIT_TXE(port->channel);
      LL_USART_ClearFlag_TC(port->channel);
      LL_USART_EnableIT_TC(port->channel);
    }
  }

  if (LL_USART_IsEnabledIT_TC(port->channel) && LL_USART_IsActiveFlag_TC(port->channel)) {
    LL_USART_ClearFlag_TC(port->channel);
    LL_USART_DisableIT_TC(port->channel);

    if (ring_buffer_available(serial->tx_buffer)) {
      if (!serial_dma_start_tx(serial)) {
        LL_USART_EnableIT_TXE(port->channel);
      }
    } else {
      if (serial->config.half_duplex)
        LL_USART_SetTransferDirection(port->channel, LL_USART_DIRECTION_RX);
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
  LL_USART_DisableIT_TXE(port->channel);
  LL_USART_DisableIT_RXNE(port->channel);
  LL_USART_ClearFlag_ORE(port->channel);
  LL_USART_Disable(port->channel);
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
#if !defined(STM32F411)
USART_IRQ_HANDLER(3)
USART_IRQ_HANDLER(4)
USART_IRQ_HANDLER(5)
#endif
#ifndef STM32G4
USART_IRQ_HANDLER(6)
#endif
#if defined(STM32F7) || defined(STM32H7)
USART_IRQ_HANDLER(7)
USART_IRQ_HANDLER(8)
#endif

#undef USART_IRQ_HANDLER
