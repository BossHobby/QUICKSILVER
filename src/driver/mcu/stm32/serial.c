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
        .rcc = RCC_APB2_GRP1(USART1),
        .dma_rx = DMA_DEVICE_SERIAL1_RX,
        .dma_tx = DMA_DEVICE_SERIAL1_TX,
#if defined(STM32F4)
        .dma_channel_rx = 4, // DMA2_Stream2_Channel4 or DMA2_Stream5_Channel4
        .dma_channel_tx = 4, // DMA2_Stream7_Channel4
#elif defined(STM32G4)
        .dma_request_rx = LL_DMAMUX_REQ_USART1_RX,
        .dma_request_tx = LL_DMAMUX_REQ_USART1_TX,
#elif defined(STM32H7)
        .dma_request_rx = LL_DMAMUX1_REQ_USART1_RX,
        .dma_request_tx = LL_DMAMUX1_REQ_USART1_TX,
#endif
    },
    {
        .channel_index = 2,
        .channel = USART2,
        .irq = USART2_IRQn,
        .rcc = RCC_APB1_GRP1(USART2),
        .dma_rx = DMA_DEVICE_SERIAL2_RX,
        .dma_tx = DMA_DEVICE_SERIAL2_TX,
#if defined(STM32F4)
        .dma_channel_rx = 4, // DMA1_Stream5_Channel4
        .dma_channel_tx = 4, // DMA1_Stream6_Channel4
#elif defined(STM32G4)
        .dma_request_rx = LL_DMAMUX_REQ_USART2_RX,
        .dma_request_tx = LL_DMAMUX_REQ_USART2_TX,
#elif defined(STM32H7)
        .dma_request_rx = LL_DMAMUX1_REQ_USART2_RX,
        .dma_request_tx = LL_DMAMUX1_REQ_USART2_TX,
#endif
    },
#if !defined(STM32F411)
    {
        .channel_index = 3,
        .channel = USART3,
        .irq = USART3_IRQn,
        .rcc = RCC_APB1_GRP1(USART3),
        .dma_rx = DMA_DEVICE_SERIAL3_RX,
        .dma_tx = DMA_DEVICE_SERIAL3_TX,
#if defined(STM32F4)
        .dma_channel_rx = 4, // DMA1_Stream1_Channel4
        .dma_channel_tx = 4, // DMA1_Stream3_Channel4 or DMA1_Stream4_Channel7
#elif defined(STM32G4)
        .dma_request_rx = LL_DMAMUX_REQ_USART3_RX,
        .dma_request_tx = LL_DMAMUX_REQ_USART3_TX,
#elif defined(STM32H7)
        .dma_request_rx = LL_DMAMUX1_REQ_USART3_RX,
        .dma_request_tx = LL_DMAMUX1_REQ_USART3_TX,
#endif
    },
    {
        .channel_index = 4,
        .channel = UART4,
        .irq = UART4_IRQn,
        .rcc = RCC_APB1_GRP1(UART4),
        .dma_rx = DMA_DEVICE_SERIAL4_RX,
        .dma_tx = DMA_DEVICE_SERIAL4_TX,
#if defined(STM32F4)
        .dma_channel_rx = 4, // DMA1_Stream2_Channel4
        .dma_channel_tx = 4, // DMA1_Stream4_Channel4
#elif defined(STM32G4)
        .dma_request_rx = LL_DMAMUX_REQ_UART4_RX,
        .dma_request_tx = LL_DMAMUX_REQ_UART4_TX,
#elif defined(STM32H7)
        .dma_request_rx = LL_DMAMUX1_REQ_UART4_RX,
        .dma_request_tx = LL_DMAMUX1_REQ_UART4_TX,
#endif
    },
    {
        .channel_index = 5,
        .channel = UART5,
        .irq = UART5_IRQn,
        .rcc = RCC_APB1_GRP1(UART5),
        .dma_rx = DMA_DEVICE_SERIAL5_RX,
        .dma_tx = DMA_DEVICE_SERIAL5_TX,
#if defined(STM32F4)
        .dma_channel_rx = 4, // DMA1_Stream0_Channel4
        .dma_channel_tx = 4, // DMA1_Stream7_Channel4
#elif defined(STM32G4)
        .dma_request_rx = LL_DMAMUX_REQ_UART5_RX,
        .dma_request_tx = LL_DMAMUX_REQ_UART5_TX,
#elif defined(STM32H7)
        .dma_request_rx = LL_DMAMUX1_REQ_UART5_RX,
        .dma_request_tx = LL_DMAMUX1_REQ_UART5_TX,
#endif
    },
#endif
#ifndef STM32G4
    {
        .channel_index = 6,
        .channel = USART6,
        .irq = USART6_IRQn,
        .rcc = RCC_APB2_GRP1(USART6),
        .dma_rx = DMA_DEVICE_SERIAL6_RX,
        .dma_tx = DMA_DEVICE_SERIAL6_TX,
#if defined(STM32F4)
        .dma_channel_rx = 5, // DMA2_Stream1_Channel5 or DMA2_Stream2_Channel5
        .dma_channel_tx = 5, // DMA2_Stream6_Channel5 or DMA2_Stream7_Channel5
#elif defined(STM32H7)
        .dma_request_rx = LL_DMAMUX1_REQ_USART6_RX,
        .dma_request_tx = LL_DMAMUX1_REQ_USART6_TX,
#endif
    },
#endif
#if defined(STM32F7) || defined(STM32H7)
    {
        .channel_index = 7,
        .channel = UART7,
        .irq = UART7_IRQn,
        .rcc = RCC_APB1_GRP1(UART7),
        .dma_rx = DMA_DEVICE_SERIAL7_RX,
        .dma_tx = DMA_DEVICE_SERIAL7_TX,
#if defined(STM32H7)
        .dma_request_rx = LL_DMAMUX1_REQ_UART7_RX,
        .dma_request_tx = LL_DMAMUX1_REQ_UART7_TX,
#endif
    },
    {
        .channel_index = 8,
        .channel = UART8,
        .irq = UART8_IRQn,
        .rcc = RCC_APB1_GRP1(UART8),
        .dma_rx = DMA_DEVICE_SERIAL8_RX,
        .dma_tx = DMA_DEVICE_SERIAL8_TX,
#if defined(STM32H7)
        .dma_request_rx = LL_DMAMUX1_REQ_UART8_RX,
        .dma_request_tx = LL_DMAMUX1_REQ_UART8_TX,
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

static const uint32_t direction_map[] = {
    [SERIAL_DIR_NONE] = LL_USART_DIRECTION_NONE,
    [SERIAL_DIR_RX] = LL_USART_DIRECTION_RX,
    [SERIAL_DIR_TX] = LL_USART_DIRECTION_TX,
    [SERIAL_DIR_TX_RX] = LL_USART_DIRECTION_TX_RX,
};

extern serial_port_t *serial_ports[SERIAL_PORT_MAX];

#define USART usart_port_defs[port]

static void serial_init_dma_rx(serial_port_t *serial, const dma_stream_def_t *dma_rx) {
  const serial_ports_t port = serial->config.port;

  dma_enable_rcc(dma_rx);

  LL_DMA_DeInit(dma_rx->port, dma_rx->stream_index);

  LL_DMA_InitTypeDef dma_init;
  LL_DMA_StructInit(&dma_init);

#if defined(STM32G4) || defined(STM32H7)
  dma_init.PeriphRequest = target.dma[USART.dma_rx].request;
  dma_init.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(USART.channel, LL_USART_DMA_REG_DATA_RECEIVE);
#elif defined(STM32F7)
  dma_init.Channel = dma_map_channel(target.dma[USART.dma_rx].channel);
  dma_init.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(USART.channel, LL_USART_DMA_REG_DATA_RECEIVE);
#else
  dma_init.Channel = dma_map_channel(target.dma[USART.dma_rx].channel);
  dma_init.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(USART.channel);
#endif

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
  LL_DMA_Init(dma_rx->port, dma_rx->stream_index, &dma_init);

  LL_USART_EnableDMAReq_RX(USART.channel);
  LL_USART_EnableIT_IDLE(USART.channel);
  LL_DMA_EnableStream(dma_rx->port, dma_rx->stream_index);

  serial->rx_buffer->tail = 0;
  serial->rx_buffer->head = 0;
}

static void serial_init_dma_tx(serial_port_t *serial, const dma_stream_def_t *dma_tx) {
  const serial_ports_t port = serial->config.port;
  dma_enable_rcc(dma_tx);

  interrupt_disable(dma_tx->irq);

  LL_DMA_DeInit(dma_tx->port, dma_tx->stream_index);

  LL_DMA_InitTypeDef dma_init;
  LL_DMA_StructInit(&dma_init);

#if defined(STM32G4) || defined(STM32H7)
  dma_init.PeriphRequest = target.dma[USART.dma_tx].request;
  dma_init.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(USART.channel, LL_USART_DMA_REG_DATA_TRANSMIT);
#elif defined(STM32F7)
  dma_init.Channel = dma_map_channel(target.dma[USART.dma_tx].channel);
  dma_init.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(USART.channel, LL_USART_DMA_REG_DATA_TRANSMIT);
#else
  dma_init.Channel = dma_map_channel(target.dma[USART.dma_tx].channel);
  dma_init.PeriphOrM2MSrcAddress = LL_USART_DMA_GetRegAddr(USART.channel);
#endif

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
  LL_DMA_Init(dma_tx->port, dma_tx->stream_index, &dma_init);
  LL_DMA_DisableStream(dma_tx->port, dma_tx->stream_index);
  LL_USART_EnableDMAReq_TX(USART.channel);

  dma_clear_flag_tc(dma_tx);
  interrupt_enable(dma_tx->irq, DMA_PRIORITY);
  LL_DMA_EnableIT_TC(dma_tx->port, dma_tx->stream_index);
  LL_DMA_EnableIT_TE(dma_tx->port, dma_tx->stream_index);
}

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

  if (!USART.channel) {
    return;
  }

  LL_USART_InitTypeDef usart_init;
  LL_USART_StructInit(&usart_init);
  usart_init.BaudRate = config.baudrate;
  usart_init.DataWidth = LL_USART_DATAWIDTH_8B;
  usart_init.StopBits = stop_bits_map[config.stop_bits];
  usart_init.Parity = LL_USART_PARITY_NONE;
  usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  usart_init.TransferDirection = direction_map[config.direction];
  usart_init.OverSampling = LL_USART_OVERSAMPLING_16;

  serial_enable_rcc(port);
  serial_disable_isr(port);

  LL_USART_Disable(USART.channel);
  LL_USART_DeInit(USART.channel);

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

#ifdef STM32H7
  LL_USART_SetTXFIFOThreshold(USART.channel, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART.channel, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART.channel);
#endif

  if (config.half_duplex) {
    LL_USART_ConfigHalfDuplexMode(USART.channel);
  }

  LL_USART_Enable(USART.channel);
  LL_USART_EnableIT_ERROR(USART.channel);

#ifdef STM32H7
  if (usart_init.TransferDirection & LL_USART_DIRECTION_RX) {
    while (!(LL_USART_IsActiveFlag_REACK(USART.channel)))
      ;
  }
  if (usart_init.TransferDirection & LL_USART_DIRECTION_TX) {
    while (!(LL_USART_IsActiveFlag_TEACK(USART.channel)))
      ;
  }
#endif

  if ((config.direction & SERIAL_DIR_RX)) {
    const dma_stream_def_t *dma_rx = serial_get_dma_rx(serial->config.port);
    if (dma_rx)
      serial_init_dma_rx(serial, dma_rx);
    else
      LL_USART_EnableIT_RXNE(USART.channel);
  }

  if ((config.direction & SERIAL_DIR_TX)) {
    const dma_stream_def_t *dma_tx = serial_get_dma_tx(serial->config.port);
    if (dma_tx)
      serial_init_dma_tx(serial, dma_tx);
    else
      LL_USART_EnableIT_TC(USART.channel);
  }

  serial_enable_isr(serial->config.port);
}

static bool serial_start_dma_tx(serial_port_t *serial) {
  const dma_stream_def_t *dma_tx = serial_get_dma_tx(serial->config.port);
  if (!dma_tx || serial_is_soft(serial->config.port)) {
    return false;
  }

#ifdef STM32F4
  // STM32F4 errata 2.2.19: check for DMA2 conflicts
  const dma_device_t tx_dev = usart_port_defs[serial->config.port].dma_tx;
  if (!dma_can_use_dma2(tx_dev)) {
    return false; // Defer serial TX if DMA2 conflicts exist
  }
#endif

  while (LL_DMA_IsEnabledStream(dma_tx->port, dma_tx->stream_index))
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

  dma_clear_flag_tc(dma_tx);
  dma_prepare_tx_memory((void *)(serial->tx_buffer->buffer + tail), serial->tx_dma_transfer_size);
  LL_DMA_SetMemoryAddress(dma_tx->port, dma_tx->stream_index, (uint32_t)(serial->tx_buffer->buffer + tail));
  LL_DMA_SetDataLength(dma_tx->port, dma_tx->stream_index, serial->tx_dma_transfer_size);
  LL_DMA_EnableStream(dma_tx->port, dma_tx->stream_index);
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
      LL_USART_DisableDirectionRx(port->channel);
      LL_USART_EnableDirectionTx(port->channel);
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
      LL_USART_EnableIT_TXE(usart_def->channel);
    }
  }

  return true;
}

static void handle_serial_isr(serial_port_t *serial) {
  const usart_port_def_t *port = &usart_port_defs[serial->config.port];
  const dma_stream_def_t *dma_rx = serial_get_dma_rx(serial->config.port);

  if (LL_USART_IsEnabledIT_TC(port->channel) && LL_USART_IsActiveFlag_TC(port->channel)) {
    LL_USART_ClearFlag_TC(port->channel);
    if (serial->tx_done && serial->config.half_duplex) {
      LL_USART_DisableDirectionTx(port->channel);
      LL_USART_EnableDirectionRx(port->channel);
    }
  }

  if (LL_USART_IsEnabledIT_TXE(port->channel) && LL_USART_IsActiveFlag_TXE(port->channel)) {
    uint8_t data = 0;
    if (ring_buffer_read(serial->tx_buffer, &data)) {
      LL_USART_TransmitData8(port->channel, data);
    } else {
      LL_USART_DisableIT_TXE(port->channel);
      serial->tx_done = true;
    }
  }

  if (LL_USART_IsEnabledIT_RXNE(port->channel) && LL_USART_IsActiveFlag_RXNE(port->channel)) {
    const volatile uint8_t data = LL_USART_ReceiveData8(port->channel);
    ring_buffer_write(serial->rx_buffer, data);
#if defined(STM32F4)
    LL_USART_ClearFlag_RXNE(port->channel);
#endif
  }

  if (LL_USART_IsActiveFlag_ORE(port->channel)) {
    LL_USART_ClearFlag_ORE(port->channel);
    ring_buffer_clear(serial->rx_buffer);
  }
  if (LL_USART_IsActiveFlag_PE(port->channel)) {
    LL_USART_ClearFlag_PE(port->channel);
    LL_USART_ReceiveData8(port->channel);
  }
  if (LL_USART_IsActiveFlag_FE(port->channel)) {
    LL_USART_ClearFlag_FE(port->channel);
    LL_USART_ReceiveData8(port->channel);
  }
  if (LL_USART_IsActiveFlag_NE(port->channel)) {
    LL_USART_ClearFlag_NE(port->channel);
    LL_USART_ReceiveData8(port->channel);
  }

  if (LL_USART_IsActiveFlag_IDLE(port->channel)) {
    LL_USART_ClearFlag_IDLE(port->channel);
  }

  if (dma_rx) {
    const uint32_t dma_head = serial->rx_buffer->size - LL_DMA_GetDataLength(dma_rx->port, dma_rx->stream_index);
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
  LL_USART_DisableIT_TXE(port->channel);
  LL_USART_DisableIT_RXNE(port->channel);
  LL_USART_DisableIT_TC(port->channel);
  LL_USART_ClearFlag_ORE(port->channel);
  LL_USART_ClearFlag_PE(port->channel);
  LL_USART_ClearFlag_FE(port->channel);
  LL_USART_ClearFlag_NE(port->channel);
  LL_USART_ClearFlag_TC(port->channel);
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

static void handle_dma_tx_isr(serial_ports_t port) {
  const dma_stream_def_t *dma_tx = serial_get_dma_tx(port);
  if (!dma_tx || !serial_ports[port]) {
    return;
  }

  serial_port_t *serial = serial_ports[port];

  // Check for any error conditions first
  if (dma_is_flag_active(dma_tx, DMA_FLAG_TE)) {
    dma_clear_flag_tc(dma_tx);
    LL_DMA_DisableStream(dma_tx->port, dma_tx->stream_index);
    serial->tx_done = true;
    return;
  }

  if (dma_is_flag_active(dma_tx, DMA_FLAG_DME)) {
    dma_clear_flag_dme(dma_tx);
  }

  if (dma_is_flag_active(dma_tx, DMA_FLAG_TC)) {
    LL_DMA_DisableStream(dma_tx->port, dma_tx->stream_index);

    // Wait for stream to be fully disabled before proceeding
    while (LL_DMA_IsEnabledStream(dma_tx->port, dma_tx->stream_index))
      ;

    const uint32_t remaining = LL_DMA_GetDataLength(dma_tx->port, dma_tx->stream_index);
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
#if !defined(STM32F411)
  case DMA_DEVICE_SERIAL3_TX:
    handle_dma_tx_isr(SERIAL_PORT3);
    break;
  case DMA_DEVICE_SERIAL4_TX:
    handle_dma_tx_isr(SERIAL_PORT4);
    break;
  case DMA_DEVICE_SERIAL5_TX:
    handle_dma_tx_isr(SERIAL_PORT5);
    break;
#endif
#ifndef STM32G4
  case DMA_DEVICE_SERIAL6_TX:
    handle_dma_tx_isr(SERIAL_PORT6);
    break;
#endif
#if defined(STM32F7) || defined(STM32H7)
  case DMA_DEVICE_SERIAL7_TX:
    handle_dma_tx_isr(SERIAL_PORT7);
    break;
  case DMA_DEVICE_SERIAL8_TX:
    handle_dma_tx_isr(SERIAL_PORT8);
    break;
#endif
  default:
    break;
  }
}