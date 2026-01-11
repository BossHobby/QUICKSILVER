#include "driver/serial.h"

#include "driver/interrupt.h"
#include "driver/serial_soft.h"

const usart_port_def_t usart_port_defs[SERIAL_PORT_MAX] = {
    {},
    {
        .channel_index = 1,
        .channel = USART1,
        .irq = USART1_IRQn,
        .rcc = RCC_APB2_GRP1(USART1),
    },
    {
        .channel_index = 2,
        .channel = USART2,
        .irq = USART2_IRQn,
        .rcc = RCC_APB1_GRP1(USART2),
    },
#if !defined(STM32F411)
    {
        .channel_index = 3,
        .channel = USART3,
        .irq = USART3_IRQn,
        .rcc = RCC_APB1_GRP1(USART3),
    },
    {
        .channel_index = 4,
        .channel = UART4,
        .irq = UART4_IRQn,
        .rcc = RCC_APB1_GRP1(UART4),
    },
    {
        .channel_index = 5,
        .channel = UART5,
        .irq = UART5_IRQn,
        .rcc = RCC_APB1_GRP1(UART5),
    },
#endif
#ifndef STM32G4
    {
        .channel_index = 6,
        .channel = USART6,
        .irq = USART6_IRQn,
        .rcc = RCC_APB2_GRP1(USART6),
    },
#endif
#if defined(STM32F7) || defined(STM32H7)
    {
        .channel_index = 7,
        .channel = UART7,
        .irq = UART7_IRQn,
        .rcc = RCC_APB1_GRP1(UART7),
    },
    {
        .channel_index = 8,
        .channel = UART8,
        .irq = UART8_IRQn,
        .rcc = RCC_APB1_GRP1(UART8),
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

  LL_USART_Disable(USART.channel);
  LL_USART_DeInit(USART.channel);

  LL_USART_InitTypeDef usart_init;
  LL_USART_StructInit(&usart_init);
  usart_init.BaudRate = config.baudrate;
  usart_init.DataWidth = LL_USART_DATAWIDTH_8B;
  usart_init.StopBits = stop_bits_map[config.stop_bits];
  usart_init.Parity = LL_USART_PARITY_NONE;
  usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  usart_init.TransferDirection = direction_map[config.direction];
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

  LL_USART_EnableIT_RXNE(usart_port_defs[serial->config.port].channel);

  serial_enable_isr(serial->config.port);
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
      ATOMIC_BLOCK(UART_PRIORITY) {
        LL_USART_EnableIT_TXE(port->channel);
        serial->tx_done = false;
      }
    }
  }

  return true;
}

static void handle_serial_isr(serial_port_t *serial) {
  const usart_port_def_t *port = &usart_port_defs[serial->config.port];

  if (LL_USART_IsActiveFlag_ORE(port->channel)) {
    LL_USART_ClearFlag_ORE(port->channel);
  }

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
      LL_USART_EnableIT_TXE(port->channel);
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