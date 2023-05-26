#include "driver/serial.h"

#include "core/profile.h"
#include "core/project.h"
#include "driver/interrupt.h"
#include "driver/rcc.h"
#include "driver/serial_soft.h"
#include "driver/time.h"
#include "io/usb_configurator.h"

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
    {
        .channel_index = 6,
        .channel = USART6,
        .irq = USART6_IRQn,
        .rcc = RCC_APB2_GRP1(USART6),
    },
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

serial_port_t *serial_ports[SERIAL_PORT_MAX];

#define USART usart_port_defs[port]

static void serial_enable_rcc(serial_ports_t port) {
  const rcc_reg_t reg = usart_port_defs[port].rcc;
  rcc_enable(reg);
}

static void serial_enable_isr(serial_ports_t port) {
  const IRQn_Type irq = usart_port_defs[port].irq;
  interrupt_enable(irq, UART_PRIORITY);
}

static void serial_disable_isr(serial_ports_t port) {
  const IRQn_Type irq = usart_port_defs[port].irq;
  interrupt_disable(irq);
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
  gpio_config_t gpio_init;
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
#if defined(STM32F7) || defined(STM32H7)
  if (invert) {
    LL_USART_SetRXPinLevel(USART.channel, LL_USART_RXPIN_LEVEL_INVERTED);
    LL_USART_SetTXPinLevel(USART.channel, LL_USART_TXPIN_LEVEL_INVERTED);
  } else {
    LL_USART_SetRXPinLevel(USART.channel, LL_USART_RXPIN_LEVEL_STANDARD);
    LL_USART_SetTXPinLevel(USART.channel, LL_USART_TXPIN_LEVEL_STANDARD);
  }
#endif
}

bool serial_is_soft(serial_ports_t port) {
  if (port < SERIAL_PORT_MAX) {
    return false;
  }
  return true;
}

static void serial_hard_init(serial_port_t *serial, serial_port_config_t config) {
  const serial_ports_t port = config.port;
  const target_serial_port_t *dev = &target.serial_ports[port];

  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_ALTERNATE;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  if (config.half_duplex) {
    gpio_init.output = GPIO_OPENDRAIN;
    gpio_init.pull = GPIO_UP_PULL;
    gpio_pin_init_tag(dev->tx, gpio_init, SERIAL_TAG(port, RES_SERIAL_TX));
  } else {
    gpio_init.output = GPIO_PUSHPULL;
    gpio_init.pull = GPIO_NO_PULL;
    gpio_pin_init_tag(dev->rx, gpio_init, SERIAL_TAG(port, RES_SERIAL_RX));
    gpio_pin_init_tag(dev->tx, gpio_init, SERIAL_TAG(port, RES_SERIAL_TX));
  }

  LL_USART_InitTypeDef usart_init;
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

#if !defined(STM32F7) && !defined(STM32H7)
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

  LL_USART_EnableIT_TC(usart_port_defs[serial->config.port].channel);
  LL_USART_EnableIT_RXNE(usart_port_defs[serial->config.port].channel);

  serial_enable_isr(serial->config.port);
}

void serial_init(serial_port_t *serial, serial_port_config_t config) {
  const serial_ports_t port = config.port;
  if (port == SERIAL_PORT_INVALID || serial == NULL) {
    return;
  }

  const target_serial_port_t *dev = &target.serial_ports[port];
  if (!target_serial_port_valid(dev)) {
    return;
  }
  serial->config = config;
  serial->tx_done = true;

  serial_ports[port] = serial;

  ring_buffer_clear(serial->rx_buffer);
  ring_buffer_clear(serial->tx_buffer);

  if (serial_is_soft(config.port)) {
    soft_serial_init(config);
  } else {
    serial_hard_init(serial, config);
  }
}

uint32_t serial_bytes_available(serial_port_t *serial) {
  return ring_buffer_available(serial->rx_buffer);
}

uint32_t serial_read_bytes(serial_port_t *serial, uint8_t *data, const uint32_t size) {
  return ring_buffer_read_multi(serial->rx_buffer, data, size);
}

bool serial_write_bytes(serial_port_t *serial, const uint8_t *data, const uint32_t size) {
  if (size == 0) {
    return true;
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
    if (!serial_is_soft(serial->config.port)) {
      const usart_port_def_t *port = &usart_port_defs[serial->config.port];
      LL_USART_DisableIT_TXE(port->channel);
    }

    written += ring_buffer_write_multi(serial->tx_buffer, data + written, size - written);
    serial->tx_done = false;

    if (!serial_is_soft(serial->config.port)) {
      const usart_port_def_t *port = &usart_port_defs[serial->config.port];
      LL_USART_EnableIT_TXE(port->channel);
    }
  }

  return true;
}

void soft_serial_tx_isr(serial_ports_t port) {
  serial_port_t *serial = serial_ports[port];

  uint8_t data = 0;
  if (ring_buffer_read(serial->tx_buffer, &data)) {
    soft_serial_write_byte(port, data);
  } else {
    soft_serial_enable_read(port);
    serial->tx_done = true;
  }
}

void soft_serial_rx_isr(serial_ports_t port) {
  serial_port_t *serial = serial_ports[port];

  const uint8_t data = soft_serial_read_byte(port);
  ring_buffer_write(serial->rx_buffer, data);
}

static void handle_serial_isr(serial_port_t *serial) {
  const usart_port_def_t *port = &usart_port_defs[serial->config.port];

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
    const uint8_t data = LL_USART_ReceiveData8(port->channel);
    ring_buffer_write(serial->rx_buffer, data);
  }

  if (LL_USART_IsActiveFlag_ORE(port->channel)) {
    LL_USART_ClearFlag_ORE(port->channel);
  }
}

static void handle_usart_isr(serial_ports_t port) {
  if (serial_ports[port]) {
    handle_serial_isr(serial_ports[port]);
    return;
  }
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
USART_IRQ_HANDLER(6)
#if defined(STM32F7) || defined(STM32H7)
USART_IRQ_HANDLER(7)
USART_IRQ_HANDLER(8)
#endif

#undef USART_IRQ_HANDLER