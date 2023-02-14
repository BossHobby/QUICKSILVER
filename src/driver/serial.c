#include "driver/serial.h"

#include "core/profile.h"
#include "driver/interrupt.h"
#include "driver/time.h"
#include "io/usb_configurator.h"
#include "project.h"

usart_ports_t serial_rx_port = USART_PORT_INVALID;
usart_ports_t serial_smart_audio_port = USART_PORT_INVALID;
usart_ports_t serial_hdzero_port = USART_PORT_INVALID;

serial_port_t *serial_ports[USART_PORTS_MAX];

#define USART usart_port_defs[port]

// FUNCTION TO SET APB CLOCK TO USART BASED ON GIVEN UART
void serial_enable_rcc(usart_ports_t port) {
  switch (usart_port_defs[port].channel_index) {
#ifdef USART1
  case 1:
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    break;
#endif
#ifdef USART2
  case 2:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    break;
#endif
#ifdef USART3
  case 3:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    break;
#endif
#ifdef UART4
  case 4:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
    break;
#endif
#ifdef UART5
  case 5:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
    break;
#endif
#ifdef USART6
  case 6:
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);
    break;
#endif
#ifdef UART7
  case 7:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART7);
    break;
#endif
#ifdef UART8
  case 8:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART8);
    break;
#endif
  }
}

void serial_enable_isr(usart_ports_t port) {
  switch (usart_port_defs[port].channel_index) {
#ifdef USART1
  case 1:
    interrupt_enable(USART1_IRQn, UART_PRIORITY);
    break;
#endif
#ifdef USART2
  case 2:
    interrupt_enable(USART2_IRQn, UART_PRIORITY);
    break;
#endif
#ifdef USART3
  case 3:
    interrupt_enable(USART3_IRQn, UART_PRIORITY);
    break;
#endif
#ifdef UART4
  case 4:
    interrupt_enable(UART4_IRQn, UART_PRIORITY);
    break;
#endif
#ifdef UART5
  case 5:
    interrupt_enable(UART5_IRQn, UART_PRIORITY);
    break;
#endif
#ifdef USART6
  case 6:
    interrupt_enable(USART6_IRQn, UART_PRIORITY);
    break;
#endif
#ifdef UART7
  case 7:
    interrupt_enable(UART7_IRQn, UART_PRIORITY);
    break;
#endif
#ifdef UART8
  case 8:
    interrupt_enable(UART8_IRQn, UART_PRIORITY);
    break;
#endif
  }
}

void serial_disable_isr(usart_ports_t port) {
  switch (usart_port_defs[port].channel_index) {
#ifdef USART1
  case 1:
    interrupt_disable(USART1_IRQn);
    break;
#endif
#ifdef USART2
  case 2:
    interrupt_disable(USART2_IRQn);
    break;
#endif
#ifdef USART3
  case 3:
    interrupt_disable(USART3_IRQn);
    break;
#endif
#ifdef UART4
  case 4:
    interrupt_disable(UART4_IRQn);
    break;
#endif
#ifdef UART5
  case 5:
    interrupt_disable(UART5_IRQn);
    break;
#endif
#ifdef USART6
  case 6:
    interrupt_disable(USART6_IRQn);
    break;
#endif
#ifdef UART7
  case 7:
    interrupt_disable(UART7_IRQn);
    break;
#endif
#ifdef UART8
  case 8:
    interrupt_disable(UART8_IRQn);
    break;
#endif
  }
}

void handle_usart_invert(usart_ports_t port, bool invert) {
#if defined(STM32F4) && (defined(USART1_INVERTER_PIN) || defined(USART2_INVERTER_PIN) || defined(USART3_INVERTER_PIN) || defined(USART4_INVERTER_PIN) || defined(USART5_INVERTER_PIN) || defined(USART6_INVERTER_PIN))
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;

  // Inverter control line, set high
  switch (usart_port_defs[profile.serial.rx].channel_index) {
  case 1:
#if defined(USART1_INVERTER_PIN)
    gpio_pin_init(&gpio_init, USART1_INVERTER_PIN);
    if (invert) {
      gpio_pin_set(USART1_INVERTER_PIN);
    } else {
      gpio_pin_reset(USART1_INVERTER_PIN);
    }
#endif
    break;
  case 2:
#if defined(USART2_INVERTER_PIN)
    gpio_pin_init(&gpio_init, USART2_INVERTER_PIN);
    if (invert) {
      gpio_pin_set(USART2_INVERTER_PIN);
    } else {
      gpio_pin_reset(USART2_INVERTER_PIN);
    }
#endif
    break;
  case 3:
#if defined(USART3_INVERTER_PIN)
    gpio_pin_init(&gpio_init, USART3_INVERTER_PIN);
    if (invert) {
      gpio_pin_set(USART3_INVERTER_PIN);
    } else {
      gpio_pin_reset(USART3_INVERTER_PIN);
    }
#endif
    break;
  case 4:
#if defined(USART4_INVERTER_PIN)
    gpio_pin_init(&gpio_init, USART4_INVERTER_PIN);
    if (invert) {
      gpio_pin_set(USART4_INVERTER_PIN);
    } else {
      gpio_pin_reset(USART4_INVERTER_PIN);
    }
#endif
    break;
  case 5:
#if defined(USART5_INVERTER_PIN)
    gpio_pin_init(&gpio_init, USART5_INVERTER_PIN);
    if (invert) {
      gpio_pin_set(USART5_INVERTER_PIN);
    } else {
      gpio_pin_reset(USART5_INVERTER_PIN);
    }
#endif
    break;
  case 6:
#if defined(USART6_INVERTER_PIN)
    gpio_pin_init(&gpio_init, USART6_INVERTER_PIN);
    if (invert) {
      gpio_pin_set(USART6_INVERTER_PIN);
    } else {
      gpio_pin_reset(USART6_INVERTER_PIN);
    }
#endif
    break;
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

void serial_port_init(usart_ports_t port, LL_USART_InitTypeDef *usart_init, bool half_duplex, bool invert) {
  if (port == USART_PORT_INVALID) {
    return;
  }

  serial_ports[port] = NULL;

  LL_USART_Disable(USART.channel);
  LL_USART_DeInit(USART.channel);

  LL_USART_Init(USART.channel, usart_init);

  handle_usart_invert(port, invert);

#if !defined(STM32F7) && !defined(STM32H7)
  LL_USART_ClearFlag_RXNE(USART.channel);
#endif
  LL_USART_ClearFlag_TC(USART.channel);

  LL_USART_DisableIT_TXE(USART.channel);
  LL_USART_DisableIT_RXNE(USART.channel);
  LL_USART_DisableIT_TC(USART.channel);

#ifdef STM32H7
  LL_USART_SetTXFIFOThreshold(USART.channel, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART.channel, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART.channel);
#endif

  if (half_duplex) {
    LL_USART_ConfigHalfDuplexMode(USART.channel);
  }

  LL_USART_Enable(USART.channel);

#ifdef STM32H7
  if (usart_init->TransferDirection & LL_USART_DIRECTION_RX) {
    while (!(LL_USART_IsActiveFlag_REACK(USART.channel)))
      ;
  }
  if (usart_init->TransferDirection & LL_USART_DIRECTION_TX) {
    while (!(LL_USART_IsActiveFlag_TEACK(USART.channel)))
      ;
  }
#endif
}

void serial_init(serial_port_t *serial, usart_ports_t port, uint32_t baudrate, uint8_t stop_bits, bool half_duplex) {
  if (port == USART_PORT_INVALID) {
    return;
  }

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  if (half_duplex) {
    gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gpio_init.Pull = LL_GPIO_PULL_NO;
    gpio_pin_init_af(&gpio_init, USART.tx_pin, USART.gpio_af);
  } else {
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_UP;
    gpio_pin_init_af(&gpio_init, USART.rx_pin, USART.gpio_af);
    gpio_pin_init_af(&gpio_init, USART.tx_pin, USART.gpio_af);
  }

  LL_USART_InitTypeDef usart_init;
  usart_init.BaudRate = baudrate;
  usart_init.DataWidth = LL_USART_DATAWIDTH_8B;
  usart_init.StopBits = stop_bits == 2 ? LL_USART_STOPBITS_2 : LL_USART_STOPBITS_1;
  usart_init.Parity = LL_USART_PARITY_NONE;
  usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  usart_init.TransferDirection = LL_USART_DIRECTION_TX_RX;
  usart_init.OverSampling = LL_USART_OVERSAMPLING_16;

  serial_port_init(port, &usart_init, half_duplex, false);

  if (serial) {
    serial_ports[port] = serial;
    serial->port = port;

    serial_enable_isr(port);

    LL_USART_EnableIT_TC(USART.channel);
    LL_USART_EnableIT_RXNE(USART.channel);
  }
}

uint32_t serial_read_bytes(serial_port_t *serial, uint8_t *data, const uint32_t size) {
  return ring_buffer_read_multi(serial->rx_buffer, data, size);
}

bool serial_write_bytes(serial_port_t *serial, const uint8_t *data, const uint32_t size) {
  if (size == 0) {
    return true;
  }

  const usart_port_def_t *port = &usart_port_defs[serial->port];

  uint32_t written = 0;
  while (written < size) {
    written += ring_buffer_write_multi(serial->tx_buffer, data + written, size - written);
    LL_USART_EnableIT_TXE(port->channel);
    __NOP();
  }

  return true;
}

bool serial_is_soft(usart_ports_t port) {
  if (port < USART_PORTS_MAX) {
    return false;
  }
  return true;
}

#ifdef STM32F4
#define USART4 UART4
#define USART5 UART5
#endif

#if defined(STM32F7) || defined(STM32H7)
#define USART4 UART4
#define USART5 UART5
#define USART7 UART7
#define USART8 UART8
#endif

#define GPIO_AF_USART1 GPIO_AF7_USART1
#define GPIO_AF_USART2 GPIO_AF7_USART2
#define GPIO_AF_USART3 GPIO_AF7_USART3
#define GPIO_AF_USART4 GPIO_AF8_UART4
#define GPIO_AF_USART5 GPIO_AF8_UART5

#ifdef STM32H7
#define GPIO_AF_USART6 GPIO_AF7_USART6
#define GPIO_AF_USART7 GPIO_AF7_UART7
#else
#define GPIO_AF_USART6 GPIO_AF8_USART6
#define GPIO_AF_USART7 GPIO_AF8_UART7
#endif

#define GPIO_AF_USART8 GPIO_AF8_UART8

#define USART_PORT(chan, rx, tx)      \
  {                                   \
      .channel_index = chan,          \
      .channel = USART##chan,         \
      .gpio_af = GPIO_AF_USART##chan, \
      .rx_pin = rx,                   \
      .tx_pin = tx,                   \
  },
#define SOFT_SERIAL_PORT(index, rx_pin, tx_pin)

usart_port_def_t usart_port_defs[USART_PORTS_MAX] = {{}, USART_PORTS};

#undef USART_PORT
#undef SOFT_SERIAL_PORT

void handle_serial_isr(serial_port_t *serial) {
  const usart_port_def_t *port = &usart_port_defs[serial->port];

  if (LL_USART_IsEnabledIT_TC(port->channel) && LL_USART_IsActiveFlag_TC(port->channel)) {
    LL_USART_ClearFlag_TC(port->channel);
  }

  if (LL_USART_IsEnabledIT_TXE(port->channel) && LL_USART_IsActiveFlag_TXE(port->channel)) {
    uint8_t data = 0;
    if (ring_buffer_read(serial->tx_buffer, &data)) {
      LL_USART_TransmitData8(port->channel, data);
    } else {
      LL_USART_DisableIT_TXE(port->channel);
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

void handle_usart_isr(usart_ports_t port) {
  if (serial_ports[port]) {
    handle_serial_isr(serial_ports[port]);
    return;
  }

#ifdef SERIAL_RX
  extern void rx_serial_isr();
  if (serial_rx_port == port) {
    rx_serial_isr();
    return;
  }
#endif

  extern void vtx_uart_isr();
  if (serial_smart_audio_port == port) {
    vtx_uart_isr();
    return;
  }

  extern void hdzero_uart_isr();
  if (serial_hdzero_port == port) {
    hdzero_uart_isr();
    return;
  }
}

// we need handlers for both U_S_ART and UART.
// simply define both for every enabled port.
#define USART_PORT(channel, rx_pin, tx_pin) \
  void USART##channel##_IRQHandler() {      \
    handle_usart_isr(USART_IDENT(channel)); \
  }                                         \
  void UART##channel##_IRQHandler() {       \
    handle_usart_isr(USART_IDENT(channel)); \
  }
#define SOFT_SERIAL_PORT(index, rx_pin, tx_pin)

USART_PORTS

#undef USART_PORT
#undef SOFT_SERIAL_PORT