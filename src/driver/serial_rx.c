#include "driver/serial.h"

#include "core/profile.h"
#include "driver/time.h"

#define USART usart_port_defs[serial_rx_port]

void serial_rx_init(rx_serial_protocol_t proto) {
#if defined(RX_DSM2) || defined(RX_DSMX)
  proto = RX_SERIAL_PROTOCOL_DSM;
#endif
#if defined(RX_SBUS)
  proto = RX_SERIAL_PROTOCOL_SBUS;
#endif
#if defined(RX_IBUS)
  proto = RX_SERIAL_PROTOCOL_IBUS;
#endif
#if defined(RX_FPORT)
  proto = RX_SERIAL_PROTOCOL_FPORT;
#endif
#if defined(RX_CRSF)
  proto = RX_SERIAL_PROTOCOL_CRSF;
#endif

  const target_serial_port_t *dev = &target.serial_ports[profile.serial.rx];
  if (!target_serial_port_valid(dev)) {
    return;
  }

  serial_rx_port = profile.serial.rx;

  if (serial_rx_port == SERIAL_PORT_INVALID) {
    return;
  }

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;

  switch (proto) {
  case RX_SERIAL_PROTOCOL_DSM:
  case RX_SERIAL_PROTOCOL_IBUS:
  case RX_SERIAL_PROTOCOL_CRSF:
    gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gpio_init.Pull = LL_GPIO_PULL_UP;
    gpio_pin_init_tag(&gpio_init, dev->rx, SERIAL_TAG(serial_rx_port, RES_SERIAL_RX));
    break;

  case RX_SERIAL_PROTOCOL_SBUS:
  case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
  case RX_SERIAL_PROTOCOL_FPORT:
  case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
  case RX_SERIAL_PROTOCOL_REDPINE:
  case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_NO;
    gpio_pin_init_tag(&gpio_init, dev->rx, SERIAL_TAG(serial_rx_port, RES_SERIAL_RX));
    break;

  default:
    // no rx-pin? maybe throw error
    break;
  }

  switch (proto) {
  case RX_SERIAL_PROTOCOL_FPORT:
  case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
  case RX_SERIAL_PROTOCOL_CRSF:
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_NO;
    gpio_pin_init_tag(&gpio_init, dev->tx, SERIAL_TAG(serial_rx_port, RES_SERIAL_TX));
    break;

  default:
    // no tx pin for the others
    break;
  }

  serial_enable_rcc(serial_rx_port);

  LL_USART_InitTypeDef usart_init;
  LL_USART_StructInit(&usart_init);

  usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  usart_init.DataWidth = LL_USART_DATAWIDTH_8B;
  usart_init.OverSampling = LL_USART_OVERSAMPLING_16;

  switch (proto) {
  case RX_SERIAL_PROTOCOL_DSM:
  case RX_SERIAL_PROTOCOL_IBUS:
    usart_init.BaudRate = 115200;
    usart_init.StopBits = LL_USART_STOPBITS_1;
    usart_init.Parity = LL_USART_PARITY_NONE;
    usart_init.TransferDirection = LL_USART_DIRECTION_RX;
    break;

  case RX_SERIAL_PROTOCOL_FPORT:
  case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
    usart_init.BaudRate = 115200;
    usart_init.StopBits = LL_USART_STOPBITS_1;
    usart_init.Parity = LL_USART_PARITY_NONE;
    usart_init.TransferDirection = LL_USART_DIRECTION_TX_RX;
    break;

  case RX_SERIAL_PROTOCOL_SBUS:
  case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
    usart_init.BaudRate = 100000;
    usart_init.StopBits = LL_USART_STOPBITS_2;
    usart_init.Parity = LL_USART_PARITY_EVEN;
    usart_init.TransferDirection = LL_USART_DIRECTION_RX;
    break;

  case RX_SERIAL_PROTOCOL_CRSF:
    usart_init.BaudRate = 420000;
    usart_init.StopBits = LL_USART_STOPBITS_1;
    usart_init.Parity = LL_USART_PARITY_NONE;
    usart_init.TransferDirection = LL_USART_DIRECTION_TX_RX;
    break;

  case RX_SERIAL_PROTOCOL_REDPINE:
  case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
    usart_init.BaudRate = 230400;
    usart_init.StopBits = LL_USART_STOPBITS_1;
    usart_init.Parity = LL_USART_PARITY_NONE;
    usart_init.TransferDirection = LL_USART_DIRECTION_RX;
    break;

  default:
    break;
  }

#if defined(INVERT_UART)
  // inversion is hard defined, always invert
  const bool invert_port = true;
#else
  // invert according to protocol
  const bool invert_port = proto == RX_SERIAL_PROTOCOL_SBUS_INVERTED || proto == RX_SERIAL_PROTOCOL_FPORT_INVERTED || proto == RX_SERIAL_PROTOCOL_REDPINE_INVERTED;
#endif

  // RX_SERIAL_PROTOCOL_FPORT_INVERTED requires half duplex off
  const bool half_duplex = proto == RX_SERIAL_PROTOCOL_FPORT;

  serial_port_init(serial_rx_port, &usart_init, half_duplex, invert_port);

  LL_USART_EnableIT_RXNE(USART.channel);

  serial_enable_isr(serial_rx_port);
}
