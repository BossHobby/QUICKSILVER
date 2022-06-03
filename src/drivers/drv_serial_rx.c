#include "drv_serial.h"

#include "drv_time.h"
#include "profile.h"

#define USART usart_port_defs[serial_rx_port]

// FUNCTION TO COMMAND EXTERNAL USART INVERTER HIGH OR LOW
// Will init gpio pins for uart inverters once on every boot and invert according to parameter
void handle_usart_invert(bool invert) {
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

// FUNCTION TO INITIALIZE USART FOR A SERIAL RX CALLED FROM RECEIVER PROTOCOL

#ifdef SERIAL_RX
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

  serial_rx_port = profile.serial.rx;

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;

  switch (proto) {
  case RX_SERIAL_PROTOCOL_DSM:
  case RX_SERIAL_PROTOCOL_IBUS:
  case RX_SERIAL_PROTOCOL_CRSF:
    gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gpio_init.Pull = LL_GPIO_PULL_UP;

    gpio_pin_init_af(&gpio_init, USART.rx_pin, USART.gpio_af);
    break;

  case RX_SERIAL_PROTOCOL_SBUS:
  case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
  case RX_SERIAL_PROTOCOL_FPORT:
  case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
  case RX_SERIAL_PROTOCOL_REDPINE:
  case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_NO;

    gpio_pin_init_af(&gpio_init, USART.rx_pin, USART.gpio_af);
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

    gpio_pin_init_af(&gpio_init, USART.tx_pin, USART.gpio_af);
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

  if (proto == RX_SERIAL_PROTOCOL_FPORT) {
    // RX_SERIAL_PROTOCOL_FPORT_INVERTED requires half duplex off
    serial_port_init(serial_rx_port, &usart_init, true);
  } else {
    serial_port_init(serial_rx_port, &usart_init, false);
  }

#if defined(INVERT_UART)
  // inversion is hard defined, always invert
  handle_usart_invert(true);
#else
  // invert according to protocol
  handle_usart_invert(proto == RX_SERIAL_PROTOCOL_SBUS_INVERTED || proto == RX_SERIAL_PROTOCOL_FPORT_INVERTED || proto == RX_SERIAL_PROTOCOL_REDPINE_INVERTED);
#endif

  LL_USART_EnableIT_RXNE(USART.channel);

  serial_enable_isr(serial_rx_port);
}
#endif
