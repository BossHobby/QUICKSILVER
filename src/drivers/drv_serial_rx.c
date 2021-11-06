#include "drv_serial.h"

#include "drv_time.h"
#include "profile.h"

#define USART usart_port_defs[serial_rx_port]

// FUNCTION TO COMMAND EXTERNAL USART INVERTER HIGH OR LOW
// Always called during boot in main.c
// Will init gpio pins for uart inverters once on every boot, but will only toggle the inverter on if hard defined INVERT_UART
// Universal serial will overwrite inverter pin state only if INVERT_UART
void usart_invert(void) {
#if defined(F4) && (defined(USART1_INVERTER_PIN) || defined(USART2_INVERTER_PIN) || defined(USART3_INVERTER_PIN) || defined(USART4_INVERTER_PIN) || defined(USART5_INVERTER_PIN) || defined(USART6_INVERTER_PIN))
  GPIO_InitTypeDef gpio_init;
  gpio_init.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;

#if defined(USART1_INVERTER_PIN)
  gpio_pin_init(&gpio_init, USART1_INVERTER_PIN);
#endif

#if defined(USART2_INVERTER_PIN)
  gpio_pin_init(&gpio_init, USART2_INVERTER_PIN);
#endif

#if defined(USART3_INVERTER_PIN)
  gpio_pin_init(&gpio_init, USART3_INVERTER_PIN);
#endif

#if defined(USART4_INVERTER_PIN)
  gpio_pin_init(&gpio_init, USART4_INVERTER_PIN);
#endif

#if defined(USART5_INVERTER_PIN)
  gpio_pin_init(&gpio_init, USART5_INVERTER_PIN);
#endif

#if defined(USART6_INVERTER_PIN)
  gpio_pin_init(&gpio_init, USART6_INVERTER_PIN);
#endif

#ifdef INVERT_UART
  // Inverter control line, set high
  switch (usart_port_defs[profile.serial.rx].channel_index) {
  case 1:
#if defined(USART1_INVERTER_PIN)
    gpio_pin_set(USART1_INVERTER_PIN);
#endif
    break;
  case 2:
#if defined(USART2_INVERTER_PIN)
    gpio_pin_set(USART2_INVERTER_PIN);
#endif
    break;
  case 3:
#if defined(USART3_INVERTER_PIN)
    gpio_pin_set(USART3_INVERTER_PIN);
#endif
    break;
  case 4:
#if defined(USART4_INVERTER_PIN)
    gpio_pin_set(USART4_INVERTER_PIN);
#endif
    break;
  case 5:
#if defined(USART5_INVERTER_PIN)
    gpio_pin_set(USART5_INVERTER_PIN);
#endif
    break;
  case 6:
#if defined(USART6_INVERTER_PIN)
    gpio_pin_set(USART6_INVERTER_PIN);
#endif
    break;
  }
#else
  // Inverter control line, set low
  switch (usart_port_defs[profile.serial.rx].channel_index) {
  case 1:
#if defined(USART1_INVERTER_PIN)
    gpio_pin_reset(USART1_INVERTER_PIN);
#endif
    break;
  case 2:
#if defined(USART2_INVERTER_PIN)
    gpio_pin_reset(USART2_INVERTER_PIN);
#endif
    break;
  case 3:
#if defined(USART3_INVERTER_PIN)
    gpio_pin_reset(USART3_INVERTER_PIN);
#endif
    break;
  case 4:
#if defined(USART4_INVERTER_PIN)
    gpio_pin_reset(USART4_INVERTER_PIN);
#endif
    break;
  case 5:
#if defined(USART5_INVERTER_PIN)
    gpio_pin_reset(USART5_INVERTER_PIN);
#endif
    break;
  case 6:
#if defined(USART6_INVERTER_PIN)
    gpio_pin_reset(USART6_INVERTER_PIN);
#endif
    break;
  }
#endif

#else
  // do nothing here, usart swap command in usart init
#endif
}

//FUNCTION TO INITIALIZE USART FOR A SERIAL RX CALLED FROM RECEIVER PROTOCOL

#ifdef SERIAL_RX
void serial_rx_init(rx_serial_protocol_t proto) {
#if defined(RX_DSM2_1024) || defined(RX_DSMX_2028)
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

  //If the board supports inversion & inversion is not hard defined, prepare it.
#if !defined(INVERT_UART)
#if defined(F4) && (defined(USART1_INVERTER_PIN) || defined(USART2_INVERTER_PIN) || defined(USART3_INVERTER_PIN) || defined(USART4_INVERTER_PIN) || defined(USART5_INVERTER_PIN) || defined(USART6_INVERTER_PIN))

  switch (usart_port_defs[profile.serial.rx].channel_index) {
  case 1:
#if defined(USART1_INVERTER_PIN)
    if (proto == RX_SERIAL_PROTOCOL_SBUS_INVERTED || proto == RX_SERIAL_PROTOCOL_FPORT_INVERTED || proto == RX_SERIAL_PROTOCOL_REDPINE_INVERTED) {
      gpio_pin_set(USART1_INVERTER_PIN);
    } else {
      gpio_pin_reset(USART1_INVERTER_PIN);
    }
#endif
    break;
  case 2:
#if defined(USART2_INVERTER_PIN)
    if (proto == RX_SERIAL_PROTOCOL_SBUS_INVERTED || proto == RX_SERIAL_PROTOCOL_FPORT_INVERTED || proto == RX_SERIAL_PROTOCOL_REDPINE_INVERTED) {
      gpio_pin_set(USART2_INVERTER_PIN);
    } else {
      gpio_pin_reset(USART2_INVERTER_PIN);
    }
#endif
    break;
  case 3:
#if defined(USART3_INVERTER_PIN)
    if (proto == RX_SERIAL_PROTOCOL_SBUS_INVERTED || proto == RX_SERIAL_PROTOCOL_FPORT_INVERTED || proto == RX_SERIAL_PROTOCOL_REDPINE_INVERTED) {
      gpio_pin_set(USART3_INVERTER_PIN);
    } else {
      gpio_pin_reset(USART3_INVERTER_PIN);
    }
#endif
    break;
  case 4:
#if defined(USART4_INVERTER_PIN)
    if (proto == RX_SERIAL_PROTOCOL_SBUS_INVERTED || proto == RX_SERIAL_PROTOCOL_FPORT_INVERTED || proto == RX_SERIAL_PROTOCOL_REDPINE_INVERTED) {
      gpio_pin_set(USART4_INVERTER_PIN);
    } else {
      gpio_pin_reset(USART4_INVERTER_PIN);
    }
#endif
    break;
  case 5:
#if defined(USART5_INVERTER_PIN)
    if (proto == RX_SERIAL_PROTOCOL_SBUS_INVERTED || proto == RX_SERIAL_PROTOCOL_FPORT_INVERTED || proto == RX_SERIAL_PROTOCOL_REDPINE_INVERTED) {
      gpio_pin_set(USART5_INVERTER_PIN);
    } else {
      gpio_pin_reset(USART5_INVERTER_PIN);
    }
#endif
    break;
  case 6:
#if defined(USART6_INVERTER_PIN)
    if (proto == RX_SERIAL_PROTOCOL_SBUS_INVERTED || proto == RX_SERIAL_PROTOCOL_FPORT_INVERTED || proto == RX_SERIAL_PROTOCOL_REDPINE_INVERTED) {
      gpio_pin_set(USART6_INVERTER_PIN);
    } else {
      gpio_pin_reset(USART6_INVERTER_PIN);
    }
#endif
    break;
  }
#endif
#endif

  serial_rx_port = profile.serial.rx;

  GPIO_InitTypeDef gpio_init;
  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

  switch (proto) {
  case RX_SERIAL_PROTOCOL_DSM:
  case RX_SERIAL_PROTOCOL_IBUS:
  case RX_SERIAL_PROTOCOL_CRSF:
    gpio_init.GPIO_OType = GPIO_OType_OD;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;

    gpio_pin_init_af(&gpio_init, USART.rx_pin, USART.gpio_af);
    break;

  case RX_SERIAL_PROTOCOL_SBUS:
  case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
  case RX_SERIAL_PROTOCOL_FPORT:
  case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
  case RX_SERIAL_PROTOCOL_REDPINE:
  case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;

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
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;

    gpio_pin_init_af(&gpio_init, USART.tx_pin, USART.gpio_af);
    break;

  default:
    // no tx pin for the others
    break;
  }

  serial_enable_rcc(serial_rx_port);

  USART_InitTypeDef usart_init;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart_init.USART_WordLength = USART_WordLength_8b;

  switch (proto) {
  case RX_SERIAL_PROTOCOL_DSM:
  case RX_SERIAL_PROTOCOL_IBUS:
    usart_init.USART_BaudRate = 115200;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_Mode = USART_Mode_Rx;
    break;

  case RX_SERIAL_PROTOCOL_FPORT:
  case RX_SERIAL_PROTOCOL_FPORT_INVERTED:
    usart_init.USART_BaudRate = 115200;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    break;

  case RX_SERIAL_PROTOCOL_SBUS:
  case RX_SERIAL_PROTOCOL_SBUS_INVERTED:
    usart_init.USART_BaudRate = 100000;
    usart_init.USART_StopBits = USART_StopBits_2;
    usart_init.USART_Parity = USART_Parity_Even;
    usart_init.USART_Mode = USART_Mode_Rx;
    break;

  case RX_SERIAL_PROTOCOL_CRSF:
    usart_init.USART_BaudRate = 420000;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    break;

  case RX_SERIAL_PROTOCOL_REDPINE:
  case RX_SERIAL_PROTOCOL_REDPINE_INVERTED:
    usart_init.USART_BaudRate = 230400;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_Mode = USART_Mode_Rx;
    break;

  default:
    break;
  }

  if (proto == RX_SERIAL_PROTOCOL_FPORT) {
    //RX_SERIAL_PROTOCOL_FPORT_INVERTED requires half duplex off
    USART_HalfDuplexCmd(USART.channel, ENABLE);
  } else {
    USART_HalfDuplexCmd(USART.channel, DISABLE);
  }

  USART_Init(USART.channel, &usart_init);

  USART_ITConfig(USART.channel, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART.channel, ENABLE);
  serial_enable_isr(serial_rx_port);
}
#endif
