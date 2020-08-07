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
  gpio_init.GPIO_Pin = gpio_pin_defs[USART1_INVERTER_PIN].pin;
  GPIO_Init(gpio_pin_defs[USART1_INVERTER_PIN].port, &gpio_init);
#endif

#if defined(USART2_INVERTER_PIN)
  gpio_init.GPIO_Pin = gpio_pin_defs[USART2_INVERTER_PIN].pin;
  GPIO_Init(gpio_pin_defs[USART2_INVERTER_PIN].port, &gpio_init);
#endif

#if defined(USART3_INVERTER_PIN)
  gpio_init.GPIO_Pin = gpio_pin_defs[USART3_INVERTER_PIN].pin;
  GPIO_Init(gpio_pin_defs[USART3_INVERTER_PIN].port, &gpio_init);
#endif

#if defined(USART4_INVERTER_PIN)
  gpio_init.GPIO_Pin = gpio_pin_defs[USART4_INVERTER_PIN].pin;
  GPIO_Init(gpio_pin_defs[USART4_INVERTER_PIN].port, &gpio_init);
#endif

#if defined(USART5_INVERTER_PIN)
  gpio_init.GPIO_Pin = gpio_pin_defs[USART5_INVERTER_PIN].pin;
  GPIO_Init(gpio_pin_defs[USART5_INVERTER_PIN].port, &gpio_init);
#endif

#if defined(USART6_INVERTER_PIN)
  gpio_init.GPIO_Pin = gpio_pin_defs[USART6_INVERTER_PIN].pin;
  GPIO_Init(gpio_pin_defs[USART6_INVERTER_PIN].port, &gpio_init);
#endif

#ifdef INVERT_UART
  // Inverter control line, set high
  switch (usart_port_defs[profile.serial.rx].channel_index) {
  case 1:
#if defined(USART1_INVERTER_PIN)
    GPIO_SetBits(gpio_pin_defs[USART1_INVERTER_PIN].port, gpio_pin_defs[USART1_INVERTER_PIN].pin);
#endif
    break;
  case 2:
#if defined(USART2_INVERTER_PIN)
    GPIO_SetBits(gpio_pin_defs[USART2_INVERTER_PIN].port, gpio_pin_defs[USART2_INVERTER_PIN].pin);
#endif
    break;
  case 3:
#if defined(USART3_INVERTER_PIN)
    GPIO_SetBits(gpio_pin_defs[USART3_INVERTER_PIN].port, gpio_pin_defs[USART3_INVERTER_PIN].pin);
#endif
    break;
  case 4:
#if defined(USART4_INVERTER_PIN)
    GPIO_SetBits(gpio_pin_defs[USART4_INVERTER_PIN].port, gpio_pin_defs[USART4_INVERTER_PIN].pin);
#endif
    break;
  case 5:
#if defined(USART5_INVERTER_PIN)
    GPIO_SetBits(gpio_pin_defs[USART5_INVERTER_PIN].port, gpio_pin_defs[USART5_INVERTER_PIN].pin);
#endif
    break;
  case 6:
#if defined(USART6_INVERTER_PIN)
    GPIO_SetBits(gpio_pin_defs[USART6_INVERTER_PIN].port, gpio_pin_defs[USART6_INVERTER_PIN].pin);
#endif
    break;
  }
#else
  // Inverter control line, set low
  switch (usart_port_defs[profile.serial.rx].channel_index) {
  case 1:
#if defined(USART1_INVERTER_PIN)
    GPIO_ResetBits(gpio_pin_defs[USART1_INVERTER_PIN].port, gpio_pin_defs[USART1_INVERTER_PIN].pin);
#endif
    break;
  case 2:
#if defined(USART2_INVERTER_PIN)
    GPIO_ResetBits(gpio_pin_defs[USART2_INVERTER_PIN].port, gpio_pin_defs[USART2_INVERTER_PIN].pin);
#endif
    break;
  case 3:
#if defined(USART3_INVERTER_PIN)
    GPIO_ResetBits(gpio_pin_defs[USART3_INVERTER_PIN].port, gpio_pin_defs[USART3_INVERTER_PIN].pin);
#endif
    break;
  case 4:
#if defined(USART4_INVERTER_PIN)
    GPIO_ResetBits(gpio_pin_defs[USART4_INVERTER_PIN].port, gpio_pin_defs[USART4_INVERTER_PIN].pin);
#endif
    break;
  case 5:
#if defined(USART5_INVERTER_PIN)
    GPIO_ResetBits(gpio_pin_defs[USART5_INVERTER_PIN].port, gpio_pin_defs[USART5_INVERTER_PIN].pin);
#endif
    break;
  case 6:
#if defined(USART6_INVERTER_PIN)
    GPIO_ResetBits(gpio_pin_defs[USART6_INVERTER_PIN].port, gpio_pin_defs[USART6_INVERTER_PIN].pin);
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
void serial_rx_init(rx_serial_protocol_t rx_serial_protocol) {
#if defined(RX_DSM2_1024) || defined(RX_DSMX_2028)
  rx_serial_protocol = RX_SERIAL_PROTOCOL_DSM;
#endif
#if defined(RX_SBUS)
  rx_serial_protocol = RX_SERIAL_PROTOCOL_SBUS;
#endif
#if defined(RX_IBUS)
  rx_serial_protocol = RX_SERIAL_PROTOCOL_IBUS;
#endif
#if defined(RX_FPORT)
  rx_serial_protocol = RX_SERIAL_PROTOCOL_FPORT;
#endif
#if defined(RX_CRSF)
  rx_serial_protocol = RX_SERIAL_PROTOCOL_CRSF;
#endif

  //If the board supports inversion & inversion is not hard defined, prepare it.
#if !defined(INVERT_UART)
#if defined(F4) && (defined(USART1_INVERTER_PIN) || defined(USART2_INVERTER_PIN) || defined(USART3_INVERTER_PIN) || defined(USART4_INVERTER_PIN) || defined(USART5_INVERTER_PIN) || defined(USART6_INVERTER_PIN))

  switch (usart_port_defs[profile.serial.rx].channel_index) {
  case 1:
#if defined(USART1_INVERTER_PIN)
    if (rx_serial_protocol == RX_SERIAL_PROTOCOL_SBUS_INVERTED || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT_INVERTED) {
      GPIO_SetBits(gpio_pin_defs[USART1_INVERTER_PIN].port, gpio_pin_defs[USART1_INVERTER_PIN].pin);
    } else {
      GPIO_ResetBits(gpio_pin_defs[USART1_INVERTER_PIN].port, gpio_pin_defs[USART1_INVERTER_PIN].pin);
    }
#endif
    break;
  case 2:
#if defined(USART2_INVERTER_PIN)
    if (rx_serial_protocol == RX_SERIAL_PROTOCOL_SBUS_INVERTED || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT_INVERTED) {
      GPIO_SetBits(gpio_pin_defs[USART2_INVERTER_PIN].port, gpio_pin_defs[USART2_INVERTER_PIN].pin);
    } else {
      GPIO_ResetBits(gpio_pin_defs[USART2_INVERTER_PIN].port, gpio_pin_defs[USART2_INVERTER_PIN].pin);
    }
#endif
    break;
  case 3:
#if defined(USART3_INVERTER_PIN)
    if (rx_serial_protocol == RX_SERIAL_PROTOCOL_SBUS_INVERTED || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT_INVERTED) {
      GPIO_SetBits(gpio_pin_defs[USART3_INVERTER_PIN].port, gpio_pin_defs[USART3_INVERTER_PIN].pin);
    } else {
      GPIO_ResetBits(gpio_pin_defs[USART3_INVERTER_PIN].port, gpio_pin_defs[USART3_INVERTER_PIN].pin);
    }
#endif
    break;
  case 4:
#if defined(USART4_INVERTER_PIN)
    if (rx_serial_protocol == RX_SERIAL_PROTOCOL_SBUS_INVERTED || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT_INVERTED) {
      GPIO_SetBits(gpio_pin_defs[USART4_INVERTER_PIN].port, gpio_pin_defs[USART4_INVERTER_PIN].pin);
    } else {
      GPIO_ResetBits(gpio_pin_defs[USART4_INVERTER_PIN].port, gpio_pin_defs[USART4_INVERTER_PIN].pin);
    }
#endif
    break;
  case 5:
#if defined(USART5_INVERTER_PIN)
    if (rx_serial_protocol == RX_SERIAL_PROTOCOL_SBUS_INVERTED || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT_INVERTED) {
      GPIO_SetBits(gpio_pin_defs[USART5_INVERTER_PIN].port, gpio_pin_defs[USART5_INVERTER_PIN].pin);
    } else {
      GPIO_ResetBits(gpio_pin_defs[USART5_INVERTER_PIN].port, gpio_pin_defs[USART5_INVERTER_PIN].pin);
    }
#endif
    break;
  case 6:
#if defined(USART6_INVERTER_PIN)
    if (rx_serial_protocol == RX_SERIAL_PROTOCOL_SBUS_INVERTED || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT_INVERTED) {
      GPIO_SetBits(gpio_pin_defs[USART6_INVERTER_PIN].port, gpio_pin_defs[USART6_INVERTER_PIN].pin);
    } else {
      GPIO_ResetBits(gpio_pin_defs[USART6_INVERTER_PIN].port, gpio_pin_defs[USART6_INVERTER_PIN].pin);
    }
#endif
    break;
  }
#endif
#endif

  serial_rx_port = profile.serial.rx;

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  if (rx_serial_protocol == RX_SERIAL_PROTOCOL_DSM || rx_serial_protocol == RX_SERIAL_PROTOCOL_IBUS || rx_serial_protocol == RX_SERIAL_PROTOCOL_CRSF) {
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  } else if (rx_serial_protocol == RX_SERIAL_PROTOCOL_SBUS || rx_serial_protocol == RX_SERIAL_PROTOCOL_SBUS_INVERTED || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT_INVERTED) {
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  }
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  serial_enable_rcc(serial_rx_port);

  if (rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT_INVERTED) {
    GPIO_InitStructure.GPIO_Pin = USART.tx_pin.pin;
    GPIO_Init(USART.tx_pin.port, &GPIO_InitStructure);

    GPIO_PinAFConfig(USART.tx_pin.port, USART.tx_pin.pin_source, USART.gpio_af);
  }
    GPIO_InitStructure.GPIO_Pin = USART.rx_pin.pin;
    GPIO_Init(USART.rx_pin.port, &GPIO_InitStructure);

    GPIO_PinAFConfig(USART.rx_pin.port, USART.rx_pin.pin_source, USART.gpio_af);


  USART_InitTypeDef USART_InitStructure;
  if (rx_serial_protocol == RX_SERIAL_PROTOCOL_DSM || rx_serial_protocol == RX_SERIAL_PROTOCOL_IBUS || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT_INVERTED) {
    USART_InitStructure.USART_BaudRate = 115200;
  } else if (rx_serial_protocol == RX_SERIAL_PROTOCOL_SBUS || rx_serial_protocol == RX_SERIAL_PROTOCOL_SBUS_INVERTED) {
    USART_InitStructure.USART_BaudRate = 100000;
  } else if (rx_serial_protocol == RX_SERIAL_PROTOCOL_CRSF) {
    USART_InitStructure.USART_BaudRate = 420000;
  }

  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  if (rx_serial_protocol == RX_SERIAL_PROTOCOL_SBUS || rx_serial_protocol == RX_SERIAL_PROTOCOL_SBUS_INVERTED) {
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
  } else {
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
  }

  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  if (rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT || rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT_INVERTED || rx_serial_protocol == RX_SERIAL_PROTOCOL_CRSF) {
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  } else {
    USART_InitStructure.USART_Mode = USART_Mode_Rx; //USART_Mode_Rx | USART_Mode_Tx;
  }

  if (rx_serial_protocol == RX_SERIAL_PROTOCOL_FPORT){     //RX_SERIAL_PROTOCOL_FPORT_INVERTED requires half duplex off) {
    USART_HalfDuplexCmd(USART.channel, ENABLE);
  } else {
    USART_HalfDuplexCmd(USART.channel, DISABLE);
  }

  USART_Init(USART.channel, &USART_InitStructure);

#ifdef F0
#ifdef INVERT_UART
  USART_InvPinCmd(USART.channel, USART_InvPin_Rx | USART_InvPin_Tx, ENABLE);
#endif
  // swap rx/tx pins - available on F0 targets
#ifdef F0_USART_PINSWAP
  USART_SWAPPinCmd(USART.channel, ENABLE);
#endif
#endif

  USART_ITConfig(USART.channel, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART.channel, ENABLE);
  serial_enable_isr(serial_rx_port);
}
#endif
