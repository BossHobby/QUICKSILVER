#include "defines.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "profile.h"

extern uint8_t rxusart;
//SET SERIAL BAUDRATE BASED ON RECEIVER PROTOCOL
//1 = DSM2/X
//2 = sbus
//3 = ibus
//4 = FPORT
//5 = CRSF

#define port_def usart_port_defs[profile.serial.rx]

//FUNCTION TO COMMAND EXTERNAL USART INVERTER HIGH OR LOW         todo: sort out target mapping tag in drv_rx_serial.h for a quick define from the taarget

void usart_invert(void) {
#if defined(F405) && defined(INVERT_UART) && defined(USART_INVERTER_PIN) && defined(USART_INVERTER_PORT)
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = USART_INVERTER_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USART_INVERTER_PORT, &GPIO_InitStructure);
#ifdef INVERT_UART
  // Inverter control line, set high
  GPIO_SetBits(USART_INVERTER_PORT, USART_INVERTER_PIN);
#else
  // Inverter control line, set low
  GPIO_ResetBits(USART_INVERTER_PORT, USART_INVERTER_PIN);
#endif
#else
  // do nothing here, usart swap command in usart init
#endif
}

//FUNCTION TO INITIALIZE USART FOR A SERIAL RX CALLED FROM RECEIVER PROTOCOL

#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT) || defined(RX_UNIFIED_SERIAL)
void usart_rx_init(uint8_t RXProtocol) {
#if defined(RX_DSM2_1024) || defined(RX_DSMX_2028)
  RXProtocol = RX_PROTOCOL_DSM;
#endif
#if defined(RX_SBUS)
  RXProtocol = RX_PROTOCOL_SBUS;
#endif
#if defined(RX_IBUS)
  RXProtocol = RX_PROTOCOL_IBUS;
#endif
#if defined(RX_FPORT)
  RXProtocol = RX_PROTOCOL_FPORT;
#endif
#if defined(RX_CRSF)
  RXProtocol = RX_PROTOCOL_CRSF;
#endif
  // make sure there is some time to program the board if SDA pins are reinitialized as GPIO
  if (gettime() < 2000000)
    return;

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  if (RXProtocol == RX_PROTOCOL_DSM || RXProtocol == RX_PROTOCOL_IBUS || RXProtocol == RX_PROTOCOL_CRSF) {
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  } else if (RXProtocol == RX_PROTOCOL_SBUS || RXProtocol == RX_PROTOCOL_FPORT) {
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  }
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  serial_enable_rcc(profile.serial.rx);

  if (RXProtocol == RX_PROTOCOL_FPORT) {
    GPIO_InitStructure.GPIO_Pin = port_def.tx_pin;
    GPIO_Init(port_def.gpio_port, &GPIO_InitStructure);

    GPIO_PinAFConfig(port_def.gpio_port, port_def.tx_pin_source, port_def.gpio_af);
  } else {
    GPIO_InitStructure.GPIO_Pin = port_def.rx_pin;
    GPIO_Init(port_def.gpio_port, &GPIO_InitStructure);

    GPIO_PinAFConfig(port_def.gpio_port, port_def.rx_pin_source, port_def.gpio_af);
  }

  USART_InitTypeDef USART_InitStructure;
  if (RXProtocol == RX_PROTOCOL_DSM || RXProtocol == RX_PROTOCOL_IBUS || RXProtocol == RX_PROTOCOL_FPORT) {
    USART_InitStructure.USART_BaudRate = 115200;
  } else if (RXProtocol == RX_PROTOCOL_SBUS) {
    USART_InitStructure.USART_BaudRate = 100000;
  } else if (RXProtocol == RX_PROTOCOL_CRSF) {
    USART_InitStructure.USART_BaudRate = 420000;
  }

  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  if (RXProtocol == RX_PROTOCOL_SBUS) {
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
  } else {
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
  }

  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  if (RXProtocol == RX_PROTOCOL_FPORT || RXProtocol == RX_PROTOCOL_CRSF) {
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  } else {
    USART_InitStructure.USART_Mode = USART_Mode_Rx; //USART_Mode_Rx | USART_Mode_Tx;
  }

  if (RXProtocol == RX_PROTOCOL_FPORT) {
    USART_HalfDuplexCmd(port_def.channel, ENABLE);
  } else {
    USART_HalfDuplexCmd(port_def.channel, DISABLE);
  }

  USART_Init(port_def.channel, &USART_InitStructure);

#ifdef F0
#ifdef INVERT_UART
  USART_InvPinCmd(port_def.channel, USART_InvPin_Rx | USART_InvPin_Tx, ENABLE);
#endif
  // swap rx/tx pins - available on F0 targets
#ifdef F0_USART_PINSWAP
  USART_SWAPPinCmd(port_def.channel, ENABLE);
#endif
#endif

  USART_ITConfig(port_def.channel, USART_IT_RXNE, ENABLE);
  USART_Cmd(port_def.channel, ENABLE);

  serial_enable_isr(profile.serial.rx);
}
#endif
