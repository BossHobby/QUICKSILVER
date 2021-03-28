#include "drv_serial.h"

#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_usart.h>

#include "profile.h"
#include "project.h"
#include "usb_configurator.h"

usart_ports_t serial_rx_port = USART_PORT_INVALID;
usart_ports_t serial_smart_audio_port = USART_PORT_INVALID;

//FUNCTION TO SET APB CLOCK TO USART BASED ON GIVEN UART
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
  }
}

void serial_enable_isr(usart_ports_t port) {
  NVIC_SetPriorityGrouping(2);

  switch (usart_port_defs[port].channel_index) {
#ifdef USART1
  case 1:
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART1_IRQn);
    break;
#endif
#ifdef USART2
  case 2:
    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART2_IRQn);
    break;
#endif
#ifdef USART3
  case 3:
    NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART3_IRQn);
    break;
#endif
#ifdef UART4
  case 4:
    NVIC_SetPriority(UART4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(UART4_IRQn);
    break;
#endif
#ifdef UART5
  case 5:
    NVIC_SetPriority(UART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(UART5_IRQn);
    break;
#endif
#ifdef USART6
  case 6:
    NVIC_SetPriority(USART6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART6_IRQn);
    break;
#endif
  }
}

#ifdef F4

#define USART4 UART4
#define USART5 UART5

#define GPIO_AF_USART1 GPIO_AF7_USART1
#define GPIO_AF_USART2 GPIO_AF7_USART2
#define GPIO_AF_USART3 GPIO_AF7_USART3
#define GPIO_AF_USART4 GPIO_AF8_UART4
#define GPIO_AF_USART5 GPIO_AF8_UART5
#define GPIO_AF_USART6 GPIO_AF8_USART6

#define USART_PORT(chan, rx, tx)      \
  {                                   \
      .channel_index = chan,          \
      .channel = USART##chan,         \
      .gpio_af = GPIO_AF_USART##chan, \
      .rx_pin = rx,                   \
      .tx_pin = tx,                   \
  },
#endif

usart_port_def_t usart_port_defs[USART_PORTS_MAX] = {
    {},
    USART_PORTS};

#undef USART_PORT

#define USART usart_port_defs[channel]
void handle_usart_isr(usart_ports_t channel) {
#ifdef SERIAL_RX
  extern void RX_USART_ISR(void);
  extern void TX_USART_ISR(void);
  if (serial_rx_port == channel) {
    if (LL_USART_IsEnabledIT_TC(USART.channel) && LL_USART_IsActiveFlag_TC(USART.channel)) {
      LL_USART_ClearFlag_TC(USART.channel);
      TX_USART_ISR();
    } else {
      RX_USART_ISR();
    }
    return;
  }
#endif
#if defined(ENABLE_SMART_AUDIO) || defined(ENABLE_TRAMP)
  extern void vtx_uart_isr(void);
  if (serial_smart_audio_port == channel) {
    vtx_uart_isr();
    return;
  }
#endif
}

// we need handlers for both U_S_ART and UART.
// simply define both for every enabled port.
#define USART_PORT(channel, rx_pin, tx_pin) \
  void USART##channel##_IRQHandler(void) {  \
    handle_usart_isr(USART_IDENT(channel)); \
  }                                         \
  void UART##channel##_IRQHandler(void) {   \
    handle_usart_isr(USART_IDENT(channel)); \
  }

USART_PORTS

#undef USART_PORT
