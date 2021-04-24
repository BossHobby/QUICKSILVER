#include "drv_serial_vtx.h"

#include "drv_serial.h"
#include "drv_time.h"
#include "util/circular_buffer.h"

#if defined(ENABLE_SMART_AUDIO) || defined(ENABLE_TRAMP)

#define USART usart_port_defs[serial_smart_audio_port]

uint8_t vtx_rx_data[VTX_BUFFER_SIZE];
volatile circular_buffer_t vtx_rx_buffer = {
    .buffer = vtx_rx_data,
    .head = 0,
    .tail = 0,
    .size = VTX_BUFFER_SIZE,
};

uint32_t vtx_last_valid_read = 0;
uint32_t vtx_last_request = 0;

volatile uint8_t vtx_transfer_done = 1;

uint8_t vtx_frame[VTX_BUFFER_SIZE];
uint8_t volatile vtx_frame_length = 0;
uint8_t volatile vtx_frame_offset = 0;

void serial_vtx_send_data(uint8_t *data, uint32_t size) {
  for (uint32_t timeout = 0x1000; vtx_transfer_done == 0 && timeout; --timeout) {
    __WFI();
  }

  vtx_transfer_done = 0;

  USART_ClearITPendingBit(USART.channel, USART_IT_RXNE);
  USART_ClearITPendingBit(USART.channel, USART_IT_TXE);
  USART_ClearITPendingBit(USART.channel, USART_IT_TC);

  vtx_frame_offset = 0;

  USART_ITConfig(USART.channel, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART.channel, USART_IT_TXE, ENABLE);
  USART_ITConfig(USART.channel, USART_IT_TC, ENABLE);

  vtx_last_request = timer_millis();
  vtx_last_valid_read = timer_millis();
}

uint8_t serial_vtx_read_byte(uint8_t *data) {
  if (circular_buffer_read(&vtx_rx_buffer, data) == 1) {
    vtx_last_valid_read = timer_millis();
    return 1;
  }
  return 0;
}

void vtx_uart_isr(void) {
  if (USART_GetITStatus(USART.channel, USART_IT_TC) != RESET) {
    USART_ClearITPendingBit(USART.channel, USART_IT_TC);
    if (vtx_frame_offset == vtx_frame_length && vtx_transfer_done == 0) {
      vtx_transfer_done = 1;
      USART_ITConfig(USART.channel, USART_IT_TXE, DISABLE);
    }
  }

  if (USART_GetITStatus(USART.channel, USART_IT_TXE) != RESET) {
    USART_ClearITPendingBit(USART.channel, USART_IT_TXE);
    if (vtx_frame_offset < vtx_frame_length) {
      USART_SendData(USART.channel, vtx_frame[vtx_frame_offset]);
      vtx_frame_offset++;
      vtx_transfer_done = 0;
    }
  }

  if (USART_GetITStatus(USART.channel, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(USART.channel, USART_IT_RXNE);
    const uint8_t data = USART_ReceiveData(USART.channel);
    circular_buffer_write(&vtx_rx_buffer, data);
  }

  if (USART_GetFlagStatus(USART.channel, USART_FLAG_ORE)) {
    USART_ClearFlag(USART.channel, USART_FLAG_ORE);
  }
}
#endif