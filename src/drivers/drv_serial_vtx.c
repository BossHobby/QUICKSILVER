#include "drv_serial_vtx.h"

#include <stm32f4xx_ll_usart.h>

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

  LL_USART_ClearFlag_RXNE(USART.channel);
  LL_USART_ClearFlag_TC(USART.channel);

  vtx_frame_offset = 0;

  LL_USART_EnableIT_RXNE(USART.channel);
  LL_USART_EnableIT_TXE(USART.channel);
  LL_USART_EnableIT_TC(USART.channel);

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
  if (LL_USART_IsActiveFlag_TC(USART.channel)) {
    LL_USART_ClearFlag_TC(USART.channel);
    if (vtx_frame_offset == vtx_frame_length && vtx_transfer_done == 0) {
      vtx_transfer_done = 1;
      LL_USART_DisableIT_TXE(USART.channel);
    }
  }

  if (LL_USART_IsActiveFlag_TXE(USART.channel)) {
    if (vtx_frame_offset < vtx_frame_length) {
      LL_USART_TransmitData8(USART.channel, vtx_frame[vtx_frame_offset]);
      vtx_frame_offset++;
      vtx_transfer_done = 0;
    }
  }

  if (LL_USART_IsActiveFlag_RXNE(USART.channel)) {
    const uint8_t data = LL_USART_ReceiveData8(USART.channel);
    LL_USART_ClearFlag_RXNE(USART.channel);
    circular_buffer_write(&vtx_rx_buffer, data);
  }

  if (LL_USART_IsActiveFlag_ORE(USART.channel)) {
    LL_USART_ClearFlag_ORE(USART.channel);
  }
}
#endif