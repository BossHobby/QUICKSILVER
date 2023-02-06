#include "driver/serial_vtx.h"

#include "driver/serial.h"
#include "driver/serial_soft.h"
#include "driver/time.h"
#include "io/usb_configurator.h"
#include "util/ring_buffer.h"

#define USART usart_port_defs[serial_smart_audio_port]

uint8_t vtx_rx_data[VTX_BUFFER_SIZE];
ring_buffer_t vtx_rx_buffer = {
    .buffer = vtx_rx_data,
    .head = 0,
    .tail = 0,
    .size = VTX_BUFFER_SIZE,
};

uint32_t vtx_last_valid_read = 0;
uint32_t vtx_last_request = 0;

volatile uint8_t vtx_transfer_done = 1;

uint8_t vtx_frame[VTX_BUFFER_SIZE];
volatile uint8_t vtx_frame_length = 0;
volatile uint8_t vtx_frame_offset = 0;

bool serial_vtx_is_ready() {
  return vtx_transfer_done;
}

bool serial_vtx_wait_for_ready() {
  const uint32_t start = time_millis();
  while (!serial_vtx_is_ready()) {
    if ((time_millis() - start) > 100) {
      return false;
    }
    __NOP();
  }
  return true;
}

bool serial_vtx_send_data(uint8_t *data, uint32_t size) {
  if (!serial_vtx_is_ready()) {
    return false;
  }

  vtx_transfer_done = 0;
  vtx_frame_offset = 0;

  if (serial_is_soft(serial_smart_audio_port)) {
    soft_serial_enable_write(serial_smart_audio_port);
  } else {
    // LL_USART_ClearFlag_RXNE(USART.channel);
    LL_USART_ClearFlag_TC(USART.channel);

    // ring_buffer_clear(&vtx_rx_buffer);

    LL_USART_EnableIT_RXNE(USART.channel);
    LL_USART_EnableIT_TXE(USART.channel);
    LL_USART_EnableIT_TC(USART.channel);

    LL_USART_DisableDirectionRx(USART.channel);
    LL_USART_EnableDirectionTx(USART.channel);
  }

  vtx_last_request = time_millis();
  vtx_last_valid_read = time_millis();

  return true;
}

uint8_t serial_vtx_read_byte(uint8_t *data) {
  if (ring_buffer_read(&vtx_rx_buffer, data) == 1) {
    vtx_last_valid_read = time_millis();
    return 1;
  }
  return 0;
}

void vtx_uart_isr() {
  if (LL_USART_IsEnabledIT_TC(USART.channel) && LL_USART_IsActiveFlag_TC(USART.channel)) {
    LL_USART_ClearFlag_TC(USART.channel);
    if (vtx_frame_offset == vtx_frame_length && vtx_transfer_done == 0) {
      LL_USART_DisableDirectionTx(USART.channel);
      LL_USART_EnableDirectionRx(USART.channel);
      vtx_transfer_done = 1;
    }
  }

  if (LL_USART_IsEnabledIT_TXE(USART.channel) && LL_USART_IsActiveFlag_TXE(USART.channel)) {
    if (vtx_frame_offset < vtx_frame_length) {
      LL_USART_TransmitData8(USART.channel, vtx_frame[vtx_frame_offset]);
      vtx_frame_offset++;
    }
    if (vtx_frame_offset == vtx_frame_length) {
      LL_USART_DisableIT_TXE(USART.channel);
    }
  }

  if (LL_USART_IsEnabledIT_RXNE(USART.channel) && LL_USART_IsActiveFlag_RXNE(USART.channel)) {
    const uint8_t data = LL_USART_ReceiveData8(USART.channel);
    ring_buffer_write(&vtx_rx_buffer, data);
  }

  if (LL_USART_IsActiveFlag_ORE(USART.channel)) {
    LL_USART_ClearFlag_ORE(USART.channel);
  }
}

void soft_serial_tx_isr() {
  if (vtx_frame_offset < vtx_frame_length) {
    soft_serial_write_byte(serial_smart_audio_port, vtx_frame[vtx_frame_offset]);
    vtx_frame_offset++;
    return;
  }
  if (vtx_frame_offset == vtx_frame_length && vtx_transfer_done == 0) {
    soft_serial_enable_read(serial_smart_audio_port);
    vtx_transfer_done = 1;
  }
}

void soft_serial_rx_isr() {
  const uint8_t data = soft_serial_read_byte(serial_smart_audio_port);
  ring_buffer_write(&vtx_rx_buffer, data);
}