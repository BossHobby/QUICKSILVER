
#include "driver/serial_vtx_msp.h"

#include <string.h>

#include "core/debug.h"
#include "core/profile.h"
#include "driver/serial.h"
#include "driver/serial_hdzero.h"
#include "io/msp.h"
#include "util/crc.h"

#define USART usart_port_defs[serial_smart_audio_port]

extern msp_t *msp_vtx;

extern uint32_t vtx_last_valid_read;
extern uint32_t vtx_last_request;

extern volatile uint8_t vtx_transfer_done;

extern uint8_t vtx_frame[VTX_BUFFER_SIZE];
extern volatile uint8_t vtx_frame_length;
extern volatile uint8_t vtx_frame_offset;

static volatile bool request_ready = false;

static void serial_msp_send(msp_magic_t magic, uint8_t direction, uint16_t cmd, const uint8_t *data, uint16_t len) {
  if (magic == MSP2_MAGIC) {
    vtx_frame_length = len + MSP2_HEADER_LEN + 1;

    vtx_frame[0] = '$';
    vtx_frame[1] = MSP2_MAGIC;
    vtx_frame[2] = '>';
    vtx_frame[3] = 0; // flag
    vtx_frame[4] = (cmd >> 0) & 0xFF;
    vtx_frame[5] = (cmd >> 8) & 0xFF;
    vtx_frame[6] = (len >> 0) & 0xFF;
    vtx_frame[7] = (len >> 8) & 0xFF;

    memcpy(vtx_frame + MSP2_HEADER_LEN, data, len);
    vtx_frame[len + MSP2_HEADER_LEN] = crc8_dvb_s2_data(0, vtx_frame + 3, len + 5);
  } else {
    vtx_frame_length = len + MSP_HEADER_LEN + 1;

    vtx_frame[0] = '$';
    vtx_frame[1] = MSP1_MAGIC;
    vtx_frame[2] = '>';
    vtx_frame[3] = len;
    vtx_frame[4] = cmd;

    memcpy(vtx_frame + MSP_HEADER_LEN, data, len);

    uint8_t chksum = len;
    for (uint8_t i = 4; i < (vtx_frame_length - 1); i++) {
      chksum ^= vtx_frame[i];
    }
    vtx_frame[len + MSP_HEADER_LEN] = chksum;
  }

  request_ready = true;
}

extern msp_t hdzero_msp;
static uint8_t msp_rx_buffer[128];
static msp_t msp = {
    .buffer = msp_rx_buffer,
    .buffer_size = 128,
    .buffer_offset = 0,
    .send = serial_msp_send,
    .device = MSP_DEVICE_VTX,
};

void serial_msp_vtx_init() {
  if (serial_hdzero_port != USART_PORT_INVALID) {
    // reuse existing msp for hdz
    msp_vtx = &hdzero_msp;
    return;
  }

  serial_smart_audio_port = profile.serial.smart_audio;
  serial_enable_rcc(serial_smart_audio_port);
  serial_vtx_wait_for_ready();

  if (serial_is_soft(serial_smart_audio_port)) {
    soft_serial_init(serial_smart_audio_port, 9600, 1);
  } else {
    serial_disable_isr(serial_smart_audio_port);

    LL_GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_pin_init_af(&GPIO_InitStructure, USART.tx_pin, USART.gpio_af);

    LL_USART_InitTypeDef usart_init;
    LL_USART_StructInit(&usart_init);
    usart_init.BaudRate = 9600;
    usart_init.DataWidth = LL_USART_DATAWIDTH_8B;
    usart_init.StopBits = LL_USART_STOPBITS_1;
    usart_init.Parity = LL_USART_PARITY_NONE;
    usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    usart_init.TransferDirection = LL_USART_DIRECTION_TX_RX;
    usart_init.OverSampling = LL_USART_OVERSAMPLING_16;
    serial_port_init(serial_smart_audio_port, &usart_init, true, false);

    LL_USART_EnableIT_RXNE(USART.channel);
    LL_USART_EnableIT_TC(USART.channel);

    serial_enable_isr(serial_smart_audio_port);
  }

  msp_vtx = &msp;
  vtx_last_valid_read = time_millis();
}

vtx_update_result_t serial_msp_vtx_update() {
  if (serial_hdzero_port != USART_PORT_INVALID) {
    if (!hdzero_is_ready()) {
      return VTX_WAIT;
    }
    return VTX_IDLE;
  }

  if (vtx_transfer_done == 0) {
    return VTX_WAIT;
  }

  static bool in_progress = false;
  static bool is_first_packet = true;

  if (request_ready) {
    if (serial_vtx_send_data(vtx_frame, vtx_frame_length)) {
      request_ready = false;
    }
    return VTX_WAIT;
  }

  uint8_t data = 0;
  while (serial_vtx_read_byte(&data)) {
    quic_debugf("MSP_VTX: read 0x%x %c", data, data);

    in_progress = true;

    msp_status_t status = msp_process_serial(msp_vtx, data);
    switch (status) {
    case MSP_ERROR:
    case MSP_EOF:
      break;
    case MSP_SUCCESS:
      in_progress = false;
      is_first_packet = false;
      if (request_ready && serial_vtx_send_data(vtx_frame, vtx_frame_length)) {
        request_ready = false;
      }
      return VTX_SUCCESS;
    }
  }

  if ((in_progress || is_first_packet) && (time_millis() - vtx_last_valid_read) > 500) {
    quic_debugf("MSP_VTX: timeout waiting for packet");
    vtx_last_valid_read = time_millis();
    return VTX_ERROR;
  }

  if (in_progress) {
    return VTX_WAIT;
  }

  return VTX_IDLE;
}