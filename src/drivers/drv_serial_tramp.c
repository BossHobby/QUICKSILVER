#include "drv_serial_tramp.h"

#include <string.h>

#include "drv_serial.h"
#include "drv_time.h"
#include "profile.h"
#include "usb_configurator.h"
#include "util/circular_buffer.h"

#ifdef ENABLE_TRAMP

#define TRAMP_BUFFER_SIZE 128

#define USART usart_port_defs[serial_smart_audio_port]

typedef enum {
  PARSER_IDLE,
  PARSER_ERROR,
  PARSER_INIT,
  PARSER_CHECK_MIRROR,
  PARSER_READ_MAGIC,
  PARSER_READ_PAYLOAD,
  PARSER_READ_CRC,
} tramp_parser_state_t;

tramp_settings_t tramp_settings;

static uint8_t tramp_rx_data[TRAMP_BUFFER_SIZE];
static volatile circular_buffer_t tramp_rx_buffer = {
    .buffer = tramp_rx_data,
    .head = 0,
    .tail = 0,
    .size = TRAMP_BUFFER_SIZE,
};

static volatile uint8_t transfer_done = 1;

static tramp_parser_state_t parser_state = PARSER_IDLE;
static uint32_t last_valid_read = 0;
static uint32_t last_request = 0;

static uint8_t frame[TRAMP_BUFFER_SIZE];
static uint8_t volatile frame_length = 0;
static uint8_t volatile frame_offset = 0;

static uint8_t crc8_data(const uint8_t *data) {
  uint8_t crc = 0;
  for (int i = 0; i < 13; i++) {
    crc += data[i];
  }
  return crc;
}

static void serial_tramp_send_data(uint8_t *data, uint32_t size) {
  while (transfer_done == 0)
    __WFI();
  transfer_done = 0;

  USART_ClearITPendingBit(USART.channel, USART_IT_RXNE);
  USART_ClearITPendingBit(USART.channel, USART_IT_TXE);
  USART_ClearITPendingBit(USART.channel, USART_IT_TC);

  frame_offset = 0;

  USART_ITConfig(USART.channel, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART.channel, USART_IT_TXE, ENABLE);

  last_request = timer_millis();
}

static uint8_t serial_tramp_read_byte(uint8_t *data) {
  if (circular_buffer_read(&tramp_rx_buffer, data) == 1) {
    last_valid_read = timer_millis();
    return 1;
  }
  return 0;
}

void serial_tramp_send_payload(uint8_t cmd, const uint16_t payload) {
  frame_length = 16;

  memset(frame, 0, frame_length);

  frame[0] = 0x0F;
  frame[1] = cmd;
  frame[2] = payload & 0xff;
  frame[3] = (payload >> 8) & 0xff;
  frame[14] = crc8_data(frame + 1);

  circular_buffer_clear(&tramp_rx_buffer);

  parser_state = PARSER_INIT;
  last_valid_read = timer_millis();
}

static void serial_tramp_reconfigure() {
  USART_Cmd(USART.channel, DISABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_pin_init_af(&GPIO_InitStructure, USART.tx_pin, USART.gpio_af);

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART.channel, &USART_InitStructure);

  USART_ClearFlag(USART.channel, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
  USART_ClearITPendingBit(USART.channel, USART_IT_RXNE | USART_IT_TC | USART_IT_TXE);

  USART_ITConfig(USART.channel, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART.channel, USART_IT_RXNE, DISABLE);
  USART_ITConfig(USART.channel, USART_IT_TC, ENABLE);

  USART_HalfDuplexCmd(USART.channel, ENABLE);
  USART_Cmd(USART.channel, ENABLE);
}

void serial_tramp_init() {
  serial_smart_audio_port = profile.serial.smart_audio;

  serial_enable_rcc(serial_smart_audio_port);
  serial_tramp_reconfigure();
  serial_enable_isr(serial_smart_audio_port);
}

static uint8_t tramp_parse_packet(uint8_t *payload) {
  switch (payload[0]) {
  case 'r':
    tramp_settings.freq_min = payload[1] | (payload[2] << 8);
    tramp_settings.freq_max = payload[3] | (payload[4] << 8);
    tramp_settings.power_max = payload[5] | (payload[6] << 8);
    break;

  case 'v':
    tramp_settings.frequency = payload[1] | (payload[2] << 8);
    tramp_settings.power = payload[3] | (payload[4] << 8);
    tramp_settings.control_mode = payload[5];
    tramp_settings.pit_mode = payload[6];
    tramp_settings.current_power = payload[7] | (payload[8] << 8);
    break;

  case 's':
    tramp_settings.temp = payload[1] | (payload[2] << 8);
    break;
  }

  return 1;
}

vtx_update_result_t serial_tramp_update() {
  if (transfer_done == 0) {
    return VTX_WAIT;
  }
  if (parser_state > PARSER_INIT && (timer_millis() - last_valid_read) > 500) {
    quic_debugf("TRAMP: timeout waiting for packet");
    parser_state = ERROR;
    return VTX_ERROR;
  }

  static uint8_t mirror_offset = 0;
  static uint8_t payload_offset = 0;

  static uint8_t payload[32];

  switch (parser_state) {
  case PARSER_ERROR:
    return VTX_ERROR;

  case PARSER_IDLE:
    return VTX_IDLE;

  case PARSER_INIT: {
    if ((timer_millis() - last_request) > 500) {
      mirror_offset = 0;
      payload_offset = 0;
      parser_state = PARSER_CHECK_MIRROR;
      serial_tramp_send_data(frame, frame_length);
    }
    return VTX_WAIT;
  }
  case PARSER_CHECK_MIRROR: {
    uint8_t data = 0;
    if (serial_tramp_read_byte(&data) == 0) {
      return VTX_WAIT;
    }

    quic_debugf("TRAMP: mirror 0x%x (%d)", data, mirror_offset);

    if (frame[mirror_offset] != data) {
      quic_debugf("TRAMP: invalid mirror (%d:0x%x)", mirror_offset, data);
      parser_state = ERROR;
      return VTX_ERROR;
    }

    mirror_offset++;

    if (mirror_offset == frame_length) {
      if (frame[1] != 'r' && frame[1] != 'v' && frame[1] != 's') {
        // param is set, this is not a query but a command
        // we are done here, no response will follow
        parser_state = PARSER_IDLE;
        return VTX_SUCCESS;
      }
      parser_state = PARSER_READ_MAGIC;
    }
    return VTX_WAIT;
  }
  case PARSER_READ_MAGIC: {
    uint8_t data = 0;
    if (serial_tramp_read_byte(&data) == 0) {
      return VTX_WAIT;
    }

    quic_debugf("TRAMP: magic 0x%x (%d)", data, payload_offset);

    if (data != 0x0F) {
      quic_debugf("TRAMP: invalid magic (%d:0x%x)", payload_offset, data);
      parser_state = ERROR;
      return VTX_ERROR;
    }
    payload_offset++;

    if (payload_offset == 1) {
      parser_state = PARSER_READ_PAYLOAD;
    }
    return VTX_WAIT;
  }
  case PARSER_READ_PAYLOAD: {
    uint8_t data = 0;
    if (serial_tramp_read_byte(&data) == 0) {
      return VTX_WAIT;
    }

    payload[payload_offset - 1] = data;
    quic_debugf("TRAMP: payload 0x%x (%d)", data, payload_offset);
    payload_offset++;

    // payload done, lets check crc
    if (payload_offset == 16) {
      parser_state = PARSER_READ_CRC;
    }

    return VTX_WAIT;
  }
  case PARSER_READ_CRC: {
    uint8_t crc = crc8_data(payload);

    if (payload[13] != crc || payload[14] != 0) {
      quic_debugf("TRAMP: invalid crc 0x%x vs 0x%x", crc, payload[13]);
      parser_state = ERROR;
      return VTX_ERROR;
    }

    if (!tramp_parse_packet(payload)) {
      parser_state = ERROR;
      return VTX_ERROR;
    }

    parser_state = PARSER_IDLE;
    return VTX_SUCCESS;
  }
  }

  return VTX_ERROR;
}

void tramp_uart_isr(void) {
  if (USART_GetITStatus(USART.channel, USART_IT_TC) != RESET) {
    USART_ClearITPendingBit(USART.channel, USART_IT_TC);
    if (frame_offset == frame_length && transfer_done == 0) {
      transfer_done = 1;
      USART_ITConfig(USART.channel, USART_IT_TXE, DISABLE);
    }
  }

  if (USART_GetITStatus(USART.channel, USART_IT_TXE) != RESET) {
    USART_ClearITPendingBit(USART.channel, USART_IT_TXE);
    if (frame_offset < frame_length) {
      USART_SendData(USART.channel, frame[frame_offset]);
      frame_offset++;
      transfer_done = 0;
    }
  }

  if (USART_GetITStatus(USART.channel, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(USART.channel, USART_IT_RXNE);
    const uint8_t data = USART_ReceiveData(USART.channel);
    circular_buffer_write(&tramp_rx_buffer, data);
  }

  if (USART_GetFlagStatus(USART.channel, USART_FLAG_ORE)) {
    USART_ClearFlag(USART.channel, USART_FLAG_ORE);
  }
}

#endif