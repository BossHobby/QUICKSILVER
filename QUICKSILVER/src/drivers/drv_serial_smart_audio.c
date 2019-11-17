#include "drv_serial_smart_audio.h"

#include <stddef.h>

#include "drv_serial.h"
#include "drv_time.h"
#include "profile.h"
#include "usb_configurator.h"

#ifdef ENABLE_SMART_AUDIO

#define SMART_AUDIO_BAUDRATE 4800
#define port_def usart_port_defs[profile.serial.smart_audio]

smart_audio_settings_t smart_audio_settings;

void serial_smart_audio_enter_tx() {
  uint32_t tmp = port_def.channel->CR1;
  tmp &= ~(USART_CR1_TE | USART_CR1_RE);
  tmp |= USART_CR1_TE;
  port_def.channel->CR1 = tmp;
}

void serial_smart_audio_enter_rx() {
  uint32_t tmp = port_def.channel->CR1;
  tmp &= ~(USART_CR1_TE | USART_CR1_RE);
  tmp |= USART_CR1_RE;
  port_def.channel->CR1 = tmp;
}

void serial_smart_audio_init(void) {
  USART_Cmd(port_def.channel, DISABLE);

  serial_enable_rcc(profile.serial.smart_audio);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = port_def.tx_pin;
  GPIO_Init(port_def.gpio_port, &GPIO_InitStructure);

  GPIO_PinAFConfig(port_def.gpio_port, port_def.tx_pin_source, port_def.gpio_af);

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = SMART_AUDIO_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_2;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(port_def.channel, &USART_InitStructure);

  USART_HalfDuplexCmd(port_def.channel, ENABLE);

  USART_Cmd(port_def.channel, ENABLE);
}

#define POLYGEN 0xd5
static uint8_t crc8(uint8_t crc, const uint8_t input) {
  crc ^= input; /* XOR-in the next input byte */

  for (int i = 0; i < 8; i++) {
    if ((crc & 0x80) != 0) {
      crc = (uint8_t)((crc << 1) ^ POLYGEN);
    } else {
      crc <<= 1;
    }
  }

  return crc;
}

static uint8_t crc8_data(const uint8_t *data, const int8_t len) {
  uint8_t crc = 0; /* start with 0 so first byte can be 'xored' in */
  for (int i = 0; i < len; i++) {
    crc = crc8(crc, data[i]);
  }
  return crc;
}

void serial_smart_audio_send_data(uint8_t *data, uint32_t size) {
  for (uint32_t i = 0; i < size; i++) {
    while (USART_GetFlagStatus(port_def.channel, USART_FLAG_TXE) == RESET)
      ;
    USART_SendData(port_def.channel, data[i]);
  }
}

uint8_t serial_smart_audio_read_byte() {
  while (USART_GetFlagStatus(port_def.channel, USART_FLAG_RXNE) == RESET)
    ;
  return USART_ReceiveData(port_def.channel);
}

uint8_t serial_smart_audio_read_byte_crc(uint8_t *crc) {
  const uint8_t data = serial_smart_audio_read_byte();
  *crc = crc8(*crc, data);
  return data;
}

void serial_smart_audio_read_packet() {
  for (uint32_t i = 0; i < 10; i++) {
    if (serial_smart_audio_read_byte() == 0xaa) {
      break;
    }
  }

  if (serial_smart_audio_read_byte() != 0x55) {
    return;
  }

  uint8_t crc = 0;
  uint8_t cmd = serial_smart_audio_read_byte_crc(&crc);
  uint8_t length = serial_smart_audio_read_byte_crc(&crc);

  uint8_t payload[length];
  for (uint8_t i = 0; i < length; i++) {
    payload[i] = serial_smart_audio_read_byte_crc(&crc);
  }

  if (cmd != 0x01 && cmd != 0x09) {
    uint8_t crc_input = serial_smart_audio_read_byte_crc(&crc);
    if (crc != crc_input) {
      quic_debugf("SMART: invalid crc 0x%x vs 0x%x", crc, crc_input);
      return;
    }
  }

  switch (cmd) {
  case 0x01:
  case 0x09:
    if (crc == payload[5]) {
      quic_debugf("SMART: invalid crc 0x%x vs 0x%x", crc, payload[4]);
      return;
    }

    smart_audio_settings.version = (cmd == 0x09 ? 2 : 1);
    smart_audio_settings.channel = payload[0];
    smart_audio_settings.power = payload[1];
    smart_audio_settings.mode = payload[2];
    smart_audio_settings.frequency = (uint16_t)(((uint16_t)payload[3] << 8) | payload[4]);
    break;

  case SA_CMD_SET_FREQUENCY:
    smart_audio_settings.frequency = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
    break;

  case SA_CMD_SET_CHANNEL:
    smart_audio_settings.channel = payload[0];
    break;

  default: {
    break;
  }
  }
}

void serial_smart_audio_send_payload(uint8_t cmd, const uint8_t *payload, const uint32_t size) {
  uint8_t frame_length = 4 + size + 1;
  uint8_t frame[frame_length];

  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = (cmd << 1) | 0x1;
  frame[3] = size;
  for (uint8_t i = 0; i < size; i++) {
    frame[i + 4] = payload[i];
  }
  frame[size + 4] = crc8_data(frame, frame_length - 1);

  serial_smart_audio_send_data(frame, frame_length);

  delay(100);
  serial_smart_audio_read_packet();
}
#endif