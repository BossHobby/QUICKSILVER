#include "drv_serial_smart_audio.h"

#include <stddef.h>

#include "drv_serial.h"
#include "drv_time.h"
#include "profile.h"
#include "usb_configurator.h"
#include "util.h"

#ifdef ENABLE_SMART_AUDIO

#define SMART_AUDIO_BAUDRATE_MIN 4650
#define SMART_AUDIO_BAUDRATE_MAX 4950
#define SMART_AUDIO_BUFFER_SIZE 512

#define USART usart_port_defs[serial_smart_audio_port]

smart_audio_settings_t smart_audio_settings;

static volatile uint8_t smart_audio_rx_data[SMART_AUDIO_BUFFER_SIZE];
static volatile circular_buffer_t smart_audio_rx_buffer = {
    .buffer = smart_audio_rx_data,
    .head = 0,
    .tail = 0,
    .size = SMART_AUDIO_BUFFER_SIZE,
};

static volatile uint8_t transfer_done = 0;
static uint32_t baud_rate = 4800;
static uint32_t packets_sent = 0;
static uint32_t packets_recv = 0;

void serial_smart_audio_reconfigure() {
  USART_Cmd(USART.channel, DISABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = USART.tx_pin;
  GPIO_Init(USART.gpio_port, &GPIO_InitStructure);
  GPIO_PinAFConfig(USART.gpio_port, USART.tx_pin_source, USART.gpio_af);

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = baud_rate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_2;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART.channel, &USART_InitStructure);

  USART_HalfDuplexCmd(USART.channel, ENABLE);
  USART_ClearFlag(USART.channel, USART_FLAG_RXNE | USART_FLAG_TC);
  USART_ClearITPendingBit(USART.channel, USART_IT_RXNE | USART_IT_TC);
  USART_ITConfig(USART.channel, USART_IT_TC, ENABLE);
  USART_Cmd(USART.channel, ENABLE);
}

void serial_smart_audio_init(void) {
  serial_smart_audio_port = profile.serial.smart_audio;

  serial_enable_rcc(serial_smart_audio_port);
  serial_smart_audio_reconfigure();
  serial_enable_isr(serial_smart_audio_port);
}

void smart_audio_uart_isr(void) {
  if (USART_GetITStatus(USART.channel, USART_IT_TC) != RESET) {
    USART_ClearITPendingBit(USART.channel, USART_IT_TC);
    USART_ITConfig(USART.channel, USART_IT_RXNE, ENABLE);
    transfer_done = 1;
  }

  if (USART_GetITStatus(USART.channel, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(USART.channel, USART_IT_RXNE);
    circular_buffer_write(&smart_audio_rx_buffer, USART_ReceiveData(USART.channel));
    packets_recv++;
  }

  if (USART_GetFlagStatus(USART.channel, USART_FLAG_ORE)) {
    USART_ClearFlag(USART.channel, USART_FLAG_ORE);
  }
}

void smart_audio_auto_baud() {
  if (packets_sent < 30) {
    return;
  }

  if (((packets_recv * 100) / packets_sent) < 70) {
    static int8_t direction = 1;
    if ((direction == 1) && (baud_rate == SMART_AUDIO_BAUDRATE_MAX)) {
      direction = -1;
    } else if ((direction == -1 && baud_rate == SMART_AUDIO_BAUDRATE_MIN)) {
      direction = 1;
    }

    baud_rate += direction * 50;
    quic_debugf("SMART_AUDIO: auto baud %d (%d)", baud_rate, direction * 50);
    serial_smart_audio_reconfigure();

    extern int lastlooptime;
    lastlooptime = gettime();
  }

  packets_sent = 0;
  packets_recv = 0;
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
  while (transfer_done == 0)
    __WFI();
  transfer_done = 0;

  for (uint32_t i = 0; i < size; i++) {
    for (uint32_t timeout = 0x1000; USART_GetFlagStatus(USART.channel, USART_FLAG_TXE) == RESET; timeout--) {
      if (timeout == 0) {
        quic_debugf("SMART_AUDIO: send timeout");
        return;
      }
      debug_timer_delay_us(1);
      __WFI();
    }
    quic_debugf("SMART_AUDIO: write 0x%x", data[i]);
    USART_SendData(USART.channel, data[i]);
    packets_sent++;
  }
}

uint8_t serial_smart_audio_read_byte() {
  uint8_t data = 0;
  for (uint32_t timeout = 100; circular_buffer_read(&smart_audio_rx_buffer, &data) == 0; timeout--) {
    if (timeout == 0) {
      quic_debugf("SMART_AUDIO: read timeout");
      return 0;
    }
    __WFI();
  }
  quic_debugf("SMART_AUDIO: read 0x%x", data);
  return data;
}

uint8_t serial_smart_audio_read_byte_crc(uint8_t *crc) {
  const uint8_t data = serial_smart_audio_read_byte();
  *crc = crc8(*crc, data);
  return data;
}

uint8_t serial_smart_audio_read_packet() {
  while (transfer_done == 0)
    __WFI();

  for (uint8_t tries = 0;; tries++) {
    if (tries >= 3) {
      quic_debugf("SMART_AUDIO: invalid first magic");
      return 0;
    }
    if (serial_smart_audio_read_byte() == 0xaa) {
      break;
    }
  }

  if (serial_smart_audio_read_byte() != 0x55) {
    quic_debugf("SMART_AUDIO: invalid second magic");
    return 0;
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
      quic_debugf("SMART_AUDIO: invalid crc 0x%x vs 0x%x", crc, crc_input);
      return 0;
    }
  }

  switch (cmd) {
  case SA_CMD_GET_SETTINGS:
  case SA_CMD_GET_SETTINGS_V2:
  case SA_CMD_GET_SETTINGS_V21:
    if (crc == payload[5]) {
      quic_debugf("SMART_AUDIO: invalid crc 0x%x vs 0x%x", crc, payload[4]);
      return 0;
    }

    smart_audio_settings.version = (cmd == SA_CMD_GET_SETTINGS ? 1 : (cmd == SA_CMD_GET_SETTINGS_V2 ? 2 : 3));
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

  case SA_CMD_SET_POWER: {
    // workaround for buggy vtxes which swap the channel and the power arguments
    const uint8_t weird_channel = (smart_audio_settings.channel / 8) * 10 + smart_audio_settings.channel % 8;
    if (payload[0] == smart_audio_settings.channel || payload[0] == weird_channel) {
      smart_audio_settings.power = payload[1];
    } else if (payload[1] == 1) {
      smart_audio_settings.power = payload[0];
    }
    break;
  }
  case SA_CMD_SET_MODE:
    smart_audio_settings.mode = payload[0];
    break;

  default:
    quic_debugf("SMART_AUDIO: invalid cmd %d (%d)", cmd, length);
    break;
  }

  return 1;
}

uint8_t serial_smart_audio_send_payload(uint8_t cmd, const uint8_t *payload, const uint32_t size) {
  uint8_t frame_length = size + 1 + 5;
  uint8_t frame[frame_length];

  frame[0] = 0x00;
  frame[1] = 0xAA;
  frame[2] = 0x55;
  frame[3] = (cmd << 1) | 0x1;
  frame[4] = size;
  for (uint8_t i = 0; i < size; i++) {
    frame[i + 5] = payload[i];
  }
  frame[size + 5] = crc8_data(frame, frame_length - 1);

  for (uint8_t tries = 0; tries < 3; tries++) {
    smart_audio_auto_baud();

    quic_debugf("SMART_AUDIO: send cmd %d (%d)", cmd, size);
    USART_ITConfig(USART.channel, USART_IT_RXNE, DISABLE);

    serial_smart_audio_send_data(frame, frame_length);

    if (serial_smart_audio_read_packet()) {
      return 1;
    }
    debug_timer_delay_us(100);
  }

  return 0;
}
#endif