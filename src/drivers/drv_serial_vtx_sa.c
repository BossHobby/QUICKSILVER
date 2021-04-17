#include "drv_serial_vtx_sa.h"

#include <stddef.h>

#include "drv_serial.h"
#include "drv_serial_vtx.h"
#include "drv_time.h"
#include "profile.h"
#include "usb_configurator.h"
#include "util.h"
#include "util/circular_buffer.h"

#ifdef ENABLE_SMART_AUDIO

#define SMART_AUDIO_BAUDRATE_MIN 4650
#define SMART_AUDIO_BAUDRATE_MAX 5050
#define SMART_AUDIO_BUFFER_SIZE 128

#define SA_HEADER_SIZE 5

#define USART usart_port_defs[serial_smart_audio_port]

typedef enum {
  PARSER_IDLE,
  PARSER_ERROR,
  PARSER_INIT,
  PARSER_CHECK_MIRROR,
  PARSER_READ_MAGIC,
  PARSER_READ_PAYLOAD,
  PARSER_READ_CRC,
} smart_audio_parser_state_t;

smart_audio_settings_t smart_audio_settings;

static uint32_t baud_rate = 4800;
static uint32_t packets_sent = 0;
static uint32_t packets_recv = 0;

static smart_audio_parser_state_t parser_state = PARSER_IDLE;

extern uint32_t vtx_last_valid_read;
extern uint32_t vtx_last_request;

extern volatile uint8_t vtx_transfer_done;

extern volatile circular_buffer_t vtx_rx_buffer;

extern uint8_t vtx_frame[VTX_BUFFER_SIZE];
extern uint8_t volatile vtx_frame_length;
extern uint8_t volatile vtx_frame_offset;

static void serial_smart_audio_reconfigure() {
  USART_Cmd(USART.channel, DISABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_pin_init_af(&GPIO_InitStructure, USART.tx_pin, USART.gpio_af);

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = baud_rate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_2;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART.channel, &USART_InitStructure);

  USART_SetAddress(USART.channel, 0);

  USART_ClearFlag(USART.channel, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
  USART_ClearITPendingBit(USART.channel, USART_IT_RXNE | USART_IT_TC | USART_IT_TXE);

  USART_ITConfig(USART.channel, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART.channel, USART_IT_RXNE, DISABLE);
  USART_ITConfig(USART.channel, USART_IT_TC, ENABLE);

  USART_HalfDuplexCmd(USART.channel, ENABLE);
  USART_Cmd(USART.channel, ENABLE);
}

static void smart_audio_auto_baud() {
  static uint8_t last_percent = 0;

  // move quickly while we have not yet found a working baud rate
  const uint8_t current_percent = ((packets_recv * 100) / packets_sent);
  if (packets_sent < (last_percent == 0 ? 3 : 10)) {
    last_percent = current_percent;
    return;
  }

  if (current_percent < 70) {
    static int8_t direction = 1;

    // if the percentage degraded, switch it up
    if (last_percent > current_percent) {
      direction = direction == 1 ? -1 : 1;
    }

    if ((direction == 1) && (baud_rate == SMART_AUDIO_BAUDRATE_MAX)) {
      direction = -1;
    } else if ((direction == -1 && baud_rate == SMART_AUDIO_BAUDRATE_MIN)) {
      direction = 1;
    }

    baud_rate += direction * 50;
    quic_debugf("SMART_AUDIO: auto baud %d (%d) change %d vs %d", baud_rate, direction * 50, last_percent, current_percent);
    serial_smart_audio_reconfigure();
    timer_delay_us(100);
  }

  last_percent = current_percent;
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

static uint8_t serial_smart_audio_read_byte_crc(uint8_t *crc, uint8_t *data) {
  if (serial_vtx_read_byte(data) == 0) {
    return 0;
  }
  *crc = crc8(*crc, *data);
  return 1;
}

static uint8_t serial_smart_audio_parse_packet(uint8_t cmd, uint8_t *payload, uint32_t length) {
  switch (cmd) {
  case SA_CMD_GET_SETTINGS:
  case SA_CMD_GET_SETTINGS_V2:
  case SA_CMD_GET_SETTINGS_V21:
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
    return 0;
  }

  packets_recv++;
  return 1;
}

void serial_smart_audio_init(void) {
  serial_smart_audio_port = profile.serial.smart_audio;

  serial_enable_rcc(serial_smart_audio_port);
  serial_smart_audio_reconfigure();
  serial_enable_isr(serial_smart_audio_port);
}

vtx_update_result_t serial_smart_audio_update() {
  if (vtx_transfer_done == 0) {
    return VTX_WAIT;
  }
  if (parser_state > PARSER_INIT && (timer_millis() - vtx_last_valid_read) > 500) {
    quic_debugf("SMART_AUDIO: timeout waiting for packet");
    parser_state = ERROR;
    return VTX_ERROR;
  }

  static const uint8_t magic_bytes[3] = {
      0x0,
      0xaa,
      0x55,
  };

  static uint8_t mirror_offset = 0;
  static uint8_t payload_offset = 0;

  static uint8_t crc = 0;
  static uint8_t cmd = 0;
  static uint8_t length = 0;
  static uint8_t payload[SMART_AUDIO_BUFFER_SIZE];

  switch (parser_state) {
  case PARSER_ERROR:
    return VTX_ERROR;

  case PARSER_IDLE:
    return VTX_IDLE;

  case PARSER_INIT: {
    if ((timer_millis() - vtx_last_request) > 500) {
      smart_audio_auto_baud();

      mirror_offset = 0;
      payload_offset = 0;
      crc = 0;
      cmd = 0;
      length = 0;
      parser_state = PARSER_CHECK_MIRROR;

      quic_debugf("SMART_AUDIO: send cmd %d (%d)", cmd, size);
      serial_vtx_send_data(vtx_frame, vtx_frame_length);
    }

    return VTX_WAIT;
  }
  case PARSER_CHECK_MIRROR: {
    uint8_t data = 0;
    if (serial_vtx_read_byte(&data) == 0) {
      return VTX_WAIT;
    }

    quic_debugf("SMART_AUDIO: mirror 0x%x (%d)", data, mirror_offset);

    // handle optional first zero byte
    if (mirror_offset == 1 && data == 0x0) {
      return VTX_WAIT;
    }

    if (vtx_frame[mirror_offset] != data) {
      if (vtx_frame[mirror_offset + 1] == data) {
        mirror_offset++;
      } else {
        quic_debugf("SMART_AUDIO: invalid mirror (%d:0x%x)", mirror_offset, data);
        parser_state = ERROR;
        return VTX_ERROR;
      }
    }

    mirror_offset++;

    if (mirror_offset == vtx_frame_length) {
      parser_state = PARSER_READ_MAGIC;
    }
    return VTX_WAIT;
  }
  case PARSER_READ_MAGIC: {
    uint8_t data = 0;
    if (serial_vtx_read_byte(&data) == 0) {
      return VTX_WAIT;
    }

    quic_debugf("SMART_AUDIO: magic 0x%x (%d)", data, payload_offset);

    if (data != magic_bytes[payload_offset] && (payload_offset != 0 || data != 0xff)) {
      quic_debugf("SMART_AUDIO: invalid magic (%d:0x%x)", payload_offset, data);
      parser_state = ERROR;
      return VTX_ERROR;
    }
    payload_offset++;

    if (payload_offset == 3) {
      parser_state = PARSER_READ_PAYLOAD;
    }
    return VTX_WAIT;
  }
  case PARSER_READ_PAYLOAD: {
    uint8_t data = 0;
    if (serial_smart_audio_read_byte_crc(&crc, &data) == 0) {
      return VTX_WAIT;
    }

    if (payload_offset == 3) {
      cmd = data;
    }
    if (payload_offset == 4) {
      length = data;
    }
    if (payload_offset >= SA_HEADER_SIZE && (payload_offset - SA_HEADER_SIZE) < length) {
      payload[payload_offset - SA_HEADER_SIZE] = data;
    }

    quic_debugf("SMART_AUDIO: payload 0x%x (%d)", data, payload_offset);
    payload_offset++;

    // payload done, lets check crc
    if (payload_offset > SA_HEADER_SIZE && (payload_offset - SA_HEADER_SIZE) == length) {
      parser_state = PARSER_READ_CRC;
    }

    return VTX_WAIT;
  }
  case PARSER_READ_CRC: {
    uint8_t data = 0;
    if (serial_vtx_read_byte(&data) == 0) {
      return VTX_WAIT;
    }

    if (data != crc) {
      quic_debugf("SMART_AUDIO: invalid crc 0x%x vs 0x%x", crc, data);
      parser_state = ERROR;
      return VTX_ERROR;
    }

    quic_debugf("SMART_AUDIO: read cmd %d (%d)", cmd, length);
    if (serial_smart_audio_parse_packet(cmd, payload, length)) {
      parser_state = PARSER_IDLE;
      return VTX_SUCCESS;
    }

    parser_state = ERROR;
    return VTX_ERROR;
  }
  }

  // we should not reach this, something is wrong
  return VTX_ERROR;
}

void serial_smart_audio_send_payload(uint8_t cmd, const uint8_t *payload, const uint32_t size) {
  vtx_frame_length = size + 2 + SA_HEADER_SIZE;

  vtx_frame[0] = 0x00;
  vtx_frame[1] = 0xAA;
  vtx_frame[2] = 0x55;
  vtx_frame[3] = (cmd << 1) | 0x1;
  vtx_frame[4] = size;
  for (uint8_t i = 0; i < size; i++) {
    vtx_frame[i + SA_HEADER_SIZE] = payload[i];
  }
  vtx_frame[size + SA_HEADER_SIZE] = crc8_data(vtx_frame + 1, vtx_frame_length - 3);
  vtx_frame[size + 1 + SA_HEADER_SIZE] = 0x00;
  circular_buffer_clear(&vtx_rx_buffer);

  parser_state = PARSER_INIT;
}
#endif
