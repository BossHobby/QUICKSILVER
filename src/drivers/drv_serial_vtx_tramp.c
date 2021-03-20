#include "drv_serial_vtx_tramp.h"

#include <string.h>

#include <stm32f4xx_ll_usart.h>

#include "drv_serial.h"
#include "drv_serial_vtx.h"
#include "drv_time.h"
#include "profile.h"
#include "usb_configurator.h"
#include "util/circular_buffer.h"

#ifdef ENABLE_TRAMP

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

static tramp_parser_state_t parser_state = PARSER_IDLE;

extern uint32_t vtx_last_valid_read;
extern uint32_t vtx_last_request;

extern volatile uint8_t vtx_transfer_done;

extern volatile circular_buffer_t vtx_rx_buffer;

extern uint8_t vtx_frame[VTX_BUFFER_SIZE];
extern volatile uint8_t vtx_frame_length;
extern volatile uint8_t vtx_frame_offset;

static uint8_t crc8_data(const uint8_t *data) {
  uint8_t crc = 0;
  for (int i = 0; i < 13; i++) {
    crc += data[i];
  }
  return crc;
}

void serial_tramp_send_payload(uint8_t cmd, const uint16_t payload) {
  vtx_frame_length = 16;

  memset(vtx_frame, 0, vtx_frame_length);

  vtx_frame[0] = 0x0F;
  vtx_frame[1] = cmd;
  vtx_frame[2] = payload & 0xff;
  vtx_frame[3] = (payload >> 8) & 0xff;
  vtx_frame[14] = crc8_data(vtx_frame + 1);

  circular_buffer_clear(&vtx_rx_buffer);

  parser_state = PARSER_INIT;
  vtx_last_valid_read = timer_millis();
}

static void serial_tramp_reconfigure() {
  LL_USART_Disable(USART.channel);

  LL_GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_pin_init_af(&GPIO_InitStructure, USART.tx_pin, USART.gpio_af);

  LL_USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.BaudRate = 9600;
  USART_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStructure.StopBits = LL_USART_STOPBITS_1;
  USART_InitStructure.Parity = LL_USART_PARITY_NONE;
  USART_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStructure.TransferDirection = LL_USART_DIRECTION_TX | LL_USART_DIRECTION_RX;
  USART_InitStructure.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART.channel, &USART_InitStructure);

  LL_USART_ClearFlag_RXNE(USART.channel);
  LL_USART_ClearFlag_TC(USART.channel);

  LL_USART_DisableIT_TXE(USART.channel);
  LL_USART_DisableIT_RXNE(USART.channel);
  LL_USART_EnableIT_TC(USART.channel);

  LL_USART_EnableHalfDuplex(USART.channel);
  LL_USART_Enable(USART.channel);
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
  if (vtx_transfer_done == 0) {
    return VTX_WAIT;
  }
  if (parser_state > PARSER_INIT && (timer_millis() - vtx_last_valid_read) > 500) {
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
    if ((timer_millis() - vtx_last_request) > 200) {
      mirror_offset = 0;
      payload_offset = 0;
      parser_state = PARSER_CHECK_MIRROR;
      serial_vtx_send_data(vtx_frame, vtx_frame_length);
    }
    return VTX_WAIT;
  }
  case PARSER_CHECK_MIRROR: {
    uint8_t data = 0;
    if (serial_vtx_read_byte(&data) == 0) {
      return VTX_WAIT;
    }

    quic_debugf("TRAMP: mirror 0x%x (%d)", data, mirror_offset);

    if (vtx_frame[mirror_offset] != data) {
      quic_debugf("TRAMP: invalid mirror (%d:0x%x)", mirror_offset, data);
      parser_state = ERROR;
      return VTX_ERROR;
    }

    mirror_offset++;

    if (mirror_offset == vtx_frame_length) {
      if (vtx_frame[1] != 'r' && vtx_frame[1] != 'v' && vtx_frame[1] != 's') {
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
    if (serial_vtx_read_byte(&data) == 0) {
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
    if (serial_vtx_read_byte(&data) == 0) {
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

#endif