#include "driver/serial_vtx_tramp.h"

#include <string.h>

#include "core/debug.h"
#include "core/profile.h"
#include "driver/serial.h"
#include "driver/serial_vtx.h"
#include "driver/time.h"
#include "util/ring_buffer.h"

#define USART usart_port_defs[serial_smart_audio_port]

typedef enum {
  PARSER_IDLE,
  PARSER_ERROR,
  PARSER_INIT,
  PARSER_READ_MAGIC,
  PARSER_READ_PAYLOAD,
  PARSER_READ_CRC,
  PRASER_WAIT_FOR_READY
} tramp_parser_state_t;

tramp_settings_t tramp_settings;

static tramp_parser_state_t parser_state = PARSER_IDLE;

extern uint32_t vtx_last_valid_read;
extern uint32_t vtx_last_request;

extern volatile uint8_t vtx_transfer_done;

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

static void serial_tramp_reconfigure() {
  serial_vtx_wait_for_ready();

  if (serial_is_soft(serial_smart_audio_port)) {
    soft_serial_init(serial_smart_audio_port, 9600, 1);
    return;
  }

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

void serial_tramp_init() {
  serial_smart_audio_port = profile.serial.smart_audio;
  if (!serial_is_soft(serial_smart_audio_port)) {
    serial_enable_rcc(serial_smart_audio_port);
  }
  serial_tramp_reconfigure();
}

static bool tramp_is_query(uint8_t cmd) {
  switch (cmd) {
  case 'r':
  case 'v':
  case 's':
    return true;
  }
  return false;
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
    tramp_settings.temp = payload[5] | (payload[6] << 8);
    break;
  }

  return 1;
}

vtx_update_result_t serial_tramp_update() {
  if (vtx_transfer_done == 0) {
    return VTX_WAIT;
  }
  if (parser_state > PARSER_INIT && (time_millis() - vtx_last_valid_read) > 500) {
    quic_debugf("TRAMP: timeout waiting for packet");
    parser_state = ERROR;
    return VTX_ERROR;
  }

  static uint8_t payload[32];
  static uint8_t payload_offset = 0;

  switch (parser_state) {
  case PARSER_ERROR:
    return VTX_ERROR;

  case PARSER_IDLE:
    return VTX_IDLE;

  case PARSER_INIT: {
    if ((time_millis() - vtx_last_request) < 200) {
      return VTX_WAIT;
    }

    payload_offset = 0;
    if (!serial_vtx_send_data(vtx_frame, vtx_frame_length)) {
      return VTX_WAIT;
    }

    for (uint32_t i = 0; i < vtx_frame_length; i++) {
      quic_debugf("TRAMP: sent  0x%x (%d)", vtx_frame[i], i);
    }

    if (tramp_is_query(vtx_frame[1])) {
      parser_state = PARSER_READ_MAGIC;
      return VTX_WAIT;
    } else {
      parser_state = PRASER_WAIT_FOR_READY;
      return VTX_WAIT;
    }
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

    parser_state = PRASER_WAIT_FOR_READY;
    return VTX_WAIT;
  }
  case PRASER_WAIT_FOR_READY: {
    if (vtx_transfer_done == 1) {
      parser_state = PARSER_IDLE;
      return VTX_SUCCESS;
    }
    return VTX_WAIT;
  }
  }

  return VTX_ERROR;
}

void serial_tramp_send_payload(uint8_t cmd, const uint16_t payload) {
  if (!serial_vtx_is_ready()) {
    return;
  }

  vtx_frame_length = 16;

  memset(vtx_frame, 0, vtx_frame_length);

  vtx_frame[0] = 0x0F;
  vtx_frame[1] = cmd;
  vtx_frame[2] = payload & 0xff;
  vtx_frame[3] = (payload >> 8) & 0xff;
  vtx_frame[14] = crc8_data(vtx_frame + 1);

  parser_state = PARSER_INIT;
  vtx_last_valid_read = time_millis();
}