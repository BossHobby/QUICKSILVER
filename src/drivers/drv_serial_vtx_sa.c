#include "drv_serial_vtx_sa.h"

#include <stddef.h>

#include "drv_serial.h"
#include "drv_serial_vtx.h"
#include "drv_time.h"
#include "io/usb_configurator.h"
#include "profile.h"
#include "util/circular_buffer.h"
#include "util/util.h"

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

extern circular_buffer_t vtx_rx_buffer;

extern uint8_t vtx_frame[VTX_BUFFER_SIZE];
extern volatile uint8_t vtx_frame_length;
extern volatile uint8_t vtx_frame_offset;

const uint8_t default_dac_power_levels[4] = {
    7,
    16,
    25,
    40,
};

static void serial_smart_audio_reconfigure() {
  serial_vtx_wait_for_ready();

  if (serial_is_soft(serial_smart_audio_port)) {
    soft_serial_init(serial_smart_audio_port, baud_rate, 2);
    return;
  }

  serial_disable_isr(serial_smart_audio_port);

  LL_GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_pin_init_af(&GPIO_InitStructure, USART.tx_pin, USART.gpio_af);

  LL_USART_InitTypeDef usart_init;
  LL_USART_StructInit(&usart_init);
  usart_init.BaudRate = baud_rate;
  usart_init.DataWidth = LL_USART_DATAWIDTH_8B;
  usart_init.StopBits = LL_USART_STOPBITS_2;
  usart_init.Parity = LL_USART_PARITY_NONE;
  usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  usart_init.TransferDirection = LL_USART_DIRECTION_TX_RX;
  usart_init.OverSampling = LL_USART_OVERSAMPLING_16;
  serial_port_init(serial_smart_audio_port, &usart_init, true, false);

  LL_USART_EnableIT_RXNE(USART.channel);
  LL_USART_EnableIT_TC(USART.channel);

  serial_enable_isr(serial_smart_audio_port);
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

    if (cmd == SA_CMD_GET_SETTINGS_V21) {
      smart_audio_settings.power = payload[5];

      for (uint8_t i = 0; i < 4; i++) {
        smart_audio_settings.dac_power_levels[i] = payload[7 + i];
      }
    } else {
      for (uint8_t i = 0; i < 4; i++) {
        smart_audio_settings.dac_power_levels[i] = default_dac_power_levels[i];
      }
    }
    break;

  case SA_CMD_SET_FREQUENCY:
    smart_audio_settings.frequency = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
    break;

  case SA_CMD_SET_CHANNEL:
    smart_audio_settings.channel = payload[0];
    break;

  case SA_CMD_SET_POWER: {
    smart_audio_settings.power = payload[0];
    break;
  }
  case SA_CMD_SET_MODE: {
    const uint8_t mode = payload[0];

    // in-range pitmode
    smart_audio_settings.mode |= ((mode >> 0) & 0x1) << 2;

    // out-range pitmode
    smart_audio_settings.mode |= ((mode >> 1) & 0x1) << 3;

    // pit mode runnig
    smart_audio_settings.mode |= ((mode >> 2) & 0x1) << 1;

    // locked bit
    smart_audio_settings.mode |= ((mode >> 3) & 0x1) << 4;
    break;
  }
  default:
    quic_debugf("SMART_AUDIO: invalid cmd %d (%d)", cmd, length);
    return 0;
  }

  packets_recv++;
  return 1;
}

void serial_smart_audio_init() {
  serial_smart_audio_port = profile.serial.smart_audio;

  serial_enable_rcc(serial_smart_audio_port);
  serial_smart_audio_reconfigure();
}

vtx_update_result_t serial_smart_audio_update() {
  if (vtx_transfer_done == 0) {
    return VTX_WAIT;
  }
  if (parser_state > PARSER_INIT && (time_millis() - vtx_last_valid_read) > 500) {
    quic_debugf("SMART_AUDIO: timeout waiting for packet");
    parser_state = ERROR;
    return VTX_ERROR;
  }

  static const uint8_t magic_bytes[2] = {
      0xaa,
      0x55,
  };

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
    if ((time_millis() - vtx_last_request) > 200) {
      for (uint32_t i = 0; i < vtx_frame_length; i++) {
        quic_debugf("SMART_AUDIO: sending  0x%x (%d)", vtx_frame[i], i);
      }

      payload_offset = 0;
      crc = 0;
      cmd = 0;
      length = 0;
      parser_state = PARSER_READ_MAGIC;

      serial_vtx_send_data(vtx_frame, vtx_frame_length);
      packets_sent++;
    }

    return VTX_WAIT;
  }
  case PARSER_READ_MAGIC: {
    uint8_t data = 0;
    if (serial_vtx_read_byte(&data) == 0) {
      return VTX_WAIT;
    }

    quic_debugf("SMART_AUDIO: magic 0x%x (%d)", data, payload_offset);

    // ignore the first magic byte, because of the framing issues it will be garbage half ot the time
    if (data != magic_bytes[payload_offset] && payload_offset != 0) {
      quic_debugf("SMART_AUDIO: invalid magic (%d:0x%x)", payload_offset, data);
      parser_state = ERROR;
      return VTX_ERROR;
    }
    if (data == magic_bytes[payload_offset]) {
      // only increment if we actually matched
      payload_offset++;
    }

    if (payload_offset == 2) {
      // account for first (skipped) byte;
      payload_offset++;
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
    if (payload_offset >= SA_HEADER_SIZE && (payload_offset - SA_HEADER_SIZE) == length) {
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
  if (!serial_vtx_is_ready()) {
    return;
  }

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

  smart_audio_auto_baud();

  parser_state = PARSER_INIT;
  vtx_last_valid_read = time_millis();
}

uint8_t smart_audio_dac_power_level_index(uint8_t dac) {
  for (uint8_t level = 0; level < 4; level++) {
    if (dac == smart_audio_settings.dac_power_levels[level]) {
      return level;
    }
  }
  return 0;
}

#endif
