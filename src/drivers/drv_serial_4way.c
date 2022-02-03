#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "drv_gpio.h"
#include "drv_motor.h"
#include "drv_serial_4way.h"
#include "drv_serial_4way_avr_bl.h"
#include "drv_serial_soft.h"
#include "drv_usb.h"
#include "led.h"
#include "usb_configurator.h"
#include "util/cbor_helper.h"

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE

#define RX_LED_OFF ledoff(1)
#define RX_LED_ON ledon(1)
#define TX_LED_OFF ledoff(2)
#define TX_LED_ON ledon(2)

#define ATMEL_DEVICE_MATCH(device_id) ((device_id == 0x9307) || (device_id == 0x930A) || \
                                       (device_id == 0x930F) || (device_id == 0x940B))

#define SILABS_DEVICE_MATCH(device_id) ((device_id == 0xF310) || (device_id == 0xF330) || \
                                        (device_id == 0xF410) || (device_id == 0xF390) || \
                                        (device_id == 0xF850) || (device_id == 0xE8B1) || \
                                        (device_id == 0xE8B2))

#define ARM_DEVICE_MATCH(device_id) ((device_id == 0x1F06) || (device_id == 0x3306) || (device_id == 0x3406))

#define INTF_MODE_IDX 3 // index for DeviceInfostate

#define ESC_COUNT 4

#define BLHELI_SETTINGS_OFFSET 0x1A00
#define BLHELI_SETTINGS_SIZE 0x70

#define SILABS_PAGE_SIZE 0x0200

static serial_esc4way_device_t device;
static gpio_pins_t esc_pins[ESC_COUNT] = {GPIO_PIN_INVALID};

#define ESC_PIN (esc_pins[device.selected_esc])

static uint8_16_u crc_in;
static uint8_16_u crc_out;

uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data) {
  crc = crc ^ ((uint16_t)data << 8);
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }
  return crc;
}

bool device_is_connected() {
  return device.info[0] > 0 && device.info[1] > 0;
}

static void device_set_disconnected() {
  memset(device.info, 0, SERIAL_4WAY_DEVICE_INFO_SIZE);
}

static uint8_t read_byte() {
  uint8_t byte = 0;
  while (usb_serial_read(&byte, 1) == 0)
    ;
  return byte;
}

static uint8_t read_byte_crc() {
  uint8_t b = read_byte();
  crc_in.word = _crc_xmodem_update(crc_in.word, b);
  return b;
}

static void write_byte(uint8_t b) {
  usb_serial_write(&b, 1);
}

static void write_byte_crc(uint8_t b) {
  write_byte(b);
  crc_out.word = _crc_xmodem_update(crc_out.word, b);
}

static uint8_t connect_esc(gpio_pins_t pin, uint8_t *data) {
  for (uint8_t i = 0; i < 3; ++i) {
    if (!avr_bl_connect(pin, data)) {
      continue;
    }

    const uint16_t device_id = (data[1] << 8) | (uint16_t)data[0];
    if (SILABS_DEVICE_MATCH(device_id)) {
      device.mode = ESC4WAY_SIL_BLB;
      return 1;
    } else if (ATMEL_DEVICE_MATCH(device_id)) {
      device.mode = ESC4WAY_ATM_BLB;
      return 1;
    } else if (ARM_DEVICE_MATCH(device_id)) {
      device.mode = ESC4WAY_ARM_BLB;
      return 1;
    }
  }
  return 0;
}

uint8_t serial_4way_init() {
  motor_set_all(0);
  motor_wait_for_ready();

  time_delay_ms(250);

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel)     \
  esc_pins[MOTOR_PIN_IDENT(port, pin)] = PIN_IDENT(port, pin); \
  avr_bl_init_pin(PIN_IDENT(port, pin));

  MOTOR_PINS

#undef MOTOR_PIN

  time_delay_ms(500);

  return ESC_COUNT;
}

void serial_4way_release() {
  time_delay_ms(10);

  motor_init();
  motor_set_all(0);
}

serial_esc4way_ack_t serial_4way_send(uint8_t cmd, serial_esc4way_payload_t payload, uint8_t *output, uint8_t *output_len) {
  serial_esc4way_ack_t ACK_OUT = ESC4WAY_ACK_OK;

  *output_len = 1;

  switch (cmd) {
  default:
    return ESC4WAY_ACK_I_INVALID_CMD;

  case ESC4WAY_INTERFACE_TEST_ALIVE: {
    if (device_is_connected()) {
      switch (device.mode) {
      case ESC4WAY_ATM_BLB:
      case ESC4WAY_SIL_BLB:
      case ESC4WAY_ARM_BLB: {
        if (!avr_bl_send_keepalive(ESC_PIN)) {
          ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
        }
        break;
      }
      default:
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      if (ACK_OUT != ESC4WAY_ACK_OK) {
        device_set_disconnected();
      }
    }
    break;
  }

  case ESC4WAY_PROTOCOL_GET_VERSION: {
    // Only interface itself, no matter what Device
    output[0] = SERIAL_4WAY_PROTOCOL_VER;
    break;
  }

  case ESC4WAY_INTERFACE_GET_NAME: {
    // Only interface itself, no matter what Device
    *output_len = strlen(SERIAL_4WAY_INTERFACE_NAME_STR);
    memcpy(output, SERIAL_4WAY_INTERFACE_NAME_STR, strlen(SERIAL_4WAY_INTERFACE_NAME_STR));
    break;
  }

  case ESC4WAY_INTERFACE_GET_VERSION: {
    // Only interface itself, no matter what Device
    output[0] = SERIAL_4WAY_VERSION_HI;
    output[1] = SERIAL_4WAY_VERSION_LO;
    *output_len = 2;
    break;
  }

  case ESC4WAY_INTERFACE_EXIT: {
    serial_4way_release();
    break;
  }

  case ESC4WAY_INTERFACE_SET_MODE: {
    if (((payload.params[0] <= ESC4WAY_ATM_BLB) || (payload.params[0] == ESC4WAY_ARM_BLB)) && (payload.params[0] >= ESC4WAY_SIL_BLB)) {
      device.mode = payload.params[0];
    } else {
      ACK_OUT = ESC4WAY_ACK_I_INVALID_PARAM;
    }
    break;
  }

  case ESC4WAY_DEVICE_RESET: {
    if (payload.params[0] >= ESC_COUNT) {
      return ESC4WAY_ACK_I_INVALID_CHANNEL;
    }

    // Channel may change here
    device.selected_esc = payload.params[0];

    bool reboot_esc = false;
    if (payload.flash_addr_l == 1) {
      reboot_esc = true;
    }

    switch (device.mode) {
    case ESC4WAY_SIL_BLB:
    case ESC4WAY_ATM_BLB:
    case ESC4WAY_ARM_BLB: {
      avr_bl_send_restart(ESC_PIN, device.info);
      if (reboot_esc) {
        avr_bl_reboot(ESC_PIN);
      }
      break;
    }
    }
    device_set_disconnected();
    break;
  }

  case ESC4WAY_DEVICE_INIT_FLASH: {
    device_set_disconnected();

    if (payload.params[0] >= ESC_COUNT) {
      return ESC4WAY_ACK_I_INVALID_CHANNEL;
    }

    //Channel may change here
    //ESC_LO or ESC_HI; Halt state for prev channel
    device.selected_esc = payload.params[0];

    if (connect_esc(ESC_PIN, device.info)) {
      device.info[INTF_MODE_IDX] = device.mode;
    } else {
      ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      device_set_disconnected();
    }

    *output_len = SERIAL_4WAY_DEVICE_INFO_SIZE;
    memcpy(output, device.info, SERIAL_4WAY_DEVICE_INFO_SIZE);
    break;
  }

  case ESC4WAY_DEVICE_PAGE_ERASE: {
    switch (device.mode) {
    case ESC4WAY_SIL_BLB:
    case ESC4WAY_ARM_BLB: {
      uint8_t addr = output[0] = payload.params[0];

      uint16_t full_addr = 0;
      if (device.mode == ESC4WAY_ARM_BLB) {
        // Address = Page * 1024
        full_addr = (addr << 2) << 8;
      } else {
        // Address = Page * 512
        full_addr = (addr << 1) << 8;
      }

      if (!avr_bl_page_erase(ESC_PIN, full_addr)) {
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
    default:
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
    }
    break;
  }

  case ESC4WAY_DEVICE_READ: {
    const uint8_t size = payload.params[0];
    const uint16_t addr = (payload.flash_addr_h << 8) | (uint16_t)(payload.flash_addr_l);

    switch (device.mode) {
    case ESC4WAY_SIL_BLB:
    case ESC4WAY_ATM_BLB:
    case ESC4WAY_ARM_BLB: {
      if (!avr_bl_read_flash(ESC_PIN, device.mode, addr, output, size)) {
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
    default:
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
    }
    if (ACK_OUT == ESC4WAY_ACK_OK) {
      *output_len = size;
    }
    break;
  }

  case ESC4WAY_DEVICE_READ_E_EPROM: {
    const uint8_t size = payload.params[0];
    const uint16_t addr = (payload.flash_addr_h << 8) | (uint16_t)(payload.flash_addr_l);

    switch (device.mode) {
    case ESC4WAY_ATM_BLB: {
      if (!avr_bl_read_eeprom(ESC_PIN, addr, output, size)) {
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
    default:
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
    }
    if (ACK_OUT == ESC4WAY_ACK_OK) {
      *output_len = size;
    }
    break;
  }

  case ESC4WAY_DEVICE_WRITE: {
    const uint8_t size = payload.params_len;
    const uint16_t addr = (payload.flash_addr_h << 8) | (uint16_t)(payload.flash_addr_l);

    switch (device.mode) {
    case ESC4WAY_SIL_BLB:
    case ESC4WAY_ATM_BLB:
    case ESC4WAY_ARM_BLB: {
      if (!avr_bl_write_flash(ESC_PIN, addr, payload.params, size)) {
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
    }
    break;
  }

  case ESC4WAY_DEVICE_WRITE_E_EPROM: {
    const uint8_t size = payload.params_len;
    const uint16_t addr = (payload.flash_addr_h << 8) | (uint16_t)(payload.flash_addr_l);

    ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;

    switch (device.mode) {
    case ESC4WAY_SIL_BLB: {
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
      break;
    }
    case ESC4WAY_ATM_BLB: {
      if (avr_bl_write_eeprom(ESC_PIN, addr, payload.params, size)) {
        ACK_OUT = ESC4WAY_ACK_OK;
      }
      break;
    }
    }
    break;
  }

  case ESC4WAY_DEVICE_VERIFY: {
    switch (device.mode) {
    case ESC4WAY_ARM_BLB: {
      const uint8_t size = payload.params_len;
      const uint16_t addr = (payload.flash_addr_h << 8) | (uint16_t)(payload.flash_addr_l);

      switch (avr_bl_verify_flash(ESC_PIN, addr, payload.params, size)) {
      case brSUCCESS:
        ACK_OUT = ESC4WAY_ACK_OK;
        break;
      case brERRORVERIFY:
        ACK_OUT = ESC4WAY_ACK_I_VERIFY_ERROR;
        break;
      default:
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
        break;
      }
      break;
    }

    default:
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
      break;
    }
    break;
  }
  }

  return ACK_OUT;
}

#define MEMBER CBOR_ENCODE_MEMBER
#define STR_MEMBER CBOR_ENCODE_STR_MEMBER
#define TSTR_MEMBER CBOR_ENCODE_TSTR_MEMBER
#define ARRAY_MEMBER CBOR_ENCODE_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_ENCODE_STR_ARRAY_MEMBER

CBOR_START_STRUCT_ENCODER(blheli_settings_t)
BLHELI_SETTINGS_MEMBERS
CBOR_END_STRUCT_ENCODER()

#undef MEMBER
#undef STR_MEMBER
#undef TSTR_MEMBER
#undef ARRAY_MEMBER
#undef STR_ARRAY_MEMBER

#define MEMBER CBOR_DECODE_MEMBER
#define STR_MEMBER CBOR_DECODE_STR_MEMBER
#define TSTR_MEMBER CBOR_DECODE_TSTR_MEMBER
#define ARRAY_MEMBER CBOR_DECODE_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_DECODE_STR_ARRAY_MEMBER

CBOR_START_STRUCT_DECODER(blheli_settings_t)
BLHELI_SETTINGS_MEMBERS
CBOR_END_STRUCT_DECODER()

#undef MEMBER
#undef STR_MEMBER
#undef TSTR_MEMBER
#undef ARRAY_MEMBER
#undef STR_ARRAY_MEMBER

serial_esc4way_ack_t serial_4way_read_settings(blheli_settings_t *settings, uint8_t esc) {
  uint8_t input[256];
  uint8_t output[256];
  uint8_t output_len = 0;

  serial_esc4way_ack_t ack = ESC4WAY_ACK_OK;
  serial_esc4way_payload_t payload = {
      .flash_addr_h = 0,
      .flash_addr_l = 0,
      .params = input,
      .params_len = 1,
  };

  payload.params[0] = esc;
  payload.params_len = 1;

  ack = serial_4way_send(ESC4WAY_DEVICE_INIT_FLASH, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_INIT_FLASH 0x%x", ack);
    return ack;
  }
  time_delay_us(500); // give the device some time to wake up

  payload.flash_addr_h = BLHELI_SETTINGS_OFFSET >> 8;
  payload.flash_addr_l = BLHELI_SETTINGS_OFFSET & 0xFF;
  payload.params[0] = BLHELI_SETTINGS_SIZE;
  payload.params_len = 1;

  ack = serial_4way_send(ESC4WAY_DEVICE_READ, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_READ 0x%x", ack);
    return ack;
  }
  time_delay_us(200);

  {
    blheli_settings_raw_t settings_raw;
    memcpy(&settings_raw, output, output_len);

    settings->MAIN_REVISION = settings_raw.MAIN_REVISION;
    settings->SUB_REVISION = settings_raw.SUB_REVISION;
    settings->LAYOUT_REVISION = settings_raw.LAYOUT_REVISION;
    settings->MOTOR_DIRECTION = settings_raw.MOTOR_DIRECTION;

    memcpy(settings->LAYOUT, settings_raw.LAYOUT, 16);
    memcpy(settings->MCU, settings_raw.MCU, 16);
    memcpy(settings->NAME, settings_raw.NAME, 16);
  }

  payload.flash_addr_h = 0;
  payload.flash_addr_l = 0;
  payload.params[0] = esc;
  payload.params_len = 1;

  ack = serial_4way_send(ESC4WAY_DEVICE_RESET, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_RESET 0x%x", ack);
    return ack;
  }
  time_delay_us(200);

  return ack;
}

serial_esc4way_ack_t serial_4way_write_settings(blheli_settings_t *settings, uint8_t esc) {
  uint8_t input[256];
  uint8_t output[256];
  uint8_t output_len = 0;

  uint8_t ack = ESC4WAY_ACK_OK;
  serial_esc4way_payload_t payload = {
      .flash_addr_h = 0,
      .flash_addr_l = 0,
      .params = input,
      .params_len = 1,
  };

  payload.params[0] = esc;
  payload.params_len = 1;

  ack = serial_4way_send(ESC4WAY_DEVICE_INIT_FLASH, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_INIT_FLASH 0x%x", ack);
    return ack;
  }
  time_delay_us(500); // give the device some time to wake up

  payload.flash_addr_h = BLHELI_SETTINGS_OFFSET >> 8;
  payload.flash_addr_l = BLHELI_SETTINGS_OFFSET & 0xFF;
  payload.params[0] = BLHELI_SETTINGS_SIZE;
  payload.params_len = 1;

  ack = serial_4way_send(ESC4WAY_DEVICE_READ, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_READ 0x%x", ack);
    return ack;
  }
  time_delay_us(200);

  blheli_settings_raw_t settings_raw;
  {
    memcpy(&settings_raw, output, output_len);

    settings_raw.MAIN_REVISION = settings->MAIN_REVISION;
    settings_raw.SUB_REVISION = settings->SUB_REVISION;
    settings_raw.LAYOUT_REVISION = settings->LAYOUT_REVISION;
    settings_raw.MOTOR_DIRECTION = settings->MOTOR_DIRECTION;

    memcpy(settings_raw.LAYOUT, settings->LAYOUT, 16);
    memcpy(settings_raw.MCU, settings->MCU, 16);
    memcpy(settings_raw.NAME, settings->NAME, 16);
  }

  payload.params[0] = BLHELI_SETTINGS_OFFSET / SILABS_PAGE_SIZE;
  payload.params_len = 1;

  ack = serial_4way_send(ESC4WAY_DEVICE_PAGE_ERASE, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_PAGE_ERASE 0x%x", ack);
    return ack;
  }
  time_delay_us(200);

  payload.flash_addr_h = BLHELI_SETTINGS_OFFSET >> 8;
  payload.flash_addr_l = BLHELI_SETTINGS_OFFSET & 0xFF;
  memcpy(payload.params, &settings_raw, BLHELI_SETTINGS_SIZE);
  payload.params_len = BLHELI_SETTINGS_SIZE;

  ack = serial_4way_send(ESC4WAY_DEVICE_WRITE, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_WRITE 0x%x", ack);
    return ack;
  }
  time_delay_us(200);

  payload.flash_addr_h = 0;
  payload.flash_addr_l = 0;
  payload.params[0] = esc;
  payload.params_len = 1;

  ack = serial_4way_send(ESC4WAY_DEVICE_RESET, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_RESET 0x%x", ack);
    return ack;
  }
  time_delay_us(200);

  return ack;
}

// Send Structure
// ESC + CMD PARAM_LEN [PARAM (if len > 0)] CRC16_Hi CRC16_Lo
// Return
// ESC CMD PARAM_LEN [PARAM (if len > 0)] + ACK (uint8_t OK or ERR) + CRC16_Hi CRC16_Lo
void serial_4way_process() {
  uint8_t input_buffer[256];
  uint8_t output_buffer[256];
  uint8_t output_len = 0;

  RX_LED_OFF;
  TX_LED_OFF;

  while (1) {
    // restart looking for new sequence from host
    uint8_t ESC;
    do {
      RX_LED_ON;
      crc_in.word = 0;
      ESC = read_byte_crc();
      RX_LED_OFF;
    } while (ESC != ESC4WAY_LOCAL_ESCAPE);

    RX_LED_ON;

    const uint8_t CMD = read_byte_crc();

    serial_esc4way_payload_t payload;
    payload.flash_addr_h = read_byte_crc();
    payload.flash_addr_l = read_byte_crc();

    payload.params_len = read_byte_crc();

    const uint16_t size = payload.params_len == 0 ? 256 : payload.params_len;
    for (uint16_t i = 0; i < size; i++) {
      input_buffer[i] = read_byte_crc();
    }

    payload.params = input_buffer;

    uint8_16_u CRC_check;
    CRC_check.bytes[1] = read_byte();
    CRC_check.bytes[0] = read_byte();

    memset(output_buffer, 0, 256);

    RX_LED_OFF;

    serial_esc4way_ack_t ACK_OUT = ESC4WAY_ACK_OK;
    if (CRC_check.word == crc_in.word) {
      TX_LED_ON;
      ACK_OUT = serial_4way_send(CMD, payload, output_buffer, &output_len);
    } else {
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CRC;
    }

    crc_out.word = 0;

    write_byte_crc(ESC4WAY_REMOTE_ESCAPE);
    write_byte_crc(CMD);
    write_byte_crc(payload.flash_addr_h);
    write_byte_crc(payload.flash_addr_l);

    write_byte_crc(output_len);
    for (uint16_t i = 0; i < output_len; i++) {
      write_byte_crc(output_buffer[i]);
    }

    write_byte_crc(ACK_OUT);
    write_byte(crc_out.bytes[1]);
    write_byte(crc_out.bytes[0]);
    TX_LED_OFF;
    RX_LED_OFF;

    if (CMD == ESC4WAY_INTERFACE_EXIT) {
      return;
    }
  };
}

#endif