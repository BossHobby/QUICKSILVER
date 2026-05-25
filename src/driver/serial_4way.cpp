#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "core/debug.h"
#include "core/profile.h"
#include "driver/gpio.h"
#include "driver/motor.h"
#include "driver/serial_4way.h"
#include "driver/serial_4way_avr_bl.h"
#include "driver/serial_soft.h"
#include "driver/servo.h"
#include "driver/usb.h"
#include "io/led.h"

#ifdef USE_SERIAL_4WAY

#define RX_LED_OFF led_off(1)
#define RX_LED_ON led_on(1)
#define TX_LED_OFF led_off(2)
#define TX_LED_ON led_on(2)

#define ATMEL_DEVICE_MATCH(device_id) ((device_id == 0x9307) || (device_id == 0x930A) || \
                                       (device_id == 0x930F) || (device_id == 0x940B))

#define SILABS_DEVICE_MATCH(device_id) ((device_id > 0xE800) && (device_id < 0xF900))

// BLHeli_32 MCU ID hi > 0x00 and < 0x90 / lo always = 0x06
#define ARM_DEVICE_MATCH(device_id) (((device_id >> 8) > 0x00 && (device_id >> 8) < 0x90) && (device_id & 0xFF) == 0x06)

#define INTF_MODE_IDX 3 // index for DeviceInfostate

static serial_esc4way_device_t device;
static gpio_pins_t esc_pins[MOTOR_PIN_MAX] = {PIN_NONE};

#define ESC_PIN (esc_pins[device.selected_esc])

#define write_byte(b) \
  { *out_buf_ptr++ = b; }

#define write_byte_crc(crc, b)          \
  {                                     \
    write_byte(b);                      \
    *crc = _crc_xmodem_update(*crc, b); \
  }

static uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data) {
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

static uint8_t read_byte_crc(uint16_t *crc) {
  uint8_t b = read_byte();
  *crc = _crc_xmodem_update(*crc, b);
  return b;
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
  motor_set_all(MOTOR_OFF);
  motor_wait_for_ready();

#ifdef USE_SERVO
  servo_stop();
#endif

  time_delay_ms(250);

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    esc_pins[i] = PIN_NONE;
  }

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    const profile_output_t *output = &profile.outputs[i];
    const bool passthrough = output && output->target_output < MOTOR_PIN_MAX &&
                             (output->protocol == OUTPUT_PROTOCOL_DSHOT || output->protocol == OUTPUT_PROTOCOL_PWM);
    const gpio_pins_t pin = passthrough ? target.outputs[output->target_output].pin : PIN_NONE;
    esc_pins[i] = pin;
    if (pin != PIN_NONE) {
      avr_bl_init_pin(pin);
    }
  }

  return MOTOR_PIN_MAX;
}

void serial_4way_release() {
  time_delay_ms(10);

  motor_init();
  motor_set_all(MOTOR_OFF);

#ifdef USE_SERVO
  servo_init();
#endif
}

serial_esc4way_ack_t serial_4way_send(uint8_t cmd, uint16_t addr, const uint8_t *input, const uint8_t input_size, uint8_t *output, uint8_t *output_size) {
  serial_esc4way_ack_t ack_out = ESC4WAY_ACK_OK;

  *output_size = 1;

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
          ack_out = ESC4WAY_ACK_D_GENERAL_ERROR;
        }
        break;
      }
      default:
        ack_out = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      if (ack_out != ESC4WAY_ACK_OK) {
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
    *output_size = strlen(SERIAL_4WAY_INTERFACE_NAME_STR);
    memcpy(output, SERIAL_4WAY_INTERFACE_NAME_STR, strlen(SERIAL_4WAY_INTERFACE_NAME_STR));
    break;
  }

  case ESC4WAY_INTERFACE_GET_VERSION: {
    // Only interface itself, no matter what Device
    output[0] = SERIAL_4WAY_VERSION_HI;
    output[1] = SERIAL_4WAY_VERSION_LO;
    *output_size = 2;
    break;
  }

  case ESC4WAY_INTERFACE_EXIT: {
    serial_4way_release();
    break;
  }

  case ESC4WAY_INTERFACE_SET_MODE: {
    if (((input[0] <= ESC4WAY_ATM_BLB) || (input[0] == ESC4WAY_ARM_BLB)) && (input[0] >= ESC4WAY_SIL_BLB)) {
      device.mode = (serial_esc4way_mode_t)input[0];
    } else {
      ack_out = ESC4WAY_ACK_I_INVALID_PARAM;
    }
    break;
  }

  case ESC4WAY_DEVICE_RESET: {
    if (input[0] >= MOTOR_PIN_MAX) {
      return ESC4WAY_ACK_I_INVALID_CHANNEL;
    }

    // Channel may change here
    device.selected_esc = input[0];

    bool reboot_esc = false;
    if (addr) {
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
    default:
      break;
    }
    }
    device_set_disconnected();
    break;
  }

  case ESC4WAY_DEVICE_INIT_FLASH: {
    device_set_disconnected();

    if (input[0] >= MOTOR_PIN_MAX) {
      return ESC4WAY_ACK_I_INVALID_CHANNEL;
    }

    // Channel may change here
    // ESC_LO or ESC_HI; Halt state for prev channel
    device.selected_esc = input[0];

    if (connect_esc(ESC_PIN, device.info)) {
      device.info[INTF_MODE_IDX] = device.mode;
      device.signature = (device.info[1] << 8) | (uint16_t)device.info[0];
    } else {
      ack_out = ESC4WAY_ACK_D_GENERAL_ERROR;
      device_set_disconnected();
    }

    *output_size = SERIAL_4WAY_DEVICE_INFO_SIZE;
    memcpy(output, device.info, SERIAL_4WAY_DEVICE_INFO_SIZE);
    break;
  }

  case ESC4WAY_DEVICE_PAGE_ERASE: {
    switch (device.mode) {
    case ESC4WAY_SIL_BLB:
    case ESC4WAY_ARM_BLB: {
      uint8_t addr = output[0] = input[0];

      uint16_t full_addr = 0;
      if (device.mode == ESC4WAY_ARM_BLB) {
        // Address = Page * 1024
        full_addr = (addr << 2) << 8;
      } else {
        // Address = Page * 512
        full_addr = (addr << 1) << 8;
      }

      if (!avr_bl_page_erase(ESC_PIN, full_addr)) {
        ack_out = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
    default:
      ack_out = ESC4WAY_ACK_I_INVALID_CMD;
    }
    break;
  }

  case ESC4WAY_DEVICE_READ: {
    const uint8_t size = input[0];

    switch (device.mode) {
    case ESC4WAY_SIL_BLB:
    case ESC4WAY_ATM_BLB:
    case ESC4WAY_ARM_BLB: {
      if (!avr_bl_read_flash(ESC_PIN, device.mode, addr, output, size)) {
        ack_out = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
    default:
      ack_out = ESC4WAY_ACK_I_INVALID_CMD;
    }
    if (ack_out == ESC4WAY_ACK_OK) {
      *output_size = size;
    }
    break;
  }

  case ESC4WAY_DEVICE_READ_E_EPROM: {
    const uint8_t size = input[0];

    switch (device.mode) {
    case ESC4WAY_ATM_BLB: {
      if (!avr_bl_read_eeprom(ESC_PIN, addr, output, size)) {
        ack_out = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
    default:
      ack_out = ESC4WAY_ACK_I_INVALID_CMD;
    }
    if (ack_out == ESC4WAY_ACK_OK) {
      *output_size = size;
    }
    break;
  }

  case ESC4WAY_DEVICE_WRITE: {
    switch (device.mode) {
    case ESC4WAY_SIL_BLB:
    case ESC4WAY_ATM_BLB:
    case ESC4WAY_ARM_BLB: {
      if (!avr_bl_write_flash(ESC_PIN, addr, input, input_size)) {
        ack_out = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    default:
      ack_out = ESC4WAY_ACK_I_INVALID_CMD;
    }
    }
    break;
  }

  case ESC4WAY_DEVICE_WRITE_E_EPROM: {
    ack_out = ESC4WAY_ACK_D_GENERAL_ERROR;

    switch (device.mode) {
    case ESC4WAY_SIL_BLB: {
      ack_out = ESC4WAY_ACK_I_INVALID_CMD;
      break;
    }
    case ESC4WAY_ATM_BLB: {
      if (avr_bl_write_eeprom(ESC_PIN, addr, input, input_size)) {
        ack_out = ESC4WAY_ACK_OK;
      }
      break;
    }
    default:
      ack_out = ESC4WAY_ACK_I_INVALID_CMD;
    }
    break;
  }

  case ESC4WAY_DEVICE_VERIFY: {
    switch (device.mode) {
    case ESC4WAY_ARM_BLB: {
      switch (avr_bl_verify_flash(ESC_PIN, addr, input, input_size)) {
      case brSUCCESS:
        ack_out = ESC4WAY_ACK_OK;
        break;
      case brERRORVERIFY:
        ack_out = ESC4WAY_ACK_I_VERIFY_ERROR;
        break;
      default:
        ack_out = ESC4WAY_ACK_D_GENERAL_ERROR;
        break;
      }
      break;
    }

    default:
      ack_out = ESC4WAY_ACK_I_INVALID_CMD;
      break;
    }
    break;
  }
  }

  return ack_out;
}


// Send Structure
// ESC + CMD PARAM_LEN [PARAM (if len > 0)] CRC16_Hi CRC16_Lo
// Return
// ESC CMD PARAM_LEN [PARAM (if len > 0)] + ACK (uint8_t OK or ERR) + CRC16_Hi CRC16_Lo
void serial_4way_process() {
  uint8_t input_buffer[256];
  uint8_t output_buffer[256];
  uint8_t output_size = 0;

  uint16_t crc_in = 0;
  uint16_t crc_out = 0;

  RX_LED_OFF;
  TX_LED_OFF;

  while (1) {
    // restart looking for new sequence from host
    uint8_t magic;
    do {
      RX_LED_ON;
      crc_in = 0;
      magic = read_byte_crc(&crc_in);
      RX_LED_OFF;
    } while (magic != ESC4WAY_LOCAL_ESCAPE);

    RX_LED_ON;

    const uint8_t cmd = read_byte_crc(&crc_in);
    const uint16_t addr = (read_byte_crc(&crc_in) << 8) | (uint16_t)(read_byte_crc(&crc_in));

    const uint8_t size = read_byte_crc(&crc_in);
    for (uint16_t i = 0; i < (size == 0 ? 256 : size); i++) {
      input_buffer[i] = read_byte_crc(&crc_in);
    }

    const uint16_t their_crc = (uint16_t)(read_byte() << 8) | (uint16_t)(read_byte());

    memset(output_buffer, 0, 256);

    RX_LED_OFF;

    serial_esc4way_ack_t ack_out = ESC4WAY_ACK_OK;
    if (their_crc == crc_in) {
      TX_LED_ON;
      ack_out = serial_4way_send(cmd, addr, input_buffer, size, output_buffer, &output_size);
    } else {
      ack_out = ESC4WAY_ACK_I_INVALID_CRC;
    }

    crc_out = 0;

    {
      uint8_t out_buf[512];
      uint8_t *out_buf_ptr = out_buf;

      write_byte_crc(&crc_out, ESC4WAY_REMOTE_ESCAPE);
      write_byte_crc(&crc_out, cmd);
      write_byte_crc(&crc_out, addr >> 8);
      write_byte_crc(&crc_out, addr & 0xFF);

      write_byte_crc(&crc_out, output_size);
      for (uint16_t i = 0; i < (output_size == 0 ? 256 : output_size); i++) {
        write_byte_crc(&crc_out, output_buffer[i]);
      }

      write_byte_crc(&crc_out, ack_out);
      write_byte(crc_out >> 8);
      write_byte(crc_out & 0xFF);

      usb_serial_write(out_buf, out_buf_ptr - out_buf);
    }

    TX_LED_OFF;
    RX_LED_OFF;

    if (cmd == ESC4WAY_INTERFACE_EXIT) {
      return;
    }
  };
}

#endif
