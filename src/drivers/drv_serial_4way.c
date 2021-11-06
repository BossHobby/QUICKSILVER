/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 * Author: 4712
*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "drv_gpio.h"
#include "drv_motor.h"
#include "drv_serial_4way.h"
#include "drv_serial_soft.h"
#include "drv_usb.h"
#include "usb_configurator.h"
#include "util/cbor_helper.h"

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE

#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#include "drv_serial_4way_avrootloader.h"
#endif

#if defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
#include "drv_serial_4way_stk500v2.h"
#endif

#define USE_TXRX_LED

#ifdef USE_TXRX_LED
#include "led.h"
#define RX_LED_OFF ledoff(1)
#define RX_LED_ON ledon(1)
#define TX_LED_OFF ledoff(2)
#define TX_LED_ON ledon(2)
#else
#define RX_LED_OFF
#define RX_LED_ON
#define TX_LED_OFF
#define TX_LED_ON
#endif

#define SERIAL_4WAY_INTERFACE_NAME_STR "m4wFCIntf"

// *** change to adapt Revision
#define SERIAL_4WAY_VER_MAIN 20
#define SERIAL_4WAY_VER_SUB_1 (uint8_t)0
#define SERIAL_4WAY_VER_SUB_2 (uint8_t)04

#define SERIAL_4WAY_PROTOCOL_VER 108
// *** end

#if (SERIAL_4WAY_VER_MAIN > 24)
#error "beware of SERIAL_4WAY_VER_SUB_1 is uint8_t"
#endif

#define SERIAL_4WAY_VERSION (uint16_t)((SERIAL_4WAY_VER_MAIN * 1000) + (SERIAL_4WAY_VER_SUB_1 * 100) + SERIAL_4WAY_VER_SUB_2)

#define SERIAL_4WAY_VERSION_HI (uint8_t)(SERIAL_4WAY_VERSION / 100)
#define SERIAL_4WAY_VERSION_LO (uint8_t)(SERIAL_4WAY_VERSION % 100)

#define ATMEL_DEVICE_MATCH ((pDeviceInfo->words[0] == 0x9307) || (pDeviceInfo->words[0] == 0x930A) || \
                            (pDeviceInfo->words[0] == 0x930F) || (pDeviceInfo->words[0] == 0x940B))

#define SILABS_DEVICE_MATCH ((pDeviceInfo->words[0] == 0xF310) || (pDeviceInfo->words[0] == 0xF330) || \
                             (pDeviceInfo->words[0] == 0xF410) || (pDeviceInfo->words[0] == 0xF390) || \
                             (pDeviceInfo->words[0] == 0xF850) || (pDeviceInfo->words[0] == 0xE8B1) || \
                             (pDeviceInfo->words[0] == 0xE8B2))

#define ARM_DEVICE_MATCH ((pDeviceInfo->words[0] == 0x1F06) || \
                          (pDeviceInfo->words[0] == 0x3306) || (pDeviceInfo->words[0] == 0x3406))

#define DEVICE_INFO_SIZE 4
#define SET_DISCONNECTED device_info.words[0] = 0

#define INTF_MODE_IDX 3 // index for DeviceInfostate

#define ESC_COUNT 4

#define BLHELI_SETTINGS_OFFSET 0x1A00
#define BLHELI_SETTINGS_SIZE 0x70

#define SILABS_PAGE_SIZE 0x0200

uint8_t selected_esc;

static gpio_pins_t esc_pins[ESC_COUNT] = {GPIO_PIN_INVALID};

static uint8_32_u device_info;
static uint8_16_u crc_in;
static uint8_16_u crc_out;
static uint8_t interface_mode;

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

bool is_mcu_connected(void) {
  return (device_info.bytes[0] > 0);
}

bool is_esc_high(uint8_t esc) {
  return gpio_pin_read(esc_pins[esc]) > 0;
}

bool is_esc_low(uint8_t esc) {
  return !is_esc_high(esc);
}

void set_esc_high(uint8_t esc) {
  gpio_pin_set(esc_pins[esc]);
}

void set_esc_low(uint8_t esc) {
  gpio_pin_reset(esc_pins[esc]);
}

void set_esc_input(uint8_t esc) {
  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.GPIO_Mode = GPIO_Mode_IN;
  gpio_init.GPIO_OType = GPIO_OType_OD;
  gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_pin_init(&gpio_init, esc_pins[esc]);
}

void set_esc_output(uint8_t esc) {
  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_pin_init(&gpio_init, esc_pins[esc]);
}

static uint8_t read_byte(void) {
  uint8_t byte = 0;
  while (usb_serial_read(&byte, 1) == 0)
    ;
  return byte;
}

static uint8_t read_byte_crc(void) {
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

static uint8_t connect_esc(uint8_32_u *pDeviceInfo) {
  for (uint8_t I = 0; I < 3; ++I) {
#if (defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) && defined(USE_SERIAL_4WAY_SK_BOOTLOADER))
    if ((interface_mode != ESC4WAY_ARM_BLB) && Stk_ConnectEx(pDeviceInfo) && ATMEL_DEVICE_MATCH) {
      interface_mode = ESC4WAY_ATM_SK;
      return 1;
    } else {
      if (BL_ConnectEx(pDeviceInfo)) {
        if (SILABS_DEVICE_MATCH) {
          interface_mode = ESC4WAY_SIL_BLB;
          return 1;
        } else if (ATMEL_DEVICE_MATCH) {
          interface_mode = ESC4WAY_ATM_BLB;
          return 1;
        } else if (ARM_DEVICE_MATCH) {
          interface_mode = ESC4WAY_ARM_BLB;
          return 1;
        }
      }
    }
#elif defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER)
    if (BL_ConnectEx(pDeviceInfo)) {
      if (SILABS_DEVICE_MATCH) {
        interface_mode = ESC4WAY_SIL_BLB;
        return 1;
      } else if (ATMEL_DEVICE_MATCH) {
        interface_mode = ESC4WAY_ATM_BLB;
        return 1;
      } else if (ARM_DEVICE_MATCH) {
        interface_mode = ESC4WAY_ARM_BLB;
        return 1;
      }
    }
#elif defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
    if (Stk_ConnectEx(pDeviceInfo)) {
      interface_mode = ESC4WAY_ATM_SK;
      if (ATMEL_DEVICE_MATCH)
        return 1;
    }
#endif
  }
  return 0;
}

uint8_t serial_4way_init() {
  motor_set_all(0);
  timer_delay_us(250000);

  // set up 1wire serial to each esc

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel)     \
  esc_pins[MOTOR_PIN_IDENT(port, pin)] = PIN_IDENT(port, pin); \
  set_esc_input(MOTOR_PIN_IDENT(port, pin));                   \
  set_esc_high(MOTOR_PIN_IDENT(port, pin));

  MOTOR_PINS

#undef MOTOR_PIN

  return ESC_COUNT;
}

void serial_4way_release() {
  motor_init();
  motor_set_all(0);
}

serial_esc4way_ack_t serial_4way_send(uint8_t cmd, serial_esc4way_payload_t payload, uint8_t *output, uint8_t *output_len) {
  serial_esc4way_ack_t ACK_OUT = ESC4WAY_ACK_OK;

  *output_len = 1;

  switch (cmd) {
  default:
    return ESC4WAY_ACK_I_INVALID_CMD;

  // ******* Interface related stuff *******
  case ESC4WAY_INTERFACE_TEST_ALIVE: {
    if (is_mcu_connected()) {
      switch (interface_mode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
      case ESC4WAY_ATM_BLB:
      case ESC4WAY_SIL_BLB:
      case ESC4WAY_ARM_BLB: {
        if (!BL_SendCMDKeepAlive()) { // SetStateDisconnected() included
          ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
        }
        break;
      }
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
      case ESC4WAY_ATM_SK: {
        if (!Stk_SignOn()) { // SetStateDisconnected();
          ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
        }
        break;
      }
#endif
      default:
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      if (ACK_OUT != ESC4WAY_ACK_OK)
        SET_DISCONNECTED;
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
#if defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) && defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
    if ((payload.params[0] <= ESC4WAY_ARM_BLB) && (payload.params[0] >= ESC4WAY_SIL_BLB))
#elif defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER)
    if (((payload.params[0] <= ESC4WAY_ATM_BLB) || (payload.params[0] == ESC4WAY_ARM_BLB)) && (payload.params[0] >= ESC4WAY_SIL_BLB))
#elif defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
    if (payload.params[0] == ESC4WAY_ATM_SK)
#endif
    {
      interface_mode = payload.params[0];
    } else {
      ACK_OUT = ESC4WAY_ACK_I_INVALID_PARAM;
    }
    break;
  }

  case ESC4WAY_DEVICE_RESET: {
    if (payload.params[0] >= ESC_COUNT) {
      return ESC4WAY_ACK_I_INVALID_CHANNEL;
    }

    bool reboot_esc = false;
    // Channel may change here
    selected_esc = payload.params[0];
    if (payload.flash_addr_l == 1) {
      reboot_esc = true;
    }

    switch (interface_mode) {
    case ESC4WAY_SIL_BLB:
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
    case ESC4WAY_ATM_BLB:
    case ESC4WAY_ARM_BLB: {
      BL_SendCMDRunRestartBootloader(&device_info);
      if (reboot_esc) {
        ESC_OUTPUT;
        set_esc_low(selected_esc);
        uint32_t m = timer_millis();
        while (timer_millis() - m < 300)
          ;
        set_esc_high(selected_esc);
        ESC_INPUT;
      }
      break;
    }
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
    case ESC4WAY_ATM_SK: {
      break;
    }
#endif
    }
    SET_DISCONNECTED;
    break;
  }

  case ESC4WAY_DEVICE_INIT_FLASH: {
    SET_DISCONNECTED;

    if (payload.params[0] >= ESC_COUNT) {
      return ESC4WAY_ACK_I_INVALID_CHANNEL;
    }

    //Channel may change here
    //ESC_LO or ESC_HI; Halt state for prev channel
    selected_esc = payload.params[0];

    if (connect_esc(&device_info)) {
      device_info.bytes[INTF_MODE_IDX] = interface_mode;
    } else {
      SET_DISCONNECTED;
      ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
    }

    *output_len = DEVICE_INFO_SIZE;
    memcpy(output, &device_info, DEVICE_INFO_SIZE);
    break;
  }

#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
  case ESC4WAY_DEVICE_ERASE_ALL: {
    switch (interface_mode) {
    case ESC4WAY_ATM_SK: {
      if (!Stk_Chip_Erase())
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      break;
    }
    default:
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
    }
    break;
  }
#endif

#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
  case ESC4WAY_DEVICE_PAGE_ERASE: {
    switch (interface_mode) {
    case ESC4WAY_SIL_BLB:
    case ESC4WAY_ARM_BLB: {
      ioMem_t mem;
      mem.D_NUM_BYTES = 0;
      mem.D_PTR_I = NULL;

      uint8_t addr = output[0] = payload.params[0];
      if (interface_mode == ESC4WAY_ARM_BLB) {
        // Address =Page * 1024
        mem.D_FLASH_ADDR_H = (addr << 2);
      } else {
        // Address =Page * 512
        mem.D_FLASH_ADDR_H = (addr << 1);
      }
      mem.D_FLASH_ADDR_L = 0;
      if (!BL_PageErase(&mem))
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      break;
    }
    default:
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
    }
    break;
  }
#endif

  //*** Device Memory Read Ops ***
  case ESC4WAY_DEVICE_READ: {
    ioMem_t mem;
    mem.D_NUM_BYTES = payload.params[0];
    mem.D_PTR_I = output;
    mem.D_FLASH_ADDR_H = payload.flash_addr_h;
    mem.D_FLASH_ADDR_L = payload.flash_addr_l;

    switch (interface_mode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
    case ESC4WAY_SIL_BLB:
    case ESC4WAY_ATM_BLB:
    case ESC4WAY_ARM_BLB: {
      if (!BL_ReadFlash(interface_mode, &mem)) {
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
    case ESC4WAY_ATM_SK: {
      if (!Stk_ReadFlash(&mem)) {
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
#endif
    default:
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
    }
    if (ACK_OUT == ESC4WAY_ACK_OK) {
      *output_len = mem.D_NUM_BYTES;
    }
    break;
  }

  case ESC4WAY_DEVICE_READ_E_EPROM: {
    ioMem_t mem;
    mem.D_NUM_BYTES = payload.params[0];
    mem.D_PTR_I = output;
    mem.D_FLASH_ADDR_H = payload.flash_addr_h;
    mem.D_FLASH_ADDR_L = payload.flash_addr_l;

    switch (interface_mode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
    case ESC4WAY_ATM_BLB: {
      if (!BL_ReadEEprom(&mem)) {
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
    case ESC4WAY_ATM_SK: {
      if (!Stk_ReadEEprom(&mem)) {
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
#endif
    default:
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
    }
    if (ACK_OUT == ESC4WAY_ACK_OK) {
      *output_len = mem.D_NUM_BYTES;
    }
    break;
  }

  //*** Device Memory Write Ops ***
  case ESC4WAY_DEVICE_WRITE: {
    ioMem_t mem;
    mem.D_NUM_BYTES = payload.params_len;
    mem.D_PTR_I = payload.params;
    mem.D_FLASH_ADDR_H = payload.flash_addr_h;
    mem.D_FLASH_ADDR_L = payload.flash_addr_l;

    switch (interface_mode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
    case ESC4WAY_SIL_BLB:
    case ESC4WAY_ATM_BLB:
    case ESC4WAY_ARM_BLB: {
      if (!BL_WriteFlash(&mem)) {
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
    case ESC4WAY_ATM_SK: {
      if (!Stk_WriteFlash(&mem)) {
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
      }
      break;
    }
#endif
    }
    break;
  }

  case ESC4WAY_DEVICE_WRITE_E_EPROM: {
    ioMem_t mem;
    mem.D_NUM_BYTES = payload.params_len;
    mem.D_PTR_I = payload.params;
    mem.D_FLASH_ADDR_H = payload.flash_addr_h;
    mem.D_FLASH_ADDR_L = payload.flash_addr_l;

    ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;

    switch (interface_mode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
    case ESC4WAY_SIL_BLB: {
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
      break;
    }
    case ESC4WAY_ATM_BLB: {
      if (BL_WriteEEprom(&mem)) {
        ACK_OUT = ESC4WAY_ACK_OK;
      }
      break;
    }
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
    case ESC4WAY_ATM_SK: {
      if (Stk_WriteEEprom(&mem)) {
        ACK_OUT = ESC4WAY_ACK_OK;
      }
      break;
    }
#endif
    }
    break;
  }
//*** Device Memory Verify Ops ***
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
  case ESC4WAY_DEVICE_VERIFY: {

    switch (interface_mode) {
    case ESC4WAY_ARM_BLB: {
      ioMem_t mem;
      mem.D_NUM_BYTES = payload.params_len;
      mem.D_PTR_I = payload.params;
      mem.D_FLASH_ADDR_H = payload.flash_addr_h;
      mem.D_FLASH_ADDR_L = payload.flash_addr_l;

#ifdef USE_FAKE_ESC
      ACK_OUT = ESC4WAY_ACK_OK;
      break;
#else
      switch (BL_VerifyFlash(&mem)) {
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
#endif
    }

    default:
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
      break;
    }
    break;
  }
#endif
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
  timer_delay_us(250000); // give the device some time to wake up

  payload.flash_addr_h = BLHELI_SETTINGS_OFFSET >> 8;
  payload.flash_addr_l = BLHELI_SETTINGS_OFFSET & 0xFF;
  payload.params[0] = BLHELI_SETTINGS_SIZE;
  payload.params_len = 1;

  ack = serial_4way_send(ESC4WAY_DEVICE_READ, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_READ 0x%x", ack);
    return ack;
  }
  timer_delay_us(200);

  memcpy(settings, output, output_len);

  payload.flash_addr_h = 0;
  payload.flash_addr_l = 0;
  payload.params[0] = esc;
  payload.params_len = 1;

  ack = serial_4way_send(ESC4WAY_DEVICE_RESET, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_RESET 0x%x", ack);
    return ack;
  }
  timer_delay_us(200);

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
  timer_delay_us(250000); // give the device some time to wake up

  payload.params[0] = BLHELI_SETTINGS_OFFSET / SILABS_PAGE_SIZE;
  payload.params_len = 1;

  ack = serial_4way_send(ESC4WAY_DEVICE_PAGE_ERASE, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_PAGE_ERASE 0x%x", ack);
    return ack;
  }
  timer_delay_us(200);

  payload.flash_addr_h = BLHELI_SETTINGS_OFFSET >> 8;
  payload.flash_addr_l = BLHELI_SETTINGS_OFFSET & 0xFF;
  memcpy(payload.params, settings, BLHELI_SETTINGS_SIZE);
  payload.params_len = BLHELI_SETTINGS_SIZE;

  ack = serial_4way_send(ESC4WAY_DEVICE_WRITE, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_WRITE 0x%x", ack);
    return ack;
  }
  timer_delay_us(200);

  payload.flash_addr_h = 0;
  payload.flash_addr_l = 0;
  payload.params[0] = esc;
  payload.params_len = 1;

  ack = serial_4way_send(ESC4WAY_DEVICE_RESET, payload, output, &output_len);
  if (ack != ESC4WAY_ACK_OK) {
    quic_debugf("ERROR ESC4WAY_DEVICE_RESET 0x%x", ack);
    return ack;
  }
  timer_delay_us(200);

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