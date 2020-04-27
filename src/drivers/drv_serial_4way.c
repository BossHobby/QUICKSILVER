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

#include "drv_motor.h"
#include "drv_serial_4way.h"
#include "drv_serial_soft.h"
#include "drv_usb.h"

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
#define SERIAL_4WAY_VER_SUB_2 (uint8_t)01

#define SERIAL_4WAY_PROTOCOL_VER 107
// *** end

#if (SERIAL_4WAY_VER_MAIN > 24)
#error "beware of SERIAL_4WAY_VER_SUB_1 is uint8_t"
#endif

#define SERIAL_4WAY_VERSION (uint16_t)((SERIAL_4WAY_VER_MAIN * 1000) + (SERIAL_4WAY_VER_SUB_1 * 100) + SERIAL_4WAY_VER_SUB_2)

#define SERIAL_4WAY_VERSION_HI (uint8_t)(SERIAL_4WAY_VERSION / 100)
#define SERIAL_4WAY_VERSION_LO (uint8_t)(SERIAL_4WAY_VERSION % 100)

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

static uint8_t escCount;

SoftSerialData_t escSerial[4] = {0};

uint8_t selected_esc;

uint8_32_u DeviceInfo;

#define DeviceInfoSize 4

bool isMcuConnected(void) {
  return (DeviceInfo.bytes[0] > 0);
}

bool isEscHi(uint8_t selEsc) {
  return GPIO_ReadInputDataBit(escSerial[selEsc].rx_port, escSerial[selEsc].rx_pin) > 0;
}

bool isEscLo(uint8_t selEsc) {
  return !isEscHi(selEsc);
}

void setEscHi(uint8_t selEsc) {
  GPIO_SetBits(escSerial[selEsc].rx_port, escSerial[selEsc].rx_pin);
}

void setEscLo(uint8_t selEsc) {
  GPIO_ResetBits(escSerial[selEsc].rx_port, escSerial[selEsc].rx_pin);
}

void setEscInput(uint8_t selEsc) {
  softserial_set_input(&escSerial[selEsc]);
}

void setEscOutput(uint8_t selEsc) {
  softserial_set_output(&escSerial[selEsc]);
}

uint8_t esc4wayInit(void) {
  // StopPwmAllMotors();
  //pwmDisableMotors();
  escCount = 4;
  motor_set_all(0);

  // set up 1wire serial to each esc

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel) \
  escSerial[MOTOR_PIN_IDENT(port, pin)] = softserial_init(GPIO##port, GPIO_Pin_##pin, GPIO##port, GPIO_Pin_##pin, 19200);

  MOTOR_PINS

#undef MOTOR_PIN

#ifdef F0
  // tx = dat (PA13), rx = clk (PA14)
  softserial_init(GPIOA, GPIO_Pin_13, GPIOA, GPIO_Pin_14, 38400);
#endif

  return escCount;
}

void esc4wayRelease(void) {
  motor_init();
  motor_set_all(0);
}

#define SET_DISCONNECTED DeviceInfo.words[0] = 0

#define INTF_MODE_IDX 3 // index for DeviceInfostate

// Interface related only
// establish and test connection to the Interface

// Send Structure
// ESC + CMD PARAM_LEN [PARAM (if len > 0)] CRC16_Hi CRC16_Lo
// Return
// ESC CMD PARAM_LEN [PARAM (if len > 0)] + ACK (uint8_t OK or ERR) + CRC16_Hi CRC16_Lo

#define ATMEL_DEVICE_MATCH ((pDeviceInfo->words[0] == 0x9307) || (pDeviceInfo->words[0] == 0x930A) || \
                            (pDeviceInfo->words[0] == 0x930F) || (pDeviceInfo->words[0] == 0x940B))

#define SILABS_DEVICE_MATCH ((pDeviceInfo->words[0] == 0xF310) || (pDeviceInfo->words[0] == 0xF330) || \
                             (pDeviceInfo->words[0] == 0xF410) || (pDeviceInfo->words[0] == 0xF390) || \
                             (pDeviceInfo->words[0] == 0xF850) || (pDeviceInfo->words[0] == 0xE8B1) || \
                             (pDeviceInfo->words[0] == 0xE8B2))

#define ARM_DEVICE_MATCH ((pDeviceInfo->words[0] == 0x1F06) || \
                          (pDeviceInfo->words[0] == 0x3306) || (pDeviceInfo->words[0] == 0x3406))

static uint8_t CurrentInterfaceMode;

static uint8_t Connect(uint8_32_u *pDeviceInfo) {
  for (uint8_t I = 0; I < 3; ++I) {
#if (defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) && defined(USE_SERIAL_4WAY_SK_BOOTLOADER))
    if ((CurrentInterfaceMode != ESC4WAY_ARM_BLB) && Stk_ConnectEx(pDeviceInfo) && ATMEL_DEVICE_MATCH) {
      CurrentInterfaceMode = ESC4WAY_ATM_SK;
      return 1;
    } else {
      if (BL_ConnectEx(pDeviceInfo)) {
        if SILABS_DEVICE_MATCH {
          CurrentInterfaceMode = ESC4WAY_SIL_BLB;
          return 1;
        } else if ATMEL_DEVICE_MATCH {
          CurrentInterfaceMode = ESC4WAY_ATM_BLB;
          return 1;
        } else if ARM_DEVICE_MATCH {
          CurrentInterfaceMode = ESC4WAY_ARM_BLB;
          return 1;
        }
      }
    }
#elif defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER)
    if (BL_ConnectEx(pDeviceInfo)) {
      if SILABS_DEVICE_MATCH {
        CurrentInterfaceMode = ESC4WAY_SIL_BLB;
        return 1;
      } else if ATMEL_DEVICE_MATCH {
        CurrentInterfaceMode = ESC4WAY_ATM_BLB;
        return 1;
      } else if ARM_DEVICE_MATCH {
        CurrentInterfaceMode = ESC4WAY_ARM_BLB;
        return 1;
      }
    }
#elif defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
    if (Stk_ConnectEx(pDeviceInfo)) {
      CurrentInterfaceMode = ESC4WAY_ATM_SK;
      if ATMEL_DEVICE_MATCH
        return 1;
    }
#endif
  }
  return 0;
}

static uint8_t ReadByte(void) {
  uint8_t byte = 0;

#ifdef F0
  // need timeout?
  softserial_read_byte(&byte);
#else
  byte = usb_serial_read_byte();
#endif
  return byte;
}

static uint8_16_u CRC_in;
static uint8_t ReadByteCrc(void) {
  uint8_t b = ReadByte();
  CRC_in.word = _crc_xmodem_update(CRC_in.word, b);
  return b;
}

static void WriteByte(uint8_t b) {
#ifdef F0
  softserial_write_byte(b);
#else
  usb_serial_write(&b, 1);
#endif
}

static uint8_16_u CRCout;
static void WriteByteCrc(uint8_t b) {
  WriteByte(b);
  CRCout.word = _crc_xmodem_update(CRCout.word, b);
}

void esc4wayProcess() {

  uint8_16_u CRC_check;
  uint8_16_u Dummy;

  uint8_t O_PARAM_LEN;
  uint8_t *O_PARAM;

  uint8_t I_PARAM[256];
  ioMem_t ioMem;

  bool isExitScheduled = false;

  RX_LED_OFF;
  TX_LED_OFF;

  while (1) {
    // restart looking for new sequence from host
    uint8_t ESC;
    do {
      RX_LED_ON;
      CRC_in.word = 0;
      ESC = ReadByteCrc();
      RX_LED_OFF;
    } while (ESC != ESC4WAY_LOCAL_ESCAPE);

    RX_LED_ON;

    Dummy.word = 0;
    O_PARAM = &Dummy.bytes[0];
    O_PARAM_LEN = 1;

    uint8_t CMD = ReadByteCrc();
    ioMem.D_FLASH_ADDR_H = ReadByteCrc();
    ioMem.D_FLASH_ADDR_L = ReadByteCrc();

    uint8_t I_PARAM_LEN = ReadByteCrc();
    for (uint16_t i = 0; i < I_PARAM_LEN; i++) {
      I_PARAM[i] = ReadByteCrc();
    }

    ioMem.D_PTR_I = I_PARAM;

    CRC_check.bytes[1] = ReadByte();
    CRC_check.bytes[0] = ReadByte();

    RX_LED_OFF;

    uint8_t ACK_OUT = ESC4WAY_ACK_OK;
    if (CRC_check.word == CRC_in.word) {
      TX_LED_ON;

      switch (CMD) {

      // ******* Interface related stuff *******
      case ESC4WAY_INTERFACE_TEST_ALIVE: {
        if (isMcuConnected()) {
          switch (CurrentInterfaceMode) {
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
        Dummy.bytes[0] = SERIAL_4WAY_PROTOCOL_VER;
        break;
      }

      case ESC4WAY_INTERFACE_GET_NAME: {
        // Only interface itself, no matter what Device
        // O_PARAM_LEN=16;
        O_PARAM_LEN = strlen(SERIAL_4WAY_INTERFACE_NAME_STR);
        O_PARAM = (uint8_t *)SERIAL_4WAY_INTERFACE_NAME_STR;
        break;
      }

      case ESC4WAY_INTERFACE_GET_VERSION: {
        // Only interface itself, no matter what Device
        // Dummy = iUart_res_InterfVersion;
        O_PARAM_LEN = 2;
        Dummy.bytes[0] = SERIAL_4WAY_VERSION_HI;
        Dummy.bytes[1] = SERIAL_4WAY_VERSION_LO;
        break;
      }

      case ESC4WAY_INTERFACE_EXIT: {
        isExitScheduled = true;
        break;
      }

      case ESC4WAY_INTERFACE_SET_MODE: {
#if defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) && defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
        if ((I_PARAM[0] <= ESC4WAY_ARM_BLB) && (I_PARAM[0] >= ESC4WAY_SIL_BLB))
#elif defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER)
        if (((I_PARAM[0] <= ESC4WAY_ATM_BLB) || (I_PARAM[0] == ESC4WAY_ARM_BLB)) && (I_PARAM[0] >= ESC4WAY_SIL_BLB))
#elif defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
        if (I_PARAM[0] == ESC4WAY_ATM_SK)
#endif
        {
          CurrentInterfaceMode = I_PARAM[0];
        } else {
          ACK_OUT = ESC4WAY_ACK_I_INVALID_PARAM;
        }
        break;
      }

      case ESC4WAY_DEVICE_RESET: {
        if (I_PARAM[0] < escCount) {
          // Channel may change here
          selected_esc = I_PARAM[0];
        } else {
          ACK_OUT = ESC4WAY_ACK_I_INVALID_CHANNEL;
          break;
        }
        switch (CurrentInterfaceMode) {
        case ESC4WAY_SIL_BLB:
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
        case ESC4WAY_ATM_BLB:
        case ESC4WAY_ARM_BLB: {
          BL_SendCMDRunRestartBootloader(&DeviceInfo);
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

        if (I_PARAM[0] < escCount) {
          //Channel may change here
          //ESC_LO or ESC_HI; Halt state for prev channel
          selected_esc = I_PARAM[0];
        } else {
          ACK_OUT = ESC4WAY_ACK_I_INVALID_CHANNEL;
          break;
        }

        O_PARAM_LEN = DeviceInfoSize; //4
        O_PARAM = (uint8_t *)&DeviceInfo;

        if (Connect(&DeviceInfo)) {
          DeviceInfo.bytes[INTF_MODE_IDX] = CurrentInterfaceMode;
        } else {
          SET_DISCONNECTED;
          ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
        }
        break;
      }

#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
      case ESC4WAY_DEVICE_ERASE_ALL: {
        switch (CurrentInterfaceMode) {
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
        switch (CurrentInterfaceMode) {
        case ESC4WAY_SIL_BLB:
        case ESC4WAY_ARM_BLB: {
          Dummy.bytes[0] = I_PARAM[0];
          if (CurrentInterfaceMode == ESC4WAY_ARM_BLB) {
            // Address =Page * 1024
            ioMem.D_FLASH_ADDR_H = (Dummy.bytes[0] << 2);
          } else {
            // Address =Page * 512
            ioMem.D_FLASH_ADDR_H = (Dummy.bytes[0] << 1);
          }
          ioMem.D_FLASH_ADDR_L = 0;
          if (!BL_PageErase(&ioMem))
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
        ioMem.D_NUM_BYTES = I_PARAM[0];

        switch (CurrentInterfaceMode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
        case ESC4WAY_SIL_BLB:
        case ESC4WAY_ATM_BLB:
        case ESC4WAY_ARM_BLB: {
          if (!BL_ReadFlash(CurrentInterfaceMode, &ioMem)) {
            ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
          }
          break;
        }
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
        case ESC4WAY_ATM_SK: {
          if (!Stk_ReadFlash(&ioMem)) {
            ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
          }
          break;
        }
#endif
        default:
          ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
        }
        if (ACK_OUT == ESC4WAY_ACK_OK) {
          O_PARAM_LEN = ioMem.D_NUM_BYTES;
          O_PARAM = (uint8_t *)&I_PARAM;
        }
        break;
      }

      case ESC4WAY_DEVICE_READ_E_EPROM: {
        ioMem.D_NUM_BYTES = I_PARAM[0];

        switch (CurrentInterfaceMode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
        case ESC4WAY_ATM_BLB: {
          if (!BL_ReadEEprom(&ioMem)) {
            ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
          }
          break;
        }
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
        case ESC4WAY_ATM_SK: {
          if (!Stk_ReadEEprom(&ioMem)) {
            ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
          }
          break;
        }
#endif
        default:
          ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
        }
        if (ACK_OUT == ESC4WAY_ACK_OK) {
          O_PARAM_LEN = ioMem.D_NUM_BYTES;
          O_PARAM = (uint8_t *)&I_PARAM;
        }
        break;
      }

      //*** Device Memory Write Ops ***
      case ESC4WAY_DEVICE_WRITE: {
        ioMem.D_NUM_BYTES = I_PARAM_LEN;

        switch (CurrentInterfaceMode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
        case ESC4WAY_SIL_BLB:
        case ESC4WAY_ATM_BLB:
        case ESC4WAY_ARM_BLB: {
          if (!BL_WriteFlash(&ioMem)) {
            ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
          }
          break;
        }
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
        case ESC4WAY_ATM_SK: {
          if (!Stk_WriteFlash(&ioMem)) {
            ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;
          }
          break;
        }
#endif
        }
        break;
      }

      case ESC4WAY_DEVICE_WRITE_E_EPROM: {
        ioMem.D_NUM_BYTES = I_PARAM_LEN;
        ACK_OUT = ESC4WAY_ACK_D_GENERAL_ERROR;

        switch (CurrentInterfaceMode) {
#ifdef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
        case ESC4WAY_SIL_BLB: {
          ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
          break;
        }
        case ESC4WAY_ATM_BLB: {
          if (BL_WriteEEprom(&ioMem)) {
            ACK_OUT = ESC4WAY_ACK_OK;
          }
          break;
        }
#endif
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER
        case ESC4WAY_ATM_SK: {
          if (Stk_WriteEEprom(&ioMem)) {
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

        switch (CurrentInterfaceMode) {
        case ESC4WAY_ARM_BLB: {
          ioMem.D_NUM_BYTES = I_PARAM_LEN;

#ifdef USE_FAKE_ESC
          ACK_OUT = ESC4WAY_ACK_OK;
          break;
#else
          ACK_OUT = BL_VerifyFlash(&ioMem);
          switch (ACK_OUT) {
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

      default:
        ACK_OUT = ESC4WAY_ACK_I_INVALID_CMD;
      }
    } else {
      ACK_OUT = ESC4WAY_ACK_I_INVALID_CRC;
    }

    CRCout.word = 0;

    WriteByteCrc(ESC4WAY_REMOTE_ESCAPE);
    WriteByteCrc(CMD);
    WriteByteCrc(ioMem.D_FLASH_ADDR_H);
    WriteByteCrc(ioMem.D_FLASH_ADDR_L);
    WriteByteCrc(O_PARAM_LEN);

    for (uint16_t i = 0; i < O_PARAM_LEN; i++) {
      WriteByteCrc(O_PARAM[i]);
    }

    WriteByteCrc(ACK_OUT);
    WriteByte(CRCout.bytes[1]);
    WriteByte(CRCout.bytes[0]);
    TX_LED_OFF;
    RX_LED_OFF;

    if (isExitScheduled) {
      esc4wayRelease();
      return;
    }
  };
}

#endif