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
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "defines.h"

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
//#define USE_SERIAL_4WAY_SK_BOOTLOADER // not implemented in Silverware yet
// #define USE_FAKE_ESC

#include "drv_serial_4way_impl.h"

typedef enum {
  ESC4WAY_SIL_C2 = 0,
  ESC4WAY_SIL_BLB = 1,
  ESC4WAY_ATM_BLB = 2,
  ESC4WAY_ATM_SK = 3,
  ESC4WAY_ARM_BLB = 4,
} serial_esc4way_mode_t;

typedef union __attribute__((packed)) {
  uint8_t bytes[2];
  uint16_t word;
} uint8_16_u;

typedef union __attribute__((packed)) {
  uint8_t bytes[4];
  uint16_t words[2];
  uint32_t dword;
} uint8_32_u;

typedef enum {
  ESC4WAY_REMOTE_ESCAPE = 0x2E, // '.'
  ESC4WAY_LOCAL_ESCAPE = 0x2F,  // '/'
} serial_esc4way_escape_t;

typedef enum {
  // Test Interface still present
  ESC4WAY_INTERFACE_TEST_ALIVE = 0x30, // '0' alive
  // RETURN: ACK

  // get Protocol Version Number 01..255
  ESC4WAY_PROTOCOL_GET_VERSION = 0x31, // '1' version
  // RETURN: uint8_t VersionNumber + ACK

  // get Version String
  ESC4WAY_INTERFACE_GET_NAME = 0x32, // '2' name
  // RETURN: String + ACK

  //get Version Number 01..255
  ESC4WAY_INTERFACE_GET_VERSION = 0x33, // '3' version
  // RETURN: uint8_t AVersionNumber + ACK

  // Exit / Restart Interface - can be used to switch to Box Mode
  ESC4WAY_INTERFACE_EXIT = 0x34, // '4' exit
  // RETURN: ACK

  // Reset the Device connected to the Interface
  ESC4WAY_DEVICE_RESET = 0x35, // '5' reset
  // RETURN: ACK

  // Get the Device ID connected
  // ESC4WAY_DEVICE_GET_ID = 0x36,      //'6' device id removed since 06/106
  // RETURN: uint8_t DeviceID + ACK

  // Initialize Flash Access for Device connected
  ESC4WAY_DEVICE_INIT_FLASH = 0x37, // '7' init flash access
  // RETURN: ACK

  // Erase the whole Device Memory of connected Device
  ESC4WAY_DEVICE_ERASE_ALL = 0x38, // '8' erase all
  // RETURN: ACK

  // Erase one Page of Device Memory of connected Device
  ESC4WAY_DEVICE_PAGE_ERASE = 0x39, // '9' page erase
  // PARAM: uint8_t APageNumber
  // RETURN: ACK

  // Read to Buffer from Device Memory of connected Device // Buffer Len is Max 256 Bytes
  // BuffLen = 0 means 256 Bytes
  ESC4WAY_DEVICE_READ = 0x3A, // ':' read Device
  // PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BuffLen[0..255]
  // RETURN: PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255] ACK

  // Write to Buffer for Device Memory of connected Device // Buffer Len is Max 256 Bytes
  // BuffLen = 0 means 256 Bytes
  ESC4WAY_DEVICE_WRITE = 0x3B, // ';' write
  // PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255]
  // RETURN: ACK

  // Set C2CK low infinite ) permanent Reset state
  ESC4WAY_DEVICE_C2_CK_LOW = 0x3C, // '<'
  // RETURN: ACK

  // Read to Buffer from Device Memory of connected Device //Buffer Len is Max 256 Bytes
  // BuffLen = 0 means 256 Bytes
  ESC4WAY_DEVICE_READ_E_EPROM = 0x3D, // '=' read Device
  // PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BuffLen[0..255]
  // RETURN: PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255] ACK

  // Write to Buffer for Device Memory of connected Device // Buffer Len is Max 256 Bytes
  // BuffLen = 0 means 256 Bytes
  ESC4WAY_DEVICE_WRITE_E_EPROM = 0x3E, // '>' write
  // PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255]
  // RETURN: ACK

  // Set Interface Mode
  ESC4WAY_INTERFACE_SET_MODE = 0x3F, // '?'
  // #define imC2 0
  // #define imSIL_BLB 1
  // #define imATM_BLB 2
  // #define imSK 3
  // PARAM: uint8_t Mode
  // RETURN: ACK or ACK_I_INVALID_CHANNEL

  //Write to Buffer for Verify Device Memory of connected Device //Buffer Len is Max 256 Bytes
  //BuffLen = 0 means 256 Bytes
  ESC4WAY_DEVICE_VERIFY = 0x40, //'@' write
  //PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255]
  //RETURN: ACK
} serial_esc4way_cmd_t;

typedef enum {
  ESC4WAY_ACK_OK = 0x00,
  //ESC4WAY_ACK_I_UNKNOWN_ERROR = 0x01,
  ESC4WAY_ACK_I_INVALID_CMD = 0x02,
  ESC4WAY_ACK_I_INVALID_CRC = 0x03,
  ESC4WAY_ACK_I_VERIFY_ERROR = 0x04,
  //ESC4WAY_ACK_D_INVALID_COMMAND = 0x05,
  //ESC4WAY_ACK_D_COMMAND_FAILED = 0x06,
  //ESC4WAY_ACK_D_UNKNOWN_ERROR = 0x07,
  ESC4WAY_ACK_I_INVALID_CHANNEL = 0x08,
  ESC4WAY_ACK_I_INVALID_PARAM = 0x09,
  ESC4WAY_ACK_D_GENERAL_ERROR = 0x0F,
} serial_esc4way_ack_t;

typedef struct {
  uint8_t num_bytes;
  uint8_t flash_addr_h;
  uint8_t flash_addr_l;
  uint8_t *params;
} serial_esc4way_payload_t;

extern uint8_t selected_esc;
//extern uint8_32_u DeviceInfo;

bool isMcuConnected(void);
uint8_t esc4wayInit(void);
struct serialPort_s;
void esc4wayProcess(void);
void esc4wayRelease(void);
#endif
