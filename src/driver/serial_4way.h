#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "core/project.h"

#define SERIAL_4WAY_INTERFACE_NAME_STR "m4wFCIntf"

#define SERIAL_4WAY_VER_MAIN 20
#define SERIAL_4WAY_VER_SUB_1 (uint8_t)0
#define SERIAL_4WAY_VER_SUB_2 (uint8_t)06

#define SERIAL_4WAY_PROTOCOL_VER 108

#define SERIAL_4WAY_VERSION (uint16_t)((SERIAL_4WAY_VER_MAIN * 1000) + (SERIAL_4WAY_VER_SUB_1 * 100) + SERIAL_4WAY_VER_SUB_2)
#define SERIAL_4WAY_VERSION_HI (uint8_t)(SERIAL_4WAY_VERSION / 100)
#define SERIAL_4WAY_VERSION_LO (uint8_t)(SERIAL_4WAY_VERSION % 100)

#define SERIAL_4WAY_DEVICE_INFO_SIZE 4

typedef enum {
  ESC4WAY_SIL_C2 = 0,
  ESC4WAY_SIL_BLB = 1,
  ESC4WAY_ATM_BLB = 2,
  ESC4WAY_ATM_SK = 3,
  ESC4WAY_ARM_BLB = 4,
} serial_esc4way_mode_t;

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

  // get Version Number 01..255
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
  // PARAM: uint8_t Mode
  // RETURN: ACK or ACK_I_INVALID_CHANNEL

  // Write to Buffer for Verify Device Memory of connected Device //Buffer Len is Max 256 Bytes
  // BuffLen = 0 means 256 Bytes
  ESC4WAY_DEVICE_VERIFY = 0x40, //'@' write
  // PARAM: uint8_t ADRESS_Hi + ADRESS_Lo + BUffLen + Buffer[0..255]
  // RETURN: ACK
} serial_esc4way_cmd_t;

typedef enum {
  ESC4WAY_ACK_OK = 0x00,
  ESC4WAY_ACK_I_UNKNOWN_ERROR = 0x01,
  ESC4WAY_ACK_I_INVALID_CMD = 0x02,
  ESC4WAY_ACK_I_INVALID_CRC = 0x03,
  ESC4WAY_ACK_I_VERIFY_ERROR = 0x04,
  ESC4WAY_ACK_D_INVALID_COMMAND = 0x05,
  ESC4WAY_ACK_D_COMMAND_FAILED = 0x06,
  ESC4WAY_ACK_D_UNKNOWN_ERROR = 0x07,
  ESC4WAY_ACK_I_INVALID_CHANNEL = 0x08,
  ESC4WAY_ACK_I_INVALID_PARAM = 0x09,
  ESC4WAY_ACK_D_GENERAL_ERROR = 0x0F,
} serial_esc4way_ack_t;

typedef struct {
  uint8_t info[SERIAL_4WAY_DEVICE_INFO_SIZE];
  serial_esc4way_mode_t mode;
  uint16_t signature;
  uint8_t selected_esc;
} serial_esc4way_device_t;


uint8_t serial_4way_init();
void serial_4way_release();

serial_esc4way_ack_t serial_4way_send(uint8_t cmd, uint16_t addr, const uint8_t *input, const uint8_t input_size, uint8_t *output, uint8_t *output_size);

void serial_4way_process();
