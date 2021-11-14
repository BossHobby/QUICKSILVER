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

#include <cbor.h>
#include <stdbool.h>
#include <stdint.h>

#include "project.h"

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
// #define USE_SERIAL_4WAY_SK_BOOTLOADER
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
  uint8_t flash_addr_h;
  uint8_t flash_addr_l;
  uint8_t *params;
  uint8_t params_len;
} serial_esc4way_payload_t;

typedef struct __attribute__((__packed__)) {
  uint8_t MAIN_REVISION;            //offset 0x00
  uint8_t SUB_REVISION;             //offset 0x01
  uint8_t LAYOUT_REVISION;          //offset 0x02
  uint8_t P_GAIN;                   //offset 0x03
  uint8_t I_GAIN;                   //offset 0x04
  uint8_t GOVERNOR_MODE;            //offset 0x05
  uint8_t LOW_VOLTAGE_LIMIT;        //offset 0x06
  uint8_t MOTOR_GAIN;               //offset 0x07
  uint8_t MOTOR_IDLE;               //offset 0x08
  uint8_t STARTUP_POWER;            //offset 0x09
  uint8_t PWM_FREQUENCY;            //offset 0x0A
  uint8_t MOTOR_DIRECTION;          //offset 0x0B
  uint8_t INPUT_PWM_POLARITY;       //offset 0x0C
  uint16_t MODE;                    //offset 0x0D
  uint8_t PROGRAMMING_BY_TX;        //offset 0x0F
  uint8_t REARM_AT_START;           //offset 0x10
  uint8_t GOVERNOR_SETUP_TARGET;    //offset 0x11
  uint8_t STARTUP_RPM;              //offset 0x12
  uint8_t STARTUP_ACCELERATION;     //offset 0x13
  uint8_t VOLT_COMP;                //offset 0x14
  uint8_t COMMUTATION_TIMING;       //offset 0x15
  uint8_t DAMPING_FORCE;            //offset 0x16
  uint8_t GOVERNOR_RANGE;           //offset 0x17
  uint8_t STARTUP_METHOD;           //offset 0x18
  uint8_t PPM_MIN_THROTTLE;         //offset 0x19
  uint8_t PPM_MAX_THROTTLE;         //offset 0x1A
  uint8_t BEEP_STRENGTH;            //offset 0x1B
  uint8_t BEACON_STRENGTH;          //offset 0x1C
  uint8_t BEACON_DELAY;             //offset 0x1D
  uint8_t THROTTLE_RATE;            //offset 0x1E
  uint8_t DEMAG_COMPENSATION;       //offset 0x1F
  uint8_t BEC_VOLTAGE;              //offset 0x20
  uint8_t PPM_CENTER_THROTTLE;      //offset 0x21
  uint8_t SPOOLUP_TIME;             //offset 0x22
  uint8_t TEMPERATURE_PROTECTION;   //offset 0x23
  uint8_t LOW_RPM_POWER_PROTECTION; //offset 0x24
  uint8_t PWM_INPUT;                //offset 0x25
  uint8_t PWM_DITHER;               //offset 0x26
  uint8_t BRAKE_ON_STOP;            //offset 0x27
  uint8_t LED_CONTROL;              //offset 0x28

  uint8_t _padding[23]; // offset 0x29

  uint8_t LAYOUT[16]; //offset 0x40
  uint8_t MCU[16];    //offset 0x50
  uint8_t NAME[16];   //offset 0x60
} blheli_settings_t;

#define BLHELI_SETTINGS_MEMBERS           \
  MEMBER(MAIN_REVISION, uint8)            \
  MEMBER(SUB_REVISION, uint8)             \
  MEMBER(LAYOUT_REVISION, uint8)          \
  MEMBER(P_GAIN, uint8)                   \
  MEMBER(I_GAIN, uint8)                   \
  MEMBER(GOVERNOR_MODE, uint8)            \
  MEMBER(LOW_VOLTAGE_LIMIT, uint8)        \
  MEMBER(MOTOR_GAIN, uint8)               \
  MEMBER(MOTOR_IDLE, uint8)               \
  MEMBER(STARTUP_POWER, uint8)            \
  MEMBER(PWM_FREQUENCY, uint8)            \
  MEMBER(MOTOR_DIRECTION, uint8)          \
  MEMBER(INPUT_PWM_POLARITY, uint8)       \
  MEMBER(MODE, uint16)                    \
  MEMBER(PROGRAMMING_BY_TX, uint8)        \
  MEMBER(REARM_AT_START, uint8)           \
  MEMBER(GOVERNOR_SETUP_TARGET, uint8)    \
  MEMBER(STARTUP_RPM, uint8)              \
  MEMBER(STARTUP_ACCELERATION, uint8)     \
  MEMBER(VOLT_COMP, uint8)                \
  MEMBER(COMMUTATION_TIMING, uint8)       \
  MEMBER(DAMPING_FORCE, uint8)            \
  MEMBER(GOVERNOR_RANGE, uint8)           \
  MEMBER(STARTUP_METHOD, uint8)           \
  MEMBER(PPM_MIN_THROTTLE, uint8)         \
  MEMBER(PPM_MAX_THROTTLE, uint8)         \
  MEMBER(BEEP_STRENGTH, uint8)            \
  MEMBER(BEACON_STRENGTH, uint8)          \
  MEMBER(BEACON_DELAY, uint8)             \
  MEMBER(THROTTLE_RATE, uint8)            \
  MEMBER(DEMAG_COMPENSATION, uint8)       \
  MEMBER(BEC_VOLTAGE, uint8)              \
  MEMBER(PPM_CENTER_THROTTLE, uint8)      \
  MEMBER(SPOOLUP_TIME, uint8)             \
  MEMBER(TEMPERATURE_PROTECTION, uint8)   \
  MEMBER(LOW_RPM_POWER_PROTECTION, uint8) \
  MEMBER(PWM_INPUT, uint8)                \
  MEMBER(PWM_DITHER, uint8)               \
  MEMBER(BRAKE_ON_STOP, uint8)            \
  MEMBER(LED_CONTROL, uint8)              \
  TSTR_MEMBER(LAYOUT, 16)                 \
  TSTR_MEMBER(MCU, 16)                    \
  TSTR_MEMBER(NAME, 16)

cbor_result_t cbor_decode_blheli_settings_t(cbor_value_t *dec, blheli_settings_t *p);
cbor_result_t cbor_encode_blheli_settings_t(cbor_value_t *enc, const blheli_settings_t *p);

uint8_t serial_4way_init();
void serial_4way_release();

serial_esc4way_ack_t serial_4way_send(uint8_t cmd, serial_esc4way_payload_t payload, uint8_t *output, uint8_t *output_len);

serial_esc4way_ack_t serial_4way_read_settings(blheli_settings_t *settings, uint8_t esc);
serial_esc4way_ack_t serial_4way_write_settings(blheli_settings_t *settings, uint8_t esc);

void serial_4way_process();

#endif
