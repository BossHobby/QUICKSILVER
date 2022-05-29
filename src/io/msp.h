#pragma once

#include <stdint.h>

#define MSP_API_VERSION 1 // out message
#define MSP_FC_VARIANT 2  // out message
#define MSP_FC_VERSION 3  // out message
#define MSP_BOARD_INFO 4  // out message
#define MSP_BUILD_INFO 5  // out message

#define MSP_FEATURE_CONFIG 36

#define MSP_BATTERY_STATE 130 // out message         Connected/Disconnected, Voltage, Current Used

#define MSP_UID 160   // out message         Unique device ID
#define MSP_MOTOR 104 // out message         motors

#define MSP_DISPLAYPORT 182

#define MSP_MOTOR_CONFIG 131 // out message         Motor configuration (min/max throttle, etc)

#define MSP_SET_MOTOR 214   // in message          PropBalance function
#define MSP_SET_4WAY_IF 245 // in message          Sets 4way interface

#define MSP_RESERVE_1 251 // reserved for system usage

#define MSP_HEADER_LEN 5
#define MSP_TLM_HEADER_LEN 3

#define MSP2_HEADER_LEN 8

#define MSP_STATUS_SEQUENCE_MASK 0x0f // 0b00001111,   // sequence number mask
#define MSP_STATUS_START_MASK 0x10    // 0b00010000,   // bit of starting frame (if 1, the frame is a first/single chunk of msp-frame)
#define MSP_STATUS_VERSION_MASK 0x60  // 0b01100000,   // MSP version mask
#define MSP_STATUS_ERROR_MASK 0x80    // 0b10000000,   // Error bit (1 if error)
#define MSP_STATUS_VERSION_SHIFT 5    // MSP version shift

#define MSP_BUILD_DATE_TIME __DATE__ __TIME__

typedef enum {
  MSP1_MAGIC = 'M',
  MSP2_MAGIC = 'X',
} msp_magic_t;

typedef enum {
  MSP_ERROR,
  MSP_EOF,
  MSP_SUCCESS,
} msp_status_t;

typedef void (*msp_send_fn_t)(msp_magic_t magic, uint8_t direction, uint16_t code, uint8_t *data, uint16_t len);

typedef struct {
  uint8_t *buffer;

  uint32_t buffer_size;
  uint32_t buffer_offset;

  msp_send_fn_t send;
} msp_t;

msp_status_t msp_process_serial(msp_t *msp, uint8_t *data, uint32_t len);
msp_status_t msp_process_telemetry(msp_t *msp, uint8_t *data, uint32_t len);