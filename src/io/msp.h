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

#define MSP_HEADER_LEN 5

#define MSP_BUILD_DATE_TIME __DATE__ __TIME__

typedef enum {
  MSP_ERROR,
  MSP_EOF,
  MSP_SUCCESS,
} msp_status_t;

typedef void (*msp_send_fn_t)(uint8_t direction, uint8_t code, uint8_t *data, uint8_t len);

typedef struct {
  uint8_t *buffer;

  uint32_t buffer_size;

  uint32_t write_offset;
  uint32_t read_offset;

  msp_send_fn_t send;
} msp_t;

void msp_push_byte(msp_t *msp, uint8_t val);
void msp_push(msp_t *msp, uint8_t *data, uint32_t size);

msp_status_t msp_process_serial(msp_t *msp);