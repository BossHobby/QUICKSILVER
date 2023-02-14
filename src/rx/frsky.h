#pragma once

#include "project.h"

#define FRSKY_SYNC_DELAY_MAX 9000
#define FRSKY_MAX_MISSING_FRAMES 150

typedef struct {
  int8_t offset;
  uint8_t idx;
  uint8_t tx_id[2];
  uint8_t hop_data[50];
  uint8_t rx_num;
  uint8_t _pad;
} rx_frsky_bind_data_t;

typedef enum {
  FRSKY_STATE_DETECT = 0,
  FRSKY_STATE_INIT,
  FRSKY_STATE_BIND,
  FRSKY_STATE_BIND_TUNING,
  FRSKY_STATE_BIND_BINDING,
  FRSKY_STATE_BIND_COMPLETE,
  FRSKY_STATE_STARTING,
  FRSKY_STATE_UPDATE,
  FRSKY_STATE_DATA,
  FRSKY_STATE_TELEMETRY,
  FRSKY_STATE_RESUME,
} frsky_state_t;

enum {
  FSSP_START_STOP = 0x7E,

  FSSP_DLE = 0x7D,
  FSSP_DLE_XOR = 0x20,

  FSSP_DATA_FRAME = 0x10,
  FSSP_MSPC_FRAME_SMARTPORT = 0x30, // MSP client frame
  FSSP_MSPC_FRAME_FPORT = 0x31,     // MSP client frame
  FSSP_MSPS_FRAME = 0x32,           // MSP server frame

  FSSP_SENSOR_ID1 = 0x1B,
  FSSP_SENSOR_ID2 = 0x0D,
  FSSP_SENSOR_ID3 = 0x34,
  FSSP_SENSOR_ID4 = 0x67
};
typedef struct {
  uint8_t frame_id;
  uint16_t value_id;
  uint32_t value;
} __attribute__((packed)) smart_port_payload_t;

void frsky_d16_write_telemetry(smart_port_payload_t *payload);
