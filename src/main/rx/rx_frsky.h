#pragma once

#include "project.h"
#include "rx_frsky.h"

#ifdef RX_FRSKY

#define FRSKY_SYNC_DELAY_MAX 9000
#define FRSKY_MAX_MISSING_FRAMES 150

typedef union {
  struct {
    int8_t offset;
    uint8_t idx;
    uint8_t tx_id[2];
    uint8_t hop_data[50];
    uint8_t rx_num;
    uint8_t _pad;
  };
  uint32_t raw[14];
} frsky_bind_data;

typedef enum {
  FRSKY_STATE_DETECT = 0,
  FRSKY_STATE_INIT,
  FRSKY_STATE_BIND,
  FRSKY_STATE_BIND_TUNING,
  FRSKY_STATE_BIND_BINDING1,
  FRSKY_STATE_BIND_BINDING2,
  FRSKY_STATE_BIND_COMPLETE,
  FRSKY_STATE_STARTING,
  FRSKY_STATE_UPDATE,
  FRSKY_STATE_DATA,
  FRSKY_STATE_TELEMETRY,
  FRSKY_STATE_RESUME,
} frsky_state_t;

#endif