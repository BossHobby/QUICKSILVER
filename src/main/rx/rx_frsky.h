#pragma once

#include "project.h"

#ifdef RX_FRSKY
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
#endif