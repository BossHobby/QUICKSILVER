#pragma once

#include "rx.h"
#include "rx_bayang.h"
#include "rx_frsky.h"
#include "rx_unified_serial.h"

typedef struct {
  float initial_pid_identifier;
  float accelcal[3];

  uint8_t flash_feature_1;
  uint8_t flash_feature_2;
} flash_storage_t;

typedef struct {
  uint8_t bind_enable;
  union {
    rx_frsky_bind_data_t frsky;
    rx_bayang_bind_data_t bayang;
    rx_unified_bind_data_t unified;
  };
} rx_bind_storage_t;

void flash_save(void);
void flash_load(void);