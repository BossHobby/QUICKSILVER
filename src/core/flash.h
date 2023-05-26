#pragma once

#include "rx/express_lrs.h"
#include "rx/flysky.h"
#include "rx/frsky.h"
#include "rx/rx.h"
#include "rx/unified_serial.h"

#define FMC_MAGIC 0x12AA0001
#define FMC_MAGIC_SIZE 4

#define TARGET_STORAGE_OFFSET 0
#define TARGET_STORAGE_SIZE FLASH_ALIGN(2048)

#define FLASH_STORAGE_OFFSET (TARGET_STORAGE_OFFSET + TARGET_STORAGE_SIZE)
#define FLASH_STORAGE_SIZE FLASH_ALIGN(32)

typedef struct {
  float accelcal[3];

  uint8_t flash_feature_1; // SETUP WIZARD
  uint8_t lvc_lower_throttle;
} flash_storage_t;

extern flash_storage_t flash_storage;

#define BIND_STORAGE_OFFSET (FLASH_STORAGE_OFFSET + FLASH_STORAGE_SIZE)
#define BIND_STORAGE_SIZE FLASH_ALIGN(64)
#define BIND_RAW_STORAGE_SIZE 60

typedef struct {
  uint8_t bind_saved;
  uint8_t _padding[3];
  union {
    rx_frsky_bind_data_t frsky;
    rx_unified_bind_data_t unified;
    rx_elrs_bind_data_t elrs;
    rx_flysky_bind_data_t flysky;
    uint8_t raw[BIND_RAW_STORAGE_SIZE];
  };
} rx_bind_storage_t;

extern rx_bind_storage_t bind_storage;

cbor_result_t cbor_encode_rx_bind_storage_t(cbor_value_t *enc, const rx_bind_storage_t *s);
cbor_result_t cbor_decode_rx_bind_storage_t(cbor_value_t *enc, rx_bind_storage_t *s);

#define PROFILE_STORAGE_OFFSET (BIND_STORAGE_OFFSET + BIND_STORAGE_SIZE)
#define PROFILE_STORAGE_SIZE FLASH_ALIGN(2048)

#define VTX_STORAGE_OFFSET (PROFILE_STORAGE_OFFSET + PROFILE_STORAGE_SIZE)
#define VTX_STORAGE_SIZE FLASH_ALIGN(512)

void flash_save();
void flash_load();
