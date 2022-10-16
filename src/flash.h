#pragma once

#include "rx.h"
#include "rx_bayang.h"
#include "rx_express_lrs.h"
#include "rx_frsky.h"
#include "rx_unified_serial.h"
#include "rx_flysky.h"

#define FLASH_STORAGE_OFFSET FLASH_ALIGN(4)
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
    rx_bayang_bind_data_t bayang;
    rx_unified_bind_data_t unified;
    rx_elrs_bind_data_t elrs;

// TODO: For some reason adding this to the union actually changes the union size!
// I get this compile error
//src\flash.c: In function 'flash_load.part.0':
//src\flash.c:120:5: warning: 'memcpy' reading 68 bytes from a region of size 64 [-Wstringop-overflow=]
//  120 |     memcpy((uint8_t *)&bind_storage, buffer, sizeof(rx_bind_storage_t));
//#define FLYSKY_BIND_DATA
#if defined(FLYSKY_BIND_DATA)
    rx_flysky_bind_data_t flysky;
#endif    
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

#define FMC_END_OFFSET (VTX_STORAGE_OFFSET + VTX_STORAGE_SIZE)

void flash_save();
void flash_load();
