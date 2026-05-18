#pragma once

#include "core/profile.h"
#include "rx/rx_spi.h"

#define FMC_MAGIC 0x12AA0001
#define FMC_MAGIC_SIZE 4

#define TARGET_STORAGE_OFFSET 0
#define TARGET_STORAGE_SIZE FLASH_ALIGN(2048)

#define FLASH_STORAGE_OFFSET (TARGET_STORAGE_OFFSET + TARGET_STORAGE_SIZE)
#define FLASH_STORAGE_SIZE FLASH_ALIGN(32)

typedef struct {
  float accelcal[3];
} flash_storage_t;

extern flash_storage_t flash_storage;

#define PROFILE_STORAGE_OFFSET (FLASH_STORAGE_OFFSET + FLASH_STORAGE_SIZE)
#define PROFILE_STORAGE_SIZE FLASH_ALIGN(4096 + 128 + 512)

void flash_save();
void flash_load();
