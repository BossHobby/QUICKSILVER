#pragma once

#include <cbor.h>

#include "blackbox.h"

#define DATA_FLASH_HEADER_MAGIC 0xdeadbeef

typedef struct {
  uint32_t page_size;
  uint32_t pages_per_sector;
  uint32_t sectors;
  uint32_t sector_size;
  uint32_t total_size;
} data_flash_bounds_t;

typedef struct {
  uint32_t start_sector;
  uint32_t entries;
} data_flash_file_t;

typedef struct {
  uint32_t magic;
  uint16_t file_num;
  data_flash_file_t files[16];
} data_flash_header_t;

void data_flash_init();
uint8_t data_flash_update(uint32_t loop);

void data_flash_reset();
void data_flash_restart();
void data_flash_finish();

cbor_result_t data_flash_read_backbox(const uint32_t addr, blackbox_t *b, const uint8_t count);
cbor_result_t data_flash_write_backbox(const blackbox_t *b);

cbor_result_t data_flash_read_header(data_flash_header_t *h);
cbor_result_t data_flash_write_header(data_flash_header_t *h);
