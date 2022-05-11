#pragma once

#include <cbor.h>

#include "io/blackbox.h"

#define DATA_FLASH_HEADER_MAGIC 0xdeadbeef

typedef struct {
  uint32_t page_size;
  uint32_t pages_per_sector;
  uint32_t sectors;
  uint32_t sector_size;
  uint64_t total_size;
} data_flash_bounds_t;

typedef struct {
  uint32_t looptime;
  uint8_t blackbox_rate;
  uint32_t start_page;
  uint32_t size;
} data_flash_file_t;

#define DATA_FLASH_FILE_MEMBERS \
  MEMBER(looptime, uint32)      \
  MEMBER(blackbox_rate, uint8)  \
  MEMBER(start_page, uint32)    \
  MEMBER(size, uint32)

// sizeof(data_flash_header_t) cannot exceed PAGE_SIZE eg 256byte
typedef struct {
  uint32_t magic;
  uint8_t file_num;
  data_flash_file_t files[8];
} data_flash_header_t;

#define DATA_FLASH_HEADER_MEMBERS \
  MEMBER(magic, uint32)           \
  MEMBER(file_num, uint8)         \
  ARRAY_MEMBER(files, 8, data_flash_file_t)

typedef enum {
  DATA_FLASH_IDLE,
  DATA_FLASH_WAIT,
  DATA_FLASH_DETECT,
  DATA_FLASH_STARTING,
  DATA_FLASH_WRITE,
} data_flash_result_t;

cbor_result_t cbor_encode_data_flash_file_t(cbor_value_t *enc, const data_flash_file_t *f);
cbor_result_t cbor_encode_data_flash_header_t(cbor_value_t *enc, const data_flash_header_t *h);

void data_flash_init();
data_flash_result_t data_flash_update();

void data_flash_reset();
void data_flash_restart(uint32_t blackbox_rate, uint32_t looptime);
void data_flash_finish();

void data_flash_read_backbox(const uint32_t file_index, const uint32_t offset, uint8_t *buffer, const uint32_t size);
cbor_result_t data_flash_write_backbox(const blackbox_t *b);
