#pragma once

#include <cbor.h>

#include "io/blackbox.h"

#define BLACKBOX_DEVICE_HEADER_MAGIC 0xdeadbeef

typedef struct {
  uint32_t page_size;
  uint32_t pages_per_sector;
  uint32_t sectors;
  uint32_t sector_size;
  uint64_t total_size;
} blackbox_device_bounds_t;

typedef struct {
  uint32_t blackbox_fieldflags;
  uint32_t looptime;
  uint8_t blackbox_rate;
  uint32_t start_page;
  uint32_t size;
} blackbox_device_file_t;

#define BLACKBOX_DEVICE_FILE_MEMBERS  \
  MEMBER(blackbox_fieldflags, uint32) \
  MEMBER(looptime, uint32)            \
  MEMBER(blackbox_rate, uint8)        \
  MEMBER(start_page, uint32)          \
  MEMBER(size, uint32)

#define BLACKBOX_DEVICE_MAX_FILES 10

// sizeof(blackbox_device_header_t) cannot exceed PAGE_SIZE eg 256byte
typedef struct {
  uint32_t magic;
  uint8_t file_num;
  blackbox_device_file_t files[BLACKBOX_DEVICE_MAX_FILES];
} blackbox_device_header_t;

#define BLACKBOX_DEVICE_HEADER_MEMBERS \
  MEMBER(magic, uint32)                \
  MEMBER(file_num, uint8)              \
  ARRAY_MEMBER(files, BLACKBOX_DEVICE_MAX_FILES, blackbox_device_file_t)

typedef enum {
  BLACKBOX_DEVICE_IDLE,
  BLACKBOX_DEVICE_WAIT,
  BLACKBOX_DEVICE_DETECT,
  BLACKBOX_DEVICE_STARTING,
  BLACKBOX_DEVICE_WRITE,
} blackbox_device_result_t;

extern blackbox_device_header_t blackbox_device_header;
extern blackbox_device_bounds_t bounds;

cbor_result_t cbor_encode_blackbox_device_file_t(cbor_value_t *enc, const blackbox_device_file_t *f);
cbor_result_t cbor_encode_blackbox_device_header_t(cbor_value_t *enc, const blackbox_device_header_t *h);

void blackbox_device_init();
blackbox_device_result_t blackbox_device_update();
uint32_t blackbox_device_usage();

void blackbox_device_reset();
bool blackbox_device_restart(uint32_t blackbox_fieldflags, uint32_t blackbox_rate, uint32_t looptime);
void blackbox_device_finish();

void blackbox_device_read_backbox(const uint32_t file_index, const uint32_t offset, uint8_t *buffer, const uint32_t size);
cbor_result_t blackbox_device_write_backbox(const uint32_t blackbox_fieldflags, const blackbox_t *b);
