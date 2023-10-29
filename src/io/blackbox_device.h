#pragma once

#include <cbor.h>

#include "io/blackbox.h"
#include "util/ring_buffer.h"

#define BLACKBOX_HEADER_MAGIC 0xdeadbeef

// max size for a given entry
#define BLACKBOX_MAX_SIZE 255

#define BLACKBOX_WRITE_BUFFER_SIZE 512
#define BLACKBOX_ENCODE_BUFFER_SIZE 8192

typedef struct {
  uint32_t page_size;
  uint32_t pages_per_sector;
  uint32_t sectors;
  uint32_t sector_size;
  uint64_t total_size;
} blackbox_device_bounds_t;

typedef enum {
  BLACKBOX_DEVICE_IDLE,
  BLACKBOX_DEVICE_WAIT,
  BLACKBOX_DEVICE_WRITE,
} blackbox_device_result_t;

typedef struct {
  void (*init)();
  blackbox_device_result_t (*update)();
  void (*reset)();
  void (*write_header)();
  void (*flush)();

  uint32_t (*usage)();
  bool (*ready)();

  void (*read)(const uint32_t file_index, const uint32_t offset, uint8_t *buffer, const uint32_t size);
  void (*write)(const uint8_t *buffer, const uint8_t size);
} blackbox_device_vtable_t;

typedef struct {
  uint32_t field_flags;
  float looptime;
  uint8_t blackbox_rate;
  uint32_t start;
  uint32_t size;
} blackbox_device_file_t;

#define BLACKBOX_DEVICE_FILE_MEMBERS \
  MEMBER(field_flags, uint32_t)      \
  MEMBER(looptime, float)            \
  MEMBER(blackbox_rate, uint8_t)     \
  MEMBER(start, uint32_t)            \
  MEMBER(size, uint32_t)

#define BLACKBOX_DEVICE_MAX_FILES 10

// sizeof(blackbox_device_header_t) cannot exceed PAGE_SIZE eg 256byte
typedef struct {
  uint32_t magic;
  uint8_t file_num;
  blackbox_device_file_t files[BLACKBOX_DEVICE_MAX_FILES];
} blackbox_device_header_t;

#define BLACKBOX_DEVICE_HEADER_MEMBERS \
  MEMBER(magic, uint32_t)              \
  MEMBER(file_num, uint8_t)            \
  ARRAY_MEMBER(files, BLACKBOX_DEVICE_MAX_FILES, blackbox_device_file_t)

extern blackbox_device_header_t blackbox_device_header;
extern blackbox_device_bounds_t blackbox_bounds;

extern ring_buffer_t blackbox_encode_buffer;
extern uint8_t blackbox_write_buffer[BLACKBOX_WRITE_BUFFER_SIZE];

cbor_result_t cbor_encode_blackbox_device_file_t(cbor_value_t *enc, const blackbox_device_file_t *f);
cbor_result_t cbor_encode_blackbox_device_header_t(cbor_value_t *enc, const blackbox_device_header_t *h);

void blackbox_device_init();
blackbox_device_result_t blackbox_device_update();
uint32_t blackbox_device_usage();

blackbox_device_file_t *blackbox_current_file();

void blackbox_device_reset();
bool blackbox_device_restart(uint32_t field_flags, uint32_t blackbox_rate, float looptime);
void blackbox_device_finish();

void blackbox_device_read(const uint32_t file_index, const uint32_t offset, uint8_t *buffer, const uint32_t size);
cbor_result_t blackbox_device_write(const uint32_t field_flags, const blackbox_t *b);
