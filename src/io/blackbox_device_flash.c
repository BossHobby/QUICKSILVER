#include "blackbox_device_flash.h"

#include "core/project.h"
#include "driver/spi_m25p16.h"
#include "util/util.h"

#define FILES_SECTOR_OFFSET blackbox_bounds.sector_size
#define PAGE_SIZE M25P16_PAGE_SIZE
#define MAX_WRITE_SIZE (128)

typedef enum {
  STATE_DETECT,
  STATE_IDLE,

  STATE_WRITE,

  STATE_READ_HEADER,

  STATE_ERASE_HEADER,
  STATE_WRITE_HEADER,
} blackbox_device_state_t;

#ifdef USE_DATA_FLASH

static blackbox_device_state_t state = STATE_DETECT;
static uint8_t should_flush = 0;

void blackbox_device_flash_init() {
  m25p16_init();

  state = STATE_DETECT;
}

blackbox_device_result_t blackbox_device_flash_update() {
  static uint32_t offset = 0;
  static uint32_t write_size = 0;

  switch (state) {
  case STATE_DETECT:
    if (m25p16_is_ready()) {
      m25p16_get_bounds(&blackbox_bounds);
      state = STATE_READ_HEADER;
    }
    return BLACKBOX_DEVICE_WAIT;

  case STATE_READ_HEADER:
    if (!m25p16_is_ready()) {
      break;
    }

    m25p16_read_addr(M25P16_READ_DATA_BYTES, 0x0, (uint8_t *)&blackbox_device_header, sizeof(blackbox_device_header_t));
    if (blackbox_device_header.magic != BLACKBOX_HEADER_MAGIC) {
      blackbox_device_header.magic = BLACKBOX_HEADER_MAGIC;
      blackbox_device_header.file_num = 0;

      state = STATE_ERASE_HEADER;
      break;
    }

    state = STATE_IDLE;
    break;

  case STATE_IDLE: {
    if (should_flush == 1) {
      state = STATE_ERASE_HEADER;
      should_flush = 0;
      break;
    }

    offset = blackbox_current_file()->start + blackbox_current_file()->size;
    if (offset >= blackbox_bounds.total_size) {
      break;
    }

    if (write_size < MAX_WRITE_SIZE) {
      write_size += ring_buffer_read_multi(&blackbox_encode_buffer, blackbox_write_buffer + write_size, (MAX_WRITE_SIZE - write_size));
    }
    if (write_size >= MAX_WRITE_SIZE || should_flush == 1) {
      state = STATE_WRITE;
    }
    break;
  }

  case STATE_WRITE: {
    if (!m25p16_page_program(offset, blackbox_write_buffer, write_size)) {
      break;
    }
    blackbox_current_file()->size += write_size;
    write_size = 0;
    state = STATE_IDLE;
    return BLACKBOX_DEVICE_WRITE;
  }

  case STATE_ERASE_HEADER: {
    if (!m25p16_is_ready()) {
      return BLACKBOX_DEVICE_WAIT;
    }
    m25p16_write_addr(M25P16_SECTOR_ERASE, 0x0, NULL, 0);
    state = STATE_WRITE_HEADER;
    return BLACKBOX_DEVICE_WAIT;
  }

  case STATE_WRITE_HEADER: {
    if (!m25p16_is_ready()) {
      return BLACKBOX_DEVICE_WAIT;
    }
    if (m25p16_page_program(0x0, (uint8_t *)&blackbox_device_header, sizeof(blackbox_device_header_t))) {
      state = STATE_IDLE;
    }
    return BLACKBOX_DEVICE_WAIT;
  }
  }

  return BLACKBOX_DEVICE_IDLE;
}

void blackbox_device_flash_reset() {
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_command(M25P16_BULK_ERASE);

  m25p16_wait_for_ready();

  state = STATE_ERASE_HEADER;
}

uint32_t blackbox_device_flash_usage() {
  if (blackbox_device_header.file_num == 0) {
    return FILES_SECTOR_OFFSET;
  }
  return blackbox_current_file()->start + blackbox_current_file()->size;
}

void blackbox_device_flash_flush() {
  should_flush = 1;
}

void blackbox_device_flash_write_header() {
  state = STATE_ERASE_HEADER;
}

bool blackbox_device_flash_ready() {
  return state == STATE_IDLE;
}

void blackbox_device_flash_write(const uint8_t *buffer, const uint8_t size) {
  if (size >= ring_buffer_free(&blackbox_encode_buffer)) {
    return;
  }

  ring_buffer_write_multi(&blackbox_encode_buffer, buffer, size);
}

void blackbox_device_flash_read(const uint32_t file_index, const uint32_t offset, uint8_t *buffer, const uint32_t size) {
  const blackbox_device_file_t *file = &blackbox_device_header.files[file_index];

  uint32_t read = 0;
  while (read < size) {
    const uint32_t read_size = min(size - read, PAGE_SIZE);

    const uint32_t abs_offset = file->start + offset + read;
    m25p16_read_addr(M25P16_READ_DATA_BYTES, abs_offset, buffer + read, read_size);

    read += read_size;
  }
}

blackbox_device_vtable_t blackbox_device_flash = {
    .init = blackbox_device_flash_init,
    .update = blackbox_device_flash_update,
    .reset = blackbox_device_flash_reset,
    .write_header = blackbox_device_flash_write_header,
    .flush = blackbox_device_flash_flush,

    .usage = blackbox_device_flash_usage,
    .ready = blackbox_device_flash_ready,

    .read = blackbox_device_flash_read,
    .write = blackbox_device_flash_write,
};

#endif