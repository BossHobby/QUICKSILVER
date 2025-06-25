#include "blackbox_device_sdcard.h"

#ifdef USE_SDCARD

#include <string.h>

#include "core/project.h"
#include "driver/blackbox/sdcard.h"

#define FLUSH_INTERVAL 8
#define FILES_SECTOR_OFFSET 1
#define PAGE_SIZE SDCARD_PAGE_SIZE

typedef enum {
  STATE_DETECT,
  STATE_IDLE,

  STATE_START_WRITE,
  STATE_FILL_WRITE_BUFFER,
  STATE_CONTINUE_WRITE,
  STATE_FINISH_WRITE,

  STATE_READ_HEADER,

  STATE_ERASE_HEADER,
  STATE_WRITE_HEADER,
} blackbox_device_state_t;

static blackbox_device_state_t state = STATE_DETECT;
static uint8_t should_flush = 0;
static uint32_t write_count = 0;

void blackbox_device_sdcard_init() {
  sdcard_init();

  state = STATE_DETECT;
  should_flush = 0;
  write_count = 0;
}

bool blackbox_device_sdcard_update() {
  static uint32_t offset = 0;
  static uint32_t write_size = PAGE_SIZE;

  sdcard_status_t sdcard_status = sdcard_update();
  if (sdcard_status != SDCARD_IDLE) {
    return false;
  }

sdcard_do_more:
  switch (state) {
  case STATE_DETECT: {
    state = STATE_READ_HEADER;
    sdcard_get_bounds(&blackbox_bounds);
    blackbox_bounds.use_4byte_addresses = false;
    return false;
  }

  case STATE_READ_HEADER: {
    if (sdcard_read_pages(blackbox_write_buffer, 0, 1)) {
      memcpy((uint8_t *)&blackbox_device_header, blackbox_write_buffer, sizeof(blackbox_device_header_t));

      if (blackbox_device_header.magic != BLACKBOX_HEADER_MAGIC) {
        blackbox_device_header.magic = BLACKBOX_HEADER_MAGIC;
        blackbox_device_header.file_num = 0;

        state = STATE_ERASE_HEADER;
        break;
      }

      state = STATE_IDLE;
      goto sdcard_do_more;
    }
    break;
  }

  case STATE_IDLE: {
    const uint32_t to_write = ring_buffer_available(&blackbox_encode_buffer);
    if (to_write >= PAGE_SIZE) {
      state = STATE_START_WRITE;
      goto sdcard_do_more;
    }
    if (should_flush == 1 && to_write > 0) {
      state = STATE_START_WRITE;
      goto sdcard_do_more;
    }
    if (should_flush == 1) {
      state = STATE_ERASE_HEADER;
      should_flush = 0;
      goto sdcard_do_more;
    }
    break;
  }

  case STATE_START_WRITE: {
    const uint32_t byte_offset = blackbox_current_file()->start + blackbox_current_file()->size;
    if (byte_offset >= blackbox_bounds.total_size) {
      state = should_flush ? STATE_ERASE_HEADER : STATE_IDLE;
      should_flush = 0;
      break;
    }

    offset = byte_offset / PAGE_SIZE;
    if (sdcard_write_pages_start(offset, FLUSH_INTERVAL)) {
      write_count = 0;
      state = STATE_FILL_WRITE_BUFFER;
    }
    break;
  }

  case STATE_FILL_WRITE_BUFFER: {
    const uint32_t to_write = ring_buffer_available(&blackbox_encode_buffer);
    const uint32_t byte_offset = blackbox_current_file()->start + blackbox_current_file()->size;
    if (byte_offset >= blackbox_bounds.total_size) {
      state = STATE_FINISH_WRITE;
      goto sdcard_do_more;
    }

    write_size = min(PAGE_SIZE, blackbox_bounds.total_size - byte_offset);
    if (to_write < PAGE_SIZE) {
      if (should_flush == 0) {
        break;
      }
      if (to_write == 0) {
        state = STATE_FINISH_WRITE;
        goto sdcard_do_more;
      }
      write_size = min(write_size, to_write);
    }

    if (ring_buffer_read_multi(&blackbox_encode_buffer, blackbox_write_buffer, write_size) != write_size) {
      break;
    }
    if (write_size < PAGE_SIZE) {
      memset(blackbox_write_buffer + write_size, 0, PAGE_SIZE - write_size);
    }
    state = STATE_CONTINUE_WRITE;
    break;
  }

  case STATE_CONTINUE_WRITE: {
    if (sdcard_write_pages_continue(blackbox_write_buffer)) {
      blackbox_current_file()->size += write_size;

      write_count++;
      if (write_count == FLUSH_INTERVAL) {
        state = STATE_FINISH_WRITE;
      } else {
        state = STATE_FILL_WRITE_BUFFER;
      }
    }
    break;
  }

  case STATE_FINISH_WRITE: {
    if (sdcard_write_pages_finish()) {
      write_count = 0;
      state = STATE_IDLE;
    }
    break;
  }

  case STATE_ERASE_HEADER: {
    memcpy(blackbox_write_buffer, (uint8_t *)&blackbox_device_header, sizeof(blackbox_device_header_t));
    state = STATE_WRITE_HEADER;
    return false;
  }

  case STATE_WRITE_HEADER: {
    if (sdcard_write_page(blackbox_write_buffer, 0)) {
      state = STATE_IDLE;
    }
    return false;
  }
  }

  return true;
}

void blackbox_device_sdcard_reset() {
  state = STATE_ERASE_HEADER;
  should_flush = 0;
  write_count = 0;
}

uint32_t blackbox_device_sdcard_usage() {
  if (blackbox_device_header.file_num == 0) {
    return FILES_SECTOR_OFFSET * PAGE_SIZE;
  }
  return blackbox_current_file()->start + blackbox_current_file()->size;
}

void blackbox_device_sdcard_stop() {
  should_flush = 1;
}

void blackbox_device_sdcard_start() {
  state = STATE_ERASE_HEADER;
  should_flush = 0;
  write_count = 0;
}

bool blackbox_device_sdcard_ready() {
  return state == STATE_IDLE;
}

bool blackbox_device_sdcard_write(const uint8_t *buffer, const uint8_t size) {
  if (size > ring_buffer_free(&blackbox_encode_buffer)) {
    return false;
  }

  return ring_buffer_write_multi(&blackbox_encode_buffer, buffer, size) == size;
}

void blackbox_device_sdcard_read(const uint32_t file_index, const uint32_t offset, uint8_t *buffer, const uint32_t size) {
  const blackbox_device_file_t *file = &blackbox_device_header.files[file_index];

  const uint32_t sector_offset = (file->start + offset) / PAGE_SIZE;
  const uint32_t sectors = size / PAGE_SIZE + (size % PAGE_SIZE ? 1 : 0);

  while (1) {
    sdcard_update();
    if (sdcard_read_pages(buffer, sector_offset, sectors)) {
      break;
    }
    __NOP();
  }
}

blackbox_device_vtable_t blackbox_device_sdcard = {
    .init = blackbox_device_sdcard_init,
    .update = blackbox_device_sdcard_update,
    .reset = blackbox_device_sdcard_reset,

    .start = blackbox_device_sdcard_start,
    .stop = blackbox_device_sdcard_stop,

    .usage = blackbox_device_sdcard_usage,
    .ready = blackbox_device_sdcard_ready,

    .read = blackbox_device_sdcard_read,
    .write = blackbox_device_sdcard_write,
};

#endif
