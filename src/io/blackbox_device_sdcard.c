#include "blackbox_device_sdcard.h"

#include <string.h>

#include "core/project.h"
#include "driver/spi_sdcard.h"

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

void blackbox_device_sdcard_init() {
  sdcard_init();

  state = STATE_DETECT;
}

blackbox_device_result_t blackbox_device_sdcard_update() {
  static uint32_t offset = 0;
  static uint32_t write_size = PAGE_SIZE;

  const uint32_t to_write = ring_buffer_available(&blackbox_encode_buffer);

  sdcard_status_t sdcard_status = sdcard_update();
  if (sdcard_status != SDCARD_IDLE) {
    return BLACKBOX_DEVICE_WAIT;
  }

sdcard_do_more:
  switch (state) {
  case STATE_DETECT: {
    state = STATE_READ_HEADER;
    sdcard_get_bounds(&blackbox_bounds);
    return BLACKBOX_DEVICE_WAIT;
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

  case STATE_IDLE:
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

  case STATE_START_WRITE: {
    offset = (blackbox_current_file()->start + blackbox_current_file()->size) / PAGE_SIZE;
    if (sdcard_write_pages_start(offset, FLUSH_INTERVAL)) {
      state = STATE_FILL_WRITE_BUFFER;
    }
    return BLACKBOX_DEVICE_WRITE;
  }

  case STATE_FILL_WRITE_BUFFER: {
    write_size = PAGE_SIZE;
    if (to_write < PAGE_SIZE) {
      if (should_flush == 0) {
        break;
      }
      if (to_write == 0) {
        state = STATE_FINISH_WRITE;
        goto sdcard_do_more;
      }

      write_size = to_write;
    }

    ring_buffer_read_multi(&blackbox_encode_buffer, blackbox_write_buffer, write_size);
    state = STATE_CONTINUE_WRITE;
    break;
  }

  case STATE_CONTINUE_WRITE: {
    static uint32_t counter = 0;
    if (sdcard_write_pages_continue(blackbox_write_buffer)) {
      blackbox_current_file()->size += write_size;

      counter++;
      if (counter == FLUSH_INTERVAL) {
        counter = 0;
        state = STATE_FINISH_WRITE;
      } else {
        state = STATE_FILL_WRITE_BUFFER;
      }
    }
    return BLACKBOX_DEVICE_WRITE;
  }

  case STATE_FINISH_WRITE: {
    if (sdcard_write_pages_finish()) {
      state = STATE_IDLE;
    }
    return BLACKBOX_DEVICE_WRITE;
  }

  case STATE_ERASE_HEADER: {
    memcpy(blackbox_write_buffer, (uint8_t *)&blackbox_device_header, sizeof(blackbox_device_header_t));
    state = STATE_WRITE_HEADER;
    return BLACKBOX_DEVICE_WAIT;
  }

  case STATE_WRITE_HEADER: {
    if (sdcard_write_page(blackbox_write_buffer, 0)) {
      state = STATE_IDLE;
    }
    return BLACKBOX_DEVICE_WAIT;
  }
  }

  return BLACKBOX_DEVICE_IDLE;
}

void blackbox_device_sdcard_reset() {
  state = STATE_ERASE_HEADER;
}

uint32_t blackbox_device_sdcard_usage() {
  if (blackbox_device_header.file_num == 0) {
    return FILES_SECTOR_OFFSET * PAGE_SIZE;
  }
  return blackbox_current_file()->start + blackbox_current_file()->size;
}

void blackbox_device_sdcard_flush() {
  should_flush = 1;
}

void blackbox_device_sdcard_write_header() {
  state = STATE_ERASE_HEADER;
}

bool blackbox_device_sdcard_ready() {
  return state == STATE_IDLE;
}

void blackbox_device_sdcard_write(const uint8_t *buffer, const uint8_t size) {
  if (size >= ring_buffer_free(&blackbox_encode_buffer)) {
    return;
  }

  ring_buffer_write_multi(&blackbox_encode_buffer, buffer, size);
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
    .write_header = blackbox_device_sdcard_write_header,
    .flush = blackbox_device_sdcard_flush,

    .usage = blackbox_device_sdcard_usage,
    .ready = blackbox_device_sdcard_ready,

    .read = blackbox_device_sdcard_read,
    .write = blackbox_device_sdcard_write,
};
