#include "blackbox_device_simulator.h"

#include <stdio.h>

#include "core/project.h"
#include "util/util.h"

#ifdef SIMULATOR

#define FILENAME "blackbox.bin"
#define FILES_SECTOR_OFFSET 1024
#define PAGE_SIZE 256

typedef enum {
  STATE_DETECT,
  STATE_IDLE,

  STATE_START_WRITE,
  STATE_FILL_WRITE_BUFFER,
  STATE_CONTINUE_WRITE,
  STATE_FINISH_WRITE,

  STATE_READ_HEADER,
  STATE_WRITE_HEADER,
} blackbox_device_state_t;

static blackbox_device_state_t state = STATE_DETECT;
static uint8_t should_flush = 0;
static FILE *file = NULL;

void blackbox_device_simulator_init() {
  file = fopen(FILENAME, "rb+");
  if (file == NULL) {
    file = fopen(FILENAME, "wb+");
  }
  state = STATE_DETECT;
}

static void simulator_read(uint32_t addr, uint8_t *data, uint32_t size) {
  fseek(file, addr, SEEK_SET);
  fread(data, size, 1, file);
}

static void simulator_write(uint32_t addr, uint8_t *data, uint32_t size) {
  fseek(file, addr, SEEK_SET);
  fwrite(data, size, 1, file);
  fflush(file);
}

blackbox_device_result_t blackbox_device_simulator_update() {
  static uint32_t offset = 0;
  static uint32_t write_size = PAGE_SIZE;

  const uint32_t to_write = ring_buffer_available(&blackbox_encode_buffer);

simulator_do_more:
  switch (state) {
  case STATE_DETECT: {
    const uint32_t size = 104857600;
    blackbox_bounds.page_size = PAGE_SIZE;
    blackbox_bounds.pages_per_sector = 1;
    blackbox_bounds.sectors = size / PAGE_SIZE;
    blackbox_bounds.sector_size = PAGE_SIZE;
    blackbox_bounds.total_size = size;
    state = STATE_READ_HEADER;
    return BLACKBOX_DEVICE_WAIT;
  }

  case STATE_READ_HEADER:
    simulator_read(0x0, (uint8_t *)&blackbox_device_header, sizeof(blackbox_device_header_t));
    if (blackbox_device_header.magic == BLACKBOX_HEADER_MAGIC) {
      state = STATE_IDLE;
      break;
    }

    blackbox_device_header.magic = BLACKBOX_HEADER_MAGIC;
    blackbox_device_header.file_num = 0;
    state = STATE_WRITE_HEADER;
    break;

  case STATE_IDLE:
    if (should_flush == 1) {
      if (to_write > 0 && offset < blackbox_bounds.total_size) {
        state = STATE_START_WRITE;
      } else {
        state = STATE_WRITE_HEADER;
        should_flush = 0;
      }
      goto simulator_do_more;
    }
    if (to_write >= PAGE_SIZE) {
      state = STATE_START_WRITE;
      goto simulator_do_more;
    }
    break;

  case STATE_START_WRITE: {
    offset = blackbox_current_file()->start + blackbox_current_file()->size;
    if (offset >= blackbox_bounds.total_size) {
      state = STATE_IDLE;
      break;
    }
    state = STATE_FILL_WRITE_BUFFER;
    goto simulator_do_more;
  }

  case STATE_FILL_WRITE_BUFFER: {
    write_size = PAGE_SIZE;
    if (to_write < PAGE_SIZE) {
      if (should_flush == 0) {
        break;
      }
      if (to_write == 0) {
        state = STATE_FINISH_WRITE;
        goto simulator_do_more;
      }

      write_size = to_write;
    }

    ring_buffer_read_multi(&blackbox_encode_buffer, blackbox_write_buffer, write_size);
    state = STATE_CONTINUE_WRITE;
    break;
  }

  case STATE_CONTINUE_WRITE: {
    simulator_write(offset, blackbox_write_buffer, PAGE_SIZE);
    blackbox_current_file()->size += write_size;
    state = STATE_FINISH_WRITE;
    return BLACKBOX_DEVICE_WRITE;
  }

  case STATE_FINISH_WRITE: {
    state = STATE_IDLE;
    goto simulator_do_more;
  }

  case STATE_WRITE_HEADER: {
    simulator_write(0x0, (uint8_t *)&blackbox_device_header, sizeof(blackbox_device_header_t));
    state = STATE_IDLE;
    return BLACKBOX_DEVICE_WAIT;
  }
  }

  return BLACKBOX_DEVICE_IDLE;
}

void blackbox_device_simulator_reset() {
  file = freopen(FILENAME, "wb+", file);
  state = STATE_WRITE_HEADER;
}

uint32_t blackbox_device_simulator_usage() {
  if (blackbox_device_header.file_num == 0) {
    return FILES_SECTOR_OFFSET;
  }
  return blackbox_current_file()->start + blackbox_current_file()->size;
}

void blackbox_device_simulator_flush() {
  should_flush = 1;
}

void blackbox_device_simulator_write_header() {
  state = STATE_WRITE_HEADER;
}

bool blackbox_device_simulator_ready() {
  return state == STATE_IDLE;
}

void blackbox_device_simulator_write(const uint8_t *buffer, const uint8_t size) {
  if (size >= ring_buffer_free(&blackbox_encode_buffer)) {
    return;
  }

  ring_buffer_write_multi(&blackbox_encode_buffer, buffer, size);
}

void blackbox_device_simulator_read(const uint32_t file_index, const uint32_t offset, uint8_t *buffer, const uint32_t size) {
  const blackbox_device_file_t *file = &blackbox_device_header.files[file_index];
  simulator_read(file->start + offset, buffer, size);
}

blackbox_device_vtable_t blackbox_device_simulator = {
    .init = blackbox_device_simulator_init,
    .update = blackbox_device_simulator_update,
    .reset = blackbox_device_simulator_reset,
    .write_header = blackbox_device_simulator_write_header,
    .flush = blackbox_device_simulator_flush,

    .usage = blackbox_device_simulator_usage,
    .ready = blackbox_device_simulator_ready,

    .read = blackbox_device_simulator_read,
    .write = blackbox_device_simulator_write,
};

#endif