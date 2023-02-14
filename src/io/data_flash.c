#include "io/data_flash.h"

#include <string.h>

#include "core/looptime.h"
#include "driver/spi_m25p16.h"
#include "driver/spi_sdcard.h"
#include "driver/time.h"
#include "io/usb_configurator.h"
#include "util/cbor_helper.h"
#include "util/ring_buffer.h"
#include "util/util.h"

#ifdef ENABLE_BLACKBOX

#ifdef USE_M25P16
#define FILES_SECTOR_OFFSET bounds.sector_size
#define PAGE_SIZE M25P16_PAGE_SIZE
#endif
#ifdef USE_SDCARD
#define FLUSH_INTERVAL 8
#define FILES_SECTOR_OFFSET 1
#define PAGE_SIZE SDCARD_PAGE_SIZE
#endif

#define BUFFER_SIZE 8192

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
} data_flash_state_t;

data_flash_bounds_t bounds;
data_flash_header_t data_flash_header;

static uint8_t encode_buffer_data[BUFFER_SIZE];
static ring_buffer_t encode_buffer = {
    .buffer = encode_buffer_data,
    .head = 0,
    .tail = 0,
    .size = BUFFER_SIZE,
};
static uint8_t write_buffer[PAGE_SIZE];
static data_flash_state_t state = STATE_DETECT;
static uint8_t should_flush = 0;

#define MEMBER CBOR_ENCODE_MEMBER
#define STR_MEMBER CBOR_ENCODE_STR_MEMBER
#define ARRAY_MEMBER CBOR_ENCODE_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_ENCODE_STR_ARRAY_MEMBER

CBOR_START_STRUCT_ENCODER(data_flash_file_t)
DATA_FLASH_FILE_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(data_flash_header_t)
DATA_FLASH_HEADER_MEMBERS
CBOR_END_STRUCT_ENCODER()

#undef MEMBER
#undef STR_MEMBER
#undef ARRAY_MEMBER
#undef STR_ARRAY_MEMBER

#define MEMBER CBOR_DECODE_MEMBER
#define STR_MEMBER CBOR_DECODE_STR_MEMBER
#define ARRAY_MEMBER CBOR_DECODE_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_DECODE_STR_ARRAY_MEMBER

CBOR_START_STRUCT_DECODER(data_flash_file_t)
DATA_FLASH_FILE_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(data_flash_header_t)
DATA_FLASH_HEADER_MEMBERS
CBOR_END_STRUCT_DECODER()

#undef MEMBER
#undef STR_MEMBER
#undef ARRAY_MEMBER
#undef STR_ARRAY_MEMBER

static data_flash_file_t *current_file() {
  return &data_flash_header.files[data_flash_header.file_num - 1];
}

data_flash_result_t data_flash_update() {
  static uint32_t offset = 0;
  static uint32_t write_size = PAGE_SIZE;

  const uint32_t to_write = ring_buffer_available(&encode_buffer);

#ifdef USE_SDCARD
  sdcard_status_t sdcard_status = sdcard_update();
  if (sdcard_status != SDCARD_IDLE) {
    if (state == STATE_DETECT) {
      return DATA_FLASH_DETECT;
    } else {
      return DATA_FLASH_WAIT;
    }
  }

sdcard_do_more:
  switch (state) {
  case STATE_DETECT: {
    state = STATE_READ_HEADER;
    sdcard_get_bounds(&bounds);
    return DATA_FLASH_DETECT;
  }

  case STATE_READ_HEADER: {
    if (sdcard_read_pages(write_buffer, 0, 1)) {
      memcpy((uint8_t *)&data_flash_header, write_buffer, sizeof(data_flash_header_t));

      if (data_flash_header.magic != DATA_FLASH_HEADER_MAGIC) {
        data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
        data_flash_header.file_num = 0;

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
    offset = FILES_SECTOR_OFFSET + current_file()->start_page + (current_file()->size / PAGE_SIZE);
    if (sdcard_write_pages_start(offset, FLUSH_INTERVAL)) {
      state = STATE_FILL_WRITE_BUFFER;
    }
    break;
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

    ring_buffer_read_multi(&encode_buffer, write_buffer, write_size);
    state = STATE_CONTINUE_WRITE;
    break;
  }

  case STATE_CONTINUE_WRITE: {
    static uint32_t counter = 0;
    if (sdcard_write_pages_continue(write_buffer)) {
      current_file()->size += write_size;

      counter++;
      if (counter == FLUSH_INTERVAL) {
        counter = 0;
        state = STATE_FINISH_WRITE;
      } else {
        state = STATE_FILL_WRITE_BUFFER;
      }
    }
    break;
  }

  case STATE_FINISH_WRITE: {
    if (sdcard_write_pages_finish()) {
      state = STATE_IDLE;
      goto sdcard_do_more;
    }
    break;
  }

  case STATE_ERASE_HEADER: {
    memcpy(write_buffer, (uint8_t *)&data_flash_header, sizeof(data_flash_header_t));
    state = STATE_WRITE_HEADER;
    return DATA_FLASH_STARTING;
  }

  case STATE_WRITE_HEADER: {
    if (sdcard_write_page(write_buffer, 0)) {
      state = STATE_IDLE;
    }
    return DATA_FLASH_STARTING;
  }
  }
#endif

#ifdef USE_M25P16
flash_do_more:
  switch (state) {
  case STATE_DETECT:
    if (!m25p16_is_ready()) {
      return DATA_FLASH_DETECT;
    }

    m25p16_get_bounds(&bounds);
    state = STATE_READ_HEADER;
    return DATA_FLASH_DETECT;

  case STATE_READ_HEADER:
    if (!m25p16_is_ready()) {
      break;
    }

    m25p16_read_addr(M25P16_READ_DATA_BYTES, 0x0, (uint8_t *)&data_flash_header, sizeof(data_flash_header_t));
    if (data_flash_header.magic != DATA_FLASH_HEADER_MAGIC) {
      data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
      data_flash_header.file_num = 0;

      state = STATE_ERASE_HEADER;
      break;
    }

    state = STATE_IDLE;
    break;

  case STATE_IDLE:
    if (should_flush == 1) {
      if (to_write > 0 && offset < bounds.total_size) {
        state = STATE_START_WRITE;
      } else {
        state = STATE_ERASE_HEADER;
        should_flush = 0;
      }
      goto flash_do_more;
    }
    if (to_write >= PAGE_SIZE) {
      state = STATE_START_WRITE;
      goto flash_do_more;
    }
    break;

  case STATE_START_WRITE: {
    offset = FILES_SECTOR_OFFSET + current_file()->start_page * PAGE_SIZE + current_file()->size;
    if (offset >= bounds.total_size) {
      state = STATE_IDLE;
      break;
    }
    state = STATE_FILL_WRITE_BUFFER;
    goto flash_do_more;
  }

  case STATE_FILL_WRITE_BUFFER: {
    write_size = PAGE_SIZE;
    if (to_write < PAGE_SIZE) {
      if (should_flush == 0) {
        break;
      }
      if (to_write == 0) {
        state = STATE_FINISH_WRITE;
        goto flash_do_more;
      }

      write_size = to_write;
    }

    ring_buffer_read_multi(&encode_buffer, write_buffer, write_size);
    state = STATE_CONTINUE_WRITE;
    break;
  }

  case STATE_CONTINUE_WRITE: {
    if (!m25p16_page_program(offset, write_buffer, PAGE_SIZE)) {
      break;
    }
    current_file()->size += write_size;
    state = STATE_FINISH_WRITE;
    return DATA_FLASH_WRITE;
  }

  case STATE_FINISH_WRITE: {
    state = STATE_IDLE;
    goto flash_do_more;
  }

  case STATE_ERASE_HEADER: {
    if (!m25p16_is_ready()) {
      return DATA_FLASH_STARTING;
    }
    m25p16_write_addr(M25P16_SECTOR_ERASE, 0x0, NULL, 0);
    state = STATE_WRITE_HEADER;
    return DATA_FLASH_STARTING;
  }

  case STATE_WRITE_HEADER: {
    if (!m25p16_is_ready()) {
      return DATA_FLASH_STARTING;
    }
    if (m25p16_page_program(0x0, (uint8_t *)&data_flash_header, sizeof(data_flash_header_t))) {
      state = STATE_IDLE;
    }
    return DATA_FLASH_STARTING;
  }
  }

  if (should_flush == 1) {
    return DATA_FLASH_STARTING;
  }
#endif

  return DATA_FLASH_IDLE;
}

void data_flash_init() {
#ifdef USE_M25P16
  m25p16_init();
#endif
#ifdef USE_SDCARD
  sdcard_init();
#endif

  data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
  data_flash_header.file_num = 0;

  state = STATE_DETECT;
}

uint32_t data_flash_usage() {
  if (data_flash_header.file_num == 0) {
    return 0;
  }
  return FILES_SECTOR_OFFSET + current_file()->start_page * PAGE_SIZE + current_file()->size;
}

void data_flash_reset() {
#ifdef USE_M25P16
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_command(M25P16_BULK_ERASE);

  m25p16_wait_for_ready();
#endif

  data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
  data_flash_header.file_num = 0;

  state = STATE_ERASE_HEADER;

  looptime_reset();
}

bool data_flash_restart(uint32_t blackbox_rate, uint32_t looptime) {
  if (data_flash_header.file_num >= DATA_FLASH_MAX_FILES) {
    return false;
  }
  if (state != STATE_IDLE) {
    return false;
  }

  uint32_t offset = 0;

  for (uint16_t i = 0; i < data_flash_header.file_num; i++) {
    const uint32_t size = data_flash_header.files[i].size;

    offset += size / PAGE_SIZE;
    if (size % PAGE_SIZE > 0) {
      offset += 1;
    }
  }

  if ((FILES_SECTOR_OFFSET + (offset + 1) * PAGE_SIZE) >= bounds.total_size) {
    // flash is full
    return false;
  }

  data_flash_header.files[data_flash_header.file_num].looptime = looptime;
  data_flash_header.files[data_flash_header.file_num].blackbox_rate = blackbox_rate;
  data_flash_header.files[data_flash_header.file_num].size = 0;
  data_flash_header.files[data_flash_header.file_num].start_page = offset;
  data_flash_header.file_num++;

  state = STATE_ERASE_HEADER;

  ring_buffer_clear(&encode_buffer);

  return true;
}

void data_flash_finish() {
  if (current_file()->size == 0) {
    // file was empty, lets remove it
    data_flash_header.file_num--;
  }

  should_flush = 1;
}

void data_flash_read_backbox(const uint32_t file_index, const uint32_t offset, uint8_t *buffer, const uint32_t size) {
  const data_flash_file_t *file = &data_flash_header.files[file_index];

#ifdef USE_M25P16
  uint32_t read = 0;
  while (read < size) {
    const uint32_t read_size = min(size - read, PAGE_SIZE);

    const uint32_t abs_offset = FILES_SECTOR_OFFSET + file->start_page * PAGE_SIZE + offset + read;
    m25p16_read_addr(M25P16_READ_DATA_BYTES, abs_offset, buffer + read, read_size);

    read += read_size;
  }
#endif
#ifdef USE_SDCARD
  const uint32_t sector_offset = FILES_SECTOR_OFFSET + file->start_page + (offset / PAGE_SIZE);
  const uint32_t sectors = size / PAGE_SIZE + (size % PAGE_SIZE ? 1 : 0);

  while (1) {
    sdcard_update();
    if (sdcard_read_pages(buffer, sector_offset, sectors)) {
      break;
    }
    __NOP();
  }
#endif
}

cbor_result_t data_flash_write_backbox(const blackbox_t *b) {
  static uint8_t buffer[PAGE_SIZE];

  cbor_value_t enc;
  cbor_encoder_init(&enc, buffer, PAGE_SIZE);

  cbor_result_t res = cbor_encode_blackbox_t(&enc, b);
  if (res < CBOR_OK) {
    return res;
  }

  const uint32_t len = cbor_encoder_len(&enc);
  if (len >= ring_buffer_free(&encode_buffer)) {
    return res;
  }

  ring_buffer_write_multi(&encode_buffer, buffer, len);
  return res;
}

#endif