#include "io/data_flash.h"

#include <string.h>

#include "drv_spi_m25p16.h"
#include "drv_spi_sdcard.h"
#include "drv_time.h"
#include "io/usb_configurator.h"
#include "util/cbor_helper.h"
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
  STATE_CONTINUE_WRITE,
  STATE_FINISH_WRITE,

  STATE_READ_HEADER,

  STATE_ERASE_HEADER,
  STATE_WRITE_HEADER,
} data_flash_state_t;

data_flash_bounds_t bounds;
data_flash_header_t data_flash_header;

static data_flash_state_t state = STATE_DETECT;
static uint8_t write_buffer[BUFFER_SIZE];
static int32_t write_offset = 0;
static int32_t written_offset = 0;
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

  const uint32_t to_write = write_offset >= written_offset ? (write_offset - written_offset) : (BUFFER_SIZE + write_offset - written_offset);

#ifdef USE_SDCARD
  sdcard_status_t sdcard_status = sdcard_update();
  if (sdcard_status != SDCARD_IDLE) {
    if (state == STATE_DETECT) {
      return DATA_FLASH_DETECT;
    } else {
      return DATA_FLASH_WAIT;
    }
  }

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
    }
    break;
  }

  case STATE_IDLE:
    if (to_write >= PAGE_SIZE) {
      state = STATE_START_WRITE;
      break;
    }
    if (should_flush == 1 && to_write > 0) {
      state = STATE_START_WRITE;
      break;
    }
    if (should_flush == 1) {
      state = STATE_ERASE_HEADER;
      should_flush = 0;
      break;
    }
    break;

  case STATE_START_WRITE: {
    offset = FILES_SECTOR_OFFSET + current_file()->start_page + (current_file()->size / PAGE_SIZE);
    if (sdcard_write_pages_start(offset, FLUSH_INTERVAL)) {
      state = STATE_CONTINUE_WRITE;
    }
    break;
  }

  case STATE_CONTINUE_WRITE: {
    uint32_t write_size = PAGE_SIZE;
    if (to_write < PAGE_SIZE) {
      if (should_flush == 0) {
        break;
      }
      if (to_write == 0) {
        state = STATE_FINISH_WRITE;
        break;
      }

      write_size = to_write;
    }

    static uint32_t counter = 0;

    if (sdcard_write_pages_continue(write_buffer + written_offset)) {
      written_offset = (written_offset + write_size) % BUFFER_SIZE;
      current_file()->size += write_size;

      counter++;
      if (counter == FLUSH_INTERVAL) {
        counter = 0;
        state = STATE_FINISH_WRITE;
      }
    }
    break;
  }

  case STATE_FINISH_WRITE: {
    if (sdcard_write_pages_finish()) {
      state = STATE_IDLE;
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
    if (to_write >= PAGE_SIZE) {
      state = STATE_START_WRITE;
      break;
    }
    if (should_flush == 1 && to_write > 0) {
      state = STATE_START_WRITE;
      break;
    }
    if (should_flush == 1) {
      state = STATE_ERASE_HEADER;
      should_flush = 0;
      break;
    }
    break;

  case STATE_START_WRITE: {
    offset = FILES_SECTOR_OFFSET + current_file()->start_page * PAGE_SIZE + current_file()->size;
    if (offset >= bounds.total_size) {
      state = STATE_IDLE;
      break;
    }
    state = STATE_CONTINUE_WRITE;
    break;
  }

  case STATE_CONTINUE_WRITE: {
    uint32_t write_size = PAGE_SIZE;
    if (to_write < PAGE_SIZE) {
      if (should_flush == 0) {
        break;
      }
      if (to_write == 0) {
        state = STATE_FINISH_WRITE;
        break;
      }

      write_size = to_write;
    }

    if (!m25p16_page_program(offset, write_buffer + written_offset, PAGE_SIZE)) {
      break;
    }
    written_offset = (written_offset + write_size) % BUFFER_SIZE;
    current_file()->size += write_size;

    state = STATE_FINISH_WRITE;

    return DATA_FLASH_WRITE;
  }

  case STATE_FINISH_WRITE: {
    state = STATE_IDLE;
    break;
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

void data_flash_reset() {
#ifdef USE_M25P16
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_command(M25P16_BULK_ERASE);

  m25p16_wait_for_ready();
#endif

  data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
  data_flash_header.file_num = 0;

  state = STATE_ERASE_HEADER;
}

void data_flash_restart(uint32_t blackbox_rate, uint32_t looptime) {
  uint32_t offset = 0;

  for (uint16_t i = 0; i < data_flash_header.file_num; i++) {
    const uint32_t size = data_flash_header.files[i].size;

    offset += size / PAGE_SIZE;
    if (size % PAGE_SIZE > 0) {
      offset += 1;
    }
  }

  if ((offset + 1) * PAGE_SIZE >= bounds.total_size) {
    // flash is full
    return;
  }

  data_flash_header.files[data_flash_header.file_num].looptime = looptime;
  data_flash_header.files[data_flash_header.file_num].blackbox_rate = blackbox_rate;
  data_flash_header.files[data_flash_header.file_num].size = 0;
  data_flash_header.files[data_flash_header.file_num].start_page = offset;
  data_flash_header.file_num++;

  state = STATE_ERASE_HEADER;

  write_offset = 0;
  written_offset = 0;
}

void data_flash_finish() {
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
    __WFI();
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
  for (uint32_t i = 0; i < len; i++) {
    write_buffer[write_offset] = buffer[i];
    write_offset = (write_offset + 1) % BUFFER_SIZE;
  }

  return res;
}

#endif