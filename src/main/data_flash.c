#include "data_flash.h"

#include <string.h>

#include "drv_spi_m25p16.h"
#include "drv_spi_sdcard.h"
#include "drv_time.h"
#include "usb_configurator.h"
#include "util/util.h"

#ifdef ENABLE_BLACKBOX

#define BLACKBOX_BUFFER_COUNT 64

#ifdef USE_M25P16
#define FILES_SECTOR_OFFSET bounds.sector_size
#define ENTRIES_PER_PAGE (M25P16_PAGE_SIZE / BLACKBOX_MAX_SIZE)
#endif
#ifdef USE_SDCARD
#define FLUSH_INTERVAL 8
#define FILES_SECTOR_OFFSET 1
#define ENTRIES_PER_PAGE (SDCARD_PAGE_SIZE / BLACKBOX_MAX_SIZE)
#endif

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
static uint8_t write_buffer[BLACKBOX_BUFFER_COUNT * BLACKBOX_MAX_SIZE];
static int32_t write_offset = 0;
static int32_t written_offset = 0;
static uint8_t should_flush = 0;

static data_flash_file_t *current_file() {
  return &data_flash_header.files[data_flash_header.file_num - 1];
}

static volatile uint32_t update_delta = 0;

data_flash_result_t data_flash_update() {
  static uint32_t offset = 0;

  const uint32_t to_write = write_offset >= written_offset ? (write_offset - written_offset) : (BLACKBOX_BUFFER_COUNT + write_offset - written_offset);

#ifdef USE_SDCARD
  uint8_t sdcard_ready = sdcard_update();

  switch (state) {
  case STATE_DETECT:
    if (sdcard_ready) {
      state = STATE_READ_HEADER;
      sdcard_get_bounds(&bounds);
    }
    return DATA_FLASH_DETECT;

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
    if (to_write >= ENTRIES_PER_PAGE) {
      state = STATE_START_WRITE;
    } else if (should_flush == 1) {
      state = STATE_ERASE_HEADER;
      should_flush = 0;
    }
    break;

  case STATE_START_WRITE: {
    offset = FILES_SECTOR_OFFSET + current_file()->start_sector + (current_file()->entries / ENTRIES_PER_PAGE);
    if (sdcard_write_pages_start(offset, FLUSH_INTERVAL)) {
      state = STATE_CONTINUE_WRITE;
    }
    break;
  }

  case STATE_CONTINUE_WRITE: {
    if (to_write < ENTRIES_PER_PAGE) {
      if (should_flush == 1) {
        state = STATE_FINISH_WRITE;
      }
      break;
    }

    static uint32_t counter = 0;

    if (sdcard_write_pages_continue(write_buffer + (written_offset * BLACKBOX_MAX_SIZE))) {
      written_offset = (written_offset + ENTRIES_PER_PAGE) % BLACKBOX_BUFFER_COUNT;
      current_file()->entries += ENTRIES_PER_PAGE;

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
    break;
  }

  case STATE_WRITE_HEADER: {
    if (sdcard_write_page(write_buffer, 0)) {
      state = STATE_IDLE;
    }
    break;
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
    if (should_flush == 1) {
      state = STATE_ERASE_HEADER;
      should_flush = 0;
    } else if (to_write >= ENTRIES_PER_PAGE) {
      state = STATE_START_WRITE;
    }
    break;

  case STATE_START_WRITE: {
    offset = FILES_SECTOR_OFFSET + current_file()->start_sector * bounds.sector_size + (current_file()->entries / ENTRIES_PER_PAGE) * M25P16_PAGE_SIZE;
    if (offset >= bounds.total_size) {
      state = STATE_IDLE;
      break;
    }
    state = STATE_CONTINUE_WRITE;
    break;
  }

  case STATE_CONTINUE_WRITE: {
    if (to_write < ENTRIES_PER_PAGE) {
      if (should_flush == 1) {
        state = STATE_FINISH_WRITE;
      }
      break;
    }
    if (!m25p16_is_ready()) {
      break;
    }

    const uint32_t index = current_file()->entries % ENTRIES_PER_PAGE;
    if (!m25p16_page_program(offset + index * BLACKBOX_MAX_SIZE, write_buffer + (written_offset * BLACKBOX_MAX_SIZE), M25P16_PAGE_SIZE)) {
      break;
    }
    written_offset = (written_offset + ENTRIES_PER_PAGE) % BLACKBOX_BUFFER_COUNT;
    current_file()->entries += ENTRIES_PER_PAGE;

    state = STATE_FINISH_WRITE;

    return DATA_FLASH_WRITE;
  }

  case STATE_FINISH_WRITE: {
    state = STATE_IDLE;
    break;
  }

  case STATE_ERASE_HEADER: {
    if (!m25p16_is_ready()) {
      break;
    }
    m25p16_write_addr(M25P16_SECTOR_ERASE, 0x0, NULL, 0);
    state = STATE_WRITE_HEADER;
    break;
  }

  case STATE_WRITE_HEADER: {
    if (!m25p16_is_ready()) {
      break;
    }
    if (!m25p16_page_program(0x0, (uint8_t *)&data_flash_header, sizeof(data_flash_header_t))) {
      break;
    }
    state = STATE_IDLE;
    break;
  }
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

void data_flash_restart(uint32_t blackbox_rate) {
  uint32_t offset = 0;

  for (uint16_t i = 0; i < data_flash_header.file_num; i++) {
    const uint32_t size = data_flash_header.files[i].entries * BLACKBOX_MAX_SIZE;

    offset += size / bounds.sector_size;
    if (size % bounds.sector_size > 0) {
      offset += 1;
    }
  }

  if ((offset + 1) >= bounds.sectors) {
    // flash is full
    return;
  }

  data_flash_header.files[data_flash_header.file_num].blackbox_rate = blackbox_rate;
  data_flash_header.files[data_flash_header.file_num].entries = 0;
  data_flash_header.files[data_flash_header.file_num].start_sector = offset;
  data_flash_header.file_num++;

  state = STATE_ERASE_HEADER;

  write_offset = 0;
  written_offset = 0;
}

void data_flash_finish() {
  should_flush = 1;
}

cbor_result_t data_flash_read_backbox(const uint32_t file_index, const uint32_t index, blackbox_t b[], const uint8_t count) {
  const data_flash_file_t *file = &data_flash_header.files[file_index];

#ifdef USE_M25P16
  for (uint32_t i = 0; i < count; i++) {
    const uint32_t offset = FILES_SECTOR_OFFSET + file->start_sector * bounds.sector_size + (index + i) * BLACKBOX_MAX_SIZE;
    m25p16_read_addr(M25P16_READ_DATA_BYTES, offset, (uint8_t *)&b[i], sizeof(blackbox_t));
  }
#endif
#ifdef USE_SDCARD
  const uint32_t offset = FILES_SECTOR_OFFSET + file->start_sector + (index / ENTRIES_PER_PAGE);
  const uint32_t sectors = count / ENTRIES_PER_PAGE + (count % ENTRIES_PER_PAGE ? 1 : 0);

  static uint8_t buf[4 * SDCARD_PAGE_SIZE];
  while (1) {
    sdcard_update();
    if (sdcard_read_pages(buf, offset, sectors)) {
      break;
    }
    time_delay_us(100);
  }

  for (uint32_t i = 0; i < count; i++) {
    memcpy(b + i, buf + ((index % ENTRIES_PER_PAGE) + i) * BLACKBOX_MAX_SIZE, sizeof(blackbox_t));
  }
#endif
  return CBOR_OK;
}

cbor_result_t data_flash_write_backbox(const blackbox_t *b) {
  memcpy(write_buffer + (write_offset * BLACKBOX_MAX_SIZE), (uint8_t *)b, sizeof(blackbox_t));
  write_offset = (write_offset + 1) % BLACKBOX_BUFFER_COUNT;
  return CBOR_OK;
}

#endif