#include "data_flash.h"

#include <string.h>

#include "drv_spi_m25p16.h"
#include "drv_spi_sdcard.h"
#include "drv_time.h"
#include "usb_configurator.h"
#include "util.h"

#ifdef ENABLE_BLACKBOX

#ifdef USE_M25P16
#define FILES_SECTOR_OFFSET bounds.sector_size
#define ENTRIES_PER_BLOCK (256 / BLACKBOX_MAX_SIZE)
#endif
#ifdef USE_SDCARD
#define FILES_SECTOR_OFFSET 1
#define FLUSH_INTERVAL 4
#define ENTRIES_PER_BLOCK (512 / BLACKBOX_MAX_SIZE)
#endif

typedef enum {
  STATE_IDLE,
  STATE_START_MULTI_WRITE,
  STATE_START_WRITE,
  STATE_CONTINUE_WRITE,
  STATE_FINISH_WRITE,
  STATE_FINISH_MULTI_WRITE,
  STATE_ERASE_HEADER,
  STATE_WRITE_HEADER,
  STATE_FLUSH,
} data_flash_state_t;

data_flash_header_t data_flash_header;

static data_flash_state_t state = STATE_IDLE;
static blackbox_t write_buffer[16];
static uint32_t write_offset = 0;
static uint32_t written_offset = 0;

static data_flash_bounds_t bounds;
static data_flash_file_t *current_file() {
  return &data_flash_header.files[data_flash_header.file_num - 1];
}

static volatile uint32_t update_delta = 0;

uint8_t data_flash_update(uint32_t loop) {
  static uint32_t offset = 0;

  const uint32_t to_write = write_offset >= written_offset ? write_offset - written_offset : 16 + write_offset - written_offset;
  uint8_t write_in_progress = 0;

#ifdef USE_SDCARD
  switch (state) {
  case STATE_IDLE:
    if (to_write > 0) {
      state = STATE_START_MULTI_WRITE;
    }
    break;

  case STATE_WAIT_FOR_READY:
    break;

  case STATE_START_MULTI_WRITE: {
    if (sdcard_start_multi_write(offset)) {
      state = STATE_START_WRITE;
    }
    break;
  }

  case STATE_START_WRITE: {
    offset = FILES_SECTOR_OFFSET + current_file()->start_sector + (current_file()->entries / ENTRIES_PER_BLOCK);
    if (sdcard_start_write_sector(offset)) {
      state = STATE_CONTINUE_WRITE;
    }
    break;
  }

  case STATE_CONTINUE_WRITE: {
    if (to_write <= 0) {
      break;
    }

    const uint32_t index = current_file()->entries % ENTRIES_PER_BLOCK;
    sdcard_continue_write_sector(index * BLACKBOX_MAX_SIZE, &write_buffer[written_offset], sizeof(blackbox_t));
    written_offset = (written_offset + 1) % 16;

    if (index == ENTRIES_PER_BLOCK - 1) {
      state = STATE_FINISH_WRITE;
    } else {
      state = STATE_WAIT_FOR_READY;
      state_after_ready = STATE_CONTINUE_WRITE;
    }

    current_file()->entries += 1;
    break;
  }

  case STATE_FINISH_WRITE: {
    static uint32_t count = 0;
    if (!sdcard_finish_write_sector(offset)) {
      break;
    }
    count++;

    if (count == FLUSH_INTERVAL) {
      count = 0;
      state = STATE_FINISH_MULTI_WRITE;
    } else {
      state = STATE_START_WRITE;
    }
    break;
  }

  case STATE_FINISH_MULTI_WRITE: {
    if (sdcard_finish_multi_write(offset)) {
      state = STATE_IDLE;
    }
    break;
  }

  case STATE_ERASE_HEADER: {
  }
  }
#endif

#ifdef USE_M25P16
  if (!m25p16_is_ready()) {
    return 0;
  }

  switch (state) {
  case STATE_IDLE:
    if (to_write > 0) {
      state = STATE_START_WRITE;
    }
    break;

  case STATE_START_WRITE: {
    offset = FILES_SECTOR_OFFSET + current_file()->start_sector * bounds.sector_size + (current_file()->entries / ENTRIES_PER_BLOCK) * 256;
    if (offset >= bounds.total_size) {
      break;
    }
    state = STATE_CONTINUE_WRITE;
    break;
  }

  case STATE_CONTINUE_WRITE: {
    if (to_write <= 0) {
      break;
    }

    const uint32_t index = current_file()->entries % ENTRIES_PER_BLOCK;
    write_in_progress = 1;
    if (!m25p16_page_program(offset + index * BLACKBOX_MAX_SIZE, (const uint8_t *)&write_buffer[written_offset], sizeof(blackbox_t))) {
      break;
    }
    written_offset = (written_offset + 1) % 16;

    if (index == ENTRIES_PER_BLOCK - 1) {
      state = STATE_FINISH_WRITE;
    } else {
      state = STATE_CONTINUE_WRITE;
    }

    current_file()->entries += 1;
    break;
  }

  case STATE_FINISH_WRITE: {
    state = STATE_IDLE;
    break;
  }

  case STATE_ERASE_HEADER: {
    m25p16_write_addr(M25P16_SECTOR_ERASE, 0x0, NULL, 0);
    state = STATE_WRITE_HEADER;
    break;
  }

  case STATE_WRITE_HEADER: {
    if (!m25p16_page_program(0x0, (uint8_t *)&data_flash_header, sizeof(data_flash_header_t))) {
      break;
    }
    state = STATE_IDLE;
    break;
  }

  case STATE_START_MULTI_WRITE:
  case STATE_FINISH_MULTI_WRITE:
    break;
  }
#endif

  return write_in_progress;
}

cbor_result_t data_flash_read_header(data_flash_header_t *h) {
#ifdef USE_M25P16
  m25p16_read_addr(M25P16_READ_DATA_BYTES, 0x0, (uint8_t *)h, sizeof(data_flash_header_t));
#endif
#ifdef USE_SDCARD
  uint8_t buf[512];
  while (!sdcard_read_sectors(buf, 0, 1))
    ;

  memcpy(h, buf, sizeof(data_flash_header_t));
#endif
  return CBOR_OK;
}

cbor_result_t data_flash_write_header(data_flash_header_t *h) {
#ifdef USE_M25P16
  m25p16_write_addr(M25P16_SECTOR_ERASE, 0x0, NULL, 0);
  m25p16_write_addr(M25P16_PAGE_PROGRAM, 0x0, (uint8_t *)h, sizeof(data_flash_header_t));
#endif
#ifdef USE_SDCARD
  uint8_t buf[512];
  memcpy(buf, h, sizeof(data_flash_header_t));

  while (!sdcard_write_sectors(buf, 0, 1))
    ;
#endif
  return CBOR_OK;
}

void data_flash_init() {
#ifdef USE_M25P16
  m25p16_init();
  m25p16_get_bounds(&bounds);
#endif
#ifdef USE_SDCARD
  sdcard_init();
  sdcard_detect();

  bounds.page_size = 512;
  bounds.pages_per_sector = 1;

  // TODO: fetch
  bounds.sectors = 512;

  bounds.sector_size = bounds.pages_per_sector * bounds.page_size;
  bounds.total_size = bounds.sector_size * bounds.sectors;
#endif

  data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
  data_flash_header.file_num = 0;
  data_flash_read_header(&data_flash_header);
  if (data_flash_header.magic != DATA_FLASH_HEADER_MAGIC) {
    data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
    data_flash_header.file_num = 0;
  }
}

void data_flash_reset() {
#ifdef USE_M25P16
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_command(M25P16_BULK_ERASE);
#endif

  data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
  data_flash_header.file_num = 0;
  data_flash_write_header(&data_flash_header);
  reset_looptime();
}

void data_flash_restart() {
  uint32_t offset = 0;

  for (uint16_t i = 0; i < data_flash_header.file_num; i++) {
    const uint32_t size = data_flash_header.files[i].entries * 0x80;

    offset += size / bounds.sector_size;
    if (size % bounds.sector_size > 0) {
      offset += 1;
    }
  }

  data_flash_header.files[data_flash_header.file_num].entries = 0;
  data_flash_header.files[data_flash_header.file_num].start_sector = offset;
  data_flash_header.file_num++;

  state = STATE_ERASE_HEADER;
}

void data_flash_finish() {
  state = STATE_ERASE_HEADER;
}

cbor_result_t data_flash_read_backbox(const uint32_t index, blackbox_t b[], const uint8_t count) {
#ifdef USE_M25P16
  for (uint32_t i = 0; i < count; i++) {
    const uint32_t offset = FILES_SECTOR_OFFSET + current_file()->start_sector * bounds.sector_size + (index + i) * BLACKBOX_MAX_SIZE;
    m25p16_read_addr(M25P16_READ_DATA_BYTES, offset, (uint8_t *)&b[i], sizeof(blackbox_t));
  }
#endif
#ifdef USE_SDCARD
  const uint32_t offset = FILES_SECTOR_OFFSET + current_file()->start_sector + (index / ENTRIES_PER_BLOCK);
  const uint32_t sectors = count / ENTRIES_PER_BLOCK + (count % ENTRIES_PER_BLOCK ? 1 : 0);

  uint8_t buf[sectors * 512];
  while (!sdcard_read_sectors(buf, offset, sectors))
    ;

  for (uint32_t i = 0; i < count; i++) {
    memcpy(b + i, buf + ((index % ENTRIES_PER_BLOCK) + i) * BLACKBOX_MAX_SIZE, sizeof(blackbox_t));
  }
#endif
  return CBOR_OK;
}

cbor_result_t data_flash_write_backbox(const blackbox_t *b) {
  write_buffer[write_offset] = *b;
  write_offset = (write_offset + 1) % 16;
  return CBOR_OK;
}

#endif