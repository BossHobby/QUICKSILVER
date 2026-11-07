#include "driver/fmc.h"

#include <string.h>

#include <hardware/flash.h>

#include "core/failloop.h"
#include "core/project.h"

uint8_t __attribute__((section(".config_flash"), aligned(FLASH_SECTOR_SIZE))) _config_flash[16384];

static uint32_t fmc_flash_offset(uint32_t addr) {
  return (uint32_t)_config_flash - XIP_BASE + addr;
}

// The SDK flash routines run from RAM and suspend XIP. Callers already hold
// interrupts off, and the second core is never started.
void fmc_lock() {}

void fmc_unlock() {}

void fmc_erase() {
  flash_range_erase(fmc_flash_offset(0), sizeof(_config_flash));
}

flash_word_t fmc_read(uint32_t addr) {
  return *((flash_word_t *)(_config_flash + addr));
}

void fmc_read_buf(uint32_t start, uint8_t *data, uint32_t size) {
  if (size < FLASH_WORD_SIZE || (size % FLASH_WORD_SIZE) || (start + size) > sizeof(_config_flash)) {
    failloop(FAILLOOP_FAULT);
  }
  memcpy(data, _config_flash + start, size);
}

// Flash programs whole 256-byte pages, and the storage blocks share pages.
// Each page is rewritten with its current contents plus the new bytes;
// reprogramming bytes with the value they already hold leaves them intact.
void fmc_write_buf(uint32_t start, uint8_t *data, uint32_t size) {
  if (size < FLASH_WORD_SIZE || (size % FLASH_WORD_SIZE) || (start + size) > sizeof(_config_flash)) {
    failloop(FAILLOOP_FAULT);
  }

  uint8_t page[FLASH_PAGE_SIZE];
  const uint32_t end = start + size;
  for (uint32_t page_start = start - start % FLASH_PAGE_SIZE; page_start < end; page_start += FLASH_PAGE_SIZE) {
    memcpy(page, _config_flash + page_start, FLASH_PAGE_SIZE);

    const uint32_t from = MAX(start, page_start);
    const uint32_t to = MIN(end, page_start + FLASH_PAGE_SIZE);
    memcpy(page + (from - page_start), data + (from - start), to - from);

    flash_range_program(fmc_flash_offset(page_start), page, FLASH_PAGE_SIZE);
  }
}
