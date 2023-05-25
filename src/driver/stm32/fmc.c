#include "driver/fmc.h"

#include <string.h>

#include "core/failloop.h"
#include "core/project.h"

#if defined(STM32F4)
#define FLASH_FLAG_ALL_ERRORS (FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)
#endif

#if defined(STM32H7)
#define FLASH_FLAG_ALL_ERRORS (FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR)
#endif

#ifdef STM32H7
#define PROGRAM_TYPE FLASH_TYPEPROGRAM_FLASHWORD
#else
#define PROGRAM_TYPE FLASH_TYPEPROGRAM_WORD
#endif

#define FLASH_PTR(offset) (_config_flash + FLASH_ALIGN(offset))

uint8_t __attribute__((section(".config_flash"))) _config_flash[16384];

void fmc_lock() {
  HAL_FLASH_Lock();
}

void fmc_unlock() {
  HAL_FLASH_Unlock();
}

uint8_t fmc_erase() {
  // clear error status
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
#ifdef STM32H7
  FLASH_Erase_Sector(FLASH_SECTOR_1, FLASH_BANK_BOTH, FLASH_VOLTAGE_RANGE_3);
#else
  FLASH_Erase_Sector(FLASH_SECTOR_3, FLASH_VOLTAGE_RANGE_3);
#endif
  return 0;
}

flash_word_t fmc_read(uint32_t addr) {
  return *((flash_word_t *)(_config_flash + addr));
}

void fmc_read_buf(uint32_t offset, uint8_t *data, uint32_t size) {
  flash_word_t *ptr = (flash_word_t *)data;

  for (uint32_t i = 0; i < (size / sizeof(flash_word_t)); i++) {
    ptr[i] = fmc_read(offset + i * sizeof(flash_word_t));
  }
}

void fmc_write(uint32_t offset, flash_word_t value) {
#ifdef STM32H7
  uint8_t data[FLASH_WORD_SIZE];
  memcpy(data, &value, sizeof(flash_word_t));

  HAL_StatusTypeDef result = HAL_FLASH_Program(PROGRAM_TYPE, (uint32_t)FLASH_PTR(offset), (uint64_t)(uint32_t)(data));
#else
  HAL_StatusTypeDef result = HAL_FLASH_Program(PROGRAM_TYPE, (uint32_t)FLASH_PTR(offset), value);
#endif

  if (result != HAL_OK) {
    fmc_lock();
    failloop(FAILLOOP_FAULT);
  }
}

void fmc_write_buf(uint32_t offset, uint8_t *data, uint32_t size) {
  for (uint32_t i = 0; i < (size / FLASH_WORD_SIZE); i++) {
    const uint32_t addr = (uint32_t)FLASH_PTR(offset + i * FLASH_WORD_SIZE);

#ifdef STM32H7
    const uint32_t ptr = (uint32_t)(data + i * FLASH_WORD_SIZE);
    HAL_StatusTypeDef result = HAL_FLASH_Program(PROGRAM_TYPE, addr, ptr);
#else
    flash_word_t *ptr = (flash_word_t *)data;
    HAL_StatusTypeDef result = HAL_FLASH_Program(PROGRAM_TYPE, addr, ptr[i]);
#endif

    if (result != HAL_OK) {
      fmc_lock();
      failloop(FAILLOOP_FAULT);
    }
  }
}
