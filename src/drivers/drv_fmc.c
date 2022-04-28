#include "drv_fmc.h"

#include "failloop.h"
#include "project.h"

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
  FLASH_Erase_Sector(FLASH_SECTOR_3, FLASH_BANK_BOTH, FLASH_VOLTAGE_RANGE_3);
#else
  FLASH_Erase_Sector(FLASH_SECTOR_3, FLASH_VOLTAGE_RANGE_3);
#endif
  return 0;
}

uint32_t fmc_read(uint32_t addr) {
  return *((uint32_t *)(_config_flash + addr * 4));
}

float fmc_read_float(uint32_t address) {
  uint32_t result = fmc_read(address);
  return *((float *)&result);
}

void fmc_write(uint32_t addr, uint32_t value) {
  HAL_StatusTypeDef result = HAL_FLASH_Program(PROGRAM_TYPE, (uint32_t)(_config_flash) + (addr * 4), value);
  if (result != HAL_OK) {
    fmc_lock();
    failloop(FAILLOOP_FAULT);
  }
}

void fmc_write_float(uint32_t addr, float value) {
  fmc_write(addr, *((uint32_t *)&value));
}
