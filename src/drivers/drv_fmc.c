#include "drv_fmc.h"

#include "failloop.h"
#include "project.h"

#ifdef STM32F4
/*
Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
Sector 8    0x08080000 - 0x0809FFFF 128 Kbytes
Sector 9    0x080A0000 - 0x080BFFFF 128 Kbytes
Sector 10   0x080C0000 - 0x080DFFFF 128 Kbytes
Sector 11   0x080E0000 - 0x080FFFFF 128 Kbytes
*/
#define FLASH_ADDR 0x0800C000 //sector 3 address
#define FLASH_FLAG_ALL_ERRORS (FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)
#endif

#ifdef STM32F7
/*
Sector 0 0x0800 0000 - 0x0800 7FFF 0x0020 0000 - 0x0020 7FFF 32 Kbytes
Sector 1 0x0800 8000 - 0x0800 FFFF 0x0020 8000 - 0x0020 FFFF 32 Kbytes
Sector 2 0x0801 0000 - 0x0801 7FFF 0x0021 0000 - 0x0021 7FFF 32 Kbytes
Sector 3 0x0801 8000 - 0x0801 FFFF 0x0021 8000 - 0x0021 FFFF 32 Kbytes
Sector 4 0x0802 0000 - 0x0803 FFFF 0x0022 0000 - 0x0023 FFFF 128 Kbytes
Sector 5 0x0804 0000 - 0x0807 FFFF 0x0024 0000 - 0x0027 FFFF 256 Kbytes
Sector 6 0x0808 0000 - 0x080B FFFF 0x0028 0000 - 0x002B FFFF 256 Kbytes
Sector 7 0x080C 0000 - 0x080F FFFF 0x002C 0000 - 0x02F FFFF 256 Kbytes
*/
#define FLASH_ADDR 0x8018000 //sector 3 address
#endif

void fmc_lock() {
  HAL_FLASH_Lock();
}

void fmc_unlock() {
  HAL_FLASH_Unlock();
}

uint8_t fmc_erase() {
  // clear error status
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  FLASH_Erase_Sector(FLASH_SECTOR_3, FLASH_VOLTAGE_RANGE_3);
  return 0;
}

uint32_t fmc_read(uint32_t addr) {
  return *((uint32_t *)(FLASH_ADDR + addr * 4));
}

float fmc_read_float(uint32_t address) {
  uint32_t result = fmc_read(address);
  return *((float *)&result);
}

void fmc_write(uint32_t addr, uint32_t value) {
  HAL_StatusTypeDef result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR + (addr * 4), value);
  if (result != HAL_OK) {
    fmc_lock();
    failloop(FAILLOOP_FAULT);
  }
}

void fmc_write_float(uint32_t addr, float value) {
  fmc_write(addr, *((uint32_t *)&value));
}
