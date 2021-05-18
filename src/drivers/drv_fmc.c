#include "drv_fmc.h"

#include "defines.h"
#include "project.h"

#ifdef F4
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
#endif

extern void failloop(int);

void fmc_lock() {
  FLASH_Lock();
}

void fmc_unlock() {
  FLASH_Unlock();
}

uint8_t fmc_erase() {
#ifdef F4
  FLASH_ClearFlag(FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR |
                  FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR |
                  FLASH_FLAG_OPERR | FLASH_FLAG_EOP); // clear error status
  int result = FLASH_EraseSector(FLASH_Sector_3, VoltageRange_3);
#endif

  if (result != FLASH_COMPLETE) {
    FLASH_Lock();
    return 1; // error occured
  }
  return 0;
}

uint32_t fmc_read(uint32_t addr) {
  return *((uint32_t *)(FLASH_ADDR + addr * 4));
}

float fmc_read_float(unsigned long address) {
  uint32_t result = fmc_read(address);
  return *((float *)&result);
}

void fmc_write(uint32_t addr, uint32_t value) {
  int result = FLASH_ProgramWord(FLASH_ADDR + (addr * 4), value);
  if (result != FLASH_COMPLETE) {
    FLASH_Lock();
    failloop(5);
  }
}

void fmc_write_float(uint32_t addr, float value) {
  fmc_write(addr, *((uint32_t *)&value));
}
