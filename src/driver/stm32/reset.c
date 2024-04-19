#include "driver/reset.h"

#include <stdint.h>

#include "core/project.h"

#ifdef STM32F4
#define BOOTLOADER_OFFSET 0x1FFF0000
#endif

#ifdef STM32G4
#define BOOTLOADER_OFFSET 0x1FFF0000
#define LL_RTC_BAK_SetRegister LL_RTC_BKP_SetRegister
#define LL_RTC_BAK_GetRegister LL_RTC_BKP_GetRegister
#endif

#ifdef STM32F7
#define BOOTLOADER_OFFSET 0x1FF00000
#endif

#ifdef STM32H7
#define BOOTLOADER_OFFSET 0x1FF09800
#endif

#define BKP_INDEX LL_RTC_BKP_DR4

static void backup_register_write(uint32_t val) {
#ifndef STM32H7
  __HAL_RCC_PWR_CLK_ENABLE();
#endif

  LL_PWR_EnableBkUpAccess();
  LL_RTC_BAK_SetRegister(RTC, BKP_INDEX, val);
  LL_PWR_DisableBkUpAccess();
}

void system_reset_to_bootloader() {
  backup_register_write(RESET_BOOTLOADER_MAGIC);
  system_reset();
}

__attribute__((__used__)) void system_check_for_bootloader() {
  const uint32_t magic = LL_RTC_BAK_GetRegister(RTC, BKP_INDEX);

  switch (magic) {
  case RESET_BOOTLOADER_MAGIC: {
    backup_register_write(0);

#ifdef STM32G4
    HAL_RCC_DeInit();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
#endif

    void (*DfuBootJump)(void) = (void (*)(void))(*((uint32_t *)(BOOTLOADER_OFFSET + 4)));
    __set_MSP(*((uint32_t *)BOOTLOADER_OFFSET));
    DfuBootJump();
    break;
  }

  case RESET_FIRMWARE_MAGIC:
    break;

  default:
    backup_register_write(RESET_FIRMWARE_MAGIC);
    system_reset();
    break;
  }
}