#include "core/reset.h"

#include <stdint.h>

#include "core/project.h"

#define BKP_INDEX LL_RTC_BKP_DR4

#define FIRMWARE_MAGIC 0xCAFEBABE
#define BOOTLOADER_MAGIC 0xDEADBEEF

#ifdef STM32F4
#define BOOTLOADER_OFFSET 0x1FFF0000
#endif

#ifdef STM32F7
#define BOOTLOADER_OFFSET 0x1FF00000
#endif

#ifdef STM32H7
#define BOOTLOADER_OFFSET 0x1FF09800
#endif

void system_reset() {
  NVIC_SystemReset();
}

static void backup_register_write(uint32_t val) {
#ifndef STM32H7
  __HAL_RCC_PWR_CLK_ENABLE();
#endif

  LL_PWR_EnableBkUpAccess();
  LL_RTC_BAK_SetRegister(RTC, BKP_INDEX, val);
  LL_PWR_DisableBkUpAccess();
}

void system_reset_to_bootloader() {
  backup_register_write(BOOTLOADER_MAGIC);
  system_reset();
}

__attribute__((__used__)) void system_check_for_bootloader() {
  const uint32_t magic = LL_RTC_BAK_GetRegister(RTC, BKP_INDEX);

  switch (magic) {
  case BOOTLOADER_MAGIC: {
    backup_register_write(0);

    void (*DfuBootJump)(void) = (void (*)(void))(*((uint32_t *)(BOOTLOADER_OFFSET + 4)));
    __set_MSP(*((uint32_t *)BOOTLOADER_OFFSET));
    DfuBootJump();
    break;
  }

  case FIRMWARE_MAGIC:
    break;

  default:
    backup_register_write(FIRMWARE_MAGIC);
    system_reset();
    break;
  }
}