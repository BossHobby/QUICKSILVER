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

    __disable_irq();

    SysTick->CTRL = 0;

    HAL_RCC_DeInit();
#ifdef STM32G4
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
#endif

    for (uint8_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++)
      NVIC->ICER[i] = 0xFFFFFFFF;
    for (uint8_t i = 0; i < sizeof(NVIC->ICPR) / sizeof(NVIC->ICPR[0]); i++)
      NVIC->ICPR[i] = 0xFFFFFFFF;

    __enable_irq();

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