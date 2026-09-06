#include "driver/reset.h"

#include <stdint.h>

#include "core/project.h"

#define BKP_INDEX ERTC_DT4
#define BOOTLOADER_OFFSET 0x1FFF0000
#define SRAM_CONFIG FLASH_EOPB0_SRAM_384K

static void backup_register_write(uint32_t val) {
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);

  pwc_battery_powered_domain_access(TRUE);
  ertc_bpr_data_write(BKP_INDEX, val);
  pwc_battery_powered_domain_access(FALSE);
}

void system_reset_to_bootloader() {
  backup_register_write(RESET_BOOTLOADER_MAGIC);
  system_reset();
}

static inline void wait_for_flash() {
  while (FLASH->sts_bit.obf != RESET)
    ;
}

static inline void write_eopb0_config(flash_usd_eopb0_type data) {
  // unlock all the flash
  FLASH->unlock = FLASH_UNLOCK_KEY1;
  FLASH->unlock = FLASH_UNLOCK_KEY2;
  FLASH->unlock2 = FLASH_UNLOCK_KEY1;
  FLASH->unlock2 = FLASH_UNLOCK_KEY2;

  // unlock user bytes
  FLASH->usd_unlock = FLASH_UNLOCK_KEY1;
  FLASH->usd_unlock = FLASH_UNLOCK_KEY2;

  // wait for unlock success
  while (FLASH->ctrl_bit.usdulks == RESET)
    ;

  // erase user flash
  FLASH->ctrl_bit.usders = TRUE;
  FLASH->ctrl_bit.erstr = TRUE;
  wait_for_flash();
  FLASH->ctrl_bit.usders = FALSE;

  // re-program FAP and set EOPB0
  FLASH->ctrl_bit.usdprgm = TRUE;
  USD->fap = (uint16_t)FAP_RELIEVE_KEY;
  USD->eopb0 = (uint16_t)data;
  wait_for_flash();
  FLASH->ctrl_bit.usdprgm = FALSE;
}

extern "C" __attribute__((__used__)) void system_check_for_bootloader() {
  if (((USD->eopb0) & 0x07) != SRAM_CONFIG) {
    write_eopb0_config(SRAM_CONFIG);
    system_reset();
  }

  const uint32_t magic = ertc_bpr_data_read(BKP_INDEX);

  switch (magic) {
  case RESET_BOOTLOADER_MAGIC: {
    backup_register_write(0);

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