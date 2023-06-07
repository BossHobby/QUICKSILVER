#include "driver/reset.h"

#include <stdint.h>

#include "core/project.h"

#define BKP_INDEX ERTC_DT4
#define BOOTLOADER_OFFSET 0x1FFF0000

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

__attribute__((__used__)) void system_check_for_bootloader() {
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