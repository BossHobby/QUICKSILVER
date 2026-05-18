#include "core/flash.h"

#include <stdlib.h>
#include <string.h>

#include "core/failloop.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/fmc.h"
#include "driver/serial.h"
#include "io/vtx.h"
#include "rx/rx.h"
#include "util/cbor_helper.h"

extern const profile_t default_profile;
extern profile_t profile;

flash_storage_t flash_storage;

static void flash_write_magic(uint8_t *data, uint32_t magic) {
  *((uint32_t *)data) = magic;
}

static bool flash_compare_magic(uint32_t addr, uint32_t magic) {
  return ((uint32_t)fmc_read(addr)) == magic;
}

void flash_save() {
  rx_stop();

  if (sizeof(profile_t) > PROFILE_STORAGE_SIZE - FMC_MAGIC_SIZE) {
    failloop(FAILLOOP_FAULT);
  }

  __disable_irq();

  uint8_t *buffer = (uint8_t *)malloc(PROFILE_STORAGE_SIZE);
  if (buffer == NULL) {
    failloop(FAILLOOP_FAULT);
  }

  fmc_unlock();
  fmc_erase();

  {
    memset(buffer, 0, TARGET_STORAGE_SIZE);
    flash_write_magic(buffer, FMC_MAGIC | TARGET_STORAGE_OFFSET);

    cbor_value_t enc;
    cbor_encoder_init(&enc, buffer + FMC_MAGIC_SIZE, TARGET_STORAGE_SIZE - FMC_MAGIC_SIZE);

    cbor_result_t res = cbor_encode_target_t(&enc, &target);
    if (res < CBOR_OK) {
      fmc_lock();
      __enable_irq();
      failloop(FAILLOOP_FAULT);
    }

    fmc_write_buf(TARGET_STORAGE_OFFSET, buffer, TARGET_STORAGE_SIZE);
  }

  {
    memset(buffer, 0, FLASH_STORAGE_SIZE);
    flash_write_magic(buffer, FMC_MAGIC | FLASH_STORAGE_OFFSET);
    memcpy(buffer + FMC_MAGIC_SIZE, (uint8_t *)&flash_storage, sizeof(flash_storage_t));
    fmc_write_buf(FLASH_STORAGE_OFFSET, buffer, FLASH_STORAGE_SIZE);
  }

  {
    memset(buffer, 0, PROFILE_STORAGE_SIZE);
    flash_write_magic(buffer, FMC_MAGIC | PROFILE_STORAGE_OFFSET);

    if (profile.receiver.bind.bind_saved == 0) {
      // reset all bind data
      memset(profile.receiver.bind.raw, 0, BIND_RAW_STORAGE_SIZE);
    }

    memcpy(buffer + FMC_MAGIC_SIZE, (uint8_t *)&profile, sizeof(profile_t));
    fmc_write_buf(PROFILE_STORAGE_OFFSET, buffer, PROFILE_STORAGE_SIZE);
  }

  fmc_lock();
  free(buffer);
  __enable_irq();
}

void flash_load() {
  uint8_t *buffer = (uint8_t *)malloc(PROFILE_STORAGE_SIZE);
  if (buffer == NULL) {
    failloop(FAILLOOP_FAULT);
  }

  if (flash_compare_magic(TARGET_STORAGE_OFFSET, (FMC_MAGIC | TARGET_STORAGE_OFFSET))) {
    fmc_read_buf(TARGET_STORAGE_OFFSET, buffer, TARGET_STORAGE_SIZE);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buffer + FMC_MAGIC_SIZE, TARGET_STORAGE_SIZE - FMC_MAGIC_SIZE);

    cbor_result_t res = cbor_decode_target_t(&dec, &target);
    if (res < CBOR_OK) {
      failloop(FAILLOOP_FAULT);
    }
  }

  if (flash_compare_magic(FLASH_STORAGE_OFFSET, (FMC_MAGIC | FLASH_STORAGE_OFFSET))) {
    fmc_read_buf(FLASH_STORAGE_OFFSET, buffer, FLASH_STORAGE_SIZE);
    memcpy((uint8_t *)&flash_storage, buffer + FMC_MAGIC_SIZE, sizeof(flash_storage_t));
  }

  profile_set_defaults();

  if (flash_compare_magic(PROFILE_STORAGE_OFFSET, (FMC_MAGIC | PROFILE_STORAGE_OFFSET))) {
    fmc_read_buf(PROFILE_STORAGE_OFFSET, buffer, PROFILE_STORAGE_SIZE);
    memcpy((uint8_t *)&profile, buffer + FMC_MAGIC_SIZE, sizeof(profile_t));
  }

#ifdef EXPRESS_LRS_UID
  if (profile.receiver.bind.bind_saved == 0) {
    const uint8_t uid[6] = {EXPRESS_LRS_UID};
    profile.receiver.bind.bind_saved = 1;

    profile.receiver.bind.elrs.is_set = 0x1;
    profile.receiver.bind.elrs.magic = 0x37;
    memcpy(profile.receiver.bind.elrs.uid, uid, 6);
  }
#endif

  free(buffer);
}
