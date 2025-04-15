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
rx_bind_storage_t bind_storage;

CBOR_START_STRUCT_ENCODER(rx_bind_storage_t)
CBOR_ENCODE_MEMBER(bind_saved, uint8_t)
CBOR_ENCODE_BSTR_MEMBER(raw, BIND_RAW_STORAGE_SIZE)
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_DECODER(rx_bind_storage_t)
CBOR_DECODE_MEMBER(bind_saved, uint8_t)
CBOR_DECODE_BSTR_MEMBER(raw, BIND_RAW_STORAGE_SIZE)
CBOR_END_STRUCT_DECODER()

static void flash_write_magic(uint8_t *data, uint32_t magic) {
  *((uint32_t *)data) = magic;
}

static bool flash_compare_magic(uint32_t addr, uint32_t magic) {
  return ((uint32_t)fmc_read(addr)) == magic;
}

void flash_save() {
  rx_stop();

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
    memset(buffer, 0, BIND_STORAGE_SIZE);
    flash_write_magic(buffer, FMC_MAGIC | BIND_STORAGE_OFFSET);

    if (bind_storage.bind_saved == 0) {
      // reset all bind data
      memset(bind_storage.raw, 0, BIND_RAW_STORAGE_SIZE);
    }

    memcpy(buffer + FMC_MAGIC_SIZE, (uint8_t *)&bind_storage, sizeof(rx_bind_storage_t));
    fmc_write_buf(BIND_STORAGE_OFFSET, buffer, BIND_STORAGE_SIZE);
  }

  {
    memset(buffer, 0, PROFILE_STORAGE_SIZE);
    flash_write_magic(buffer, FMC_MAGIC | PROFILE_STORAGE_OFFSET);

    cbor_value_t enc;
    cbor_encoder_init(&enc, buffer + FMC_MAGIC_SIZE, PROFILE_STORAGE_SIZE - FMC_MAGIC_SIZE);

    cbor_result_t res = cbor_encode_profile_t(&enc, &profile);
    if (res < CBOR_OK) {
      fmc_lock();
      __enable_irq();
      failloop(FAILLOOP_FAULT);
    }

    fmc_write_buf(PROFILE_STORAGE_OFFSET, buffer, PROFILE_STORAGE_SIZE);
  }

#ifdef USE_VTX
  {
    flash_write_magic(buffer, FMC_MAGIC | VTX_STORAGE_OFFSET);

    cbor_value_t enc;
    cbor_encoder_init(&enc, buffer + FMC_MAGIC_SIZE, VTX_STORAGE_SIZE - FMC_MAGIC_SIZE);

    cbor_result_t res = cbor_encode_vtx_settings_t(&enc, &vtx_settings);
    if (res < CBOR_OK) {
      fmc_lock();
      __enable_irq();
      failloop(FAILLOOP_FAULT);
    }

    fmc_write_buf(VTX_STORAGE_OFFSET, buffer, VTX_STORAGE_SIZE);
  }
#endif

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

  if (flash_compare_magic(BIND_STORAGE_OFFSET, (FMC_MAGIC | BIND_STORAGE_OFFSET))) {
    fmc_read_buf(BIND_STORAGE_OFFSET, buffer, BIND_STORAGE_SIZE);
    memcpy((uint8_t *)&bind_storage, buffer + FMC_MAGIC_SIZE, sizeof(rx_bind_storage_t));

  } else {
#ifdef EXPRESS_LRS_UID
    const uint8_t uid[6] = {EXPRESS_LRS_UID};
    bind_storage.bind_saved = 1;

    bind_storage.elrs.is_set = 0x1;
    bind_storage.elrs.magic = 0x37;
    memcpy(bind_storage.elrs.uid, uid, 6);
#endif
  }

  profile_set_defaults();

  if (flash_compare_magic(PROFILE_STORAGE_OFFSET, (FMC_MAGIC | PROFILE_STORAGE_OFFSET))) {
    fmc_read_buf(PROFILE_STORAGE_OFFSET, buffer, PROFILE_STORAGE_SIZE);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buffer + FMC_MAGIC_SIZE, PROFILE_STORAGE_SIZE - FMC_MAGIC_SIZE);

    cbor_result_t res = cbor_decode_profile_t(&dec, &profile);
    if (res < CBOR_OK) {
      failloop(FAILLOOP_FAULT);
    }
  }

#ifdef USE_VTX
  if (flash_compare_magic(VTX_STORAGE_OFFSET, (FMC_MAGIC | VTX_STORAGE_OFFSET))) {
    fmc_read_buf(VTX_STORAGE_OFFSET, buffer, VTX_STORAGE_SIZE);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buffer + FMC_MAGIC_SIZE, VTX_STORAGE_SIZE - FMC_MAGIC_SIZE);

    cbor_result_t res = cbor_decode_vtx_settings_t(&dec, &vtx_settings);
    if (res < CBOR_OK) {
      failloop(FAILLOOP_FAULT);
    }
  }
#endif

  free(buffer);
}
