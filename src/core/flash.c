#include "core/flash.h"

#include <string.h>

#include "core/failloop.h"
#include "core/profile.h"
#include "driver/fmc.h"
#include "driver/serial.h"
#include "io/vtx.h"
#include "project.h"
#include "util/cbor_helper.h"

#define FMC_HEADER 0x12AA0001

extern const profile_t default_profile;
extern profile_t profile;

flash_storage_t flash_storage;
rx_bind_storage_t bind_storage;

CBOR_START_STRUCT_ENCODER(rx_bind_storage_t)
CBOR_ENCODE_MEMBER(bind_saved, uint8)
CBOR_ENCODE_BSTR_MEMBER(raw, BIND_RAW_STORAGE_SIZE)
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_DECODER(rx_bind_storage_t)
CBOR_DECODE_MEMBER(bind_saved, uint8)
CBOR_DECODE_BSTR_MEMBER(raw, BIND_RAW_STORAGE_SIZE)
CBOR_END_STRUCT_DECODER()

void flash_save() {
  fmc_unlock();
  fmc_erase();

  fmc_write(0, FMC_HEADER);

  {
    uint8_t buffer[FLASH_STORAGE_SIZE];

    memcpy(buffer, (uint8_t *)&flash_storage, sizeof(flash_storage_t));

    fmc_write_buf(FLASH_STORAGE_OFFSET, buffer, FLASH_STORAGE_SIZE);
  }

  {
    uint8_t buffer[BIND_STORAGE_SIZE];
    memset(buffer, 0, BIND_STORAGE_SIZE);

    if (bind_storage.bind_saved == 0) {
      // reset all bind data
      memset(bind_storage.raw, 0, BIND_RAW_STORAGE_SIZE);
    }

    memcpy(buffer, (uint8_t *)&bind_storage, sizeof(rx_bind_storage_t));

    fmc_write_buf(BIND_STORAGE_OFFSET, buffer, BIND_STORAGE_SIZE);
  }

  {
    uint8_t buffer[PROFILE_STORAGE_SIZE];

    cbor_value_t enc;
    cbor_encoder_init(&enc, buffer, PROFILE_STORAGE_SIZE);

    cbor_result_t res = cbor_encode_profile_t(&enc, &profile);
    if (res < CBOR_OK) {
      fmc_lock();
      failloop(FAILLOOP_FAULT);
    }

    fmc_write_buf(PROFILE_STORAGE_OFFSET, buffer, PROFILE_STORAGE_SIZE);
  }

  {
    uint8_t buffer[VTX_STORAGE_SIZE];

    cbor_value_t enc;
    cbor_encoder_init(&enc, buffer, VTX_STORAGE_SIZE);

    cbor_result_t res = cbor_encode_vtx_settings_t(&enc, &vtx_settings);
    if (res < CBOR_OK) {
      fmc_lock();
      failloop(FAILLOOP_FAULT);
    }

    fmc_write_buf(VTX_STORAGE_OFFSET, buffer, VTX_STORAGE_SIZE);
  }

  fmc_write(FMC_END_OFFSET, FMC_HEADER);
  fmc_lock();
}

void flash_load() {
  // check if saved data is present
  if (fmc_read(0) != FMC_HEADER || fmc_read(FMC_END_OFFSET) != FMC_HEADER) {
    // Flash was empty, load defaults?

#ifdef EXPRESS_LRS_UID
    const uint8_t uid[6] = {EXPRESS_LRS_UID};
    bind_storage.bind_saved = 1;

    bind_storage.elrs.is_set = 0x1;
    bind_storage.elrs.magic = 0x37;
    memcpy(bind_storage.elrs.uid, uid, 6);
#endif
    return;
  }

  {
    uint8_t buffer[FLASH_STORAGE_SIZE];

    fmc_read_buf(FLASH_STORAGE_OFFSET, buffer, FLASH_STORAGE_SIZE);
    memcpy((uint8_t *)&flash_storage, buffer, sizeof(flash_storage_t));
  }

  {
    uint8_t buffer[BIND_STORAGE_SIZE];

    fmc_read_buf(BIND_STORAGE_OFFSET, buffer, BIND_STORAGE_SIZE);
    memcpy((uint8_t *)&bind_storage, buffer, sizeof(rx_bind_storage_t));

#ifdef RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND
    extern int rx_bind_load;
    rx_bind_load = bind_storage.bind_saved;
#endif
  }

  {
    uint8_t buffer[PROFILE_STORAGE_SIZE];

    fmc_read_buf(PROFILE_STORAGE_OFFSET, buffer, PROFILE_STORAGE_SIZE);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buffer, PROFILE_STORAGE_SIZE);

    cbor_result_t res = cbor_decode_profile_t(&dec, &profile);
    if (res < CBOR_OK) {
      fmc_lock();
      failloop(FAILLOOP_FAULT);
    }
  }

  {
    uint8_t buffer[VTX_STORAGE_SIZE];

    fmc_read_buf(VTX_STORAGE_OFFSET, buffer, VTX_STORAGE_SIZE);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buffer, VTX_STORAGE_SIZE);

    cbor_result_t res = cbor_decode_vtx_settings_t(&dec, &vtx_settings);
    if (res < CBOR_OK) {
      fmc_lock();
      failloop(FAILLOOP_FAULT);
    }
  }
}
