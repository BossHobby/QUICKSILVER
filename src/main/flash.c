#include "flash.h"

#include <string.h>

#include "drv_fmc.h"
#include "drv_serial.h"
#include "profile.h"
#include "project.h"
#include "util/cbor_helper.h"
#include "vtx.h"

#define FMC_HEADER 0x12AA0001

extern const profile_t default_profile;
extern profile_t profile;

extern vtx_settings_t vtx_settings;

static float initial_pid_identifier = -10;

flash_storage_t flash_storage;
rx_bind_storage_t bind_storage;

float flash_get_hard_coded_pid_identifier() {
  float result = 0;

  for (int i = 0; i < 3; i++) {
    result += profile.pid.pid_rates[0].kp.axis[i] * (i + 1) * (1) * 0.932f;
    result += profile.pid.pid_rates[0].ki.axis[i] * (i + 1) * (2) * 0.932f;
    result += profile.pid.pid_rates[0].kd.axis[i] * (i + 1) * (3) * 0.932f;
  }
  return result;
}

void flash_hard_coded_pid_identifier() {
  initial_pid_identifier = flash_get_hard_coded_pid_identifier();
}

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
    memset(buffer, 0, FLASH_STORAGE_SIZE);

    flash_storage.pid_identifier = initial_pid_identifier;

    memcpy(buffer, (uint8_t *)&flash_storage, sizeof(flash_storage_t));

    uint32_t *proxy = (uint32_t *)buffer;
    for (int i = 0; i < (FLASH_STORAGE_SIZE / 4); i++) {
      fmc_write((FLASH_STORAGE_OFFSET / 4) + i, proxy[i]);
    }
  }

  {
    uint8_t buffer[BIND_STORAGE_SIZE];
    memset(buffer, 0, BIND_STORAGE_SIZE);

    if (bind_storage.bind_saved == 0) {
      // reset all bind data
      memset(bind_storage.raw, 0, BIND_RAW_STORAGE_SIZE);
    }

    memcpy(buffer, (uint8_t *)&bind_storage, sizeof(rx_bind_storage_t));

    uint32_t *proxy = (uint32_t *)buffer;
    for (int i = 0; i < (BIND_STORAGE_SIZE / 4); i++) {
      fmc_write((BIND_STORAGE_OFFSET / 4) + i, proxy[i]);
    }
  }

  {
    uint8_t buffer[PROFILE_STORAGE_SIZE];
    memset(buffer, 0, PROFILE_STORAGE_SIZE);

    cbor_value_t enc;
    cbor_encoder_init(&enc, buffer, PROFILE_STORAGE_SIZE);
    cbor_encode_profile_t(&enc, &profile);

    uint32_t *proxy = (uint32_t *)buffer;
    for (int i = 0; i < (PROFILE_STORAGE_SIZE / 4); i++) {
      fmc_write((PROFILE_STORAGE_OFFSET / 4) + i, proxy[i]);
    }
  }

  {
    uint8_t buffer[VTX_STORAGE_SIZE];
    memset(buffer, 0, VTX_STORAGE_SIZE);

    cbor_value_t enc;
    cbor_encoder_init(&enc, buffer, VTX_STORAGE_SIZE);
    cbor_encode_vtx_settings_t(&enc, &vtx_settings);

    uint32_t *proxy = (uint32_t *)buffer;
    for (int i = 0; i < (VTX_STORAGE_SIZE / 4); i++) {
      fmc_write((VTX_STORAGE_OFFSET / 4) + i, proxy[i]);
    }
  }

  fmc_write(FMC_END_OFFSET / 4, FMC_HEADER);
  fmc_lock();
}

void flash_load() {
  // check if saved data is present
  if (FMC_HEADER != fmc_read(0) || FMC_HEADER != fmc_read(FMC_END_OFFSET / 4)) {
    // Flash was empty, load defaults?
    return;
  }

  {
    uint8_t buffer[FLASH_STORAGE_SIZE];
    memset(buffer, 0, FLASH_STORAGE_SIZE);

    uint32_t *proxy = (uint32_t *)buffer;
    for (int i = 0; i < (FLASH_STORAGE_SIZE / 4); i++) {
      proxy[i] = fmc_read((FLASH_STORAGE_OFFSET / 4) + i);
    }

    memcpy((uint8_t *)&flash_storage, buffer, sizeof(flash_storage_t));
  }

  {
    uint8_t buffer[BIND_STORAGE_SIZE];
    memset(buffer, 0, BIND_STORAGE_SIZE);

    uint32_t *proxy = (uint32_t *)buffer;
    for (int i = 0; i < (BIND_STORAGE_SIZE / 4); i++) {
      proxy[i] = fmc_read((BIND_STORAGE_OFFSET / 4) + i);
    }

    memcpy((uint8_t *)&bind_storage, buffer, sizeof(rx_bind_storage_t));

#ifdef RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND
    extern int rx_bind_load;
    rx_bind_load = bind_storage.bind_saved;
#endif
  }

  {
    uint8_t buffer[PROFILE_STORAGE_SIZE];
    memset(buffer, 0, PROFILE_STORAGE_SIZE);

    uint32_t *proxy = (uint32_t *)buffer;
    for (int i = 0; i < (PROFILE_STORAGE_SIZE / 4); i++) {
      proxy[i] = fmc_read((PROFILE_STORAGE_OFFSET / 4) + i);
    }

    cbor_value_t dec;
    cbor_decoder_init(&dec, buffer, PROFILE_STORAGE_SIZE);
    cbor_decode_profile_t(&dec, &profile);

    // values in profile.c (was pid.c) changed, overwrite with defaults form profile.c
    if (flash_storage.pid_identifier != initial_pid_identifier) {
      profile.pid = default_profile.pid;
    }
  }

  {
    uint8_t buffer[VTX_STORAGE_SIZE];
    memset(buffer, 0, VTX_STORAGE_SIZE);

    uint32_t *proxy = (uint32_t *)buffer;
    for (int i = 0; i < (VTX_STORAGE_SIZE / 4); i++) {
      proxy[i] = fmc_read((VTX_STORAGE_OFFSET / 4) + i);
    }

    cbor_value_t dec;
    cbor_decoder_init(&dec, buffer, VTX_STORAGE_SIZE);
    cbor_decode_vtx_settings_t(&dec, &vtx_settings);
  }
}
