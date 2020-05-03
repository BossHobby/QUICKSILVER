#include <string.h>

#include "drv_fmc.h"
#include "drv_serial.h"
#include "profile.h"
#include "project.h"
#include "rx.h"

extern const profile_t default_profile;
extern profile_t profile;
extern float accelcal[];

#define FMC_HEADER 0x12AA0001
#define FRSKY_BIND_OFFSET 57

float initial_pid_identifier = -10;

float flash_get_hard_coded_pid_identifier(void) {
  float result = 0;

  for (int i = 0; i < 3; i++) {
    result += profile.pid.pid_rates[0].kp.axis[i] * (i + 1) * (1) * 0.932f;
    result += profile.pid.pid_rates[0].ki.axis[i] * (i + 1) * (2) * 0.932f;
    result += profile.pid.pid_rates[0].kd.axis[i] * (i + 1) * (3) * 0.932f;
  }
  return result;
}

void flash_hard_coded_pid_identifier(void) {
  initial_pid_identifier = flash_get_hard_coded_pid_identifier();
}

void flash_save(void) {

  fmc_unlock();
  fmc_erase();

  unsigned long addresscount = 0;

  writeword(addresscount++, FMC_HEADER);

  fmc_write_float(addresscount++, initial_pid_identifier);

  fmc_write_float(addresscount++, accelcal[0]);
  fmc_write_float(addresscount++, accelcal[1]);
  fmc_write_float(addresscount++, accelcal[2]);

#ifdef RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND
  // autobind info
  extern char rfchannel[4];
  extern char rxaddress[5];
  extern int telemetry_enabled;
  extern int rx_bind_enable;

  // save radio bind info
  if (rx_bind_enable) {
    writeword(50, rxaddress[4] | telemetry_enabled << 8);
    writeword(51, rxaddress[0] | (rxaddress[1] << 8) | (rxaddress[2] << 16) | (rxaddress[3] << 24));
    writeword(52, rfchannel[0] | (rfchannel[1] << 8) | (rfchannel[2] << 16) | (rfchannel[3] << 24));
  } else {
    // this will leave 255's so it will be picked up as disabled
  }
#endif

#ifdef SWITCHABLE_FEATURE_1
  extern int flash_feature_1;

  //save filter cut info

  if (flash_feature_1) {
    fmc_write_float(53, 1);
  } else {
    fmc_write_float(53, 0);
  }
#endif

#ifdef SWITCHABLE_FEATURE_2
  extern int flash_feature_2;

  //save LVC info

  if (flash_feature_2) {
    fmc_write_float(54, 1);
  } else {
    fmc_write_float(54, 0);
  }
#endif

#if defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_UNIFIED_SERIAL)
  extern int rx_bind_enable;
  if (rx_bind_enable) {
    fmc_write_float(56, 1);
  } else {
    fmc_write_float(56, 0);
  }
#endif

#ifdef RX_UNIFIED_SERIAL
  if (rx_bind_enable) {
    extern rx_serial_protocol_t rx_serial_protocol;
    writeword(50, rx_serial_protocol);
  } else {
    writeword(50, 0);
  }
#endif

#ifdef RX_FRSKY
  extern int rx_bind_enable;
  extern frsky_bind_data frsky_bind;

  frsky_bind_data dummy_frsky_bind = {
      .offset = 0xff,
      .idx = 0xff,
  };

  for (int i = 0; i < sizeof(frsky_bind_data) / 4; i++) {
    if (rx_bind_enable == 1) {
      // we want to bind to next bootup
      // so lets write dummy data
      writeword(i + FRSKY_BIND_OFFSET, dummy_frsky_bind.raw[i]);
    } else {
      writeword(i + FRSKY_BIND_OFFSET, frsky_bind.raw[i]);
    }
  }
#endif
  {
    uint8_t buffer[PROFILE_FLASH_SIZE];
    memset(buffer, 0, PROFILE_FLASH_SIZE);

    cbor_value_t enc;
    cbor_encoder_init(&enc, buffer, PROFILE_FLASH_SIZE);
    cbor_encode_profile_t(&enc, &profile);

    uint32_t *proxy = (uint32_t *)buffer;
    for (int i = 0; i < (PROFILE_FLASH_SIZE / 8); i++) {
      writeword(i + 256, proxy[i]);
    }
  }

  writeword(256 + (PROFILE_FLASH_SIZE / 8), FMC_HEADER);
  fmc_lock();
}

void flash_load(void) {

  unsigned long addresscount = 0;
  // check if saved data is present
  if (FMC_HEADER == fmc_read(addresscount++) && FMC_HEADER == fmc_read(256 + (PROFILE_FLASH_SIZE / 8))) {

    float saved_pid_identifier = fmc_read_float(addresscount++);

    accelcal[0] = fmc_read_float(addresscount++);
    accelcal[1] = fmc_read_float(addresscount++);
    accelcal[2] = fmc_read_float(addresscount++);

#ifdef RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND
    extern char rfchannel[4];
    extern char rxaddress[5];
    extern int telemetry_enabled;
    extern int rx_bind_load;
    extern int rx_bind_enable;

    // save radio bind info

    int temp = fmc_read(52);
    int error = 0;
    for (int i = 0; i < 4; i++) {
      if (((temp >> (i * 8)) & 0xff) > 127) {
        error = 1;
      }
    }

    if (!error) {
      rx_bind_load = rx_bind_enable = 1;

      rxaddress[4] = fmc_read(50);

      telemetry_enabled = fmc_read(50) >> 8;
      int temp = fmc_read(51);
      for (int i = 0; i < 4; i++) {
        rxaddress[i] = temp >> (i * 8);
      }

      temp = fmc_read(52);
      for (int i = 0; i < 4; i++) {
        rfchannel[i] = temp >> (i * 8);
      }
    }
#endif

#if defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_UNIFIED_SERIAL)
    extern int rx_bind_enable;
    rx_bind_enable = fmc_read_float(56);
#endif

#ifdef RX_UNIFIED_SERIAL
    extern rx_serial_protocol_t rx_serial_protocol;
    if (rx_bind_enable != 1) {
      rx_serial_protocol = 0;
    } else {
      rx_serial_protocol = fmc_read(50);
    }

#endif

#ifdef SWITCHABLE_FEATURE_1
    extern int flash_feature_1;
    flash_feature_1 = fmc_read_float(53);
#endif

#ifdef SWITCHABLE_FEATURE_2
    extern int flash_feature_2;
    flash_feature_2 = fmc_read_float(54);
#endif

#ifdef RX_FRSKY
    extern int rx_bind_enable;

    // only load data if we did not just overwrite it
    if (rx_bind_enable != 1) {
      extern frsky_bind_data frsky_bind;
      for (int i = 0; i < sizeof(frsky_bind_data) / 4; i++) {
        frsky_bind.raw[i] = fmc_read(i + FRSKY_BIND_OFFSET);
      }
    }
#endif

    //profile
    {
      uint8_t buffer[PROFILE_FLASH_SIZE];
      memset(buffer, 0, PROFILE_FLASH_SIZE);

      uint32_t *proxy = (uint32_t *)buffer;
      for (int i = 0; i < (PROFILE_FLASH_SIZE / 8); i++) {
        proxy[i] = fmc_read(i + 256);
      }

      cbor_value_t dec;
      cbor_decoder_init(&dec, buffer, PROFILE_FLASH_SIZE);
      cbor_decode_profile_t(&dec, &profile);

      // values in profile.c (was pid.c) changed, overwrite with defaults form profile.c
      if (saved_pid_identifier != initial_pid_identifier) {
        profile.pid = default_profile.pid;
      }
    }

  } else {
    // Flash was empty, load defaults?
  }
}
