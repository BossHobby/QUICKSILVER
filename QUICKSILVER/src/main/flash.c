#include <string.h>

#include "defines.h"
#include "drv_fmc.h"
#include "profile.h"
#include "project.h"
#include "rx.h"

extern const profile_t default_profile;
extern profile_t profile;
extern float accelcal[];

#define FMC_HEADER 0x12AA0001
#define FRSKY_BIND_OFFSET OSD_FLASH_START + OSD_NUMBER_ELEMENTS

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

#if defined(RX_DSMX_2048) || defined(RX_DSM2_1024)
  extern int rx_bind_enable;
  if (rx_bind_enable) {
    fmc_write_float(56, 1);
  } else {
    fmc_write_float(56, 0);
  }
#endif

#ifdef ENABLE_OSD //currently starts at address 57
  extern unsigned long osd_element[OSD_NUMBER_ELEMENTS];
  for (int i = 0; i < OSD_NUMBER_ELEMENTS; i++) {
    writeword(i + OSD_FLASH_START, osd_element[i]);
  }

#endif

#ifdef RX_FRSKY //currently starts at address 66
  extern frsky_bind_data frsky_bind;
  for (int i = 0; i < sizeof(frsky_bind_data) / 4; i++) {
    writeword(i + FRSKY_BIND_OFFSET, frsky_bind.raw[i]);
  }
#endif
  {
    uint8_t buffer[1024];
    memset(buffer, 0, 1024);

    cbor_value_t enc;
    cbor_encoder_init(&enc, buffer, 1024);
    cbor_encode_profile_t(&enc, &profile);

    uint32_t *proxy = (uint32_t *)buffer;
    for (int i = 0; i < 256; i++) {
      writeword(i + 256, proxy[i]);
    }
  }

  writeword(512, FMC_HEADER);
  fmc_lock();
}

void flash_load(void) {

  unsigned long addresscount = 0;
  // check if saved data is present
  if (FMC_HEADER == fmc_read(addresscount++) && FMC_HEADER == fmc_read(512)) {

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

#ifdef SWITCHABLE_FEATURE_1
    extern int flash_feature_1;
    flash_feature_1 = fmc_read_float(53);
#endif

#ifdef SWITCHABLE_FEATURE_2
    extern int flash_feature_2;
    flash_feature_2 = fmc_read_float(54);
#endif

#if defined(RX_DSMX_2048) || defined(RX_DSM2_1024)
    extern int rx_bind_enable;
    rx_bind_enable = fmc_read_float(56);
#endif

#ifdef ENABLE_OSD
    extern unsigned long osd_element[OSD_NUMBER_ELEMENTS];
    for (int i = 0; i < OSD_NUMBER_ELEMENTS; i++) {
      osd_element[i] = fmc_read(i + OSD_FLASH_START);
    }
#endif

#ifdef RX_FRSKY //currently starts at address 57
    extern frsky_bind_data frsky_bind;
    for (int i = 0; i < sizeof(frsky_bind_data) / 4; i++) {
      frsky_bind.raw[i] = fmc_read(i + FRSKY_BIND_OFFSET);
    }
#endif

    //profile
    {
      uint8_t buffer[1024];
      memset(buffer, 0, 1024);
      uint32_t *proxy = (uint32_t *)buffer;
      for (int i = 0; i < 256; i++) {
        proxy[i] = fmc_read(i + 256);
      }

      cbor_value_t enc;
      cbor_decoder_init(&enc, buffer, 1024);
      cbor_decode_profile_t(&enc, &profile);

      // values in profile.c (was pid.c) changed, overwrite with defaults form profile.c
      if (saved_pid_identifier != initial_pid_identifier) {
        profile.pid = default_profile.pid;
      }
    }

  } else {
    // Flash was empty, load defaults?
  }
}
