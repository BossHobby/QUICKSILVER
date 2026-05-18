#pragma once

#include <cbor.h>

#include "core/profile.h"

#define VTX_APPLY_TRIES 50

typedef enum {
  VTX_DETECT_WAIT,
  VTX_DETECT_ERROR,
  VTX_DETECT_SUCCESS,
} vtx_detect_status_t;

typedef struct {
  vtx_protocol_t protocol;
  vtx_band_t band;
  vtx_channel_t channel;
  vtx_pit_mode_t pit_mode;
  vtx_power_level_t power_level;
  vtx_power_table_t power_table;
} vtx_status_t;

#define VTX_STATUS_MEMBERS     \
  MEMBER(protocol, uint8_t)    \
  MEMBER(band, uint8_t)        \
  MEMBER(channel, uint8_t)     \
  MEMBER(pit_mode, uint8_t)    \
  MEMBER(power_level, uint8_t) \
  MEMBER(power_table, vtx_power_table_t)

extern vtx_status_t vtx_actual;

typedef struct {
  void (*init)(void);
  vtx_detect_status_t (*update)(vtx_status_t *);
  bool (*set_frequency)(vtx_band_t, vtx_channel_t);
  bool (*set_power_level)(vtx_power_level_t);
  bool (*set_pit_mode)(vtx_pit_mode_t);
} vtx_device_t;

void vtx_init();
void vtx_update();

uint16_t vtx_frequency_from_channel(vtx_band_t band, vtx_channel_t channel);
int8_t vtx_find_frequency_index(uint16_t frequency);
vtx_power_level_t vtx_power_level_index(vtx_power_table_t *power_table, uint16_t power);

cbor_result_t cbor_encode_vtx_status_t(cbor_value_t *enc, const vtx_status_t *vtx);
