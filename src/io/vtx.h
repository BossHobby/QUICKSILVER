#pragma once

#include <cbor.h>

#define VTX_SETTINGS_MAGIC 0xdeed
#define VTX_APPLY_TRIES 10

typedef enum {
  VTX_DETECT_WAIT,
  VTX_DETECT_ERROR,
  VTX_DETECT_SUCCESS,
  VTX_DETECT_UPDATE
} vtx_detect_status_t;

typedef enum {
  VTX_BAND_A,
  VTX_BAND_B,
  VTX_BAND_E,
  VTX_BAND_F,
  VTX_BAND_R,
  VTX_BAND_L,

  VTX_BAND_MAX
} vtx_band_t;

typedef enum {
  VTX_CHANNEL_1,
  VTX_CHANNEL_2,
  VTX_CHANNEL_3,
  VTX_CHANNEL_4,
  VTX_CHANNEL_5,
  VTX_CHANNEL_6,
  VTX_CHANNEL_7,
  VTX_CHANNEL_8,

  VTX_CHANNEL_MAX,
} vtx_channel_t;

typedef enum {
  VTX_POWER_LEVEL_1,
  VTX_POWER_LEVEL_2,
  VTX_POWER_LEVEL_3,
  VTX_POWER_LEVEL_4,
  VTX_POWER_LEVEL_5,

  VTX_POWER_LEVEL_MAX,
} vtx_power_level_t;

typedef enum {
  VTX_PIT_MODE_OFF,
  VTX_PIT_MODE_ON,
  VTX_PIT_MODE_NO_SUPPORT,

  VTX_PIT_MODE_MAX,
} vtx_pit_mode_t;

typedef enum {
  VTX_PROTOCOL_INVALID,
  VTX_PROTOCOL_TRAMP,
  VTX_PROTOCOL_SMART_AUDIO,
  VTX_PROTOCOL_MSP_VTX,

  VTX_PROTOCOL_MAX,
} vtx_protocol_t;

typedef struct {
  uint8_t levels;
  char labels[VTX_POWER_LEVEL_MAX][3];
  uint16_t values[VTX_POWER_LEVEL_MAX];
} vtx_power_table_t;

#define VTX_POWER_TABLE_MEMBERS                     \
  MEMBER(levels, uint8)                             \
  TSTR_ARRAY_MEMBER(labels, VTX_POWER_LEVEL_MAX, 3) \
  ARRAY_MEMBER(values, VTX_POWER_LEVEL_MAX, uint16)

typedef struct {
  uint16_t magic;

  vtx_protocol_t protocol;
  vtx_protocol_t detected;

  vtx_band_t band;
  vtx_channel_t channel;

  vtx_pit_mode_t pit_mode;

  vtx_power_level_t power_level;
  vtx_power_table_t power_table;
} vtx_settings_t;

#define VTX_SETTINGS_MEMBERS \
  MEMBER(magic, uint16)      \
  MEMBER(protocol, uint8)    \
  MEMBER(detected, uint8)    \
  MEMBER(band, uint8)        \
  MEMBER(channel, uint8)     \
  MEMBER(pit_mode, uint8)    \
  MEMBER(power_level, uint8) \
  MEMBER(power_table, vtx_power_table_t)

extern vtx_settings_t vtx_settings;

void vtx_init();
void vtx_update();

void vtx_set(vtx_settings_t *vtx);

uint16_t vtx_frequency_from_channel(vtx_band_t band, vtx_channel_t channel);
int8_t vtx_find_frequency_index(uint16_t frequency);
vtx_power_level_t vtx_power_level_index(vtx_power_table_t *power_table, uint16_t power);

cbor_result_t cbor_encode_vtx_settings_t(cbor_value_t *enc, const vtx_settings_t *vtx);
cbor_result_t cbor_decode_vtx_settings_t(cbor_value_t *dec, vtx_settings_t *vtx);