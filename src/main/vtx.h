#pragma once

#include <cbor.h>

typedef enum {
  VTX_BAND_A,
  VTX_BAND_B,
  VTX_BAND_E,
  VTX_BAND_F,
  VTX_BAND_R,

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

typedef struct {
  vtx_band_t band;
  vtx_channel_t channel;
} vtx_settings_t;

void vtx_init();
void vtx_update();

void vtx_set(vtx_settings_t *vtx);
void vtx_set_frequency(vtx_band_t band, vtx_channel_t channel);

cbor_result_t cbor_encode_vtx_settings_t(cbor_value_t *enc, const vtx_settings_t *vtx);
cbor_result_t cbor_decode_vtx_settings_t(cbor_value_t *dec, vtx_settings_t *vtx);