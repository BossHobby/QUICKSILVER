#pragma once

#include "profile.h"

typedef struct {
  uint32_t time;

  float cpu_load;

  float vbat_filter;

  float gyro_raw[3];
  float gyro_filter[3];
  float gyro_vector[3];

  float rx_raw[4];
  float rx_filter[4];
  uint8_t rx_aux[AUX_CHANNEL_MAX];

  float accel_raw[3];
  float accel_filter[3];

  float pid_output[3];
} blackbox_t;

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b);
cbor_result_t cbor_encode_compact_blackbox_t(cbor_value_t *enc, const blackbox_t *b);
cbor_result_t cbor_decode_compact_blackbox_t(cbor_value_t *dec, blackbox_t *b);

void blackbox_init();
void blackbox_update();