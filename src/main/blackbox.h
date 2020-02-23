#pragma once

#include "profile.h"

typedef struct {
  uint32_t time;     // 4
  float cpu_load;    // 8
  float vbat_filter; // 12

  float gyro_raw[3];    // 24
  float gyro_filter[3]; // 36
  float gyro_vector[3]; // 48

  float rx_raw[4];    // 64
  float rx_filter[4]; // 80
  uint32_t rx_aux;    // 84

  float accel_raw[3];    // 96
  float accel_filter[3]; // 108

  float pid_output[3]; // 120
} blackbox_t;

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b);
cbor_result_t cbor_encode_compact_blackbox_t(cbor_value_t *enc, const blackbox_t *b);
cbor_result_t cbor_decode_compact_blackbox_t(cbor_value_t *dec, blackbox_t *b);

void blackbox_init();
void blackbox_update();