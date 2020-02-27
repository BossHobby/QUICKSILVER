#pragma once

#include "profile.h"

typedef struct {
  uint32_t time;        // 4
  uint16_t cpu_load;    // 6
  uint16_t vbat_filter; // 8

  float gyro_raw[3];    // 20
  float gyro_filter[3]; // 32
  float gyro_vector[3]; // 44

  float rx_raw[4];    // 60
  float rx_filter[4]; // 76
  uint32_t rx_aux;    // 80

  float accel_raw[3];    // 92
  float accel_filter[3]; // 104

  float pid_output[3]; // 116
} __attribute__((packed)) blackbox_t;

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b);
cbor_result_t cbor_encode_compact_blackbox_t(cbor_value_t *enc, const blackbox_t *b);
cbor_result_t cbor_decode_compact_blackbox_t(cbor_value_t *dec, blackbox_t *b);

void blackbox_init();
void blackbox_update();