#pragma once

#include "profile.h"

typedef struct __attribute__((packed)) {
  uint32_t time;        // 4
  uint16_t cpu_load;    // 6
  uint16_t vbat_filter; // 8

  compact_vec4_t rx_raw;    // 16
  compact_vec4_t rx_filter; // 24
  uint32_t rx_aux;          // 28

  compact_vec3_t accel_raw;    // 34
  compact_vec3_t accel_filter; // 40

  compact_vec3_t gyro_raw;    // 46
  compact_vec3_t gyro_filter; // 52

  compact_vec3_t gyro_vector; // 58
  compact_vec3_t pid_output;  // 64
} blackbox_t;

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b);

void blackbox_init();
void blackbox_update();