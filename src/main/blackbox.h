#pragma once

#include "profile.h"

typedef struct __attribute__((packed)) {
  uint32_t time;
  uint16_t cpu_load;
  uint16_t vbat_filter;

  vec4_t rx_raw;
  vec4_t rx_filter;
  uint32_t rx_aux;

  compact_vec3_t accel_raw;
  compact_vec3_t accel_filter;

  compact_vec3_t gyro_raw;
  compact_vec3_t gyro_filter;

  vec3_t gyro_vector;
  vec3_t pid_output;
} blackbox_t;

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b);

void blackbox_init();
void blackbox_update();