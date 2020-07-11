#pragma once

#include "profile.h"

typedef struct {
  uint32_t loop;
  uint32_t time;

  compact_vec3_t pid_p_term;
  compact_vec3_t pid_i_term;
  compact_vec3_t pid_d_term;

  compact_vec4_t rx;
  compact_vec4_t setpoint;

  compact_vec3_t accel_raw;
  compact_vec3_t accel_filter;

  compact_vec3_t gyro_raw;
  compact_vec3_t gyro_filter;

  compact_vec4_t motor;

  uint16_t cpu_load;
} blackbox_t;

#define BLACKBOX_MAX_SIZE 128

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b);

void blackbox_init();
void blackbox_update();