#pragma once

#include "core/profile.h"
#include "util/util.h"

#define BLACKBOX_VERSION MAKE_SEMVER(0, 1, 0)

#define BLACKBOX_SCALE 1000
#define BLACKBOX_DEBUG_SIZE 10

typedef enum {
  BBOX_FIELD_LOOP,
  BBOX_FIELD_TIME,
  BBOX_FIELD_PID_P_TERM,
  BBOX_FIELD_PID_I_TERM,
  BBOX_FIELD_PID_D_TERM,
  BBOX_FIELD_RX,
  BBOX_FIELD_SETPOINT,
  BBOX_FIELD_ACCEL_RAW,
  BBOX_FIELD_ACCEL_FILTER,
  BBOX_FIELD_GYRO_RAW,
  BBOX_FIELD_GYRO_FILTER,
  BBOX_FIELD_MOTOR,
  BBOX_FIELD_CPU_LOAD,
  BBOX_FIELD_DEBUG,

  BBOX_FIELD_MAX,
} blackbox_field_t;

typedef enum {
  BBOX_DEBUG_DYN_NOTCH = 0x1 << 0,
} blackbox_debug_flag_t;

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

  int16_t debug[BLACKBOX_DEBUG_SIZE];
} blackbox_t;

// Blackbox fields (should align with above structure)

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b, const uint32_t field_flags);

void blackbox_init();
void blackbox_set_debug(blackbox_debug_flag_t flag, uint8_t index, int16_t data);
void blackbox_update();