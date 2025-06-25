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

// Frame types for I-frame/P-frame encoding
typedef enum {
  BLACKBOX_FRAME_I = 0,  // Intra frame - complete data
  BLACKBOX_FRAME_P = 1,  // Predicted frame - delta from previous
} blackbox_frame_type_t;

// Special flag to indicate frame type is stored in upper bit of field flags
#define BLACKBOX_FRAME_TYPE_BIT (1UL << 31)

// Blackbox fields (should align with above structure)

cbor_result_t cbor_encode_blackbox_frame(cbor_value_t *enc, const blackbox_t *current, const blackbox_t *previous, blackbox_frame_type_t frame_type, const uint32_t field_flags);

void blackbox_init();
void blackbox_set_debug(blackbox_debug_flag_t flag, uint8_t index, int16_t data);
void blackbox_update();