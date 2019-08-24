#pragma once

#include <cbor.h>

#include "project.h"

// Utility
typedef union {
  struct {
    float roll;
    float pitch;
    float yaw;
  };
  float axis[3];
} vector_t;

// Rates
typedef enum {
  RATE_MODE_SILVERWARE,
  RATE_MODE_BETAFLIGHT,
} rate_modes_t;

typedef struct {
  vector_t max_rate;
  vector_t acro_expo;
  vector_t angle_expo;
} rate_mode_silverware_t;

#define SILVERWARE_RATE_MEMBERS \
  MEMBER(max_rate, vector_t)    \
  MEMBER(acro_expo, vector_t)   \
  MEMBER(angle_expo, vector_t)

typedef struct {
  vector_t rc_rate;
  vector_t super_rate;
  vector_t expo;
} rate_mode_betaflight_t;

#define BETAFLIGHT_RATE_MEMBERS \
  MEMBER(rc_rate, vector_t)     \
  MEMBER(super_rate, vector_t)  \
  MEMBER(expo, vector_t)

typedef struct {
  rate_modes_t mode;
  rate_mode_silverware_t silverware;
  rate_mode_betaflight_t betaflight;
  float level_max_angle;
  float low_rate_mulitplier;
  float sticks_deadband;
} rate_t;

#define RATE_MEMBERS                         \
  MEMBER(mode, uint8)                        \
  MEMBER(silverware, rate_mode_silverware_t) \
  MEMBER(betaflight, rate_mode_betaflight_t) \
  MEMBER(level_max_angle, float)             \
  MEMBER(low_rate_mulitplier, float)         \
  MEMBER(sticks_deadband, float)

typedef struct {
  vector_t kp;
  vector_t ki;
  vector_t kd;
} pid_rate_t;

#define PID_RATE_MEMBERS \
  MEMBER(kp, vector_t)   \
  MEMBER(ki, vector_t)   \
  MEMBER(kd, vector_t)

// Full Profile
typedef struct {
  rate_t rate;
  pid_rate_t pid;
} profile_t;

#define PROFILE_MEMBERS \
  MEMBER(rate, rate_t)  \
  MEMBER(pid, pid_rate_t)

cbor_result_t cbor_decode_vector_t(cbor_value_t *enc, vector_t *vec);
cbor_result_t cbor_decode_profile_t(cbor_value_t *enc, profile_t *p);

cbor_result_t cbor_encode_vector_t(cbor_value_t *it, vector_t vec);
cbor_result_t cbor_encode_profile_t(cbor_value_t *it, profile_t p);