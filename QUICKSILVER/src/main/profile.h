#pragma once

#include "nanocbor/nanocbor.h"
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

// Full Profile
typedef struct {
  rate_modes_t rate_mode;
  rate_mode_silverware_t silverware_rate;
  rate_mode_betaflight_t betaflight_rate;

  float low_rate_mulitplier;
} profile_t;

#define PROFILE_MEMBERS                           \
  MEMBER(rate_mode, uint8)                        \
  MEMBER(silverware_rate, rate_mode_silverware_t) \
  MEMBER(betaflight_rate, rate_mode_betaflight_t) \
  MEMBER(low_rate_mulitplier, float)

void nanocbor_fmt_vector_t(nanocbor_encoder_t *enc, vector_t vec);
void nanocbor_fmt_profile_t(nanocbor_encoder_t *enc, profile_t p);

void nanocbor_get_vector_t(nanocbor_value_t *it, vector_t *vec);
void nanocbor_get_profile_t(nanocbor_value_t *it, profile_t *p);