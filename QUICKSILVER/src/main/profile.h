#pragma once

#include "config.h"

// Utility
typedef struct {
  float roll;
  float pitch;
  float yaw;
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

typedef struct {
  vector_t rc_rate;
  vector_t super_rate;
  vector_t expo;
} rate_mode_betaflight_t;

typedef union {
  rate_mode_silverware_t silverware;
  rate_mode_betaflight_t betaflight;
} rate_t;

// Full Profile
typedef struct {
  rate_modes_t rate_mode;
  rate_t rate;

  float low_rate_mulitplier;
} profile_t;