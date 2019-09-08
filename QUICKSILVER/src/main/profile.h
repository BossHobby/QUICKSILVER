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

typedef struct {
  uint8_t invert_yaw;
} motor_t;

#define MOTOR_MEMBERS \
  MEMBER(invert_yaw, uint8)

typedef struct {
  uint8_t lipo_cell_count;
  uint8_t pid_voltage_compensation;
  float vbattlow;
  float actual_battery_voltage;
  float reported_telemetry_voltage;
} voltage_t;

#define VOLTAGE_MEMBERS                   \
  MEMBER(lipo_cell_count, uint8)          \
  MEMBER(pid_voltage_compensation, uint8) \
  MEMBER(vbattlow, float)                 \
  MEMBER(actual_battery_voltage, float)   \
  MEMBER(reported_telemetry_voltage, float)

typedef struct {
  aux_channel_t aux[AUX_FUNCTION_MAX];
} channel_t;

// Full Profile
typedef struct {
  motor_t motor;
  rate_t rate;
  channel_t channel;
  pid_rate_t pid;
  voltage_t voltage;
} profile_t;

#define PROFILE_MEMBERS      \
  MEMBER(motor, motor_t)     \
  MEMBER(rate, rate_t)       \
  MEMBER(channel, channel_t) \
  MEMBER(pid, pid_rate_t)    \
  MEMBER(voltage, voltage_t)

void profile_set_defaults();

cbor_result_t cbor_encode_vector_t(cbor_value_t *enc, const vector_t *vec);
cbor_result_t cbor_decode_profile_t(cbor_value_t *dec, profile_t *p);

cbor_result_t cbor_encode_profile_t(cbor_value_t *enc, const profile_t *p);
cbor_result_t cbor_decode_vector_t(cbor_value_t *dec, vector_t *vec);