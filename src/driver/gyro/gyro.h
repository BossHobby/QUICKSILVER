#pragma once

#include <stdint.h>

#include "util/vector.h"

typedef enum {
  GYRO_TYPE_INVALID,

  GYRO_TYPE_MPU6000,
  GYRO_TYPE_MPU6500,

  GYRO_TYPE_ICM20601,
  GYRO_TYPE_ICM20602,
  GYRO_TYPE_ICM20608,
  GYRO_TYPE_ICM20689,

  GYRO_TYPE_ICM42605,
  GYRO_TYPE_ICM42688P,

  GYRO_TYPE_BMI270,
  GYRO_TYPE_BMI323,

  GYRO_TYPE_ICM42622P,
  GYRO_TYPE_ICM42686P,

  GYRO_TYPE_LSM6DSO,
  GYRO_TYPE_LSM6DSV16X,
  GYRO_TYPE_LSM6DSK320X,
} gyro_types_t;

typedef struct {
  vec3_t gyro;
  vec3_t accel;
  float temp;
} gyro_data_t;

// CPU cycles; zero period means acquiring or unavailable.
typedef struct {
  uint32_t period;
  uint32_t phase; // Safe DRDY phase, including MPU6000's delayed sample.
  uint32_t sample; // Latest observed edge, for freshness.
} gyro_clock_t;

extern gyro_types_t gyro_type;

// Captures the EXTI-owned clock with interrupts masked.
gyro_clock_t gyro_clock_snapshot();
float gyro_update_period();
bool gyro_exti_state();

gyro_types_t gyro_init();
gyro_data_t gyro_read();
void gyro_calibrate();
