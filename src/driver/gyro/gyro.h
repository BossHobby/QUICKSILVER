#pragma once

#include "driver/spi.h"
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
} gyro_types_t;

typedef struct {
  vec3_t gyro;
  vec3_t accel;
  float temp;
} gyro_data_t;

extern gyro_types_t gyro_type;

float gyro_update_period();

uint8_t gyro_int();
gyro_data_t gyro_read();
void gyro_calibrate();
void gyro_wait_for_ready();