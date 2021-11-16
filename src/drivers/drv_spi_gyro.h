#pragma once

#include "util/vector.h"

typedef enum {
  MPU6XXX,
  ICM20601,
  ICM20608,
  ICM20602,
  ICM42605,
  BMI270,
} gyro_types_t;

typedef struct {
  vec3_t gyro;
  vec3_t accel;
  float temp;
} gyro_data_t;

uint8_t spi_gyro_init();
gyro_data_t spi_gyro_read();