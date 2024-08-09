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

struct gyro_device_t {
  gyro_types_t (*detect)();
  void (*configure)();
  void (*read)(gyro_data_t *);
  void (*start_read)(spi_txn_done_fn_t); // Null for polling-only drivers.
  void (*decode)(gyro_data_t *);
  float period_us;
};

extern gyro_types_t gyro_type;

float gyro_update_period();

gyro_types_t gyro_init();
gyro_data_t gyro_read();
void gyro_calibrate();
