#include "driver/spi_gyro.h"

#include "core/project.h"
#include "driver/spi.h"
#include "driver/time.h"

#include "driver/spi_bmi270.h"
#include "driver/spi_bmi323.h"
#include "driver/spi_icm42605.h"
#include "driver/spi_mpu6xxx.h"

#ifdef USE_GYRO

gyro_types_t gyro_type = GYRO_TYPE_INVALID;

spi_bus_device_t gyro_bus = {};

static gyro_types_t gyro_spi_detect() {
  gyro_types_t type = GYRO_TYPE_INVALID;

  switch (type) {
  case GYRO_TYPE_INVALID:
    // FALLTHROUGH

  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20602:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20689:
    type = mpu6xxx_detect();
    if (type != GYRO_TYPE_INVALID) {
      break;
    }
    // FALLTHROUGH

  case GYRO_TYPE_ICM42605:
  case GYRO_TYPE_ICM42688P:
    type = icm42605_detect();
    if (type != GYRO_TYPE_INVALID) {
      break;
    }
    // FALLTHROUGH

  case GYRO_TYPE_BMI270:
    type = bmi270_detect();
    if (type != GYRO_TYPE_INVALID) {
      break;
    }
    // FALLTHROUGH
  case GYRO_TYPE_BMI323:
    type = bmi323_detect();
    if (type != GYRO_TYPE_INVALID) {
      break;
    }
    // FALLTHROUGH
  default:
    break;
  }

  return type;
}

uint8_t gyro_spi_init() {
#ifdef GYRO_INT
  // Interrupt GPIO
  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_INPUT;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_UP_PULL;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_pin_init(GYRO_INT, gpio_init);
#endif

  if (!target_gyro_spi_device_valid(&target.gyro)) {
    return GYRO_TYPE_INVALID;
  }

  gyro_bus.port = target.gyro.port;
  gyro_bus.nss = target.gyro.nss;
  spi_bus_device_init(&gyro_bus);

  gyro_type = gyro_spi_detect();

  switch (gyro_type) {
  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20602:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20689:
    mpu6xxx_configure();
    break;

  case GYRO_TYPE_ICM42605:
  case GYRO_TYPE_ICM42688P:
    icm42605_configure();
    break;

  case GYRO_TYPE_BMI270:
    bmi270_configure();
    break;
  case GYRO_TYPE_BMI323:
    bmi323_configure();
    break;

  default:
    break;
  }

  return gyro_type;
}

float gyro_update_period() {
  switch (gyro_type) {
  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20602:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20689:
    return 125.0f;

  case GYRO_TYPE_ICM42605:
  case GYRO_TYPE_ICM42688P:
    return 125.0f;

  case GYRO_TYPE_BMI270:
  case GYRO_TYPE_BMI323:
    return 312.5f;

  default:
    return 250.0f;
  }
}

gyro_data_t gyro_spi_read() {
  static gyro_data_t data;

  switch (gyro_type) {
  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20602:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20689: {
    mpu6xxx_read_gyro_data(&data);
    break;
  }

  case GYRO_TYPE_ICM42605:
  case GYRO_TYPE_ICM42688P: {
    icm42605_read_gyro_data(&data);
    break;
  }

  case GYRO_TYPE_BMI270: {
    bmi270_read_gyro_data(&data);
    break;
  }
  case GYRO_TYPE_BMI323: {
    bmi323_read_gyro_data(&data);
    break;
  }

  default:
    break;
  }

  return data;
}

void gyro_spi_calibrate() {
  switch (gyro_type) {
  case GYRO_TYPE_BMI270: {
    bmi270_calibrate();
    break;
  }

  default:
    break;
  }
}
#else
float gyro_update_period() {
  return 250.0f;
}
#endif