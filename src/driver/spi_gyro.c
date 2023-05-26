#include "driver/spi_gyro.h"

#include "core/project.h"
#include "driver/spi.h"
#include "driver/time.h"

#include "driver/spi_bmi270.h"
#include "driver/spi_icm42605.h"
#include "driver/spi_mpu6xxx.h"

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
  case GYRO_TYPE_ICM20649:
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
  gpio_init.drive =GPIO_DRIVE_HIGH;
  gpio_pin_init( GYRO_INT, gpio_init);
#endif

  if (!target_spi_device_valid(&target.gyro)) {
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
  case GYRO_TYPE_ICM20649:
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

  default:
    break;
  }

  return gyro_type;
}

gyro_data_t gyro_spi_read() {
  gyro_data_t data;

  switch (gyro_type) {
  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20602:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20649:
  case GYRO_TYPE_ICM20689: {
    uint8_t buf[14];
    mpu6xxx_read_data(MPU_RA_ACCEL_XOUT_H, buf, 14);

    data.accel.axis[0] = -(int16_t)((buf[0] << 8) | buf[1]);
    data.accel.axis[1] = -(int16_t)((buf[2] << 8) | buf[3]);
    data.accel.axis[2] = (int16_t)((buf[4] << 8) | buf[5]);

    data.temp = (float)((int16_t)((buf[6] << 8) | buf[7])) / 333.87f + 21.f;

    data.gyro.axis[1] = (int16_t)((buf[8] << 8) | buf[9]);
    data.gyro.axis[0] = (int16_t)((buf[10] << 8) | buf[11]);
    data.gyro.axis[2] = (int16_t)((buf[12] << 8) | buf[13]);
    break;
  }

  case GYRO_TYPE_ICM42605:
  case GYRO_TYPE_ICM42688P: {
    uint8_t buf[14];
    icm42605_read_data(ICM42605_TEMP_DATA1, buf, 14);

    data.temp = (float)((int16_t)((buf[0] << 8) | buf[1])) / 132.48f + 25.f;

    data.accel.axis[0] = -(int16_t)((buf[2] << 8) | buf[3]);
    data.accel.axis[1] = -(int16_t)((buf[4] << 8) | buf[5]);
    data.accel.axis[2] = (int16_t)((buf[6] << 8) | buf[7]);

    data.gyro.axis[1] = (int16_t)((buf[8] << 8) | buf[9]);
    data.gyro.axis[0] = (int16_t)((buf[10] << 8) | buf[11]);
    data.gyro.axis[2] = (int16_t)((buf[12] << 8) | buf[13]);

    break;
  }

  case GYRO_TYPE_BMI270: {
    bmi270_read_gyro_data(&data);
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