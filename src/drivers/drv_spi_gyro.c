#include "drv_spi_gyro.h"

#include "drv_time.h"
#include "project.h"

#include "drv_spi_bmi270.h"
#include "drv_spi_icm42605.h"
#include "drv_spi_mpu6xxx.h"

uint8_t spi_gyro_init() {
  switch (GYRO_TYPE) {
  case ICM42605:
    return icm42605_configure();
  case BMI270:
    return bmi270_configure();
  case MPU6XXX:
  case ICM20601:
  case ICM20608:
  case ICM20602:
  default:
    return mpu6xxx_configure();
  }
}

gyro_data_t spi_gyro_read() {
  gyro_data_t data;

  switch (GYRO_TYPE) {
  case ICM42605: {
    uint8_t buf[14];
    icm42605_read_data(ICM42605_TEMP_DATA1, buf, 14);

    data.temp = (float)((int16_t)((buf[0] << 8) + buf[1])) / 132.48f + 25.f;

    data.accel.axis[0] = -(int16_t)((buf[2] << 8) + buf[3]);
    data.accel.axis[1] = -(int16_t)((buf[4] << 8) + buf[5]);
    data.accel.axis[2] = (int16_t)((buf[6] << 8) + buf[7]);

    data.gyro.axis[1] = (int16_t)((buf[8] << 8) + buf[9]);
    data.gyro.axis[0] = (int16_t)((buf[10] << 8) + buf[11]);
    data.gyro.axis[2] = (int16_t)((buf[12] << 8) + buf[13]);

    break;
  }
  case BMI270: {
    uint8_t buf[12];
    bmi270_read_data(BMI270_REG_ACC_DATA_X_LSB, buf, 12);

    data.accel.axis[0] = -(int16_t)((buf[1] << 8) + buf[0]);
    data.accel.axis[1] = -(int16_t)((buf[3] << 8) + buf[2]);
    data.accel.axis[2] = (int16_t)((buf[5] << 8) + buf[4]);

    data.gyro.axis[1] = (int16_t)((buf[7] << 8) + buf[6]);
    data.gyro.axis[0] = (int16_t)((buf[9] << 8) + buf[8]);
    data.gyro.axis[2] = (int16_t)((buf[11] << 8) + buf[10]);

    bmi270_read_data(BMI270_REG_TEMPERATURE_LSB, buf, 3);
    data.temp = 23 + ((int32_t)((int16_t)((buf[2] << 8) | buf[1])) >> 9);

    break;
  }
  case MPU6XXX:
  case ICM20601:
  case ICM20608:
  case ICM20602:
  default: {
    uint8_t buf[14];
    mpu6xxx_read_data(MPU_RA_ACCEL_XOUT_H, buf, 14);

    data.accel.axis[0] = -(int16_t)((buf[0] << 8) + buf[1]);
    data.accel.axis[1] = -(int16_t)((buf[2] << 8) + buf[3]);
    data.accel.axis[2] = (int16_t)((buf[4] << 8) + buf[5]);

    data.temp = (float)((int16_t)((buf[6] << 8) + buf[7])) / 333.87f + 21.f;

    data.gyro.axis[1] = (int16_t)((buf[8] << 8) + buf[9]);
    data.gyro.axis[0] = (int16_t)((buf[10] << 8) + buf[11]);
    data.gyro.axis[2] = (int16_t)((buf[12] << 8) + buf[13]);
  }
  }

  return data;
}