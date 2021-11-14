#include "drv_spi_gyro.h"

#include "drv_spi_mpu6xxx.h"
#include "drv_time.h"
#include "project.h"

static uint8_t mpu6xxx_configure() {
  mpu6xxx_init();

  mpu6xxx_write(MPU_RA_PWR_MGMT_1, MPU_BIT_H_RESET); //reg 107 soft reset  MPU_BIT_H_RESET
  timer_delay_us(100000);
  mpu6xxx_write(MPU_RA_SIGNAL_PATH_RESET, MPU_RESET_SIGNAL_PATHWAYS);
  timer_delay_us(100000);
  mpu6xxx_write(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROX); //reg 107 set pll clock to 1 for x axis reference
  timer_delay_us(100000);
  mpu6xxx_write(MPU_RA_USER_CTRL, MPU_BIT_I2C_IF_DIS); //reg 106 to 16 enabling spi
  timer_delay_us(1500);
  mpu6xxx_write(MPU_RA_PWR_MGMT_2, MPU_BITS_STDBY_MODE_OFF); //reg 108 disable standbye mode to 0
  timer_delay_us(1500);
  mpu6xxx_write(MPU_RA_SMPLRT_DIV, MPU_BIT_SMPLRT_DIVIDER_OFF); //reg 25 sample rate divider to 0
  timer_delay_us(1500);
  mpu6xxx_write(MPU_RA_CONFIG, MPU_BITS_DLPF_CFG_256HZ); //reg 26 dlpf to 0 - 8khz
  timer_delay_us(1500);
  mpu6xxx_write(MPU_RA_ACCEL_CONFIG, MPU_BITS_FS_16G); //reg 28 accel scale to 16G
  timer_delay_us(1500);
  mpu6xxx_write(MPU_RA_GYRO_CONFIG, MPU_BITS_FS_2000DPS); //reg 27 gyro scale to 2000deg/s
  timer_delay_us(1500);
  mpu6xxx_write(MPU_RA_INT_ENABLE, MPU_BIT_INT_STATUS_DATA); //reg 56 data ready enable interrupt to 1
  timer_delay_us(1500);

  return mpu6xxx_read(MPU_RA_WHO_AM_I);
}

uint8_t spi_gyro_init() {
  switch (GYRO_TYPE) {
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