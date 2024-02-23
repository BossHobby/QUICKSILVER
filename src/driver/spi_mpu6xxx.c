#include "driver/spi_mpu6xxx.h"

#include <stdio.h>

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/spi.h"
#include "driver/spi_gyro.h"
#include "driver/time.h"
#include "util/util.h"

#ifdef USE_GYRO

#define MPU6000_ID (0x68)
#define MPU6500_ID (0x70)

#define ICM20601_ID (0xAC)
#define ICM20602_ID (0x12)
#define ICM20608_ID (0xAF)
#define ICM20689_ID (0x98)

#define SPI_SPEED_INIT MHZ_TO_HZ(0.5)

extern spi_bus_device_t gyro_bus;

static uint32_t mpu6xxx_fast_divider() {
  switch (gyro_type) {
  default:
  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20689:
    return MHZ_TO_HZ(8);

  case GYRO_TYPE_ICM20602:
    return MHZ_TO_HZ(10.5);

  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
    return MHZ_TO_HZ(21);
  }

  return SPI_SPEED_INIT;
}

uint8_t mpu6xxx_detect() {
  time_delay_ms(100);

  const uint8_t id = mpu6xxx_read(MPU_RA_WHO_AM_I);
  switch (id) {
  case MPU6000_ID:
    return GYRO_TYPE_MPU6000;
  case MPU6500_ID:
    return GYRO_TYPE_MPU6500;
  case ICM20601_ID:
    return GYRO_TYPE_ICM20601;
  case ICM20602_ID:
    return GYRO_TYPE_ICM20602;
  case ICM20608_ID:
    return GYRO_TYPE_ICM20608;
  case ICM20689_ID:
    return GYRO_TYPE_ICM20689;
  default:
    return GYRO_TYPE_INVALID;
  }
}

void mpu6xxx_configure() {
  mpu6xxx_write(MPU_RA_PWR_MGMT_1, MPU_BIT_H_RESET); // reg 107 soft reset  MPU_BIT_H_RESET
  time_delay_ms(100);
  mpu6xxx_write(MPU_RA_SIGNAL_PATH_RESET, MPU_RESET_SIGNAL_PATHWAYS);
  time_delay_ms(100);
  mpu6xxx_write(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROX); // reg 107 set pll clock to 1 for x axis reference
  time_delay_ms(100);
  mpu6xxx_write(MPU_RA_USER_CTRL, MPU_BIT_I2C_IF_DIS); // reg 106 to 16 enabling spi
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_PWR_MGMT_2, MPU_BITS_STDBY_MODE_OFF); // reg 108 disable standbye mode to 0
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_SMPLRT_DIV, MPU_BIT_SMPLRT_DIVIDER_OFF); // reg 25 sample rate divider to 0
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_CONFIG, MPU_BITS_DLPF_CFG_256HZ); // reg 26 dlpf to 0 - 8khz
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_ACCEL_CONFIG, MPU_BITS_FS_16G); // reg 28 accel scale to 16G
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_GYRO_CONFIG, MPU_BITS_FS_2000DPS); // reg 27 gyro scale to 2000deg/s
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_INT_ENABLE, MPU_BIT_INT_STATUS_DATA); // reg 56 data ready enable interrupt to 1
  time_delay_us(1500);
}

// blocking dma read of a single register
uint8_t mpu6xxx_read(uint8_t reg) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_INIT);

  uint8_t buffer[2] = {reg | 0x80, 0x00};

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(buffer, buffer, 2),
  };
  spi_seg_submit_wait(&gyro_bus, segs);

  return buffer[1];
}

// blocking dma write of a single register
void mpu6xxx_write(uint8_t reg, uint8_t data) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_INIT);

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg, data),
  };
  spi_seg_submit_wait(&gyro_bus, segs);
}

void mpu6xxx_read_gyro_data(gyro_data_t *data) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, mpu6xxx_fast_divider());

  uint8_t buf[14];
  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(MPU_RA_ACCEL_XOUT_H | 0x80),
      spi_make_seg_buffer(buf, NULL, 14),
  };
  spi_seg_submit_wait(&gyro_bus, segs);

  data->accel.pitch = -(int16_t)((buf[0] << 8) | buf[1]);
  data->accel.roll = -(int16_t)((buf[2] << 8) | buf[3]);
  data->accel.yaw = (int16_t)((buf[4] << 8) | buf[5]);

  data->temp = (float)((int16_t)((buf[6] << 8) | buf[7])) / 333.87f + 21.f;

  data->gyro.pitch = (int16_t)((buf[8] << 8) | buf[9]);
  data->gyro.roll = (int16_t)((buf[10] << 8) | buf[11]);
  data->gyro.yaw = (int16_t)((buf[12] << 8) | buf[13]);
}

#endif