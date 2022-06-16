#include "drv_spi_mpu6xxx.h"

#include <stdio.h>

#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_spi_gyro.h"
#include "drv_time.h"
#include "project.h"

#define MPU6000_ID (0x68)
#define MPU6500_ID (0x70)

#define ICM20601_ID (0xAC)
#define ICM20602_ID (0x12)
#define ICM20608_ID (0xAF)
#define ICM20649_ID (0xE1)
#define ICM20689_ID (0x98)

#define SPI_SPEED_INIT MHZ_TO_HZ(0.5)

extern DMA_RAM spi_bus_device_t gyro_bus;

static uint32_t mpu6xxx_slow_divider() {
  return spi_find_divder(SPI_SPEED_INIT);
}

static uint32_t mpu6xxx_fast_divider() {
  switch (gyro_type) {
  default:
  case GYRO_TYPE_ICM20649:
    return spi_find_divder(MHZ_TO_HZ(7));

  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20689:
    return spi_find_divder(MHZ_TO_HZ(8));

  case GYRO_TYPE_ICM20602:
    return spi_find_divder(MHZ_TO_HZ(10.5));

  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
    return spi_find_divder(MHZ_TO_HZ(21));
  }

  return spi_find_divder(SPI_SPEED_INIT);
}

uint8_t mpu6xxx_detect() {
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
  case ICM20649_ID:
    return GYRO_TYPE_ICM20649;
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
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, mpu6xxx_slow_divider());

  uint8_t buffer[2] = {reg | 0x80, 0x00};

  spi_txn_t *txn = spi_txn_init(&gyro_bus, NULL);
  spi_txn_add_seg(txn, buffer, buffer, 2);
  spi_txn_submit(txn);

  spi_txn_wait(&gyro_bus);

  return buffer[1];
}

// blocking dma write of a single register
void mpu6xxx_write(uint8_t reg, uint8_t data) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, mpu6xxx_slow_divider());

  spi_txn_t *txn = spi_txn_init(&gyro_bus, NULL);
  spi_txn_add_seg_const(txn, reg);
  spi_txn_add_seg_const(txn, data);
  spi_txn_submit(txn);

  spi_txn_wait(&gyro_bus);
}

void mpu6xxx_read_data(uint8_t reg, uint8_t *data, uint32_t size) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, mpu6xxx_fast_divider());

  spi_txn_t *txn = spi_txn_init(&gyro_bus, NULL);
  spi_txn_add_seg_const(txn, reg | 0x80);
  spi_txn_add_seg(txn, data, NULL, size);
  spi_txn_submit(txn);

  spi_txn_wait(&gyro_bus);
}
