#include "drv_spi_icm42605.h"

#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_spi_gyro.h"
#include "drv_time.h"
#include "project.h"

#define ICM42605_ID (0x42)
#define ICM42688P_ID (0x47)

#define SPI_SPEED_SLOW MHZ_TO_HZ(0.5)
#define SPI_SPEED_FAST MHZ_TO_HZ(24)

extern spi_bus_device_t gyro_bus;

uint8_t icm42605_detect() {
  const uint8_t id = icm42605_read(ICM42605_WHO_AM_I);
  switch (id) {
  case ICM42605_ID:
    return GYRO_TYPE_ICM42605;
  case ICM42688P_ID:
    return GYRO_TYPE_ICM42688P;
  default:
    return GYRO_TYPE_INVALID;
  }
}

void icm42605_configure() {
  icm42605_write(ICM42605_PWR_MGMT0, 0x00); // reset
  time_delay_ms(150);

  icm42605_write(ICM42605_PWR_MGMT0, ICM42605_PWR_MGMT0_ACCEL_MODE_LN | ICM42605_PWR_MGMT0_GYRO_MODE_LN | ICM42605_PWR_MGMT0_TEMP_DISABLE_OFF);
  time_delay_ms(15);

  icm42605_write(ICM42605_GYRO_CONFIG0, ICM42605_GFS_2000DPS | ICM42605_GODR_8000Hz);
  time_delay_ms(15);

  icm42605_write(ICM42605_ACCEL_CONFIG0, ICM42605_AFS_16G | ICM42605_AODR_8000Hz);
  time_delay_ms(15);

  icm42605_write(ICM42605_GYRO_ACCEL_CONFIG0, (14 << 4) | 14); // low latency
  time_delay_ms(15);

  icm42605_write(ICM42605_INT_CONFIG, ICM42605_INT1_MODE_PULSED | ICM42605_INT1_DRIVE_CIRCUIT_PP | ICM42605_INT1_POLARITY_ACTIVE_HIGH);
  time_delay_ms(15);

  icm42605_write(ICM42605_INT_CONFIG0, ICM42605_UI_DRDY_INT_CLEAR_ON_SBR);
  time_delay_ms(100);
}

uint8_t icm42605_read(uint8_t reg) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  uint8_t buffer[2] = {reg | 0x80, 0x00};

  spi_txn_t *txn = spi_txn_init(&gyro_bus, NULL);
  spi_txn_add_seg(txn, buffer, buffer, 2);
  spi_txn_submit(txn);

  spi_txn_wait(&gyro_bus);

  return buffer[1];
}

void icm42605_write(uint8_t reg, uint8_t data) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  spi_txn_t *txn = spi_txn_init(&gyro_bus, NULL);
  spi_txn_add_seg_const(txn, reg);
  spi_txn_add_seg_const(txn, data);
  spi_txn_submit(txn);

  spi_txn_wait(&gyro_bus);
}

void icm42605_read_data(uint8_t reg, uint8_t *data, uint32_t size) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_FAST);

  spi_txn_t *txn = spi_txn_init(&gyro_bus, NULL);
  spi_txn_add_seg_const(txn, reg | 0x80);
  spi_txn_add_seg(txn, data, NULL, size);
  spi_txn_submit(txn);

  spi_txn_wait(&gyro_bus);
}