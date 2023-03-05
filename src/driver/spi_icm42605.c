#include "driver/spi_icm42605.h"

#include "driver/gpio.h"
#include "driver/spi.h"
#include "driver/spi_gyro.h"
#include "driver/time.h"
#include "project.h"

#define ICM42605_ID (0x42)
#define ICM42688P_ID (0x47)

#define SPI_SPEED_SLOW MHZ_TO_HZ(0.5)
#define SPI_SPEED_FAST MHZ_TO_HZ(24)

#define AAF_DELT 6
#define AAF_DELTSQR 36
#define AAF_BITSHIFT 10

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

  // gyro off & bank 0
  icm42605_write(ICM42605_PWR_MGMT0, 0x0);
  time_delay_ms(1);

  icm42605_write(ICM42605_GYRO_CONFIG1, 0x1a);  // 3rd order filter
  icm42605_write(ICM42605_ACCEL_CONFIG1, 0x15); // 3rd order filter

  icm42605_write(ICM42605_GYRO_ACCEL_CONFIG0, (14 << 4) | 14); // low latency
  icm42605_write(ICM42605_INT_CONFIG, ICM42605_INT1_MODE_PULSED | ICM42605_INT1_DRIVE_CIRCUIT_PP | ICM42605_INT1_POLARITY_ACTIVE_HIGH);
  icm42605_write(ICM42605_INT_CONFIG0, ICM42605_UI_DRDY_INT_CLEAR_ON_SBR);

  // bank 1
  icm42605_write(ICM42605_REG_BANK_SEL, 1);
  icm42605_write(ICM42605_GYRO_CONFIG_STATIC3, AAF_DELT);
  icm42605_write(ICM42605_GYRO_CONFIG_STATIC4, AAF_DELTSQR & 0xFF);
  icm42605_write(ICM42605_GYRO_CONFIG_STATIC5, (AAF_DELTSQR >> 8) | (AAF_BITSHIFT << 4));

  // bank 2
  icm42605_write(ICM42605_REG_BANK_SEL, 2);
  icm42605_write(ICM42605_ACCEL_CONFIG_STATIC2, AAF_DELT << 1);
  icm42605_write(ICM42605_ACCEL_CONFIG_STATIC3, AAF_DELTSQR & 0xFF);
  icm42605_write(ICM42605_ACCEL_CONFIG_STATIC4, (AAF_DELTSQR >> 8) | (AAF_BITSHIFT << 4));

  // gyro on & bank 0
  icm42605_write(ICM42605_REG_BANK_SEL, 0);
  icm42605_write(ICM42605_PWR_MGMT0, ICM42605_PWR_MGMT0_ACCEL_MODE_LN | ICM42605_PWR_MGMT0_GYRO_MODE_LN | ICM42605_PWR_MGMT0_TEMP_DISABLE_OFF);
  time_delay_ms(1);

  icm42605_write(ICM42605_GYRO_CONFIG0, ICM42605_GFS_2000DPS | ICM42605_GODR_8000Hz);
  time_delay_ms(15);

  icm42605_write(ICM42605_ACCEL_CONFIG0, ICM42605_AFS_16G | ICM42605_AODR_8000Hz);
  time_delay_ms(15);
}

uint8_t icm42605_read(uint8_t reg) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  uint8_t buffer[2] = {reg | 0x80, 0x00};

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(buffer, buffer, 2),
  };
  spi_seg_submit_wait(&gyro_bus, segs);

  return buffer[1];
}

void icm42605_write(uint8_t reg, uint8_t data) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg),
      spi_make_seg_const(data),
  };
  spi_seg_submit_wait(&gyro_bus, segs);
}

void icm42605_read_data(uint8_t reg, uint8_t *data, uint32_t size) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_FAST);

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg | 0x80),
      spi_make_seg_buffer(data, NULL, size),
  };
  spi_seg_submit_wait(&gyro_bus, segs);
}