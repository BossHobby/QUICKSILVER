#include "driver/spi_icm42605.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/spi.h"
#include "driver/spi_gyro.h"
#include "driver/time.h"
#include "util/util.h"

#ifdef USE_GYRO

#define ICM42605_ID (0x42)
#define ICM42688P_ID (0x47)

#define SPI_SPEED_SLOW MHZ_TO_HZ(0.5)
#define SPI_SPEED_FAST MHZ_TO_HZ(24)

// select 213hz filter
#define AAF_DELT 5
#define AAF_DELTSQR 25
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

  icm42605_write(ICM42605_GYRO_ACCEL_CONFIG0, (15 << 4) | 15); // low latency
  icm42605_write(ICM42605_INT_CONFIG, ICM42605_INT1_MODE_PULSED | ICM42605_INT1_DRIVE_CIRCUIT_PP | ICM42605_INT1_POLARITY_ACTIVE_HIGH);
  icm42605_write(ICM42605_INT_CONFIG0, ICM42605_UI_DRDY_INT_CLEAR_ON_SBR);

  {
    // Disable AFSR to prevent stalls in gyro output
    // Undocumented setting see
    // https://github.com/ArduPilot/ardupilot/issues/25025
    // https://github.com/ArduPilot/ardupilot/pull/25332
    uint8_t val = icm42605_read(ICM42605_INTF_CONFIG1);
    val &= ~ICM42605_INTF_CONFIG1_AFSR_MASK;
    val |= ICM42605_INTF_CONFIG1_AFSR_DISABLE;
    icm42605_write(ICM42605_INTF_CONFIG1, val);
  }

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
      spi_make_seg_const(reg, data),
  };
  spi_seg_submit_wait(&gyro_bus, segs);
}

void icm42605_read_gyro_data(gyro_data_t *data) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_FAST);

  uint8_t buf[14];
  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(ICM42605_TEMP_DATA1 | 0x80),
      spi_make_seg_buffer(buf, NULL, 14),
  };
  spi_seg_submit_wait(&gyro_bus, segs);

  data->temp = (float)((int16_t)((buf[0] << 8) | buf[1])) / 132.48f + 25.f;

  data->accel.pitch = -(int16_t)((buf[2] << 8) | buf[3]);
  data->accel.roll = -(int16_t)((buf[4] << 8) | buf[5]);
  data->accel.yaw = (int16_t)((buf[6] << 8) | buf[7]);

  data->gyro.pitch = (int16_t)((buf[8] << 8) | buf[9]);
  data->gyro.roll = (int16_t)((buf[10] << 8) | buf[11]);
  data->gyro.yaw = (int16_t)((buf[12] << 8) | buf[13]);
}
#endif