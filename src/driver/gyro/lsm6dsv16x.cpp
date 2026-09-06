#include "driver/gyro/lsm6dsv16x.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/gyro/gyro.h"
#include "driver/spi.h"
#include "driver/time.h"
#include "util/util.h"

#ifdef USE_GYRO

#define SPI_SPEED_SLOW MHZ_TO_HZ(0.5)
#define SPI_SPEED_FAST MHZ_TO_HZ(10)

extern spi_bus_device_t gyro_bus;
extern uint8_t gyro_buf[32];

gyro_types_t lsm6dsv16x_detect() {
  const uint8_t id = lsm6dsv16x_read(LSM6DSV_WHO_AM_I);
  switch (id) {
  case LSM6DSV16X_CHIP_ID:
    return GYRO_TYPE_LSM6DSV16X;
  case LSM6DSK320X_CHIP_ID:
    return GYRO_TYPE_LSM6DSK320X;
  default:
    return GYRO_TYPE_INVALID;
  }
}

void lsm6dsv16x_configure() {
  // Software reset
  lsm6dsv16x_write(LSM6DSV_CTRL3, LSM6DSV_CTRL3_SW_RESET);
  time_delay_ms(10);

  // Wait for reset to complete
  while (lsm6dsv16x_read(LSM6DSV_CTRL3) & LSM6DSV_CTRL3_SW_RESET) {
    time_delay_ms(1);
  }

  // Auto-increment and BDU
  lsm6dsv16x_write(LSM6DSV_CTRL3, LSM6DSV_CTRL3_IF_INC | LSM6DSV_CTRL3_BDU);

  // Select high-accuracy ODR mode 1
  lsm6dsv16x_write(LSM6DSV_HAODR_CFG, LSM6DSV_HAODR_MODE1);

  // Configure accelerometer: high accuracy mode, 16G, 1kHz
  lsm6dsv16x_write(LSM6DSV_CTRL1,
                   (LSM6DSV_CTRL1_OP_MODE_XL_HIGH_ACCURACY << 4) |
                       (LSM6DSV_CTRL1_ODR_XL_1000HZ << 0));

  // Configure gyro: high accuracy mode, 2000DPS, 8kHz, LPF1 ~288Hz
  lsm6dsv16x_write(LSM6DSV_CTRL6,
                   LSM6DSV_CTRL6_LPF1_G_BW_288HZ |
                       LSM6DSV_CTRL6_FS_G_2000DPS);

  lsm6dsv16x_write(LSM6DSV_CTRL2,
                   (LSM6DSV_CTRL2_OP_MODE_G_HIGH_ACCURACY << 4) |
                       (LSM6DSV_CTRL2_ODR_G_8000HZ << 0));

  // Enable gyro LPF1
  lsm6dsv16x_write(LSM6DSV_CTRL7, LSM6DSV_CTRL7_LPF1_G_EN);

  // Enable accel LPF2
  lsm6dsv16x_write(LSM6DSV_CTRL8, LSM6DSV_CTRL8_FS_XL_16G);
  lsm6dsv16x_write(LSM6DSV_CTRL9, LSM6DSV_CTRL9_LPF2_XL_EN);

  // Pulsed DRDY
  lsm6dsv16x_write(LSM6DSV_CTRL4, LSM6DSV_CTRL4_DRDY_PULSED);

  // Enable INT1 for gyro DRDY
  lsm6dsv16x_write(LSM6DSV_INT1_CTRL, LSM6DSV_INT1_CTRL_INT1_DRDY_G);
}

uint8_t lsm6dsv16x_read(uint8_t reg) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  uint8_t buffer[2] = {reg | 0x80, 0x00};

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(buffer, buffer, 2),
  };
  spi_seg_submit_wait(&gyro_bus, segs);

  return buffer[1];
}

void lsm6dsv16x_write(uint8_t reg, uint8_t data) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg, data),
  };
  spi_seg_submit_wait(&gyro_bus, segs);
}

void lsm6dsv16x_read_gyro_data(gyro_data_t *data) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_FAST);
  spi_txn_wait(&gyro_bus);

  // Temperature: 16-bit signed, 256 LSB/°C, offset 0°C
  data->temp = (float)((int16_t)((gyro_buf[0] << 8) | gyro_buf[1])) / 256.0f;

  // Accel
  data->accel.pitch = -(int16_t)((gyro_buf[2] << 8) | gyro_buf[3]);
  data->accel.roll = -(int16_t)((gyro_buf[4] << 8) | gyro_buf[5]);
  data->accel.yaw = (int16_t)((gyro_buf[6] << 8) | gyro_buf[7]);

  // Gyro
  data->gyro.pitch = (int16_t)((gyro_buf[8] << 8) | gyro_buf[9]);
  data->gyro.roll = (int16_t)((gyro_buf[10] << 8) | gyro_buf[11]);
  data->gyro.yaw = (int16_t)((gyro_buf[12] << 8) | gyro_buf[13]);

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(LSM6DSV_OUT_TEMP_L | 0x80),
      spi_make_seg_buffer(gyro_buf, NULL, 14),
  };
  spi_seg_submit(&gyro_bus, segs);
  while (!spi_txn_continue(&gyro_bus))
    ;
}

#endif
