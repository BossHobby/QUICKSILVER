#include "driver/gyro/lsm6dso.h"

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

gyro_types_t lsm6dso_detect() {
  const uint8_t id = lsm6dso_read(LSM6DSO_REG_WHO_AM_I);
  if (id == LSM6DSO_CHIP_ID) {
    return GYRO_TYPE_LSM6DSO;
  }
  return GYRO_TYPE_INVALID;
}

void lsm6dso_configure() {
  // Reset device
  lsm6dso_write(LSM6DSO_REG_CTRL3_C, 0x01);
  time_delay_ms(100);

  // Configure interrupt pins
  lsm6dso_write(LSM6DSO_REG_INT1_CTRL, 0x02); // gyro data ready on INT1
  lsm6dso_write(LSM6DSO_REG_INT2_CTRL, 0x02); // gyro data ready on INT2

  // Configure accelerometer: 833Hz ODR, 16G scale, LPF1 output
  lsm6dso_write(LSM6DSO_REG_CTRL1_XL, (0x07 << 4) | (0x01 << 2) | (0x00 << 1));

  // Configure gyro: 6664Hz ODR, 2000DPS scale
  lsm6dso_write(LSM6DSO_REG_CTRL2_G, (0x0A << 4) | (0x03 << 2));

  // Control register 3: BDU, active high, push-pull, 4-wire SPI, auto-increment
  lsm6dso_write(LSM6DSO_REG_CTRL3_C, 0x44 | 0x04);

  // Control register 4: disable I2C, enable gyro LPF1
  lsm6dso_write(LSM6DSO_REG_CTRL4_C, 0x06);

  // Control register 6: accel high performance, gyro LPF1 ~335Hz
  lsm6dso_write(LSM6DSO_REG_CTRL6_C, 0x00);

  // Control register 9: disable I3C
  lsm6dso_write(LSM6DSO_REG_CTRL9_XL, 0x02);
}

uint8_t lsm6dso_read(uint8_t reg) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  uint8_t buffer[2] = {reg | 0x80, 0x00};

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(buffer, buffer, 2),
  };
  spi_seg_submit_wait(&gyro_bus, segs);

  return buffer[1];
}

void lsm6dso_write(uint8_t reg, uint8_t data) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg, data),
  };
  spi_seg_submit_wait(&gyro_bus, segs);
}

void lsm6dso_read_gyro_data(gyro_data_t *data) {
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
      spi_make_seg_const(LSM6DSO_REG_OUT_TEMP_L | 0x80),
      spi_make_seg_buffer(gyro_buf, NULL, 14),
  };
  spi_seg_submit(&gyro_bus, segs);
  while (!spi_txn_continue(&gyro_bus))
    ;
}

#endif
