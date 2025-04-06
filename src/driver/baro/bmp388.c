#include "driver/baro/bmp388.h"

#include "driver/i2c.h"

#ifdef USE_BARO

#define BMP388_I2C_ADDR (0x76)        // same as BMP280/BMP180
#define BMP388_DEFAULT_CHIP_ID (0x50) // from https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3_defs.h#L130

#define BMP388_CMD_REG (0x7E)
#define BMP388_RESERVED_UPPER_REG (0x7D)
// everything between BMP388_RESERVED_UPPER_REG and BMP388_RESERVED_LOWER_REG is reserved.
#define BMP388_RESERVED_LOWER_REG (0x20)
#define BMP388_CONFIG_REG (0x1F)
#define BMP388_RESERVED_0x1E_REG (0x1E)
#define BMP388_ODR_REG (0x1D)
#define BMP388_OSR_REG (0x1C)
#define BMP388_PWR_CTRL_REG (0x1B)
#define BMP388_IF_CONFIG_REG (0x1A)
#define BMP388_INT_CTRL_REG (0x19)
#define BMP388_FIFO_CONFIG_2_REG (0x18)
#define BMP388_FIFO_CONFIG_1_REG (0x17)
#define BMP388_FIFO_WTM_1_REG (0x16)
#define BMP388_FIFO_WTM_0_REG (0x15)
#define BMP388_FIFO_DATA_REG (0x14)
#define BMP388_FIFO_LENGTH_1_REG (0x13)
#define BMP388_FIFO_LENGTH_0_REG (0x12)
#define BMP388_INT_STATUS_REG (0x11)
#define BMP388_EVENT_REG (0x10)
#define BMP388_SENSORTIME_3_REG (0x0F) // BME780 only
#define BMP388_SENSORTIME_2_REG (0x0E)
#define BMP388_SENSORTIME_1_REG (0x0D)
#define BMP388_SENSORTIME_0_REG (0x0C)
#define BMP388_RESERVED_0x0B_REG (0x0B)
#define BMP388_RESERVED_0x0A_REG (0x0A)

// see friendly register names below
#define BMP388_DATA_5_REG (0x09)
#define BMP388_DATA_4_REG (0x08)
#define BMP388_DATA_3_REG (0x07)
#define BMP388_DATA_2_REG (0x06)
#define BMP388_DATA_1_REG (0x05)
#define BMP388_DATA_0_REG (0x04)

#define BMP388_STATUS_REG (0x03)
#define BMP388_ERR_REG (0x02)
#define BMP388_RESERVED_0x01_REG (0x01)
#define BMP388_CHIP_ID_REG (0x00)

// friendly register names, from datasheet 4.3.4
#define BMP388_PRESS_MSB_23_16_REG BMP388_DATA_2_REG
#define BMP388_PRESS_LSB_15_8_REG BMP388_DATA_1_REG
#define BMP388_PRESS_XLSB_7_0_REG BMP388_DATA_0_REG

// friendly register names, from datasheet 4.3.5
#define BMP388_TEMP_MSB_23_16_REG BMP388_DATA_5_REG
#define BMP388_TEMP_LSB_15_8_REG BMP388_DATA_4_REG
#define BMP388_TEMP_XLSB_7_0_REG BMP388_DATA_3_REG

#define BMP388_DATA_FRAME_SIZE ((BMP388_DATA_5_REG - BMP388_DATA_0_REG) + 1) // +1 for inclusive

// from Datasheet 3.3
#define BMP388_MODE_SLEEP (0x00)
#define BMP388_MODE_FORCED (0x01)
#define BMP388_MODE_NORMAL (0x03)

#define BMP388_CALIRATION_LOWER_REG (0x30)        // See datasheet 4.3.19, "calibration data"
#define BMP388_TRIMMING_NVM_PAR_T1_LSB_REG (0x31) // See datasheet 3.11.1 "Memory map trimming coefficients"
#define BMP388_TRIMMING_NVM_PAR_P11_REG (0x45)    // See datasheet 3.11.1 "Memory map trimming coefficients"
#define BMP388_CALIRATION_UPPER_REG (0x57)

#define BMP388_TRIMMING_DATA_LENGTH ((BMP388_TRIMMING_NVM_PAR_P11_REG - BMP388_TRIMMING_NVM_PAR_T1_LSB_REG) + 1) // +1 for inclusive

#define BMP388_OVERSAMP_1X (0x00)
#define BMP388_OVERSAMP_2X (0x01)
#define BMP388_OVERSAMP_4X (0x02)
#define BMP388_OVERSAMP_8X (0x03)
#define BMP388_OVERSAMP_16X (0x04)
#define BMP388_OVERSAMP_32X (0x05)

// INT_CTRL register
#define BMP388_INT_OD_BIT 0
#define BMP388_INT_LEVEL_BIT 1
#define BMP388_INT_LATCH_BIT 2
#define BMP388_INT_FWTM_EN_BIT 3
#define BMP388_INT_FFULL_EN_BIT 4
#define BMP388_INT_RESERVED_5_BIT 5
#define BMP388_INT_DRDY_EN_BIT 6
#define BMP388_INT_RESERVED_7_BIT 7

#define BMP388_PRESSURE_OSR (BMP388_OVERSAMP_8X)
#define BMP388_TEMPERATURE_OSR (BMP388_OVERSAMP_1X << 3)

#define BMP388_DRDY_PRESS (1 << 5)
#define BMP388_DRDY_TEMP (1 << 6)

typedef struct {
  uint16_t t1;
  uint16_t t2;
  int8_t t3;
  int16_t p1;
  int16_t p2;
  int8_t p3;
  int8_t p4;
  uint16_t p5;
  uint16_t p6;
  int8_t p7;
  int8_t p8;
  int16_t p9;
  int8_t p10;
  int8_t p11;
} __attribute__((packed)) bmp388_calib_param_t;
static_assert(sizeof(bmp388_calib_param_t) == BMP388_TRIMMING_DATA_LENGTH, "bmp388_calib_param_t has wrong size");

extern i2c_bus_device_t baro_bus;
static bmp388_calib_param_t bmp388_cal;

static baro_types_t bmp388_init() {
  baro_bus.address = BMP388_I2C_ADDR;

  const uint8_t chip_id = i2c_read_reg(&baro_bus, BMP388_CHIP_ID_REG);
  if (chip_id != BMP388_DEFAULT_CHIP_ID) {
    return BARO_TYPE_INVALID;
  }

  i2c_write_reg(&baro_bus, BMP388_CMD_REG, 0xB6);
  time_delay_ms(10);

  i2c_read_reg_bytes(&baro_bus, BMP388_TRIMMING_NVM_PAR_T1_LSB_REG, (uint8_t *)&bmp388_cal, sizeof(bmp388_cal));
  i2c_write_reg(&baro_bus, BMP388_ODR_REG, 0x2);
  i2c_write_reg(&baro_bus, BMP388_CONFIG_REG, 0x2);
  i2c_write_reg(&baro_bus, BMP388_OSR_REG, BMP388_PRESSURE_OSR | BMP388_TEMPERATURE_OSR);
  i2c_write_reg(&baro_bus, BMP388_PWR_CTRL_REG, BMP388_MODE_NORMAL << 4 | 1 << 1 | 1 << 0);

  return BARO_TYPE_BMP388;
}

static bool bmp388_get_pressure(float *pressure) {
  static uint8_t status = 0;
  if (!i2c_read_async(&baro_bus, BMP388_STATUS_REG, &status, 1))
    return false;

  if ((status & BMP388_DRDY_PRESS) == 0 || (status & BMP388_DRDY_TEMP) == 0)
    return false;

  const uint32_t uncomp_pressure = baro_buf[0] << 0 | baro_buf[1] << 8 | baro_buf[2] << 16;
  const uint32_t uncomp_temperature = baro_buf[3] << 0 | baro_buf[4] << 8 | baro_buf[5] << 16;

  const int64_t temp_data1 = (int64_t)(uncomp_temperature - ((int64_t)256 * bmp388_cal.t1));
  const int64_t temp_data2 = (int64_t)(bmp388_cal.t2 * temp_data1);
  const int64_t temp_data3 = (int64_t)(temp_data1 * temp_data1);
  const int64_t temp_data4 = (int64_t)temp_data3 * bmp388_cal.t3;
  const int64_t temp_data5 = (int64_t)((int64_t)(temp_data2 * 262144) + temp_data4);

  const int64_t t_lin = (int64_t)(temp_data5 / 4294967296);

  int64_t partial_data1 = (int64_t)(t_lin * t_lin);
  int64_t partial_data2 = (int64_t)(partial_data1 / 64);
  int64_t partial_data3 = (int64_t)((partial_data2 * t_lin) / 256);
  int64_t partial_data4 = (int64_t)((bmp388_cal.p8 * partial_data3) / 32);
  int64_t partial_data5 = (int64_t)((bmp388_cal.p7 * partial_data1) * 16);
  int64_t partial_data6 = (int64_t)((bmp388_cal.p6 * t_lin) * 4194304);

  const int64_t offset = (int64_t)((bmp388_cal.p5 * 140737488355328) + partial_data4 + partial_data5 + partial_data6);
  partial_data2 = (int64_t)((bmp388_cal.p4 * partial_data3) / 32);
  partial_data4 = (int64_t)((bmp388_cal.p3 * partial_data1) * 4);
  partial_data5 = (int64_t)((bmp388_cal.p2 - (int32_t)16384) * t_lin * 2097152);

  const int64_t sensitivity = (int64_t)(((bmp388_cal.p1 - (int32_t)16384) * 70368744177664) + partial_data2 + partial_data4 + partial_data5);
  partial_data1 = (int64_t)((sensitivity / 16777216) * uncomp_pressure);
  partial_data2 = (int64_t)(bmp388_cal.p10 * t_lin);
  partial_data3 = (int64_t)(partial_data2 + ((int32_t)65536 * bmp388_cal.p9));
  partial_data4 = (int64_t)((partial_data3 * uncomp_pressure) / (int32_t)8192);
  partial_data5 = (int64_t)((uncomp_pressure * (partial_data4 / 10)) / (int32_t)512);
  partial_data5 = (int64_t)(partial_data5 * 10);
  partial_data6 = (int64_t)(uncomp_pressure * uncomp_pressure);

  partial_data2 = (int64_t)((bmp388_cal.p11 * partial_data6) / (int32_t)65536);
  partial_data3 = (int64_t)((int64_t)(partial_data2 * uncomp_pressure) / 128);
  partial_data4 = (int64_t)((offset / 4) + partial_data1 + partial_data5 + partial_data3);

  const uint64_t comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776);
  *pressure = comp_press / 100.f;

  i2c_read_reg_bytes(&baro_bus, BMP388_DATA_0_REG, baro_buf, sizeof(baro_buf));

  return true;
}

baro_interface_t bmp388_interface = {
    .init = bmp388_init,
    .get_pressure = bmp388_get_pressure,
};

#endif