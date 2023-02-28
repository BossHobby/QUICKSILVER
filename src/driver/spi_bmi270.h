#pragma once

#include <stdint.h>

#include "driver/spi_gyro.h"

#define BMI270_REG_CHIP_ID 0x00
#define BMI270_REG_ERR_REG 0x02
#define BMI270_REG_STATUS 0x03
#define BMI270_REG_ACC_DATA_X_LSB 0x0C
#define BMI270_REG_SENSORTIME_0 0x18
#define BMI270_REG_SENSORTIME_1 0x19
#define BMI270_REG_SENSORTIME_2 0x1A
#define BMI270_REG_EVENT 0x1B
#define BMI270_REG_INT_STATUS_0 0x1C
#define BMI270_REG_INT_STATUS_1 0x1D
#define BMI270_REG_INTERNAL_STATUS 0x21
#define BMI270_REG_TEMPERATURE_LSB 0x22
#define BMI270_REG_TEMPERATURE_MSB 0x23
#define BMI270_REG_FIFO_LENGTH_LSB 0x24
#define BMI270_REG_FIFO_LENGTH_MSB 0x25
#define BMI270_REG_FIFO_DATA 0x26
#define BMI270_REG_FEAT_PAGE 0x2F
#define BMI270_REG_ACC_CONF 0x40
#define BMI270_REG_ACC_RANGE 0x41
#define BMI270_REG_GYRO_CONF 0x42
#define BMI270_REG_GYRO_RANGE 0x43
#define BMI270_REG_AUX_CONF 0x44
#define BMI270_REG_FIFO_DOWNS 0x45
#define BMI270_REG_FIFO_WTM_0 0x46
#define BMI270_REG_FIFO_WTM_1 0x47
#define BMI270_REG_FIFO_CONFIG_0 0x48
#define BMI270_REG_FIFO_CONFIG_1 0x49
#define BMI270_REG_SATURATION 0x4A
#define BMI270_REG_INT1_IO_CTRL 0x53
#define BMI270_REG_INT2_IO_CTRL 0x54
#define BMI270_REG_INT_LATCH 0x55
#define BMI270_REG_INT1_MAP_FEAT 0x56
#define BMI270_REG_INT2_MAP_FEAT 0x57
#define BMI270_REG_INT_MAP_DATA 0x58
#define BMI270_REG_INIT_CTRL 0x59
#define BMI270_REG_INIT_DATA 0x5E
#define BMI270_REG_GYR_CRT_CONF 0x69
#define BMI270_REG_ACC_SELF_TEST 0x6D
#define BMI270_REG_GYR_SELF_TEST_AXES 0x6E
#define BMI270_REG_OFFSET_6 0x77
#define BMI270_REG_PWR_CONF 0x7C
#define BMI270_REG_PWR_CTRL 0x7D
#define BMI270_REG_CMD 0x7E

#define BMI270_REG_FEATURES_0_GYR_GAIN_STATUS 0x38
#define BMI270_REG_FEATURES_0_GYR_CAS 0x3C
#define BMI270_REG_FEATURES_1_G_TRIG_1 0x32
#define BMI270_REG_FEATURES_1_GEN_SET_1 0x34

#define BMI270_CMD_SOFTRESET 0xB6
#define BMI270_CMD_FIFOFLUSH 0xB0
#define BMI270_PWR_CTRL 0x0E                    // enable gyro, acc and temp sensors
#define BMI270_PWR_CONF 0x02                    // disable advanced power save, enable FIFO self-wake
#define BMI270_ACC_CONF_ODR800 0x0B             // set acc sample rate to 800hz
#define BMI270_ACC_CONF_ODR1600 0x0C            // set acc sample rate to 1600hz
#define BMI270_ACC_CONF_BWP 0x02                // set acc filter in normal mode
#define BMI270_ACC_CONF_HP 0x01                 // set acc in high performance mode
#define BMI270_ACC_RANGE_8G 0x02                // set acc to 8G full scale
#define BMI270_ACC_RANGE_16G 0x03               // set acc to 16G full scale
#define BMI270_GYRO_CONF_ODR3200 0x0D           // set gyro sample rate to 3200hz
#define BMI270_GYRO_CONF_BWP 0x00               // set gyro filter in osr4 mode
#define BMI270_GYRO_CONF_NOISE_PERF 0x01        // set gyro in high performance noise mode
#define BMI270_GYRO_CONF_FILTER_PERF 0x01       // set gyro in high performance filter mode
#define BMI270_GYRO_CONF_OFFSET_6 0x40          // enable offset compensation
#define BMI270_CONF_FEATURES_1_GEN_SET_1 0x0288 // enable gyr_self_off

// set gyro to 2000dps full scale
// for some reason you have to enable the ois_range bit (bit 3) for 2000dps as well
// or else the gyro scale will be 250dps when in prefiltered FIFO mode (not documented in datasheet!)
#define BMI270_GYRO_RANGE_2000DPS 0x08

#define BMI270_INT_MAP_DATA_DRDY_INT1 0x04 // enable the data ready interrupt pin 1
#define BMI270_INT_MAP_FIFO_WM_INT1 0x02   // enable the FIFO watermark interrupt pin 1
#define BMI270_INT1_IO_CTRL_PINMODE 0x0A   // active high, push-pull, output enabled, input disabled
#define BMI270_FIFO_CONFIG_0 0x00          // don't stop when full, disable sensortime frame
#define BMI270_FIFO_CONFIG_1 0x80          // only gyro data in FIFO, use headerless mode
#define BMI270_FIFO_DOWNS 0x00             // select unfiltered gyro data with no downsampling (6.4KHz samples)
#define BMI270_FIFO_WTM_0 0x06             // set the FIFO watermark level to 1 gyro sample (6 bytes)
#define BMI270_FIFO_WTM_1 0x00             // FIFO watermark MSB

#define BMI270_GYRO_CAS_MASK 0x7F
#define BMI270_GYRO_CAS_SIGN_BIT_MASK 0x40

uint8_t bmi270_detect();
void bmi270_configure();
void bmi270_calibrate();

void bmi270_write(uint8_t reg, uint8_t data, uint32_t delay);
void bmi270_write16(uint8_t reg, uint16_t data, uint32_t delay);
void bmi270_write_data(uint8_t reg, uint8_t *data, uint32_t size, uint32_t delay);

uint8_t bmi270_read(uint8_t reg);
uint16_t bmi270_read16(uint8_t reg);
void bmi270_read_data(uint8_t reg, uint8_t *data, uint32_t size);
void bmi270_read_gyro_data(gyro_data_t *data);