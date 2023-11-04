#pragma once

#include <stdint.h>

#include "driver/spi_gyro.h"

#define BMI323_REG_CHIP_ID 0x00
#define BMI323_WHO_AMI 0x43
#define BMI323_REG_ERR_REG 0x01
#define BMI323_REG_STATUS 0x03

#define BMI323_REG_ACC_DATA_X_LSB 0x03

#define BMI323_REG_SENSORTIME_0 0x0A
#define BMI323_REG_SENSORTIME_1 0x0B
// Overflow state
#define BMI323_REG_SAT_FLAGS 0x0C

// ACC_CONF GYRO_CONF 16bit
#define BMI323_REG_ACC_CONF 0x20
#define BMI323_REG_GYRO_CONF 0x21

// Interupt status for pull
#define BMI323_REG_INT_STATUS_INT1 0x0D
#define BMI323_REG_INT_STATUS_INT2 0x0E
#define BMI323_REG_INT_STATUS_IBI 0x0F

// Interupt output conf
#define BMI323_REG_IO_INT_CTRL 0x38
#define BMI323_REG_INT_LATCH_CONF 0x39
#define BMI323_REG_INT_MAP2 0x3B

// SPI3 CONF
#define BMI323_REG_IO_SPI_IF 0x50
#define BMI323_DAT_IO_SPI_3_EN 0x01
#define BMI323_DAT_IO_SPI_4_DEFAULT 0x00

// OFFSET AND GAIN FOR CALIBRATION
#define BMI323_REG_ACC_DP_OFF_X 0x60
#define BMI323_REG_ACC_DP_DGAIN_X 0x61
#define BMI323_REG_ACC_DP_OFF_Y 0x62
#define BMI323_REG_ACC_DP_DGAIN_Y 0x63
#define BMI323_REG_ACC_DP_OFF_Z 0x64
#define BMI323_REG_ACC_DP_DGAIN_Z 0x65

#define BMI323_REG_GYRO_DP_OFF_X 0x66
#define BMI323_REG_GYRO_DP_DGAIN_X 0x67
#define BMI323_REG_GYRO_DP_OFF_Y 0x68
#define BMI323_REG_GYRO_DP_DGAIN_Y 0x69
#define BMI323_REG_GYRO_DP_OFF_Z 0x6A
#define BMI323_REG_GYRO_DP_DGAIN_Z 0x6B

#define BMI323_REG_CMD 0x7E
#define BMI323_REG_CFG_RES 0x7F

/******************************************************************************/
/*!        Accelerometer Macro Definitions For CONF              */
/******************************************************************************/
/*!  Accelerometer Bandwidth parameters */
#define BMI323_ACC_AVG1 UINT8_C(0x00)
#define BMI323_ACC_AVG2 UINT8_C(0x01)
#define BMI323_ACC_AVG4 UINT8_C(0x02)
#define BMI323_ACC_AVG8 UINT8_C(0x03)
#define BMI323_ACC_AVG16 UINT8_C(0x04)
#define BMI323_ACC_AVG32 UINT8_C(0x05)
#define BMI323_ACC_AVG64 UINT8_C(0x06)

/*! Accelerometer Output Data Rate */
#define BMI323_ACC_ODR_0_78HZ UINT8_C(0x01)
#define BMI323_ACC_ODR_1_56HZ UINT8_C(0x02)
#define BMI323_ACC_ODR_3_125HZ UINT8_C(0x03)
#define BMI323_ACC_ODR_6_25HZ UINT8_C(0x04)
#define BMI323_ACC_ODR_12_5HZ UINT8_C(0x05)
#define BMI323_ACC_ODR_25HZ UINT8_C(0x06)
#define BMI323_ACC_ODR_50HZ UINT8_C(0x07)
#define BMI323_ACC_ODR_100HZ UINT8_C(0x08)
#define BMI323_ACC_ODR_200HZ UINT8_C(0x09)
#define BMI323_ACC_ODR_400HZ UINT8_C(0x0A)
#define BMI323_ACC_ODR_800HZ UINT8_C(0x0B)
#define BMI323_ACC_ODR_1600HZ UINT8_C(0x0C)
#define BMI323_ACC_ODR_3200HZ UINT8_C(0x0D)
#define BMI323_ACC_ODR_6400HZ UINT8_C(0x0E)

/*! Accelerometer G Range */
#define BMI323_ACC_RANGE_2G UINT8_C(0x00)
#define BMI323_ACC_RANGE_4G UINT8_C(0x01)
#define BMI323_ACC_RANGE_8G UINT8_C(0x02)
#define BMI323_ACC_RANGE_16G UINT8_C(0x03)

/*! Accelerometer mode */
#define BMI3_ACC_MODE_DISABLE UINT8_C(0x00)
#define BMI3_ACC_MODE_LOW_PWR UINT8_C(0x03)
#define BMI3_ACC_MODE_NORMAL UINT8_C(0X04)
#define BMI3_ACC_MODE_HIGH_PERF UINT8_C(0x07)

/*! Accelerometer bandwidth */
#define BMI3_ACC_BW_ODR_HALF UINT8_C(0)
#define BMI3_ACC_BW_ODR_QUARTER UINT8_C(1)

/******************************************************************************/
/*!       Gyroscope Macro Definitions For Conf            */
/******************************************************************************/
/*!  Gyroscope Bandwidth parameters */
#define BMI323_GYR_AVG1 UINT8_C(0x00)
#define BMI323_GYR_AVG2 UINT8_C(0x01)
#define BMI323_GYR_AVG4 UINT8_C(0x02)
#define BMI323_GYR_AVG8 UINT8_C(0x03)
#define BMI323_GYR_AVG16 UINT8_C(0x04)
#define BMI323_GYR_AVG32 UINT8_C(0x05)
#define BMI323_GYR_AVG64 UINT8_C(0x06)

/*! Gyroscope Output Data Rate */
#define BMI323_GYR_ODR_0_78HZ UINT8_C(0x01)
#define BMI323_GYR_ODR_1_56HZ UINT8_C(0x02)
#define BMI323_GYR_ODR_3_125HZ UINT8_C(0x03)
#define BMI323_GYR_ODR_6_25HZ UINT8_C(0x04)
#define BMI323_GYR_ODR_12_5HZ UINT8_C(0x05)
#define BMI323_GYR_ODR_25HZ UINT8_C(0x06)
#define BMI323_GYR_ODR_50HZ UINT8_C(0x07)
#define BMI323_GYR_ODR_100HZ UINT8_C(0x08)
#define BMI323_GYR_ODR_200HZ UINT8_C(0x09)
#define BMI323_GYR_ODR_400HZ UINT8_C(0x0A)
#define BMI323_GYR_ODR_800HZ UINT8_C(0x0B)
#define BMI323_GYR_ODR_1600HZ UINT8_C(0x0C)
#define BMI323_GYR_ODR_3200HZ UINT8_C(0x0D)
#define BMI323_GYR_ODR_6400HZ UINT8_C(0x0E)

/*! Gyroscope DPS Range */
#define BMI323_GYR_RANGE_125DPS UINT8_C(0x00)
#define BMI323_GYR_RANGE_250DPS UINT8_C(0x01)
#define BMI323_GYR_RANGE_500DPS UINT8_C(0x02)
#define BMI323_GYR_RANGE_1000DPS UINT8_C(0x03)
#define BMI323_GYR_RANGE_2000DPS UINT8_C(0x04)

/*! Gyroscope mode */
#define BMI323_GYR_MODE_DISABLE UINT8_C(0x00)
#define BMI323_GYR_MODE_SUSPEND UINT8_C(0X01)
#define BMI323_GYR_MODE_LOW_PWR UINT8_C(0x03)
#define BMI323_GYR_MODE_NORMAL UINT8_C(0X04)
#define BMI323_GYR_MODE_HIGH_PERF UINT8_C(0x07)

/*! Gyroscope bandwidth */
#define BMI323_GYR_BW_ODR_HALF UINT8_C(0)
#define BMI323_GYR_BW_ODR_QUARTER UINT8_C(1)

/******************************************************************************/
/*! @name BMI3 Interrupt Modes Conf*/
/******************************************************************************/
/*! Non latched */
#define BMI3_INT_NON_LATCH UINT8_C(0)

#define BMI3_INT_LATCH_EN UINT8_C(1)
#define BMI3_INT_LATCH_DISABLE UINT8_C(0)

/*! BMI3 Interrupt Pin Behavior */
#define BMI3_INT_PUSH_PULL UINT8_C(0)
#define BMI3_INT_OPEN_DRAIN UINT8_C(1)

/*! BMI3 Interrupt Pin Level */
#define BMI3_INT_ACTIVE_LOW UINT8_C(0)
#define BMI3_INT_ACTIVE_HIGH UINT8_C(1)

/*! BMI3 Interrupt Output Enable */
#define BMI3_INT_OUTPUT_DISABLE UINT8_C(0)
#define BMI3_INT_OUTPUT_ENABLE UINT8_C(1)

/******************************************************************************/
/*! @name       Status macros                                                 */
/******************************************************************************/
#define BMI3_STATUS_POR UINT8_C(0x01)
#define BMI3_STATUS_DRDY_TEMP UINT8_C(0x20)
#define BMI3_STATUS_DRDY_GYR UINT8_C(0x40)
#define BMI3_STATUS_DRDY_ACC UINT8_C(0x80)

/******************************************************************************/
/*! @name       Gyro self-calibration/self-test coefficient macros  */
/******************************************************************************/
#define BMI3_SC_ST_VALUE_0 UINT16_C(0x5A2E)
#define BMI3_SC_ST_VALUE_1 UINT16_C(0x9219)
#define BMI3_SC_ST_VALUE_2 UINT16_C(0x5637)
#define BMI3_SC_ST_VALUE_3 UINT16_C(0xFFE8)
#define BMI3_SC_ST_VALUE_4 UINT16_C(0xFFEF)
#define BMI3_SC_ST_VALUE_5 UINT16_C(0x000D)
#define BMI3_SC_ST_VALUE_6 UINT16_C(0x07CA)
#define BMI3_SC_ST_VALUE_7 UINT16_C(0xFFCD)
#define BMI3_SC_ST_VALUE_8 UINT16_C(0xEF6C)

#define BMI3_SC_SENSITIVITY_EN UINT8_C(1)
#define BMI3_SC_OFFSET_EN UINT8_C(2)

/*! Self-calibration enable disable macros */
#define BMI3_SC_APPLY_CORR_DIS UINT8_C(0)
#define BMI3_SC_APPLY_CORR_EN UINT8_C(4)

/* CMD DATA*/
#define BMI323_CMD_SOFT_RESET UINT16_C(0xDEAF)
#define BMI323_CMD_SELFTEST UINT16_C(0x0100)
#define BMI323_CMD_GYRO_SELF_CALI UINT16_C(0x0101)
#define BMI323_CMD_GYRO_CALI_ABORT UINT16_C(0x200)
#define BMI323_CMD_I3C_SYNC_CONF UINT16_C(0x201)
#define BMI323_CMD_AXIS_MAP UINT16_C(0x300)

uint8_t bmi323_detect();
void bmi323_configure();

void bmi3_write8(uint8_t reg, uint8_t data, uint32_t delay);
void bmi3_write16(uint8_t reg, uint16_t data, uint32_t delay);
void bmi3_write_data(uint8_t reg, uint8_t *data, uint32_t size, uint32_t delay);

uint8_t bmi3_read8(uint8_t reg);
uint16_t bmi3_read16(uint8_t reg);
void bmi323_read_data(uint8_t reg, uint8_t *data, uint32_t size);

void bmi323_read_gyro_data(gyro_data_t *data);