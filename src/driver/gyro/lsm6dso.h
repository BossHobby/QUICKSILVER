#pragma once

#include "driver/gyro/gyro.h"

#define LSM6DSO_REG_INT1_CTRL 0x0D
#define LSM6DSO_REG_INT2_CTRL 0x0E
#define LSM6DSO_REG_WHO_AM_I 0x0F
#define LSM6DSO_REG_CTRL1_XL 0x10
#define LSM6DSO_REG_CTRL2_G 0x11
#define LSM6DSO_REG_CTRL3_C 0x12
#define LSM6DSO_REG_CTRL4_C 0x13
#define LSM6DSO_REG_CTRL5_C 0x14
#define LSM6DSO_REG_CTRL6_C 0x15
#define LSM6DSO_REG_CTRL7_G 0x16
#define LSM6DSO_REG_CTRL8_XL 0x17
#define LSM6DSO_REG_CTRL9_XL 0x18
#define LSM6DSO_REG_CTRL10_C 0x19
#define LSM6DSO_REG_STATUS 0x1E
#define LSM6DSO_REG_OUT_TEMP_L 0x20
#define LSM6DSO_REG_OUT_TEMP_H 0x21
#define LSM6DSO_REG_OUTX_L_G 0x22
#define LSM6DSO_REG_OUTX_H_G 0x23
#define LSM6DSO_REG_OUTY_L_G 0x24
#define LSM6DSO_REG_OUTY_H_G 0x25
#define LSM6DSO_REG_OUTZ_L_G 0x26
#define LSM6DSO_REG_OUTZ_H_G 0x27
#define LSM6DSO_REG_OUTX_L_A 0x28
#define LSM6DSO_REG_OUTX_H_A 0x29
#define LSM6DSO_REG_OUTY_L_A 0x2A
#define LSM6DSO_REG_OUTY_H_A 0x2B
#define LSM6DSO_REG_OUTZ_L_A 0x2C
#define LSM6DSO_REG_OUTZ_H_A 0x2D

#define LSM6DSO_CHIP_ID 0x6C

gyro_types_t lsm6dso_detect();
void lsm6dso_configure();
void lsm6dso_read_gyro_data(gyro_data_t *data);

uint8_t lsm6dso_read(uint8_t reg);
void lsm6dso_write(uint8_t reg, uint8_t data);
