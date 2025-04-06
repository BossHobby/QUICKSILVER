#pragma once

#define USE_SPI
#define USE_SERIAL
#define USE_RX_UNIFIED
#define USE_BLACKBOX
#define USE_ADC

#ifndef SIMULATOR
#define USE_GYRO
#define USE_BARO
#define USE_SOFT_SERIAL
#define USE_SDCARD
#define USE_DATA_FLASH
#define USE_I2C

#define USE_VTX
#define USE_DIGITAL_VTX
#define USE_MAX7456
#define USE_RGB_LED

#define USE_MOTOR_DSHOT
#define USE_MOTOR_PWM

#ifndef AT32F4
#define USE_RX_SPI_FRSKY
#define USE_RX_SPI_FLYSKY
#define USE_RX_SPI_EXPRESS_LRS
#endif
#endif