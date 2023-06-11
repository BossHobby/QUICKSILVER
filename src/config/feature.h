#pragma once

#define USE_GYRO
#define USE_SDCARD
#define USE_DATA_FLASH
#define USE_BLACKBOX

#define USE_MAX7456

#define USE_MOTOR_DSHOT
#define USE_MOTOR_PWM

#ifndef AT32F4
#define USE_RX_SPI_FRSKY
#define USE_RX_SPI_FLYSKY
#define USE_RX_SPI_EXPRESS_LRS
#endif