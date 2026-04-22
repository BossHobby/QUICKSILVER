#pragma once

#if !defined(VEHICLE_MULTI) && !defined(VEHICLE_ROVER) && !defined(VEHICLE_WING)
#define VEHICLE_MULTI
#endif

#ifdef VEHICLE_ROVER
#define USE_SERVO
#endif

#define USE_SPI
#define USE_SERIAL
#define USE_RX_UNIFIED
#define USE_BLACKBOX
#define USE_ADC
#define USE_GPS

#ifndef SIMULATOR
#define USE_GYRO
#define USE_SOFT_SERIAL
#define USE_SDCARD
#define USE_DATA_FLASH

#define USE_VTX
#define USE_DIGITAL_VTX
#define USE_MAX7456
#define USE_RGB_LED

#ifndef VEHICLE_ROVER
#define USE_MOTOR_DSHOT
#endif
#define USE_MOTOR_PWM
#define USE_SERIAL_4WAY

#ifndef AT32F4
#define USE_RX_SPI_FRSKY
#define USE_RX_SPI_FLYSKY
#define USE_RX_SPI_EXPRESS_LRS
#endif
#endif
