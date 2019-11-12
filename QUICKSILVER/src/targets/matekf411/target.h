#include "config.h"

#define MatekF411

#define F411
#define F4
#define F405

// #define USB_DETECT_PIN GPIO_Pin_5
// #define USB_DETECT_PORT GPIOC

//LEDS
#define LED_NUMBER 2
// red led
#define LED1PIN GPIO_Pin_13
#define LED1PORT GPIOC
#define LED1_INVERT
// green led
#define LED2PIN GPIO_Pin_9
#define LED2PORT GPIOB
#define LED2_INVERT

#define FPV_PIN GPIO_Pin_12
#define FPV_PORT GPIOB

//SPI, I2C & GYRO
#define MPU6XXX_SPI1
#define MPU6XXX_NSS_PA4
#define MPU6XXX_INT_PIN GPIO_Pin_1
#define MPU6XXX_INT_PORT GPIOA

#define USE_DUMMY_I2C //todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro

#define SENSOR_ROTATE_90_CCW
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x71

// SPI PINS DEFINITONS & RADIO
#define SOFTSPI_NONE

#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2

#define SMART_AUDIO_USART USART_PORT1

#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define RX_USART USART_PORT2
#endif

// OSD
#define ENABLE_OSD
#define MAX7456_SPI2
#define MAX7456_NSS_PB12

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_0
#define BATTERYPORT GPIOB
#define BATTERY_ADC_CHANNEL ADC_Channel_8

#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 10000
#endif

#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 1000
#endif

#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
//S3_OUT
#define MOTOR_PIN0 MOTOR_PIN_PB6
//S4_OUT
#define MOTOR_PIN1 MOTOR_PIN_PB7
//S1_OUT
#define MOTOR_PIN2 MOTOR_PIN_PB4
//S2_OUT
#define MOTOR_PIN3 MOTOR_PIN_PB5
