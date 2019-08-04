#include "config.h"

#define Betafpv_F4

#define F411
#define F405

// #define USB_DETECT_PIN GPIO_Pin_5
// #define USB_DETECT_PORT GPIOC

//LEDS
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_13
#define LED1PORT GPIOC
#define LED1_INVERT
#define LED2PIN GPIO_Pin_9
#define LED2PORT GPIOB
#define LED2_INVERT

// #define BUZZER_PIN GPIO_Pin_6
// #define BUZZER_PIN_PORT GPIOB
// #define BUZZER_INVERT

#define FPV_PIN GPIO_Pin_12
#define FPV_PORT GPIOB

//SPI, I2C & GYRO
#define MPU6XXX_SPI1
#define MPU6XXX_NSS_PA4
#define MPU6XXX_INT_PIN GPIO_Pin_1
#define MPU6XXX_INT_PORT GPIOA
#define USE_DUMMY_I2C //todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
//#define I2C_SDAPIN GPIO_Pin_10
//#define I2C_SDAPORT GPIOA
//#define I2C_SCLPIN GPIO_Pin_9
//#define I2C_SCLPORT GPIOA
//#define I2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x71
//#define SENSOR_ROTATE_90_CCW
//#define SENSOR_ROTATE_90_CW
//#define SENSOR_FLIP_180

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART1_PA10PA9
#define USART2_PA3PA2
#define SOFTSPI_NONE
#endif
#ifndef SOFTSPI_NONE
#define RADIO_CHECK
#define SPI_MISO_PIN GPIO_Pin_10
#define SPI_MISO_PORT GPIOA
#define SPI_MOSI_PIN GPIO_Pin_9
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_6
#define SPI_CLK_PORT GPIOC
#define SPI_SS_PIN GPIO_Pin_7
#define SPI_SS_PORT GPIOC
#endif

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_0
#define BATTERYPORT GPIOB
#define BATTERY_ADC_CHANNEL ADC_Channel_12
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
#define MOTOR0_PIN_PB10
#define MOTOR1_PIN_PB6
#define MOTOR2_PIN_PB7
#define MOTOR3_PIN_PB8

// PWM PIN INITIALIZATION
#define PWM_PB10
#define PWM_PB6
#define PWM_PB7
#define PWM_PB8
