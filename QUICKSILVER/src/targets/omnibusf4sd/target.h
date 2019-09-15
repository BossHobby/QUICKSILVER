#include "config.h"

#define F405
#define OmnibusF4SD

#define USB_DETECT_PIN GPIO_Pin_5
#define USB_DETECT_PORT GPIOC

//LEDS
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_5
#define LED1PORT GPIOB
#define LED1_INVERT
#define BUZZER_PIN GPIO_Pin_4
#define BUZZER_PIN_PORT GPIOB
#define BUZZER_INVERT
#define FPV_PIN GPIO_Pin_13
#define FPV_PORT GPIOA

//SPI, I2C & GYRO
#define MPU6XXX_SPI1
#define MPU6XXX_NSS_PA4
#define MPU6XXX_INT_PC4
#define USE_DUMMY_I2C //todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x71
#if defined(LuxF4osd)
#define SENSOR_ROTATE_90_CCW
#else
#if defined(OmnibusF4SD)
#define SENSOR_ROTATE_90_CW
#endif
//#define SENSOR_FLIP_180  //Bobnova orientation
#endif

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT) || defined(RX_UNIFIED_SERIAL)
//   //not created yet
//#define UART3_INVERTER_PIN PC9 // Omnibus F4 Pro Corner
#ifdef OmnibusF4SD
#define USART_INVERTER_PIN GPIO_Pin_8
#define USART_INVERTER_PORT GPIOC
#define USART1_PA10PA9
#define USART3_PB11PB10
#define USART6_PC7PC6
#endif
#if defined(OmnibusF4) || defined(LuxF4osd)
#define USART1_PA10PA9
#define USART_INVERTER_PIN GPIO_Pin_0
#define USART_INVERTER_PORT GPIOC
#define USART3_PB11PB10
#define USART4_PA1PA0
#endif
#define SOFTSPI_NONE
#endif
#ifndef SOFTSPI_NONE
#define RADIO_CHECK
#define SPI_MISO_PIN GPIO_Pin_1
#define SPI_MISO_PORT GPIOA
#define SPI_MOSI_PIN GPIO_Pin_4
#define SPI_MOSI_PORT GPIOB
#define SPI_CLK_PIN GPIO_Pin_10
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_6
#define SPI_SS_PORT GPIOB
#endif

//OSD
#define ENABLE_OSD
#define MAX7456_SPI3
#define MAX7456_NSS_PA15

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
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
//OmniF4SD
#define MOTOR0_PIN_PB1
#define MOTOR1_PIN_PA2
#define MOTOR2_PIN_PB0
#define MOTOR3_PIN_PA3

// NFE 2in squirt motors
//#define MOTOR2_PIN_PB1
//#define MOTOR3_PIN_PA2
//#define MOTOR0_PIN_PB0
//#define MOTOR1_PIN_PA3

// NFE 3in 4100kv motors
//#define MOTOR0_PIN_PB1
//#define MOTOR1_PIN_PB0
//#define MOTOR2_PIN_PA3
//#define MOTOR3_PIN_PA2

//BOBNOVA MOTORS  BOBNOVA MOTORS
//#define MOTOR0_PIN_PB0
//#define MOTOR1_PIN_PB1
//#define MOTOR2_PIN_PA3
//#define MOTOR3_PIN_PA2

