#include "config.h"

#define F4
#define F405
#define Pyrodrone_F4

//LEDS
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_4
#define LED1PORT GPIOB
#define LED1_INVERT
#define BUZZER_PIN GPIO_Pin_5
#define BUZZER_PIN_PORT GPIOB
#define BUZZER_INVERT
#define FPV_PIN GPIO_Pin_15
#define FPV_PORT GPIOA

//SPI, I2C & GYRO
#define MPU6XXX_SPI1
#define MPU6XXX_NSS_PA4
#define MPU6XXX_INT_PC4
#define USE_DUMMY_I2C //todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
//#define I2C_SDAPIN GPIO_Pin_10
//#define I2C_SDAPORT GPIOA
//#define I2C_SCLPIN GPIO_Pin_9
//#define I2C_SCLPORT GPIOA
//#define I2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69
#define GYRO_ID_1 0x68
//#define GYRO_ID_2 0x73
//#define GYRO_ID_3 0x78
//#define GYRO_ID_4 0x72
//#define SENSOR_ROTATE_90_CCW
#define SENSOR_ROTATE_90_CW

// SPI PINS DEFINITONS & RADIO
#define USART_INVERTER_PIN GPIO_Pin_3
#define USART_INVERTER_PORT GPIOC

#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2     \
  USART3_PB11PB10   \
  USART4_PA1PA0     \
  USART6_PC7PC6

#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define RX_USART USART_PORT1
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

//OSD
#define ENABLE_OSD
#define MAX7456_SPI2
#define MAX7456_NSS_PB12

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
#define MOTOR_PIN0 MOTOR_PIN_PC9
#define MOTOR_PIN1 MOTOR_PIN_PA8
#define MOTOR_PIN2 MOTOR_PIN_PB1
#define MOTOR_PIN3 MOTOR_PIN_PB0
