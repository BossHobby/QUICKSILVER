#include "config.h"

#define MatekF405

#define F4
#define F405

#define USB_DETECT_PIN GPIO_Pin_12
#define USB_DETECT_PORT GPIOB

//LEDS
#define LED_NUMBER 2
// red led
#define LED1PIN GPIO_Pin_9
#define LED1PORT GPIOB
#define LED1_INVERT
// green led
#define LED2PIN GPIO_Pin_14
#define LED2PORT GPIOA
#define LED2_INVERT

#define BUZZER_PIN GPIO_Pin_13
#define BUZZER_PIN_PORT GPIOC

#define FPV_PIN GPIO_Pin_12
#define FPV_PORT GPIOB

//SPI, I2C & GYRO
#define ICM20602_SPI1
#define ICM20602_NSS_PC2
#define ICM20602_INT_PC3

#define USE_DUMMY_I2C //todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro

#define SENSOR_ROTATE_90_CCW
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x12
#define GYRO_ID_3 0x69

// SPI PINS DEFINITONS & RADIO
#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2     \
  USART3_PC11PC10   \
  USART4_PA1PA0     \
  USART5_PD2PC12

#define SOFTSPI_NONE
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define RX_USART USART_PORT1
#endif

// OSD
#define ENABLE_OSD
#define MAX7456_SPI2
#define MAX7456_NSS_PB10

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_5
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_15

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
#define MOTOR_PIN0 MOTOR_PIN_PC8
//S4_OUT
#define MOTOR_PIN1 MOTOR_PIN_PC9
//S1_OUT
#define MOTOR_PIN2 MOTOR_PIN_PC6
//S2_OUT
#define MOTOR_PIN3 MOTOR_PIN_PC7
