#include "config.h"
#include "config_helper.h"

#define F0
#define E011

//LEDS & BUZZER
#define LED_NUMBER 2
#define LED1PIN PIN_A2
#define LED2PIN PIN_A3
#define LED1_INVERT
#define LED2_INVERT
#define BUZZER_PIN PIN_A14
#define FPV_PIN GPIO_Pin_13
#define FPV_PORT GPIOA

//I2C & GYRO
#define USE_SOFTWARE_I2C
#define SOFTI2C_SPEED_FAST
#define SOFTI2C_PUSHPULL_CLK
#define I2C_SDAPIN GPIO_Pin_10
#define I2C_SDAPORT GPIOA
#define I2C_SCLPIN GPIO_Pin_9
#define I2C_SCLPORT GPIOA
#define I2C_GYRO_ADDRESS 0x68
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x98 // new id
#define GYRO_ID_3 0x7D
#define GYRO_ID_4 0x72
#define SENSOR_ROTATE_90_CW

// SPI RX PINS DEFINITONS & RADIO
#define USART_PORTS \
  USART1_SDA

#ifdef SERIAL_RX
#define F0_USART_PINSWAP
#define SOFTSPI_NONE
#define RX_USART USART_PORT1
#else
#define SOFTSPI_3WIRE
#define SPI_MOSI_PIN GPIO_Pin_0
#define SPI_MOSI_PORT GPIOF
#define SPI_CLK_PIN GPIO_Pin_1
#define SPI_CLK_PORT GPIOF
#define SPI_SS_PIN GPIO_Pin_0
#define SPI_SS_PORT GPIOA
#define RADIO_XN297L
#define RADIO_CHECK
#endif

//VOLTAGE DIVIDER
#define BATTERYPIN PIN_A5
#define BATTERY_ADC_CHANNEL ADC_Channel_5
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 10000
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 10000
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 2.8
#endif

// Assingment of pin to motor
// motor 0 back-left
#define MOTOR_PIN0 MOTOR_PIN_PA6
// motor 1 front-left
#define MOTOR_PIN1 MOTOR_PIN_PA4
// motor 2 back-right
#define MOTOR_PIN2 MOTOR_PIN_PB1
// motor 3 front-right
#define MOTOR_PIN3 MOTOR_PIN_PA7