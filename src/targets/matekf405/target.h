#include "config.h"
#include "config_helper.h"

#define MatekF405

#define F4
#define F405

#define USB_DETECT_PIN GPIO_Pin_12
#define USB_DETECT_PORT GPIOB

//LEDS
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_9 // red led
#define LED1PORT GPIOB
#define LED2PIN GPIO_Pin_14 // green led
#define LED2PORT GPIOA
#define LED2_INVERT
#define BUZZER_PIN GPIO_Pin_13
#define BUZZER_PIN_PORT GPIOC
#define FPV_PIN GPIO_Pin_13
#define FPV_PORT GPIOA

//GYRO
#define ICM20602_SPI_PORT SPI_PORT1
#define ICM20602_NSS PIN_C2
#define ICM20602_INT PIN_C3
#define USE_DUMMY_I2C
#define SENSOR_ROTATE_90_CCW
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x12
#define GYRO_ID_3 0x69

// SPI PINS DEFINITONS & RADIO
#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2     \
  USART3_PC11PC10   \
  USART4_PA1PA0
// USART5_PD2PC12 we do not support "split" spi pins at the moment

#define SOFTSPI_NONE
#ifdef SERIAL_RX
#define RX_USART USART_PORT1
#endif

// OSD
#define ENABLE_OSD
#define MAX7456_SPI_PORT SPI_PORT2
#define MAX7456_NSS PIN_B10

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
