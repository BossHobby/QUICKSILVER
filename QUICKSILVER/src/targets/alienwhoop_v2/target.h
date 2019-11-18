#include "config.h"

#define F4
#define F405
#define Alienwhoop_V2

//LEDS
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_12
#define LED1PORT GPIOC
#define LED2PIN GPIO_Pin_2
#define LED2PORT GPIOD
#define LED2_INVERT
#define BUZZER_PIN GPIO_Pin_2
#define BUZZER_PIN_PORT GPIOA
#define BUZZER_INVERT
#define FPV_PIN GPIO_Pin_13
#define FPV_PORT GPIOA

//GYRO
#define MPU6XXX_SPI1
#define MPU6XXX_NSS_PA4
#define MPU6XXX_INT_PC4
#define USE_DUMMY_I2C
#define GYRO_ID_1 0x70
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x71
#define GYRO_ID_4 0x72
#define SENSOR_ROTATE_90_CW

//RADIO
#define USART_PORTS \
  USART2_PA3PA2     \
  USART3_PC11PC10

#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT) || defined(RX_UNIFIED_SERIAL)
#define RX_USART USART_PORT2
#define SOFTSPI_NONE
#endif

#ifndef SOFTSPI_NONE
#define RADIO_CHECK
#define SPI_MISO_PIN GPIO_Pin_3
#define SPI_MISO_PORT GPIOA
#define SPI_MOSI_PIN GPIO_Pin_2
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_11
#define SPI_CLK_PORT GPIOC
#define SPI_SS_PIN GPIO_Pin_10
#define SPI_SS_PORT GPIOC
#endif

//VOLTAGE DIVIDER
#define DISABLE_ADC

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PC6
#define MOTOR_PIN1 MOTOR_PIN_PC7
#define MOTOR_PIN2 MOTOR_PIN_PC8
#define MOTOR_PIN3 MOTOR_PIN_PC9
