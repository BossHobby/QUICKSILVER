#include "config.h"

#define NFE_Breadboard

#define F4
#define F405

//#define USB_DETECT_PIN GPIO_Pin_5
//#define USB_DETECT_PORT GPIOC

//LEDS, Buzzer, FPV
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_4
#define LED1PORT GPIOA
#define LED1_INVERT
#define BUZZER_PIN GPIO_Pin_13
#define BUZZER_PIN_PORT GPIOC
#define BUZZER_INVERT
#define FPV_PIN GPIO_Pin_13
#define FPV_PORT GPIOA

//SPI, I2C & GYRO
#define MPU6XXX_SPI2
#define MPU6XXX_NSS_PB12
#define MPU6XXX_INT_PA8
#define USE_DUMMY_I2C
//#define SENSOR_ROTATE_90_CCW
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x71
//#define DISABLE_GYRO_CHECK

// SPI PINS DEFINITONS & RADIO
#define USART_INVERTER_PIN GPIO_Pin_8
#define USART_INVERTER_PORT GPIOC

#define USART_PORTS \
  USART1_PB7PB6     \
  USART2_PA3PA2

#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT) || defined(RX_UNIFIED_SERIAL)
#define RX_USART USART_PORT2
#define SOFTSPI_NONE
#endif

#ifndef SOFTSPI_NONE //sort this out later for use of bayang
#define RADIO_CHECK
#define SPI_MISO_PIN GPIO_Pin_2
#define SPI_MISO_PORT GPIOA
#define SPI_MOSI_PIN GPIO_Pin_3
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_6
#define SPI_CLK_PORT GPIOB
#define SPI_SS_PIN GPIO_Pin_7
#define SPI_SS_PORT GPIOB
#endif

// OSD
#define ENABLE_OSD
#define MAX7456_SPI2
#define MAX7456_NSS_PA10

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_5
#define BATTERYPORT GPIOA
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
#define MOTOR_PIN0 MOTOR_PIN_PB8
//S4_OUT
#define MOTOR_PIN1 MOTOR_PIN_PB1
//S1_OUT
#define MOTOR_PIN2 MOTOR_PIN_PA1
//S2_OUT
#define MOTOR_PIN3 MOTOR_PIN_PA7
