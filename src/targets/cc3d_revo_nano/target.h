#include "config.h"
#include "config_helper.h"

#define CC3D_Revo_Nano

#define F4
#define F411

//PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PB3PB4PB5

#define USART_PORTS \
  USART1_PB7PB6     \
  USART2_PA3PA2

//***********MOTORS NOT YET SUPPORTED*************

//LEDS, Buzzer, FPV
#define LED_NUMBER 2
#define LED1PIN PIN_C14
#define LED2PIN PIN_C13
#define LED1_INVERT
#define BUZZER_PIN PIN_C13
#define BUZZER_INVERT
#define FPV_PIN PIN_A13

//GYRO
#define MPU6XXX_SPI_PORT SPI_PORT2
#define MPU6XXX_NSS PIN_B12
#define MPU6XXX_INT PIN_A15
#define SENSOR_ROTATE_180
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x71
//#define DISABLE_GYRO_CHECK

//RADIO
#define USART2_INVERTER_PIN PIN_C15

#ifdef SERIAL_RX
#define RX_USART USART_PORT2
#define SOFTSPI_NONE
#endif

#ifndef SOFTSPI_NONE
#define RADIO_CHECK
#define SPI_MISO_PIN GPIO_Pin_7
#define SPI_MISO_PORT GPIOB
#define SPI_MOSI_PIN GPIO_Pin_6
#define SPI_MOSI_PORT GPIOB
#define SPI_CLK_PIN GPIO_Pin_3
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_2
#define SPI_SS_PORT GPIOA
#endif

//VOLTAGE DIVIDER
#define BATTERYPIN PIN_A6
#define BATTERY_ADC_CHANNEL ADC_Channel_6

#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 10000
#endif

#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 1000
#endif

#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS		dummy pins					***********MOTORS NOT YET SUPPORTED*************
#define MOTOR_PIN0 MOTOR_PIN_PB8
#define MOTOR_PIN1 MOTOR_PIN_PB9
#define MOTOR_PIN2 MOTOR_PIN_PB3
#define MOTOR_PIN3 MOTOR_PIN_PB10
