#include "config.h"
#include "config_helper.h"

#define BluejayF4

#define F4
#define F405

//PORTS
#define SPI_PORTS \
  SPI1_PA5PA6PA7  \
  SPI3_PC10PC11PC12

#define USART_PORTS \
  USART1_PA10PA9    \
  USART3_PB11PB10   \
  USART6_PC7PC6

//LEDS
//target has third led on b06 not supported by QS
#define LED_NUMBER 2
#define LED1PIN PIN_B5
#define LED1_INVERT
#define LED2PIN PIN_B4
#define BUZZER_PIN PIN_C1
#define BUZZER_INVERT
#define FPV_PIN PIN_A13

//GYRO
#define ICM20608_SPI_PORT SPI_PORT1
#define ICM20608_NSS PIN_C4
#define ICM20608_INT PIN_C5
#define GYRO_ID_1 0xAF
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x71
#define SENSOR_ROTATE_90_CW

//RADIO
#define USART6_INVERTER_PIN PIN_B15

#ifdef SERIAL_RX
#define SOFTSPI_NONE
#define RX_USART USART_PORT1
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
//not supported by fc

//VOLTAGE DIVIDER
#define BATTERYPIN PIN_C3
#define BATTERY_ADC_CHANNEL ADC_Channel_13
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
#define MOTOR_PIN0 MOTOR_PIN_PA3
#define MOTOR_PIN1 MOTOR_PIN_PA2
#define MOTOR_PIN2 MOTOR_PIN_PA1
#define MOTOR_PIN3 MOTOR_PIN_PA0

// BLACKBOX
//resource SDCARD_CS 1 A15  SPI3
//resource SDCARD_DETECT 1 D02
//resource FLASH_CS 1 B07 SPI3 - alternative dataflash chip to sdcard
