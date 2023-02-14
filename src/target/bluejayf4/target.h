#include "config.h"

// PORTS
#define SPI_PORTS \
  SPI1_PA5PA6PA7  \
  SPI3_PC10PC11PC12

#define USART_PORTS \
  USART1_PA10PA9    \
  USART3_PB11PB10   \
  USART6_PC7PC6

// LEDS
// target has third led on b06 not supported by QS
#define LED_NUMBER 2
#define LED1PIN PIN_B5
#define LED1_INVERT
#define LED2PIN PIN_B4
#define BUZZER_PIN PIN_C1
#define BUZZER_INVERT
#define FPV_PIN PIN_A13

// GYRO
#define GYRO_SPI_PORT SPI_PORT1
#define GYRO_NSS PIN_C4
#define GYRO_INT PIN_C5
#define GYRO_ORIENTATION GYRO_ROTATE_90_CW

// RADIO
#define USART6_INVERTER_PIN PIN_B15

// OSD
// not supported by fc

// VOLTAGE DIVIDER
#define VBAT_PIN PIN_C3
#define VBAT_DIVIDER_R1 10000
#define VBAT_DIVIDER_R2 1000

#define IBAT_PIN PIN_C2

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PA3
#define MOTOR_PIN1 MOTOR_PIN_PA2
#define MOTOR_PIN2 MOTOR_PIN_PA1
#define MOTOR_PIN3 MOTOR_PIN_PA0

// BLACKBOX
// resource SDCARD_CS 1 A15  SPI3
// resource SDCARD_DETECT 1 D02
// resource FLASH_CS 1 B07 SPI3 - alternative dataflash chip to sdcard
