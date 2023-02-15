#include "config.h"

// PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PB3PB4PB5

#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2

// LEDS
#define LED_NUMBER 2
#define LED1PIN PIN_C13
#define LED1_INVERT
#define LED2PIN PIN_B9 // RX LED

#define BUZZER_PIN PIN_C15
//#define BUZZER_INVERT

// #define FPV_PIN LL_GPIO_PIN_13
// #define FPV_PORT GPIOA

// GYRO
#define GYRO_SPI_PORT SPI_PORT1
#define GYRO_NSS PIN_A4
#define GYRO_INT PIN_A1
#define GYRO_ORIENTATION GYRO_ROTATE_90_CCW

// RADIO
// PB3/PB4/PB5 (SPI3) is used for spi connection to A7105
// PA15 is used for SPI CSN(NSS)
// PA14 is used as external interrupt with rising edge trigger connected to A7105
// PB9 is used as output (push/pull, no pullup/pulldown) for RX LED
// PB2 is used as input (pullup) for RX bind pushbutton. Not used with Quicksilver
#define USE_A7105
#define A7105_SPI_PORT SPI_PORT3
#define A7105_NSS_PIN PIN_A15
#define A7105_GIO1_PIN PIN_A14 // EXTI15_10_IRQHandler

// OSD
#define USE_MAX7456
#define MAX7456_SPI_PORT SPI_PORT2
#define MAX7456_NSS PIN_B12

// VOLTAGE DIVIDER
#define VBAT_PIN PIN_B0
#define VBAT_DIVIDER_R1 10000
#define VBAT_DIVIDER_R2 1000

#define IBAT_PIN PIN_B1

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PB7
#define MOTOR_PIN1 MOTOR_PIN_PB8
#define MOTOR_PIN2 MOTOR_PIN_PB10
#define MOTOR_PIN3 MOTOR_PIN_PB6
