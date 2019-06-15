#include "config.h"

//TODO:  MOVE TARGET DEFINES TO THEIR OWN FILE AND REORDER THE .H HIERCHARCHY TO config.h->targets.h->defines.h   Correct all relavent #includes related to this change

#ifdef BWHOOP
#define F0

//LEDS & BUZZER
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_2
#define LED1PORT GPIOA
#define LED2PIN GPIO_Pin_3
#define LED2PORT GPIOA
#define LED1_INVERT
#define LED2_INVERT
#define BUZZER_PIN GPIO_Pin_14
#define BUZZER_PIN_PORT GPIOA

//I2C & GYRO
#define USE_HARDWARE_I2C
#define HW_I2C_SPEED_FAST2
#define HW_I2C_PINS_PA910
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

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART1_SDA
#define F0_USART_PINSWAP
#define SOFTSPI_NONE
#else
#define SOFTSPI_3WIRE
#define SPI_MOSI_PIN GPIO_Pin_0
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_1
#define SPI_CLK_PORT GPIOF
#define SPI_SS_PIN GPIO_Pin_0
#define SPI_SS_PORT GPIOF
#define RADIO_XN297L
#define RADIO_CHECK
#endif

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_5
#define BATTERYPORT GPIOA
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

// MOTOR PINS
#define MOTOR0_PIN_PB1 // motor 0 back-left
#define MOTOR1_PIN_PA4 // motor 1 front-left
#define MOTOR2_PIN_PA6 // motor 2 back-right
#define MOTOR3_PIN_PA7 // motor 3 front-right

// PWM PIN INITIALIZATION
#define PWM_PA4
#define PWM_PA6
#define PWM_PA7
#define PWM_PB1
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef E011
#define F0

//LEDS & BUZZER
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_2
#define LED1PORT GPIOA
#define LED2PIN GPIO_Pin_3
#define LED2PORT GPIOA
#define LED1_INVERT
#define LED2_INVERT
#define BUZZER_PIN GPIO_Pin_14
#define BUZZER_PIN_PORT GPIOA

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
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS)
#define USART1_SDA
#define F0_USART_PINSWAP
#define SOFTSPI_NONE
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
#define BATTERYPIN GPIO_Pin_5
#define BATTERYPORT GPIOA
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
#define MOTOR0_PIN_PA6 // motor 0 back-left
#define MOTOR1_PIN_PA4 // motor 1 front-left
#define MOTOR2_PIN_PB1 // motor 2 back-right
#define MOTOR3_PIN_PA7 // motor 3 front-right

// pwm pin initialization
#define PWM_PA4
#define PWM_PA6
#define PWM_PA7
#define PWM_PB1
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef H8mini_blue_board
#define F0

//LEDS & BUZZER
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_1
#define LED1PORT GPIOF
#define LED2PIN GPIO_Pin_3
#define LED2PORT GPIOA
#define BUZZER_PIN GPIO_Pin_14
#define BUZZER_PIN_PORT GPIOA

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
#define GYRO_ID_2 0x78 // common h8 gyro
#define GYRO_ID_3 0x7D
#define GYRO_ID_4 0x72
#define SENSOR_ROTATE_180

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART1_SDA
#define F0_USART_PINSWAP
#define SOFTSPI_NONE
#else
#define SOFTSPI_3WIRE
#define SPI_MOSI_PIN GPIO_Pin_1
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_2
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_3
#define SPI_SS_PORT GPIOA
#define RADIO_XN297L
#define RADIO_CHECK
#endif

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_5
#define BATTERYPORT GPIOA
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
#define MOTOR0_PIN_PA6 // motor 0 back-left
#define MOTOR1_PIN_PA4 // motor 1 front-left
#define MOTOR2_PIN_PB1 // motor 2 back-right
#define MOTOR3_PIN_PA7 // motor 3 front-right

// pwm pin initialization
#define PWM_PA4
#define PWM_PA6
#define PWM_PA7
#define PWM_PB1
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef Alienwhoop_ZERO
#define F0

//LEDS & BUZZER
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_0
#define LED1PORT GPIOF
#define LED2PIN GPIO_Pin_0
#define LED2PORT GPIOA
#define BUZZER_PIN GPIO_Pin_14
#define BUZZER_PIN_PORT GPIOA

//I2C & GYRO
#define USE_HARDWARE_I2C
#define HW_I2C_SPEED_FAST2
#define HW_I2C_PINS_PA910
#define I2C_SDAPIN GPIO_Pin_10
#define I2C_SDAPORT GPIOA
#define I2C_SCLPIN GPIO_Pin_9
#define I2C_SCLPORT GPIOA
#define I2C_GYRO_ADDRESS 0x68
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x98 // new id
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x7D
#define SENSOR_ROTATE_90_CCW

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#if defined(RX_FPORT)
	#define F0_USART_PINSWAP
#endif
#define USART1_PA3PA2
#define SOFTSPI_NONE
#else
#define SOFTSPI_3WIRE
#define SPI_MOSI_PIN GPIO_Pin_3
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_2
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_1
#define SPI_SS_PORT GPIOA
#define RADIO_CHECK
#define RADIO_XN297L
#endif

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_5
#define BATTERYPORT GPIOA
#define BATTERY_ADC_CHANNEL ADC_Channel_5
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 2000
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 1000
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
#define MOTOR0_PIN_PA7
#define MOTOR1_PIN_PB1
#define MOTOR2_PIN_PA4
#define MOTOR3_PIN_PA6

// pwm pin initialization
#define PWM_PA4
#define PWM_PA6
#define PWM_PA7
#define PWM_PB1
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(CC3D_REVO_F4)
#define F405

//LEDS
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_5
#define LED1PORT GPIOB
#define LED1_INVERT
#define LED2PIN GPIO_Pin_4
#define LED2PORT GPIOB
#define LED1_INVERT
//#define LED2_INVERT
#define BUZZER_PIN				GPIO_Pin_6
#define BUZZER_PIN_PORT		GPIOB
#define BUZZER_INVERT

//SPI, I2C & GYRO
#define MPU6XXX_SPI1
#define MPU6XXX_NSS_PA4
#define MPU6XXX_INT_PC4
#define USE_DUMMY_I2C									//todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
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
//#define SENSOR_ROTATE_90_CW
//#define SENSOR_FLIP_180


// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART1_PA10PA9
#define USART_INVERTER_PIN GPIO_Pin_0
#define USART_INVERTER_PORT GPIOC
#define USART3_PB11PB10
#define USART4_PA1PA0
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

//VOLTAGE DIVIDER
#define DISABLE_ADC
#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_12
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 4700
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 2200
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
#define MOTOR0_PIN_PA3
#define MOTOR1_PIN_PA2
#define MOTOR2_PIN_PB0
#define MOTOR3_PIN_PB1

// PWM PIN INITIALIZATION
//#define PWM_PA0
//#define PWM_PA1
#define PWM_PA2
#define PWM_PA3
//#define PWM_PA4
//#define PWM_PA5
//#define PWM_PA6
//#define PWM_PA7
//#define PWM_PA8
//#define PWM_PA9
//#define PWM_PA10
//#define PWM_PA11
#define PWM_PB0
#define PWM_PB1
//#define PWM_PC9
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(OmnibusF4SD) || defined(OmnibusF4) || defined(LuxF4osd)
#define F405

//LEDS
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_5
#define LED1PORT GPIOB
#define LED1_INVERT
#define BUZZER_PIN GPIO_Pin_4
#define BUZZER_PIN_PORT GPIOB
#define BUZZER_INVERT

//SPI, I2C & GYRO
#define DISABLE_GYRO_CHECK
#define MPU6XXX_SPI1
#define MPU6XXX_NSS_PA4
#define MPU6XXX_INT_PC4
#define USE_DUMMY_I2C									//todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
//#define I2C_SDAPIN GPIO_Pin_10
//#define I2C_SDAPORT GPIOA
//#define I2C_SCLPIN GPIO_Pin_9
//#define I2C_SCLPORT GPIOA
//#define I2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x71
#define SENSOR_ROTATE_90_CCW
#if defined(LuxF4osd)
#define SENSOR_ROTATE_90_CCW
#else
#if  defined(OmnibusF4SD)
#define SENSOR_ROTATE_90_CW
#endif
#define SENSOR_FLIP_180
#endif

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
//   //not created yet
//#define UART3_INVERTER_PIN PC9 // Omnibus F4 Pro Corner
#ifdef OmnibusF4SD
#define USART_INVERTER_PIN GPIO_Pin_8
#define USART_INVERTER_PORT GPIOC
#define USART1_PA10PA9
#define USART3_PB11PB10
#define USART6_PC7PC6
#endif
#if defined(OmnibusF4) || defined(LuxF4osd)
#define USART1_PA10PA9
#define USART_INVERTER_PIN GPIO_Pin_0
#define USART_INVERTER_PORT GPIOC
#define USART3_PB11PB10
#define USART4_PA1PA0
#endif
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

//VOLTAGE DIVIDER
#define DISABLE_ADC
#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_12
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 4700
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 2200
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
//OmniF4SD
#ifdef OmnibusF4SD
#define MOTOR2_PIN_PA3
#define MOTOR3_PIN_PA2
#define MOTOR0_PIN_PB0
#define MOTOR1_PIN_PB1

#else //OmnibusF4
#define MOTOR0_PIN_PA3
#define MOTOR1_PIN_PA2
#define MOTOR2_PIN_PB0
#define MOTOR3_PIN_PB1

//pyroflip
//#define MOTOR0_PIN_PA8
//#define MOTOR1_PIN_PC9
//#define MOTOR2_PIN_PB1
//#define MOTOR3_PIN_PB0
#endif


#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef Alienwhoop_V2
#define F405

//LEDS
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_12
#define LED1PORT GPIOC
#define LED2PIN GPIO_Pin_2
#define LED2PORT GPIOD
//#define LED1_INVERT
#define LED2_INVERT
#define BUZZER_PIN GPIO_Pin_2
#define BUZZER_PIN_PORT GPIOA
#define BUZZER_INVERT

//SPI, I2C & GYRO
#define MPU6XXX_SPI1
#define MPU6XXX_NSS_PA4
#define MPU6XXX_INT_PC4
#define USE_DUMMY_I2C									//todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
//#define I2C_SDAPIN GPIO_Pin_10
//#define I2C_SDAPORT GPIOA
//#define I2C_SCLPIN GPIO_Pin_9
//#define I2C_SCLPORT GPIOA
//#define I2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69
#define GYRO_ID_1 0x70
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x71
//#define GYRO_ID_4 0x72 
#define SENSOR_ROTATE_90_CW

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART2_PA3PA2
#define USART3_PC11PC10
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

//VOLTAGE DIVIDER
#define DISABLE_ADC
/*#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_12
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 4700
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 2200
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif */

// MOTOR PINS
#define MOTOR0_PIN_PC6
#define MOTOR1_PIN_PC7
#define MOTOR2_PIN_PC8
#define MOTOR3_PIN_PC9

// PWM PIN INITIALIZATION
//#define PWM_PA0
//#define PWM_PA1
//#define PWM_PA2
//#define PWM_PA3
//#define PWM_PA4
//#define PWM_PA5
//#define PWM_PA6
//#define PWM_PA7
//#define PWM_PA8
//#define PWM_PA9
//#define PWM_PA10
//#define PWM_PA11
//#define PWM_PB0
//#define PWM_PB1
#define PWM_PC6
#define PWM_PC7
#define PWM_PC8
#define PWM_PC9
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef CLRacing_F4
#define F405

//LEDS
#define LED_NUMBER 1
#define LED1PIN 					GPIO_Pin_5
#define LED1PORT 					GPIOB
#define LED1_INVERT
#define BUZZER_PIN				GPIO_Pin_4
#define BUZZER_PIN_PORT		GPIOB
#define BUZZER_INVERT

//SPI, I2C & GYRO
#define MPU6XXX_SPI1
#define MPU6XXX_NSS_PA4
#define MPU6XXX_INT_PC4
#define USE_DUMMY_I2C									//todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
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
//#define DISABLE_GYRO_CHECK

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART1_PA10PA9
#define USART_INVERTER_PIN GPIO_Pin_0
#define USART_INVERTER_PORT GPIOC
#define USART3_PB11PB10
#define USART4_PA1PA0
#define USART6_PC7PC6
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

//VOLTAGE DIVIDER
#define DISABLE_ADC
#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_12
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 4700
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 2200
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
#define MOTOR0_PIN_PA3
#define MOTOR1_PIN_PA2
#define MOTOR2_PIN_PB0
#define MOTOR3_PIN_PB1
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef Raceflight_Revolt
#define F405

//LEDS & BUZZER
#define LED_NUMBER 1
#define LED1_INVERT
#define LED1PIN						GPIO_Pin_5
#define LED1PORT 					GPIOB
#define BUZZER_PIN				GPIO_Pin_4
#define BUZZER_PIN_PORT		GPIOB
#define BUZZER_INVERT

//SPI, I2C & GYRO
#define ICM20602_SPI1
#define ICM20602_NSS_PA4
#define ICM20602_INT_PC4
#define USE_DUMMY_I2C									//todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
//#define I2C_SDAPIN GPIO_Pin_10
//#define I2C_SDAPORT GPIOA
//#define I2C_SCLPIN GPIO_Pin_9
//#define I2C_SCLPORT GPIOA
//#define I2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x12
#define GYRO_ID_3 0x69
//#define GYRO_ID_4 0x72
//#define SENSOR_ROTATE_90_CCW
#define SENSOR_FLIP_180
#define SENSOR_ROTATE_90_CW
//#define DISABLE_GYRO_CHECK

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART1_PA10PA9
#define USART_INVERTER_PIN GPIO_Pin_0
#define USART_INVERTER_PORT GPIOC
#define USART3_PB11PB10
#define USART4_PA1PA0
#define USART6_PC7PC6
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
#define MOTOR0_PIN_PA3
#define MOTOR1_PIN_PB1
#define MOTOR2_PIN_PA2
#define MOTOR3_PIN_PB0
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef FortiniF4osd
#define F405
#define SYS_CLOCK_FREQ_HZ 168000000
#define PWM_CLOCK_FREQ_HZ 84000000
#define TICK_CLOCK_FREQ_HZ 21000000

//LEDS
#define LED_NUMBER 2
#define LED1PIN 					GPIO_Pin_5
#define LED1PORT 					GPIOB
#define LED2PIN 					GPIO_Pin_6
#define LED2PORT 					GPIOB
#define LED1_INVERT
#define LED2_INVERT
#define BUZZER_PIN				GPIO_Pin_4
#define BUZZER_PIN_PORT		GPIOB
#define BUZZER_INVERT

//SPI, I2C & GYRO
#define ICM20602_SPI1
#define ICM20602_NSS_PA8
#define ICM20602_INT_PC4
#define USE_DUMMY_I2C									//todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
//#define I2C_SDAPIN GPIO_Pin_10
//#define I2C_SDAPORT GPIOA
//#define I2C_SCLPIN GPIO_Pin_9
//#define I2C_SCLPORT GPIOA
//#define I2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69
#define GYRO_ID_1 0x12
#define GYRO_ID_2 0xaf
#define GYRO_ID_3 0xac
#define GYRO_ID_4 0x98
#define SENSOR_ROTATE_90_CCW
//#define SENSOR_FLIP_180
//#define SENSOR_ROTATE_90_CW
//#define DISABLE_GYRO_CHECK

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS)
#define USART1_PA10PA9
#define USART_INVERTER_PIN GPIO_Pin_15
#define USART_INVERTER_PORT GPIOC
#define USART3_PB11PB10
#define USART4_PA1PA0
#define USART6_PC7PC6
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

//VOLTAGE DIVIDER
#define DISABLE_ADC
#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_12
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 4700
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 2200
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
#define MOTOR0_PIN_PA3
#define MOTOR1_PIN_PA2
#define MOTOR2_PIN_PB0
#define MOTOR3_PIN_PB1

#endif

