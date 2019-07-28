#define F0
#define Alienwhoop_ZERO

//LEDS & BUZZER
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_0
#define LED1PORT GPIOF
#define LED2PIN GPIO_Pin_0
#define LED2PORT GPIOA
#define BUZZER_PIN GPIO_Pin_14
#define BUZZER_PIN_PORT GPIOA
#define FPV_PIN GPIO_Pin_13
#define FPV_PORT GPIOA

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