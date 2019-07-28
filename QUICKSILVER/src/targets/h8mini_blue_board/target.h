#define F0
#define H8mini_blue_board

//LEDS & BUZZER
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_1
#define LED1PORT GPIOF
#define LED2PIN GPIO_Pin_3
#define LED2PORT GPIOA
#define BUZZER_PIN GPIO_Pin_14
#define BUZZER_PIN_PORT GPIOA
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