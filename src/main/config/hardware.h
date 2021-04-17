#pragma once

#include "gpio_pins.h"
#include "motor_pins.h"
#include "spi_ports.h"
#include "usart_ports.h"

#include "target.h"

// the error codes indicate a failure that prevents normal operation
// led flash codes - the quad will not fly / bind if flashing a code
// 2 - low battery at powerup - if enabled by config.h "#define STOP_LOWBATTERY"
// 3 - radio chip not found
// 4 - Gyro not found - maybe i2c speed
// 5 - clock , intterrupts , systick , gcc bad code , bad memory access (code issues like bad pointers)- this should not come up
// 6 - loop time issue - if loop time exceeds 20mS
// 7 - i2c error  - triggered by hardware i2c driver only
// 8 - i2c error main loop  - triggered by hardware i2c driver only

#ifdef F0
#define ENABLE_OVERCLOCK
#ifdef ENABLE_OVERCLOCK
#define SYS_CLOCK_FREQ_HZ 64000000
#define PWM_CLOCK_FREQ_HZ 64000000
#define TICK_CLOCK_FREQ_HZ 8000000
#define LOOPTIME 1000
#else
#define SYS_CLOCK_FREQ_HZ 48000000
#define PWM_CLOCK_FREQ_HZ 48000000
#define TICK_CLOCK_FREQ_HZ 6000000
#define LOOPTIME 1000
#endif
#endif

#ifdef F4

#define ENABLE_SMART_AUDIO
//#define ENABLE_TRAMP

#ifdef F411
#define SYS_CLOCK_FREQ_HZ 84000000
#define PWM_CLOCK_FREQ_HZ 84000000
#define TICK_CLOCK_FREQ_HZ 10500000
#define LOOPTIME 250
#endif

#ifdef F405
#define SYS_CLOCK_FREQ_HZ 168000000
#define PWM_CLOCK_FREQ_HZ 84000000
#define TICK_CLOCK_FREQ_HZ 21000000
#ifdef SOFTSPI_NONE
#define LOOPTIME 125
#else
#define LOOPTIME 250
#endif
#endif

#if defined(F411) && defined(F405)
#error "multiple chip types (F411, F405) defined!"
#endif

#endif

#ifdef BRUSHLESS_TARGET
// pwm pin initialization
//#define USE_PWM_DRIVER
//#define USE_ESC_DRIVER       //todo:  evaluate need for this to stay if focused on quadcopters
#define USE_DSHOT_DMA_DRIVER
//#define USE_DSHOT_DRIVER_BETA
#endif

#ifdef BRUSHED_TARGET
// pwm pin initialization
#define USE_PWM_DRIVER
//#define USE_ESC_DRIVER       //todo:  evaluate need for this to stay if focused on quadcopters
//#define USE_DSHOT_DMA_DRIVER
//#define USE_DSHOT_DRIVER_BETA
#endif

//*************************************Features that still need to be moved into targets and checked for compatability************************************************

#define GYRO_LOW_PASS_FILTER 0

// RGB led type ws2812 - ws2813
// numbers over 8 could decrease performance
//#define RGB_LED_NUMBER 0
//#define RGB_LED_DMA

// pin / port for the RGB led ( programming port ok )
//#define RGB_PIN GPIO_Pin_11
//#define RGB_PORT GPIOA

// pin for fpv switch ( turns off at failsafe )
// GPIO_Pin_13 // SWDAT - GPIO_Pin_14 // SWCLK
// if programming pin, will not flash after bind

// BUZZER pin settings - buzzer active "high"
// SWDAT and SWCLK pins OK here
// GPIO_Pin_13 // SWDAT - GPIO_Pin_14 // SWCLK
//#define BUZZER_PIN       GPIO_Pin_14
//#define BUZZER_PIN_PORT  GPIOA
// x (micro)seconds after loss of tx or low bat before buzzer starts
//#define BUZZER_DELAY     30e6

//*************************************Legacy stuff moved into targets - remaining for future reference************************************************

// i2c driver to use ( dummy - disables i2c )
// hardware i2c used PB6 and 7 by default ( can also use PA9 and 10)
//#define USE_HARDWARE_I2C
//#define USE_SOFTWARE_I2C
//#define USE_DUMMY_I2C

// I2C speed: fast = no delays
// slow1 = for i2c without pull-up resistors
// slow2 = i2c failsafe speed
//#define SOFTI2C_SPEED_FAST
//#define SOFTI2C_SPEED_SLOW1
//#define SOFTI2C_SPEED_SLOW2

// hardware i2c speed ( 1000, 400 , 200 , 100Khz)
//#define HW_I2C_SPEED_FAST2
//#define HW_I2C_SPEED_FAST
//#define HW_I2C_SPEED_SLOW1
//#define HW_I2C_SPEED_SLOW2

// pins for hw i2c , select one only
// select pins PB6 and PB7 OR select pins PA9 and PA10
//#define HW_I2C_PINS_PB67
//#define HW_I2C_PINS_PA910

// disable the check for known gyro that causes the 4 times flash
//#define DISABLE_GYRO_CHECK

// disable lvc functions
//#define DISABLE_ADC

// pwm driver = brushed motors
// esc driver = servo type signal for brushless esc

//**DO NOT ENABLE ESC DRIVER WITH BRUSHED MOTORS ATTACHED**

//#define USE_PWM_DRIVER
//#define USE_ESC_DRIVER
//#define USE_DSHOT_DMA_DRIVER
//#define USE_DSHOT_DRIVER_BETA

//FC must have MOSFETS and motor pulldown resistors removed. MAY NOT WORK WITH ALL ESCS
#ifdef BRUSHLESS_TARGET
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#endif

// pwm pins disable
// disable all pwm pins / function
//#define DISABLE_PWM_PINS

#if defined(BRUSHLESS_TARGET) && defined(INVERTED_ENABLE)
// Enable this for 3D. The 'Motor Direction' setting in BLHeliSuite must
// be set to 'Bidirectional' (or 'Bidirectional Rev.') accordingly:
#define BIDIRECTIONAL
#endif

//***********************************************END LEGACY REFERENCE LIST*****************************************************
