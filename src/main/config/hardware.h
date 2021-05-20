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
// 4 - Gyro not found
// 5 - clock , intterrupts , systick , gcc bad code , bad memory access (code issues like bad pointers)- this should not come up
// 6 - loop time issue - if loop time exceeds 20mS
// 7 - spi error  - triggered by hardware spi driver only
// 8 - i2c error main loop  - triggered by depreciated hardware i2c driver only


#ifdef F4

#define ENABLE_SMART_AUDIO
#define ENABLE_TRAMP
#define GYRO_LOW_PASS_FILTER 0

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
// dshot pin initialization & usb interface to esc
#define USE_DSHOT_DMA_DRIVER
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#endif

#ifdef BRUSHED_TARGET
// pwm pin initialization
#define USE_PWM_DRIVER
#endif

#if defined(BRUSHLESS_TARGET) && defined(INVERTED_ENABLE)  //WARNING _ HAS NEVER BEEN TESTED
// Enable this for 3D. The 'Motor Direction' setting in BLHeliSuite must
// be set to 'Bidirectional' (or 'Bidirectional Rev.') accordingly:
#define BIDIRECTIONAL
#endif
//*************************************Features that still need to be moved into targets and checked for compatability************************************************



// RGB led type ws2812 - ws2813
// numbers over 8 could decrease performance
//#define RGB_LED_NUMBER 0
//#define RGB_LED_DMA

// pin / port for the RGB led ( programming port ok )
//#define RGB_PIN GPIO_Pin_11
//#define RGB_PORT GPIOA

//*************************************Legacy stuff moved into targets - remaining for future reference************************************************

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

// disable the check for known gyro that causes the 4 times flash
//#define DISABLE_GYRO_CHECK

// disable lvc functions
//#define DISABLE_ADC

// disable all pwm pins / function
//#define DISABLE_PWM_PINS



//***********************************************END LEGACY REFERENCE LIST*****************************************************
