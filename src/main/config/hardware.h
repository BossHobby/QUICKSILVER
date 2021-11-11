#pragma once

#include "gpio_pins.h"
#include "motor_pins.h"
#include "spi_ports.h"
#include "usart_ports.h"

#include "target.h"

#ifdef STM32F4

#define ENABLE_SMART_AUDIO
#define ENABLE_TRAMP

#ifdef STM32F411
#define SYS_CLOCK_FREQ_HZ 84000000
#define PWM_CLOCK_FREQ_HZ 84000000
#define TICK_CLOCK_FREQ_HZ 10500000
#define LOOPTIME LOOPTIME_4K
#endif

#ifdef STM32F405
#define SYS_CLOCK_FREQ_HZ 168000000
#define PWM_CLOCK_FREQ_HZ 84000000
#define TICK_CLOCK_FREQ_HZ 21000000

#ifdef SOFTSPI_NONE
#define LOOPTIME LOOPTIME_8K
#else
#define LOOPTIME LOOPTIME_4K
#endif

#endif

#if defined(STM32F411) && defined(STM32F405)
#error "multiple chip types (STM32F411, STM32F405) defined!"
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

#if defined(BRUSHLESS_TARGET) && defined(INVERTED_ENABLE) //WARNING _ HAS NEVER BEEN TESTED
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
//#define RGB_PIN LL_GPIO_PIN_11
//#define RGB_PORT GPIOA
