#pragma once

#include "gpio_pins.h"
#include "motor_pins.h"
#include "spi_ports.h"
#include "usart_ports.h"

#include "target.h"

#ifdef STM32F4

#ifdef STM32F411
#define SYS_CLOCK_FREQ_HZ 108000000
#define PWM_CLOCK_FREQ_HZ 108000000
#define SPI_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 2)

#define LOOPTIME LOOPTIME_4K
#endif

#ifdef STM32F405
#define SYS_CLOCK_FREQ_HZ 168000000
#define PWM_CLOCK_FREQ_HZ 84000000
#define SPI_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 4)

#if !defined(USE_SOFT_SPI_4WIRE) && !defined(USE_SOFT_SPI_3WIRE)
#define LOOPTIME LOOPTIME_8K
#else
#define LOOPTIME LOOPTIME_4K
#endif

#endif

#if defined(STM32F411) && defined(STM32F405)
#error "multiple chip types (STM32F411, STM32F405) defined!"
#endif

#endif

#ifdef STM32F7

#define SYS_CLOCK_FREQ_HZ 216000000
#define PWM_CLOCK_FREQ_HZ 216000000
#define SPI_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 4)

#define LOOPTIME LOOPTIME_8K

#define WITHIN_DTCM_RAM(p) (((uint32_t)p & 0xffff0000) == 0x20000000)

#endif

#ifdef USE_FAST_RAM
#define FAST_RAM __attribute__((section(".fast_ram"), aligned(4)))
#else
#define FAST_RAM
#endif

#define ENABLE_SMART_AUDIO
#define ENABLE_TRAMP

#ifdef BRUSHLESS_TARGET
// dshot pin initialization & usb interface to esc
#define USE_DSHOT_DMA_DRIVER
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#endif

#ifdef BRUSHED_TARGET
// pwm pin initialization
#define USE_PWM_DRIVER
#endif

#if defined(USE_M25P16) || defined(USE_SDCARD)
#define ENABLE_BLACKBOX
#endif

//*************************************Features that still need to be moved into targets and checked for compatability************************************************
// RGB led type ws2812 - ws2813
// numbers over 8 could decrease performance
//#define RGB_LED_NUMBER 0
//#define RGB_LED_DMA

// pin / port for the RGB led ( programming port ok )
//#define RGB_PIN LL_GPIO_PIN_11
//#define RGB_PORT GPIOA
