#pragma once

#include "gpio_pins.h"
#include "motor_pins.h"
#include "spi_ports.h"
#include "usart_ports.h"

#include "target.h"

#ifdef STM32F411
#define SYS_CLOCK_FREQ_HZ 120000000L
#define PWM_CLOCK_FREQ_HZ 120000000L
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

#ifdef STM32F7
#define SYS_CLOCK_FREQ_HZ 216000000
#define PWM_CLOCK_FREQ_HZ 216000000
#define SPI_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 4)

#define LOOPTIME LOOPTIME_8K

#define WITHIN_DTCM_RAM(p) (((uint32_t)p & 0xffff0000) == 0x20000000)
#define WITHIN_DMA_RAM(p) (false)
#endif

#ifdef STM32H7
#define SYS_CLOCK_FREQ_HZ 480000000
#define PWM_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 2)
#define SPI_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 4)

#define LOOPTIME LOOPTIME_8K

#define WITHIN_DTCM_RAM(p) (((uint32_t)p & 0xfffe0000) == 0x20000000)
#define WITHIN_DMA_RAM(p) (((uint32_t)p & 0xfffe0000) == 0x30000000)
#endif

#ifdef USE_FAST_RAM
#define FAST_RAM __attribute__((section(".fast_ram"), aligned(4)))
#else
#define FAST_RAM
#endif

#ifdef USE_DMA_RAM
#define DMA_RAM __attribute__((section(".dma_ram"), aligned(32)))
#else
#define DMA_RAM
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

#if defined(USE_M25P16) || defined(USE_SDCARD)
#define ENABLE_BLACKBOX
#endif

#define SERIAL_RX

#if defined(USE_CC2500)
#define RX_FRSKY
#endif

#if defined(USE_A7105)
#define RX_FLYSKY
#endif

#if defined(USE_SX127X) || defined(USE_SX128X)
#define RX_EXPRESS_LRS
#endif

//*************************************Features that still need to be moved into targets and checked for compatability************************************************
// RGB led type ws2812 - ws2813
// numbers over 8 could decrease performance
// #define RGB_LED_NUMBER 0
// #define RGB_LED_DMA

// pin / port for the RGB led ( programming port ok )
// #define RGB_PIN LL_GPIO_PIN_11
// #define RGB_PORT GPIOA
