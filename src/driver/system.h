#pragma once

#ifdef STM32F4
#include <stm32f4xx.h>
#include <stm32f4xx_hal_flash.h>
#include <stm32f4xx_ll_adc.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_dma.h>
#include <stm32f4xx_ll_exti.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_pwr.h>
#include <stm32f4xx_ll_rtc.h>
#include <stm32f4xx_ll_spi.h>
#include <stm32f4xx_ll_system.h>
#include <stm32f4xx_ll_tim.h>
#include <stm32f4xx_ll_usart.h>
#endif

#ifdef STM32F7
#include <stm32f7xx.h>
#include <stm32f7xx_hal_flash.h>
#include <stm32f7xx_ll_adc.h>
#include <stm32f7xx_ll_bus.h>
#include <stm32f7xx_ll_dma.h>
#include <stm32f7xx_ll_exti.h>
#include <stm32f7xx_ll_gpio.h>
#include <stm32f7xx_ll_pwr.h>
#include <stm32f7xx_ll_rtc.h>
#include <stm32f7xx_ll_spi.h>
#include <stm32f7xx_ll_system.h>
#include <stm32f7xx_ll_tim.h>
#include <stm32f7xx_ll_usart.h>
#endif

#ifdef STM32H7
#include <stm32h7xx.h>
#include <stm32h7xx_hal_flash.h>
#include <stm32h7xx_ll_adc.h>
#include <stm32h7xx_ll_bus.h>
#include <stm32h7xx_ll_dma.h>
#include <stm32h7xx_ll_exti.h>
#include <stm32h7xx_ll_gpio.h>
#include <stm32h7xx_ll_pwr.h>
#include <stm32h7xx_ll_rtc.h>
#include <stm32h7xx_ll_spi.h>
#include <stm32h7xx_ll_system.h>
#include <stm32h7xx_ll_tim.h>
#include <stm32h7xx_ll_usart.h>
#endif

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

#define LOOPTIME LOOPTIME_8K
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
