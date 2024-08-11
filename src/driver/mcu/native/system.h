#pragma once

#include <stdint.h>

#define __NOP() asm("nop")
#define __disable_irq() asm("nop")
#define __enable_irq() asm("nop")

#define SYS_CLOCK_FREQ_HZ 500000000

#define LOOPTIME_MAX 250

typedef uint32_t IRQn_Type;

typedef uint32_t gpio_port_t;
typedef uint32_t spi_port_t;
typedef uint32_t timer_dev_t;
typedef uint32_t usart_dev_t;
typedef uint32_t dma_stream_def_t;

#include "gpio.h"
#include "time.h"