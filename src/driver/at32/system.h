#pragma once

#ifdef AT32F4
#include <at32f435_437_clock.h>
#include <at32f435_437_conf.h>
#endif

#ifdef AT32F435
#define SYS_CLOCK_FREQ_HZ 288000000
#define PWM_CLOCK_FREQ_HZ 288000000
#define SPI_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 2)

#define LOOPTIME_MAX 125

#define UID_BASE 0x1FFFF7E8
#endif

typedef gpio_type gpio_port_t;
typedef spi_type spi_port_t;
typedef tmr_type timer_dev_t;
typedef usart_type usart_dev_t;

typedef struct {
  uint32_t device;

  dma_type *port;
  uint8_t port_index;

  dma_channel_type *channel;
  uint8_t channel_index;

  uint32_t request;
  dmamux_channel_type *mux;

  IRQn_Type irq;
} dma_stream_def_t;

#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "rcc.h"
#include "time.h"