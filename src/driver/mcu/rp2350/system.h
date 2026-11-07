#pragma once

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/resets.h>
#include <hardware/structs/i2c.h>
#include <hardware/structs/io_bank0.h>
#include <hardware/structs/pwm.h>
#include <hardware/structs/sio.h>
#include <hardware/structs/spi.h>
#include <hardware/uart.h>
#include <pico.h>

// CMSIS core (NVIC, SysTick, DWT, IRQn_Type); after the SDK headers, whose
// address map it would otherwise redefine.
#include <RP2350.h>

// The SDK's MIN/MAX macros would shadow the firmware's helpers in util.h,
// and the CMSIS peripheral pointers TIMER0/TIMER1 the timer_index_t names.
#undef MIN
#undef MAX
#undef TIMER0
#undef TIMER1

// clk_sys and clk_peri as configured by the SDK's runtime_init_clocks().
#define SYS_CLOCK_FREQ_HZ 150000000
#define PWM_CLOCK_FREQ_HZ SYS_CLOCK_FREQ_HZ
#define SPI_CLOCK_FREQ_HZ SYS_CLOCK_FREQ_HZ

#define LOOPTIME_MAX 125

// CHIPID and RANDID, the first six OTP rows, as read through the ECC window.
#define UID_BASE OTP_DATA_BASE

typedef sio_hw_t gpio_port_t;
typedef spi_hw_t spi_port_t;
typedef i2c_hw_t i2c_port_t;
typedef pwm_slice_hw_t timer_dev_t;
typedef uart_hw_t usart_dev_t;
// The hardware channel number of a dma_stream_t.
typedef uint8_t dma_stream_def_t;

#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "time.h"
