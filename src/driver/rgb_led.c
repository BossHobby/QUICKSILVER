#include "driver/rgb_led.h"

#include <stdbool.h>
#include <string.h>

#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/timer.h"
#include "flight/control.h"
#include "project.h"
#include "util/util.h"

#if defined(RGB_PIN) && defined(RGB_LED_DMA)

#define RGB_BIT_TIME ((PWM_CLOCK_FREQ_HZ / 800000) - 1)
#define RGB_T0H_TIME (RGB_BIT_TIME / 3)
#define RGB_T1H_TIME ((RGB_BIT_TIME / 3) * 2)

#define RGB_BITS_LED 24

#define RGB_BUFFER_SIZE (RGB_BITS_LED * RGB_LED_MAX + 2)

static volatile bool rgb_dma_busy = false;
static volatile uint32_t rgb_timer_buffer[RGB_BUFFER_SIZE];
static uint32_t rgb_timer_buffer_count = 0;

static const dma_stream_def_t *rgb_dma = &dma_stream_defs[RGB_LED_DMA];

static void rgb_enable_dma_request() {
#if RGB_TIMER_CHANNEL == 1
  LL_TIM_EnableDMAReq_CC1(RGB_TIMER);
#elif RGB_TIMER_CHANNEL == 2
  LL_TIM_EnableDMAReq_CC2(RGB_TIMER);
#elif RGB_TIMER_CHANNEL == 3
  LL_TIM_EnableDMAReq_CC3(RGB_TIMER);
#elif RGB_TIMER_CHANNEL == 4
  LL_TIM_EnableDMAReq_CC4(RGB_TIMER);
#endif
  LL_TIM_EnableCounter(RGB_TIMER);
}

static void rgb_disable_dma_request() {
  LL_TIM_DisableCounter(RGB_TIMER);
#if RGB_TIMER_CHANNEL == 1
  LL_TIM_DisableDMAReq_CC1(RGB_TIMER);
#elif RGB_TIMER_CHANNEL == 2
  LL_TIM_DisableDMAReq_CC2(RGB_TIMER);
#elif RGB_TIMER_CHANNEL == 3
  LL_TIM_DisableDMAReq_CC3(RGB_TIMER);
#elif RGB_TIMER_CHANNEL == 4
  LL_TIM_DisableDMAReq_CC4(RGB_TIMER);
#endif
}

static void rgb_init_io() {
  LL_GPIO_InitTypeDef GPIO_InitStructure;

  // Config pin for digital output
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStructure.Pin = RGB_PIN;

  // Bind IO pin to the alternate function
  gpio_pin_init_af(&GPIO_InitStructure, RGB_PIN, RGB_TIM_AF);
  gpio_pin_reset(RGB_PIN);
}

static void rgb_init_tim() {
  LL_TIM_OC_InitTypeDef TIM_OCInitStructure;

  // Clock Enable (DMA)
  dma_enable_rcc(RGB_LED_DMA);

  // Timer init
  timer_init(RGB_TIMER, 1, RGB_BIT_TIME);

  TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OCInitStructure.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OCInitStructure.CompareValue = 0;
  TIM_OCInitStructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
#if RGB_TIMER_CHANNEL == 1
  LL_TIM_OC_Init(RGB_TIMER, LL_TIM_CHANNEL_CH1, &TIM_OCInitStructure);
  LL_TIM_OC_EnablePreload(RGB_TIMER, LL_TIM_CHANNEL_CH1);
#elif RGB_TIMER_CHANNEL == 2
  LL_TIM_OC_Init(RGB_TIMER, LL_TIM_CHANNEL_CH2, &TIM_OCInitStructure);
  LL_TIM_OC_EnablePreload(RGB_TIMER, LL_TIM_CHANNEL_CH2);
#elif RGB_TIMER_CHANNEL == 3
  LL_TIM_OC_Init(RGB_TIMER, LL_TIM_CHANNEL_CH3, &TIM_OCInitStructure);
  LL_TIM_OC_EnablePreload(RGB_TIMER, LL_TIM_CHANNEL_CH3);
#elif RGB_TIMER_CHANNEL == 4
  LL_TIM_OC_Init(RGB_TIMER, LL_TIM_CHANNEL_CH4, &TIM_OCInitStructure);
  LL_TIM_OC_EnablePreload(RGB_TIMER, LL_TIM_CHANNEL_CH4);
#endif
  LL_TIM_EnableARRPreload(RGB_TIMER);
}

static void rgb_init_dma() {
  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);

  dma_clear_flag_tc(rgb_dma->port, rgb_dma->stream_index);
  LL_DMA_DeInit(rgb_dma->port, rgb_dma->stream_index);
#ifdef STM32H7
  DMA_InitStructure.PeriphRequest = rgb_dma->request;
#else
  DMA_InitStructure.Channel = rgb_dma->channel;
#endif
#if RGB_TIMER_CHANNEL == 1
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&RGB_TIMER->CCR1;
#elif RGB_TIMER_CHANNEL == 2
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&RGB_TIMER->CCR2;
#elif RGB_TIMER_CHANNEL == 3
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&RGB_TIMER->CCR3;
#elif RGB_TIMER_CHANNEL == 4
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&RGB_TIMER->CCR4;
#endif
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructure.NbData = RGB_BUFFER_SIZE;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)rgb_timer_buffer;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD; // 32bit
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_2;
  DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(rgb_dma->port, rgb_dma->stream_index, &DMA_InitStructure);

  interrupt_enable(rgb_dma->irq, DMA_PRIORITY);
}

bool rgb_led_busy() {
  return rgb_dma_busy;
}

void rgb_led_init() {
  rgb_init_io();
  rgb_init_tim();
  rgb_init_dma();
}

void rgb_led_set_value(uint32_t *values, uint32_t count) {
  if (rgb_dma_busy) {
    return;
  }

  uint32_t offset = 0;

  // generate rgb dma packet of pulse width timings for all LEDs
  rgb_timer_buffer[offset++] = 0;
  for (uint32_t i = 0; i < count; i++) {
    const uint32_t value = values[i];

    // rgb_led_value contains a (32bit) int that contains the RGB values in G R B format already
    // Test each bit and assign the T1H or T0H depending on whether it is 1 or 0.
    for (int32_t j = RGB_BITS_LED - 1; j >= 0; j--) {
      rgb_timer_buffer[offset++] = ((value >> j) & 0x1) ? RGB_T1H_TIME : RGB_T0H_TIME;
    }
  }
  rgb_timer_buffer[offset++] = 0;

  rgb_timer_buffer_count = offset;
}

void rgb_led_send() {
  if (rgb_dma_busy) {
    return;
  }
  rgb_dma_busy = true;

  dma_prepare_tx_memory((void *)rgb_timer_buffer, sizeof(rgb_timer_buffer));

#if RGB_TIMER_CHANNEL == 1
  LL_DMA_SetPeriphAddress(rgb_dma->port, rgb_dma->stream_index, (uint32_t)&RGB_TIMER->CCR1);
#elif RGB_TIMER_CHANNEL == 2
  LL_DMA_SetPeriphAddress(rgb_dma->port, rgb_dma->stream_index, (uint32_t)&RGB_TIMER->CCR2);
#elif RGB_TIMER_CHANNEL == 3
  LL_DMA_SetPeriphAddress(rgb_dma->port, rgb_dma->stream_index, (uint32_t)&RGB_TIMER->CCR3);
#elif RGB_TIMER_CHANNEL == 4
  LL_DMA_SetPeriphAddress(rgb_dma->port, rgb_dma->stream_index, (uint32_t)&RGB_TIMER->CCR4);
#endif

  LL_DMA_SetMemoryAddress(rgb_dma->port, rgb_dma->stream_index, (uint32_t)rgb_timer_buffer);
  LL_DMA_SetDataLength(rgb_dma->port, rgb_dma->stream_index, rgb_timer_buffer_count);

  LL_DMA_EnableIT_TC(rgb_dma->port, rgb_dma->stream_index);
  LL_DMA_EnableStream(rgb_dma->port, rgb_dma->stream_index);

  rgb_enable_dma_request();
}

void rgb_dma_isr() {
  dma_clear_flag_tc(rgb_dma->port, rgb_dma->stream_index);
  rgb_disable_dma_request();
  LL_DMA_DisableStream(rgb_dma->port, rgb_dma->stream_index);

  rgb_dma_busy = false;
}

#else
void rgb_led_init() {}
void rgb_led_set_value(uint32_t *values, uint32_t count) {}
void rgb_led_send() {}
bool rgb_led_busy() {
  return false;
}
#endif
