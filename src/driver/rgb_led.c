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

#ifdef RGB_PIN
void rgb_send(int data);

#ifdef RGB_LED_DMA

#define RGB_BIT_TIME ((PWM_CLOCK_FREQ_HZ / 800000) - 1)
#define RGB_T0H_TIME (RGB_BIT_TIME / 3)
#define RGB_T1H_TIME ((RGB_BIT_TIME / 3) * 2)
#define RGB_BITS_LED 24
#define RGB_DELAY_BUF 42
#define RGB_BUFFER_SIZE (RGB_BITS_LED * RGB_LED_MAX + RGB_DELAY_BUF)
extern int rgb_led_value[];

volatile int rgb_dma_phase = 0; // 3:rgb data ready
                                // 2:rgb dma buffer ready
                                // 1:rgb dma busy
                                // 0:idle

volatile uint32_t rgb_timer_buffer[RGB_BUFFER_SIZE] = {0}; // DMA buffer: Array of PWM duty cycle timings

const dma_stream_def_t *rgb_dma = &dma_stream_defs[RGB_LED_DMA];

void rgb_init_io() {
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

void rgb_init_tim() {
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

void rgb_init_dma() {
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
  ;
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
}

void rgb_init_nvic() {
  interrupt_enable(rgb_dma->irq, DMA_PRIORITY);
}

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

void rgb_dma_buffer_making() {
  // generate rgb dma packet of pulse width timings for all LEDs
  for (uint32_t n = 0; n < RGB_LED_NUMBER; n++) {
    // rgb_led_value contains a (32bit) int that contains the RGB values in G R B format already
    // Test each bit and assign the T1H or T0H depending on whether it is 1 or 0.
    for (size_t i = 0; i < RGB_BITS_LED; i++) {
      rgb_timer_buffer[(n * RGB_BITS_LED) + i] = (rgb_led_value[n] & (1 << ((RGB_BITS_LED - 1) - i))) ? RGB_T1H_TIME : RGB_T0H_TIME;
    }
  }
  for (uint32_t n = (RGB_LED_NUMBER * RGB_BITS_LED); n < RGB_BUFFER_SIZE; n++) {
    rgb_timer_buffer[n] = 0;
  }
}

void rgb_dma_trigger() {
  rgb_init_dma();
  LL_DMA_EnableIT_TC(rgb_dma->port, rgb_dma->stream_index);
  LL_DMA_EnableStream(rgb_dma->port, rgb_dma->stream_index);
  rgb_enable_dma_request();
}

void rgb_init() {

  rgb_init_io();
  rgb_init_tim();
  rgb_init_nvic();
  rgb_init_dma();

  for (int i = 0; i < RGB_LED_NUMBER; i++) {
    rgb_led_value[i] = 0;
  }
  if (!rgb_dma_phase)
    rgb_dma_phase = 3;
}

void rgb_send(int data) {
  if (!rgb_dma_phase)
    rgb_dma_phase = 3;
}

void rgb_dma_start() {
  if (rgb_dma_phase <= 1)
    return;

  if (rgb_dma_phase == 3) {
    rgb_dma_buffer_making();
    rgb_dma_phase = 2;
    return;
  }

  rgb_dma_phase = 1;
  rgb_dma_trigger();
}

void rgb_dma_isr() {
  dma_clear_flag_tc(rgb_dma->port, rgb_dma->stream_index);
  rgb_disable_dma_request();
  LL_DMA_DisableStream(rgb_dma->port, rgb_dma->stream_index);

  // Set phase to idle
  rgb_dma_phase = 0;

  // Set phase to idle
  rgb_dma_phase = 0;
}

#endif

#else
// rgb led not found
// some dummy headers just in case
void rgb_init() {
}

void rgb_send(int data) {
}
#endif
