#include "driver/rgb_led.h"

#include <stdbool.h>
#include <string.h>

#include "core/failloop.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/timer.h"

#if defined(USE_RGB_LED)

#define TIMER_HZ PWM_CLOCK_FREQ_HZ
#define TIMER_DIV ((PWM_CLOCK_FREQ_HZ / TIMER_HZ) - 1)

#define RGB_BIT_TIME ((TIMER_HZ / 800000) - 1)
#define RGB_T0H_TIME ((RGB_BIT_TIME / 3) + 1)
#define RGB_T1H_TIME ((RGB_BIT_TIME / 3) * 2 + 1)

#define RGB_BITS_LED 24
#define RGB_BUFFER_SIZE (RGB_BITS_LED * RGB_LED_MAX + 40)

extern resource_tag_t rgb_timer_tag;
extern volatile bool rgb_dma_busy;

static DMA_RAM uint32_t rgb_timer_buffer[RGB_BUFFER_SIZE];
static uint32_t rgb_timer_buffer_count = 0;

void rgb_led_init() {
  const gpio_pins_t pin = target.rgb_led;
  if (pin == PIN_NONE) {
    return;
  }

  rgb_timer_tag = target.dma[DMA_DEVICE_RGB].tag;
  if (rgb_timer_tag == 0) {
    return;
  }

  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_ALTERNATE;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_DOWN_PULL;
  gpio_pin_init_tag(pin, gpio_init, rgb_timer_tag);

  const timer_channel_t ch = TIMER_TAG_CH(rgb_timer_tag);
  const timer_index_t tim = TIMER_TAG_TIM(rgb_timer_tag);
  const timer_def_t *def = &timer_defs[tim];

  rcc_enable(def->rcc);

  LL_TIM_InitTypeDef tim_init;
  LL_TIM_StructInit(&tim_init);
  tim_init.Autoreload = RGB_BIT_TIME;
  tim_init.Prescaler = TIMER_DIV;
  tim_init.ClockDivision = 0;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(def->instance, &tim_init);
  LL_TIM_EnableARRPreload(def->instance);
  LL_TIM_DisableMasterSlaveMode(def->instance);

  LL_TIM_OC_InitTypeDef tim_oc_init;
  LL_TIM_OC_StructInit(&tim_oc_init);
  tim_oc_init.CompareValue = 0;
  tim_oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  tim_oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  tim_oc_init.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  tim_oc_init.OCNState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCNPolarity = LL_TIM_OCPOLARITY_LOW;
  tim_oc_init.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(def->instance, timer_channel_val(ch), &tim_oc_init);
  LL_TIM_OC_EnablePreload(def->instance, timer_channel_val(ch));
  LL_TIM_EnableAllOutputs(def->instance);

  const dma_stream_def_t *rgb_dma = &dma_stream_defs[target.dma[DMA_DEVICE_RGB].dma];
  dma_enable_rcc(rgb_dma);
  LL_DMA_DeInit(rgb_dma->port, rgb_dma->stream_index);

  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);
#if defined(STM32H7) || defined(STM32G4)
  DMA_InitStructure.PeriphRequest = target.dma[DMA_DEVICE_RGB].request;
#else
  DMA_InitStructure.Channel = dma_map_channel(target.dma[DMA_DEVICE_RGB].channel);
#endif
  DMA_InitStructure.PeriphOrM2MSrcAddress = timer_channel_addr(def->instance, ch);
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)rgb_timer_buffer;
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructure.NbData = RGB_BUFFER_SIZE;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_VERYHIGH;
#ifndef STM32G4
  DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
#endif
  LL_DMA_Init(rgb_dma->port, rgb_dma->stream_index, &DMA_InitStructure);

  LL_DMA_EnableIT_TC(rgb_dma->port, rgb_dma->stream_index);
  interrupt_enable(rgb_dma->irq, DMA_PRIORITY);
}

void rgb_led_set_value(uint32_t value, uint32_t count) {
  if (rgb_led_busy()) {
    return;
  }

  uint32_t offset = 0;
  for (uint32_t i = 0; i < 20; i++) {
    rgb_timer_buffer[offset++] = 0;
  }
  for (uint32_t i = 0; i < count; i++) {
    // rgb_led_value contains a (32bit) int that contains the RGB values in G R B format already
    // Test each bit and assign the T1H or T0H depending on whether it is 1 or 0.
    for (int32_t j = RGB_BITS_LED - 1; j >= 0; j--) {
      rgb_timer_buffer[offset++] = ((value >> j) & 0x1) ? RGB_T1H_TIME : RGB_T0H_TIME;
    }
  }
  for (uint32_t i = 0; i < 20; i++) {
    rgb_timer_buffer[offset++] = 0;
  }
  rgb_timer_buffer_count = offset;
}

void rgb_led_send() {
  if (rgb_led_busy()) {
    return;
  }

  rgb_dma_busy = true;

  const timer_channel_t ch = TIMER_TAG_CH(rgb_timer_tag);
  const timer_def_t *tim = &timer_defs[TIMER_TAG_TIM(rgb_timer_tag)];

  const dma_stream_def_t *rgb_dma = &dma_stream_defs[target.dma[DMA_DEVICE_RGB].dma];

  dma_clear_flag_tc(rgb_dma);
  dma_prepare_tx_memory((void *)rgb_timer_buffer, sizeof(rgb_timer_buffer));

  LL_DMA_SetMemoryAddress(rgb_dma->port, rgb_dma->stream_index, (uint32_t)rgb_timer_buffer);
  LL_DMA_SetDataLength(rgb_dma->port, rgb_dma->stream_index, rgb_timer_buffer_count);

  LL_DMA_EnableStream(rgb_dma->port, rgb_dma->stream_index);
  timer_enable_dma_request(TIMER_TAG_TIM(rgb_timer_tag), ch, true);
  LL_TIM_EnableCounter(tim->instance);
}

void rgb_dma_isr(const dma_device_t dev) {
  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[dev].dma];

  dma_clear_flag_tc(dma);
  LL_DMA_DisableStream(dma->port, dma->stream_index);

  const timer_index_t tim = TIMER_TAG_TIM(rgb_timer_tag);
  const timer_channel_t ch = TIMER_TAG_CH(rgb_timer_tag);
  const timer_def_t *def = &timer_defs[tim];

  LL_TIM_DisableCounter(def->instance);
  timer_enable_dma_request(tim, ch, false);

  rgb_dma_busy = false;
}

#endif