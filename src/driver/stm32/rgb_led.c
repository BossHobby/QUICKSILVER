#include "driver/rgb_led.h"

#include <stdbool.h>
#include <string.h>

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

static resource_tag_t timer_tag = 0;
static const dma_assigment_t *rgb_dma = NULL;

static volatile bool rgb_dma_busy = false;
static DMA_RAM uint32_t rgb_timer_buffer[RGB_BUFFER_SIZE];
static uint32_t rgb_timer_buffer_count = 0;

static const gpio_af_t *rgb_led_find_af(gpio_pins_t pin) {
  for (uint32_t j = 0; j < GPIO_AF_MAX; j++) {
    const gpio_af_t *func = &gpio_pin_afs[j];
    if (func->pin != pin ||
        RESOURCE_TAG_TYPE(func->tag) != RESOURCE_TIM ||
        TIMER_TAG_CH(func->tag) == TIMER_CH1N ||
        TIMER_TAG_CH(func->tag) == TIMER_CH2N ||
        TIMER_TAG_CH(func->tag) == TIMER_CH3N ||
        TIMER_TAG_CH(func->tag) == TIMER_CH4N) {
      continue;
    }

    if (timer_alloc_tag(TIMER_USE_RGB_LED, func->tag)) {
      return func;
    }
  }

  return NULL;
}

void rgb_led_init() {
  const gpio_pins_t pin = target.rgb_led;
  if (pin == PIN_NONE) {
    return;
  }

  const gpio_af_t *func = rgb_led_find_af(pin);
  if (func == NULL) {
    return;
  }

  timer_tag = func->tag;
  rgb_dma = dma_alloc(DMA_DEVICE_RGB, timer_tag);
  if (rgb_dma == NULL) {
    return;
  }

  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_ALTERNATE;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_DOWN_PULL;
  gpio_pin_init_af(pin, gpio_init, func->af);

  const timer_channel_t ch = TIMER_TAG_CH(func->tag);
  const timer_index_t tim = TIMER_TAG_TIM(func->tag);
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

  dma_enable_rcc(rgb_dma);
  LL_DMA_DeInit(rgb_dma->def->port, rgb_dma->def->stream_index);

  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);
#if defined(STM32H7) || defined(STM32G4)
  DMA_InitStructure.PeriphRequest = rgb_dma->chan->request;
#else
  DMA_InitStructure.Channel = rgb_dma->chan->channel;
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
  LL_DMA_Init(rgb_dma->def->port, rgb_dma->def->stream_index, &DMA_InitStructure);

  LL_DMA_EnableIT_TC(rgb_dma->def->port, rgb_dma->def->stream_index);
  interrupt_enable(rgb_dma->def->irq, DMA_PRIORITY);
}

bool rgb_led_busy() {
  return rgb_dma == NULL || rgb_dma_busy;
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
  rgb_dma_busy = true;

  const timer_channel_t ch = TIMER_TAG_CH(timer_tag);
  const timer_index_t tim = TIMER_TAG_TIM(timer_tag);
  const timer_def_t *def = &timer_defs[tim];

  dma_clear_flag_tc(rgb_dma->def);
  dma_prepare_tx_memory((void *)rgb_timer_buffer, sizeof(rgb_timer_buffer));

  LL_DMA_SetMemoryAddress(rgb_dma->def->port, rgb_dma->def->stream_index, (uint32_t)rgb_timer_buffer);
  LL_DMA_SetDataLength(rgb_dma->def->port, rgb_dma->def->stream_index, rgb_timer_buffer_count);

  LL_DMA_EnableStream(rgb_dma->def->port, rgb_dma->def->stream_index);
  timer_enable_dma_request(tim, ch, true);
  LL_TIM_EnableCounter(def->instance);
}

void rgb_dma_isr(const dma_assigment_t *ass) {
  dma_clear_flag_tc(ass->def);
  LL_DMA_DisableStream(ass->def->port, ass->def->stream_index);

  const uint8_t ch = TIMER_TAG_CH(timer_tag);
  const uint8_t tim = TIMER_TAG_TIM(timer_tag);
  const timer_def_t *def = &timer_defs[tim];

  LL_TIM_DisableCounter(def->instance);
  timer_enable_dma_request(tim, ch, false);

  rgb_dma_busy = false;
}

#endif