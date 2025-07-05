#include "driver/rgb_led.h"

#include <stdbool.h>
#include <string.h>

#include "core/failloop.h"
#include "core/project.h"
#include "core/target.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/timer.h"

#if defined(USE_RGB_LED)

#define TIMER_HZ PWM_CLOCK_FREQ_HZ
#define TIMER_DIV ((PWM_CLOCK_FREQ_HZ / TIMER_HZ) - 1)

// WS2812B timing requirements:
// T0H: 350ns ±150ns
// T1H: 700ns ±150ns
// Period: 1250ns ±600ns (800kHz)
#define RGB_BIT_TIME ((TIMER_HZ / 800000) - 1)
#define RGB_T0H_TIME ((RGB_BIT_TIME / 3) + 1)
#define RGB_T1H_TIME ((RGB_BIT_TIME / 3) * 2 + 1)

#define RGB_BITS_LED 24
#define RGB_RESET_BITS 20 // >50us reset time at 800kHz
#define RGB_BUFFER_SIZE (RGB_BITS_LED * RGB_LEDS_PER_UPDATE + (RGB_RESET_BITS * 2))

extern resource_tag_t rgb_timer_tag;
extern volatile bool rgb_dma_busy;

extern uint32_t led_values[RGB_LED_MAX]; // From io/rgb_led.c

static DMA_RAM uint32_t rgb_timer_buffer[RGB_BUFFER_SIZE];
static uint32_t rgb_timer_buffer_count = 0;

static uint32_t led_count = 0;
static uint32_t batch_index = 0;

void rgb_led_init() {
  const gpio_pins_t pin = target.rgb_led;
  if (pin == PIN_NONE || target.dma[DMA_DEVICE_RGB].dma == 0) {
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

  LL_DMA_InitTypeDef dma_init;
  LL_DMA_StructInit(&dma_init);
#if defined(STM32H7) || defined(STM32G4)
  dma_init.PeriphRequest = target.dma[DMA_DEVICE_RGB].request;
#else
  dma_init.Channel = dma_map_channel(target.dma[DMA_DEVICE_RGB].channel);
#endif
  dma_init.PeriphOrM2MSrcAddress = timer_channel_addr(def->instance, ch);
  dma_init.MemoryOrM2MDstAddress = (uint32_t)rgb_timer_buffer;
  dma_init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  dma_init.NbData = RGB_BUFFER_SIZE;
  dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  dma_init.Mode = LL_DMA_MODE_NORMAL;
  dma_init.Priority = LL_DMA_PRIORITY_VERYHIGH;
#ifndef STM32G4
  dma_init.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  dma_init.MemBurst = LL_DMA_MBURST_SINGLE;
  dma_init.PeriphBurst = LL_DMA_PBURST_SINGLE;
#endif
  LL_DMA_Init(rgb_dma->port, rgb_dma->stream_index, &dma_init);
  LL_DMA_DisableStream(rgb_dma->port, rgb_dma->stream_index);

  LL_DMA_EnableIT_TC(rgb_dma->port, rgb_dma->stream_index);
  dma_clear_flag_tc(rgb_dma);
  interrupt_enable(rgb_dma->irq, DMA_PRIORITY);

  // Mark RGB LED feature as available
  target_set_feature(FEATURE_RGB_LED);
}

void rgb_led_send(uint32_t count) {
  if (rgb_led_busy()) {
    return;
  }

  if (count > RGB_LED_MAX) {
    count = RGB_LED_MAX;
  }

  // Store count and reset batch index if starting new transmission
  if (batch_index == 0) {
    led_count = count;
  }

  // Exit if no LEDs to send
  if (led_count == 0) {
    return;
  }

  uint32_t batch_start = batch_index * RGB_LEDS_PER_UPDATE;
  if (batch_start >= led_count) {
    return;
  }

  uint32_t batch_count = RGB_LEDS_PER_UPDATE;
  if (batch_start + batch_count > led_count) {
    batch_count = led_count - batch_start;
  }

  uint32_t offset = 0;

  // Add reset signal at start of first batch
  if (batch_start == 0) {
    for (uint32_t i = 0; i < RGB_RESET_BITS; i++) {
      rgb_timer_buffer[offset++] = 0;
    }
  }

  // Encode LED data for this batch
  for (uint32_t i = 0; i < batch_count; i++) {
    uint32_t value = led_values[batch_start + i];
    for (int32_t j = RGB_BITS_LED - 1; j >= 0; j--) {
      rgb_timer_buffer[offset++] = ((value >> j) & 0x1) ? RGB_T1H_TIME : RGB_T0H_TIME;
    }
  }

  // Add reset signal at end of last batch
  if (batch_start + batch_count >= led_count) {
    for (uint32_t i = 0; i < RGB_RESET_BITS; i++) {
      rgb_timer_buffer[offset++] = 0;
    }
  }

  rgb_timer_buffer_count = offset;
  rgb_dma_busy = true;

  const timer_channel_t ch = TIMER_TAG_CH(rgb_timer_tag);
  const timer_def_t *tim = &timer_defs[TIMER_TAG_TIM(rgb_timer_tag)];

  const dma_stream_def_t *rgb_dma = &dma_stream_defs[target.dma[DMA_DEVICE_RGB].dma];

  dma_clear_flag_tc(rgb_dma);
  dma_prepare_tx_memory((void *)rgb_timer_buffer, rgb_timer_buffer_count * sizeof(uint32_t));

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

  // Move to next batch
  batch_index++;

  // Check if we have more batches to send
  const uint32_t total_batches = (led_count + RGB_LEDS_PER_UPDATE - 1) / RGB_LEDS_PER_UPDATE;
  if (batch_index < total_batches) {
    // Send next batch
    rgb_led_send(led_count);
  } else {
    // All batches sent, reset for next update
    batch_index = 0;
  }
}

#endif