#include "driver/rgb_led.h"

#include <stdbool.h>
#include <string.h>

#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/timer.h"

#if defined(USE_RGB_LED)

#define DMA_DEV DMA_DEVICE_TIM1_CH2

#define TIMER_HZ 48000000
#define TIMER_DIV ((PWM_CLOCK_FREQ_HZ / TIMER_HZ) - 1)

#define RGB_BIT_TIME ((TIMER_HZ / 800000) - 1)
#define RGB_T0H_TIME ((RGB_BIT_TIME / 3) + 1)
#define RGB_T1H_TIME ((RGB_BIT_TIME / 3) * 2 + 1)

#define RGB_BITS_LED 24
#define RGB_BUFFER_SIZE (RGB_BITS_LED * RGB_LED_MAX + 40)

static resource_tag_t timer_tag;
static const dma_assigment_t *rgb_dma;

static volatile bool rgb_dma_busy = false;
static DMA_RAM uint32_t rgb_timer_buffer[RGB_BUFFER_SIZE];
static uint32_t rgb_timer_buffer_count = 0;

static uint32_t timer_channel_addr(timer_dev_t *timer, timer_channel_t chan) {
  switch (chan) {
  case TIMER_CH1:
  case TIMER_CH1N:
    return (uint32_t)(&timer->c1dt);
  case TIMER_CH2:
  case TIMER_CH2N:
    return (uint32_t)(&timer->c2dt);
  case TIMER_CH3:
  case TIMER_CH3N:
    return (uint32_t)(&timer->c3dt);
  case TIMER_CH4:
    return (uint32_t)(&timer->c4dt);
  default:
    return 0;
  }
}

static const gpio_af_t *rgb_led_find_af(gpio_pins_t pin) {
  for (uint32_t j = 0; j < GPIO_AF_MAX; j++) {
    const gpio_af_t *func = &gpio_pin_afs[j];
    if (func->pin != pin || RESOURCE_TAG_TYPE(func->tag) != RESOURCE_TIM) {
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
  rgb_dma = dma_alloc(timer_tag, DMA_DEVICE_RGB);

  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_ALTERNATE;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;
  gpio_pin_init_af(pin, gpio_init, func->af);

  const timer_channel_t ch = TIMER_TAG_CH(func->tag);
  const timer_index_t tim = TIMER_TAG_TIM(func->tag);
  const timer_def_t *def = &timer_defs[tim];

  rcc_enable(def->rcc);

  tmr_counter_enable(def->instance, FALSE);

  tmr_base_init(def->instance, RGB_BIT_TIME, TIMER_DIV);
  tmr_cnt_dir_set(def->instance, TMR_COUNT_UP);
  tmr_clock_source_div_set(def->instance, TMR_CLOCK_DIV1);

  tmr_output_config_type tim_oc_init;
  tmr_output_default_para_init(&tim_oc_init);
  tim_oc_init.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tim_oc_init.oc_idle_state = FALSE;
  tim_oc_init.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tim_oc_init.oc_output_state = TRUE;
  tmr_output_channel_config(def->instance, timer_channel_val(ch), &tim_oc_init);

  tmr_channel_value_set(def->instance, timer_channel_val(ch), 0);
  tmr_output_channel_buffer_enable(def->instance, timer_channel_val(ch), TRUE);
  tmr_channel_enable(def->instance, timer_channel_val(ch), TRUE);

  tmr_period_buffer_enable(def->instance, TRUE);

  dma_enable_rcc(rgb_dma);
  dma_reset(rgb_dma->def->channel);

  dma_init_type init;
  init.peripheral_base_addr = timer_channel_addr(def->instance, ch);
  init.memory_base_addr = (uint32_t)rgb_timer_buffer;
  init.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  init.buffer_size = RGB_BUFFER_SIZE;
  init.peripheral_inc_enable = FALSE;
  init.memory_inc_enable = TRUE;
  init.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
  init.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
  init.loop_mode_enable = FALSE;
  init.priority = DMA_PRIORITY_MEDIUM;
  dma_init(rgb_dma->def->channel, &init);
  dmamux_init(rgb_dma->def->mux, rgb_dma->request);

  interrupt_enable(rgb_dma->def->irq, DMA_PRIORITY);

  tmr_output_enable(def->instance, TRUE);
}

bool rgb_led_busy() {
  return rgb_dma_busy;
}

void rgb_led_set_value(uint32_t value, uint32_t count) {
  if (rgb_dma_busy) {
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
  if (rgb_dma_busy) {
    return;
  }
  rgb_dma_busy = true;

  dma_prepare_tx_memory((void *)rgb_timer_buffer, sizeof(rgb_timer_buffer));
  dma_clear_flag_tc(rgb_dma);

  const timer_channel_t ch = TIMER_TAG_CH(timer_tag);
  const timer_index_t tim = TIMER_TAG_TIM(timer_tag);
  const timer_def_t *def = &timer_defs[tim];

  rgb_dma->def->channel->paddr = timer_channel_addr(def->instance, ch);
  rgb_dma->def->channel->maddr = (uint32_t)rgb_timer_buffer;
  rgb_dma->def->channel->dtcnt = rgb_timer_buffer_count;

  dma_interrupt_enable(rgb_dma->def->channel, DMA_FDT_INT, TRUE);

  timer_enable_dma_request(tim, ch, TRUE);
  tmr_counter_value_set(def->instance, 0);

  tmr_counter_enable(def->instance, TRUE);
  dma_channel_enable(rgb_dma->def->channel, TRUE);
}

void rgb_dma_isr(const dma_assigment_t *ass) {
  dma_clear_flag_tc(ass);
  dma_channel_enable(ass->def->channel, FALSE);

  const uint8_t ch = TIMER_TAG_CH(timer_tag);
  const uint8_t tim = TIMER_TAG_TIM(timer_tag);
  const timer_def_t *def = &timer_defs[tim];

  tmr_counter_enable(def->instance, FALSE);
  timer_enable_dma_request(tim, ch, FALSE);

  rgb_dma_busy = false;
}

#endif