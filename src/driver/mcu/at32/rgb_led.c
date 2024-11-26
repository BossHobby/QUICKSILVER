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

  const dma_stream_def_t *rgb_dma = &dma_stream_defs[target.dma[DMA_DEVICE_RGB].dma];
  dma_enable_rcc(rgb_dma);
  dma_reset(rgb_dma->stream);

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
  dma_init(rgb_dma->stream, &init);
  dmamux_init(rgb_dma->mux, target.dma[DMA_DEVICE_RGB].request);

  interrupt_enable(rgb_dma->irq, DMA_PRIORITY);

  tmr_output_enable(def->instance, TRUE);
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

  rgb_dma->stream->paddr = timer_channel_addr(tim->instance, ch);
  rgb_dma->stream->maddr = (uint32_t)rgb_timer_buffer;
  rgb_dma->stream->dtcnt = rgb_timer_buffer_count;

  dma_interrupt_enable(rgb_dma->stream, DMA_FDT_INT, TRUE);

  dma_channel_enable(rgb_dma->stream, TRUE);
  timer_enable_dma_request(TIMER_TAG_TIM(rgb_timer_tag), ch, TRUE);
  tmr_counter_enable(tim->instance, TRUE);
}

void rgb_dma_isr(const dma_device_t dev) {
  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[dev].dma];

  dma_clear_flag_tc(dma);
  dma_channel_enable(dma->stream, FALSE);

  const timer_index_t tim = TIMER_TAG_TIM(rgb_timer_tag);
  const timer_channel_t ch = TIMER_TAG_CH(rgb_timer_tag);
  const timer_def_t *def = &timer_defs[tim];

  tmr_counter_enable(def->instance, FALSE);
  timer_enable_dma_request(tim, ch, FALSE);

  rgb_dma_busy = false;
}

#endif