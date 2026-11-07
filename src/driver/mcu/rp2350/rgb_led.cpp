#include "driver/rgb_led.h"

#include <hardware/pio.h>

#include "core/failloop.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"

#if defined(USE_RGB_LED)

#include "rgb_led.pio.h"

// PIO0 and PIO1 run DShot.
#define RGB_PIO pio2

static constexpr uint32_t RGB_BIT_HZ = 800000;
static constexpr uint32_t RGB_CYCLES_PER_BIT = 10;
static constexpr uint32_t RGB_BITS_LED = 24;

extern volatile bool rgb_dma_busy;

// One GRB value per LED in the upper 24 bits, shifted out MSB first.
static DMA_RAM uint32_t rgb_buffer[RGB_LED_MAX];
static uint32_t rgb_buffer_count = 0;

static uint32_t rgb_sm;
static dma_stream_t rgb_dma = DMA_STREAM_INVALID;

void rgb_led_init() {
  const gpio_pins_t pin = target.rgb_led;
  if (pin == PIN_NONE) {
    return;
  }
  const uint32_t index = gpio_pin_defs[pin].pin_index;

  rgb_dma = target.dma[DMA_DEVICE_RGB].dma;
  if (rgb_dma == DMA_STREAM_INVALID)
    rgb_dma = dma_claim_unused(DMA_DEVICE_RGB);
  if (rgb_dma == DMA_STREAM_INVALID)
    failloop(FAILLOOP_DMA);

  // A PIO block reaches 32 consecutive pins, from GPIO0 or GPIO16.
  if (index >= 32)
    pio_set_gpio_base(RGB_PIO, 16);

  rgb_sm = pio_claim_unused_sm(RGB_PIO, true);
  const uint32_t offset = pio_add_program(RGB_PIO, &rgb_led_program);

  pio_gpio_init(RGB_PIO, index);
  gpio_set_pulls(index, false, true);
  gpio_set_drive_strength(index, GPIO_DRIVE_STRENGTH_12MA);

  pio_sm_config config = rgb_led_program_get_default_config(offset);
  sm_config_set_set_pins(&config, index, 1);
  sm_config_set_out_pins(&config, index, 1);
  sm_config_set_out_shift(&config, false, true, RGB_BITS_LED);
  sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);
  sm_config_set_clkdiv(&config, (float)SYS_CLOCK_FREQ_HZ / (RGB_BIT_HZ * RGB_CYCLES_PER_BIT));
  pio_sm_init(RGB_PIO, rgb_sm, offset, &config);
  pio_sm_set_consecutive_pindirs(RGB_PIO, rgb_sm, index, 1, true);
  pio_sm_set_enabled(RGB_PIO, rgb_sm, true);

  const uint32_t channel = dma_stream_channel(rgb_dma);
  dma_channel_config dma = dma_channel_get_default_config(channel);
  channel_config_set_transfer_data_size(&dma, DMA_SIZE_32);
  channel_config_set_dreq(&dma, pio_get_dreq(RGB_PIO, rgb_sm, true));
  channel_config_set_read_increment(&dma, true);
  channel_config_set_write_increment(&dma, false);
  dma_channel_configure(channel, &dma, &RGB_PIO->txf[rgb_sm], rgb_buffer, 0, false);
  dma_channel_set_irq0_enabled(channel, true);
  interrupt_enable(DMA_IRQ_0_IRQn, DMA_PRIORITY);
}

void rgb_led_set_value(uint32_t value, uint32_t count) {
  if (rgb_led_busy()) {
    return;
  }

  // value holds the 24-bit G R B colour.
  count = MIN(count, RGB_LED_MAX);
  for (uint32_t i = 0; i < count; i++) {
    rgb_buffer[i] = value << (32 - RGB_BITS_LED);
  }
  rgb_buffer_count = count;
}

void rgb_led_send() {
  if (rgb_led_busy() || rgb_dma == DMA_STREAM_INVALID) {
    return;
  }

  rgb_dma_busy = true;
  dma_channel_transfer_from_buffer_now(dma_stream_channel(rgb_dma), rgb_buffer, rgb_buffer_count);
}

void rgb_dma_isr(const dma_device_t dev) {
  rgb_dma_busy = false;
}

#endif
