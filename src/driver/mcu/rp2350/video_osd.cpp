#include "driver/osd/video_osd.h"

#include <stdlib.h>
#include <string.h>

#include <hardware/flash.h>
#include <hardware/pio.h>

#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/time.h"

#ifdef USE_VIDEO_OSD

#include "video_osd.pio.h"

// The sync state machine signals each displayed line, the pixel state machine
// shifts it out of a 2-bit framebuffer fed by one DMA channel, restarted at
// every vertical sync. Characters are drawn into the framebuffer when the OSD
// layer pushes them; blinking is done by the OSD layer.
//
// Without a camera the generator state machine produces PAL timing from a
// line table on a second DMA channel and triggers the pixel lines itself. The
// pixel state machine then drives only the white pin, so the generator owns
// the enable pin and transparent pixels show black. The sync state machine
// keeps watching the camera input; the OSD thread switches back once a
// camera's fields arrive.

static constexpr uint32_t VIDEO_OSD_PIO_HZ = 150000000;
static_assert(VIDEO_OSD_PIO_HZ == SYS_CLOCK_FREQ_HZ);

// Loaded into the sync state machine's X when a pulse turns out longer than
// an equalizing one, 545 cycles after its falling edge. It counts down every
// two cycles while the pulse is low, then every cycle: IRQ 4 comes 629 + this
// cycles after the falling edge of a 4.7 us (705 cycle) horizontal sync,
// 10.5 us. A pulse that is still low when it runs out, 16.3 us in, is broad.
static constexpr uint32_t VIDEO_OSD_SYNC_MIDDLE = 946;

static constexpr uint32_t VIDEO_OSD_CHAR_WIDTH = 12;
static constexpr uint32_t VIDEO_OSD_CHAR_HEIGHT = 18;
static constexpr uint32_t VIDEO_OSD_CHAR_BYTES = 54;

// 368 pixels a line at 16 per word; the 360 of the character grid start four
// pixels in.
static constexpr uint32_t VIDEO_OSD_LINE_WORDS = 23;
static constexpr uint32_t VIDEO_OSD_LEFT_PIXELS = 4;
// Lines counted after the vertical sync before the first character row: the
// sync state machine counts from the first horizontal sync after the
// equalizing pulses (PAL line 6, NTSC line 10).
static constexpr uint32_t VIDEO_OSD_TOP_LINES = 17;
// PAL shows all 16 rows; NTSC ends its field after about 13.
static constexpr uint32_t VIDEO_OSD_LINES = VIDEO_OSD_TOP_LINES + SD_ROWS * VIDEO_OSD_CHAR_HEIGHT;

// Pixel codes: bit 0 selects white, bit 1 switches the video to the overlay.
static constexpr uint32_t VIDEO_OSD_TRANSPARENT = 0b00;
static constexpr uint32_t VIDEO_OSD_BLACK = 0b10;
static constexpr uint32_t VIDEO_OSD_WHITE = 0b11;

// A field is 20 ms on PAL and 16.7 ms on NTSC; no sync for 100 ms means no
// camera.
static constexpr uint32_t VIDEO_OSD_PAL_MIN_PERIOD_US = 18300;
static constexpr uint32_t VIDEO_OSD_SYNC_TIMEOUT_US = 100000;
// Broad pulses repeat every half line during vertical sync; only the first
// one of a field starts its period.
static constexpr uint32_t VIDEO_OSD_FIELD_MIN_US = 3000;
// Without a camera the sync comparator's inputs both idle near ground and
// its output chatters, now and then like a broad pulse. Only a period between
// NTSC's 16.7 ms and PAL's 20 ms fields, with margin, counts as a camera.
static constexpr uint32_t VIDEO_OSD_FIELD_VALID_MIN_US = 15000;
static constexpr uint32_t VIDEO_OSD_FIELD_VALID_MAX_US = 22000;

// Free-running interlaced PAL: 625 lines of 64 us in two fields of 312.5.
// Each field opens with five broad and five equalizing half-line pulses and
// ends with five equalizing ones. The first field then has 305 full lines
// (6 to 310); the second starts half a line later with a black half line,
// has 304 full lines (319 to 622) and ends on a half line. Pixel lines start
// 10.5 us after the sync edge, as the sync state machine times them, and
// count from the first full line like a camera's fields.
static constexpr uint32_t VIDEO_OSD_GEN_LINE = 9600;
static constexpr uint32_t VIDEO_OSD_GEN_HALF_LINE = VIDEO_OSD_GEN_LINE / 2;
static constexpr uint32_t VIDEO_OSD_GEN_HSYNC = 705;      // 4.7 us
static constexpr uint32_t VIDEO_OSD_GEN_EQUALIZING = 353; // 2.35 us
static constexpr uint32_t VIDEO_OSD_GEN_BROAD = 4096;     // 27.3 us
static constexpr uint32_t VIDEO_OSD_GEN_ACTIVE = 1575;    // 10.5 us
static constexpr uint32_t VIDEO_OSD_GEN_PULSES = 5;
static constexpr uint32_t VIDEO_OSD_GEN_FIRST_LINES = 305;
static constexpr uint32_t VIDEO_OSD_GEN_SECOND_LINES = 304;
static_assert(VIDEO_OSD_GEN_FIRST_LINES == VIDEO_OSD_LINES);
// Two words per pulse, three per full line, one for the second field's
// leading half line, two for its trailing one, and one ending each field.
static constexpr uint32_t VIDEO_OSD_GEN_WORDS = 2 * (3 * VIDEO_OSD_GEN_PULSES * 2 + 1) +
                                                3 * (VIDEO_OSD_GEN_FIRST_LINES + VIDEO_OSD_GEN_SECOND_LINES) + 1 + 2;
// A table word lasts 7 cycles plus 4 per count.
static constexpr uint32_t VIDEO_OSD_GEN_MIN_CYCLES = 7;

// One 64-byte slot per character: a whole number per flash page and sector.
static constexpr uint32_t VIDEO_OSD_FONT_SLOT = 64;
static constexpr uint32_t VIDEO_OSD_FONT_SIZE = 256 * VIDEO_OSD_FONT_SLOT;

static uint32_t video_osd_framebuffer[VIDEO_OSD_LINES][VIDEO_OSD_LINE_WORDS];
static uint32_t video_osd_gen_table[VIDEO_OSD_GEN_WORDS];

extern const uint8_t video_osd_default_font[256][VIDEO_OSD_CHAR_BYTES];
// Characters whose flash slot is erased; they draw from the default font.
// Uploaded fonts use 01 for transparent pixels, so a written slot never reads
// as all ones.
static uint32_t video_osd_font_erased[256 / 32];

// Reserved by the linker script outside every loaded section, so firmware
// images leave an uploaded font in place.
extern "C" const uint8_t _osd_font_start[];
extern "C" const uint8_t _osd_font_end[];
#define video_osd_font _osd_font_start

static PIO video_osd_pio;
static uint32_t video_osd_sync_sm;
static uint32_t video_osd_pixel_sm;
static uint32_t video_osd_pixel_offset;
static dma_stream_t video_osd_dma = DMA_STREAM_INVALID;
// Claimed only when the block has a third state machine and room for the
// program; without it the OSD needs a camera.
static int32_t video_osd_gen_sm = -1;
static uint32_t video_osd_gen_offset;
static dma_stream_t video_osd_gen_dma = DMA_STREAM_INVALID;
static uint32_t video_osd_white_pin;

// Written by the vertical sync interrupt: the last broad pulse that started a
// field, and the last one that completed a camera field.
static uint32_t video_osd_last_field_start;
static volatile uint32_t video_osd_last_vsync;
static volatile uint32_t video_osd_field_us;
// Switched by the OSD thread, read by the vertical sync interrupt.
static volatile bool video_osd_free_run;
// The system last reported to the OSD layer; NTSC moves the bottom rows up.
static osd_system_t video_osd_system = OSD_SYS_NONE;

static const IRQn_Type video_osd_irqs[NUM_PIOS] = {
    PIO0_IRQ_1_IRQn,
    PIO1_IRQ_1_IRQn,
    PIO2_IRQ_1_IRQn,
};

static bool video_osd_slot_erased(uint8_t c) {
  const uint8_t *slot = &video_osd_font[c * VIDEO_OSD_FONT_SLOT];
  for (uint32_t i = 0; i < VIDEO_OSD_CHAR_BYTES; i++) {
    if (slot[i] != 0xFF)
      return false;
  }
  return true;
}

static void video_osd_update_erased(uint8_t c) {
  const uint32_t bit = 1u << (c % 32);
  if (video_osd_slot_erased(c))
    video_osd_font_erased[c / 32] |= bit;
  else
    video_osd_font_erased[c / 32] &= ~bit;
}

static const uint8_t *video_osd_glyph(uint8_t c) {
  if (video_osd_font_erased[c / 32] & (1u << (c % 32)))
    return video_osd_default_font[c];
  return &video_osd_font[c * VIDEO_OSD_FONT_SLOT];
}

// Both state machines share IRQ 4, so they need one block with two free
// state machines, room for both programs, and the pins in its window.
static bool video_osd_claim(uint32_t lowest, uint32_t highest, uint32_t *sync_offset) {
  static const PIO candidates[] = {pio2, pio1, pio0};
  for (PIO pio : candidates) {
    const uint32_t base = pio_get_gpio_base(pio);
    if (lowest < base || highest >= base + 32) {
      bool unused = true;
      for (uint32_t sm = 0; sm < NUM_PIO_STATE_MACHINES; sm++)
        unused &= !pio_sm_is_claimed(pio, sm);
      if (!unused || pio_set_gpio_base(pio, highest >= 32 ? 16 : 0) != PICO_OK)
        continue;
    }

    const int32_t sync = pio_claim_unused_sm(pio, false);
    if (sync < 0)
      continue;
    const int32_t pixel = pio_claim_unused_sm(pio, false);
    if (pixel < 0) {
      pio_sm_unclaim(pio, sync);
      continue;
    }
    if (!pio_can_add_program(pio, &video_osd_sync_program)) {
      pio_sm_unclaim(pio, sync);
      pio_sm_unclaim(pio, pixel);
      continue;
    }
    *sync_offset = pio_add_program(pio, &video_osd_sync_program);
    if (!pio_can_add_program(pio, &video_osd_pixels_program)) {
      pio_remove_program(pio, &video_osd_sync_program, *sync_offset);
      pio_sm_unclaim(pio, sync);
      pio_sm_unclaim(pio, pixel);
      continue;
    }
    video_osd_pixel_offset = pio_add_program(pio, &video_osd_pixels_program);

    video_osd_pio = pio;
    video_osd_sync_sm = sync;
    video_osd_pixel_sm = pixel;

    // The generator triggers the pixel state machine through IRQ 4, so it
    // shares the block too.
    video_osd_gen_sm = pio_claim_unused_sm(pio, false);
    if (video_osd_gen_sm >= 0) {
      if (pio_can_add_program(pio, &video_osd_generate_program)) {
        video_osd_gen_offset = pio_add_program(pio, &video_osd_generate_program);
      } else {
        pio_sm_unclaim(pio, video_osd_gen_sm);
        video_osd_gen_sm = -1;
      }
    }
    return true;
  }
  return false;
}

static void video_osd_restart_pixels() {
  PIO pio = video_osd_pio;
  const uint32_t sm = video_osd_pixel_sm;
  const uint32_t channel = dma_stream_channel(video_osd_dma);

  dma_channel_abort(channel);
  pio_sm_set_enabled(pio, sm, false);
  pio_sm_clear_fifos(pio, sm);
  pio_sm_restart(pio, sm);
  pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 0));
  pio_sm_exec(pio, sm, pio_encode_jmp(video_osd_pixel_offset));
  pio_interrupt_clear(pio, 4);
  pio_sm_set_enabled(pio, sm, true);

  dma_channel_transfer_from_buffer_now(channel, video_osd_framebuffer, VIDEO_OSD_LINES * VIDEO_OSD_LINE_WORDS);
}

static void video_osd_restart_generator() {
  dma_channel_transfer_from_buffer_now(dma_stream_channel(video_osd_gen_dma), video_osd_gen_table, VIDEO_OSD_GEN_WORDS);
}

static void video_osd_vsync_isr() {
  PIO pio = video_osd_pio;

  if (pio_interrupt_get(pio, video_osd_sync_sm)) {
    pio_interrupt_clear(pio, video_osd_sync_sm);

    const uint32_t now = time_micros();
    const uint32_t period = now - video_osd_last_field_start;
    if (period >= VIDEO_OSD_FIELD_MIN_US)
      video_osd_last_field_start = now;
    if (period >= VIDEO_OSD_FIELD_VALID_MIN_US && period <= VIDEO_OSD_FIELD_VALID_MAX_US) {
      video_osd_field_us = period;
      video_osd_last_vsync = now;
      // A field with missed lines leaves words queued; start each one clean.
      // While free-running the camera's fields are only recorded.
      if (!video_osd_free_run)
        video_osd_restart_pixels();
    }
  }

  // Each field ends with this interrupt, while the generator still has its
  // FIFO to play. After the second field the table DMA has finished, and it
  // restarts before the generator runs dry.
  if (video_osd_gen_sm >= 0 && pio_interrupt_get(pio, video_osd_gen_sm)) {
    pio_interrupt_clear(pio, video_osd_gen_sm);
    if (video_osd_free_run) {
      if (!dma_channel_is_busy(dma_stream_channel(video_osd_gen_dma)))
        video_osd_restart_generator();
      video_osd_restart_pixels();
    }
  }
}

static uint32_t *video_osd_gen_word(uint32_t *word, uint16_t instr, uint32_t cycles) {
  *word = (uint32_t)instr << 16 | (cycles - VIDEO_OSD_GEN_MIN_CYCLES) / 4;
  return word + 1;
}

static uint32_t *video_osd_gen_pulses(uint32_t *word, uint32_t low) {
  for (uint32_t i = 0; i < VIDEO_OSD_GEN_PULSES; i++) {
    word = video_osd_gen_word(word, pio_encode_set(pio_pins, 0), low);
    word = video_osd_gen_word(word, pio_encode_set(pio_pins, 1), VIDEO_OSD_GEN_HALF_LINE - low);
  }
  return word;
}

static uint32_t *video_osd_gen_lines(uint32_t *word, uint32_t lines) {
  for (uint32_t line = 0; line < lines; line++) {
    word = video_osd_gen_word(word, pio_encode_set(pio_pins, 0), VIDEO_OSD_GEN_HSYNC);
    word = video_osd_gen_word(word, pio_encode_set(pio_pins, 1), VIDEO_OSD_GEN_ACTIVE - VIDEO_OSD_GEN_HSYNC);
    word = video_osd_gen_word(word, pio_encode_irq_set(false, 4), VIDEO_OSD_GEN_LINE - VIDEO_OSD_GEN_ACTIVE);
  }
  return word;
}

static void video_osd_gen_build() {
  uint32_t *word = video_osd_gen_table;

  word = video_osd_gen_pulses(word, VIDEO_OSD_GEN_BROAD);
  word = video_osd_gen_pulses(word, VIDEO_OSD_GEN_EQUALIZING);
  word = video_osd_gen_lines(word, VIDEO_OSD_GEN_FIRST_LINES);
  word = video_osd_gen_pulses(word, VIDEO_OSD_GEN_EQUALIZING);
  word = video_osd_gen_word(word, pio_encode_irq_set(true, 0), VIDEO_OSD_GEN_MIN_CYCLES);

  word = video_osd_gen_pulses(word, VIDEO_OSD_GEN_BROAD);
  word = video_osd_gen_pulses(word, VIDEO_OSD_GEN_EQUALIZING);
  word = video_osd_gen_word(word, pio_encode_set(pio_pins, 1), VIDEO_OSD_GEN_HALF_LINE);
  word = video_osd_gen_lines(word, VIDEO_OSD_GEN_SECOND_LINES);
  word = video_osd_gen_word(word, pio_encode_set(pio_pins, 0), VIDEO_OSD_GEN_HSYNC);
  word = video_osd_gen_word(word, pio_encode_set(pio_pins, 1), VIDEO_OSD_GEN_HALF_LINE - VIDEO_OSD_GEN_HSYNC);
  word = video_osd_gen_pulses(word, VIDEO_OSD_GEN_EQUALIZING);
  video_osd_gen_word(word, pio_encode_irq_set(true, 0), VIDEO_OSD_GEN_MIN_CYCLES);
}

// Runs on the OSD thread. The pixel state machine drives both pins with a
// camera and only the white pin while the generator owns enable.
static void video_osd_set_free_run(bool free_run) {
  PIO pio = video_osd_pio;
  const uint32_t white = video_osd_white_pin;
  const uint32_t pins = free_run ? 1 : 2;

  ATOMIC_BLOCK(EXTI_PRIORITY) {
    video_osd_free_run = free_run;

    pio_sm_set_enabled(pio, video_osd_gen_sm, false);
    dma_channel_abort(dma_stream_channel(video_osd_gen_dma));
    pio_sm_clear_fifos(pio, video_osd_gen_sm);
    pio_sm_restart(pio, video_osd_gen_sm);
    pio_sm_exec(pio, video_osd_gen_sm, pio_encode_jmp(video_osd_gen_offset));
    pio_interrupt_clear(pio, video_osd_gen_sm);

    pio_sm_set_enabled(pio, video_osd_pixel_sm, false);
    pio_sm_set_out_pins(pio, video_osd_pixel_sm, white, pins);
    pio_sm_set_set_pins(pio, video_osd_pixel_sm, white, pins);
    video_osd_restart_pixels();

    if (free_run) {
      video_osd_restart_generator();
      pio_sm_set_enabled(pio, video_osd_gen_sm, true);
    }
  }
}

bool video_osd_init() {
  if ((uint32_t)(_osd_font_end - _osd_font_start) < VIDEO_OSD_FONT_SIZE)
    return false;
  for (uint32_t c = 0; c < 256; c++)
    video_osd_update_erased(c);

  const uint32_t white = gpio_pin_defs[target.video_osd.white].pin_index;
  const uint32_t enable = gpio_pin_defs[target.video_osd.enable].pin_index;
  const uint32_t sync = gpio_pin_defs[target.video_osd.sync].pin_index;
  // Both outputs are written by one OUT: enable must follow white.
  if (enable != white + 1)
    return false;

  uint32_t sync_offset = 0;
  if (!video_osd_claim(MIN(white, sync), MAX(enable, sync), &sync_offset))
    return false;

  video_osd_dma = dma_claim_unused(DMA_DEVICE_OSD);
  if (video_osd_dma == DMA_STREAM_INVALID)
    return false;

  if (video_osd_gen_sm >= 0) {
    video_osd_gen_dma = dma_claim_unused(DMA_DEVICE_OSD);
    if (video_osd_gen_dma == DMA_STREAM_INVALID) {
      pio_remove_program(video_osd_pio, &video_osd_generate_program, video_osd_gen_offset);
      pio_sm_unclaim(video_osd_pio, video_osd_gen_sm);
      video_osd_gen_sm = -1;
    }
  }

  PIO pio = video_osd_pio;
  video_osd_white_pin = white;
  const float clkdiv = (float)SYS_CLOCK_FREQ_HZ / VIDEO_OSD_PIO_HZ;

  gpio_init(sync);
  gpio_set_dir(sync, false);
  gpio_set_pulls(sync, false, false);

  const uint32_t outputs[] = {white, enable};
  for (const uint32_t pin : outputs) {
    gpio_put(pin, false);
    gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
    pio_gpio_init(pio, pin);
  }
  // Enable drives the analog switch select: a soft edge injects less charge
  // into the video path when it switches between camera and overlay.
  gpio_set_slew_rate(enable, GPIO_SLEW_RATE_SLOW);
  gpio_set_drive_strength(enable, GPIO_DRIVE_STRENGTH_2MA);

  pio_sm_config config = video_osd_pixels_program_get_default_config(video_osd_pixel_offset);
  sm_config_set_out_pins(&config, white, 2);
  sm_config_set_set_pins(&config, white, 2);
  sm_config_set_out_shift(&config, true, false, 32);
  sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);
  sm_config_set_clkdiv(&config, clkdiv);
  pio_sm_init(pio, video_osd_pixel_sm, video_osd_pixel_offset, &config);
  pio_sm_set_consecutive_pindirs(pio, video_osd_pixel_sm, white, 2, true);

  config = video_osd_sync_program_get_default_config(sync_offset);
  sm_config_set_in_pins(&config, sync);
  sm_config_set_jmp_pin(&config, sync);
  sm_config_set_clkdiv(&config, clkdiv);
  pio_sm_init(pio, video_osd_sync_sm, sync_offset, &config);
  // ISR keeps the count that times IRQ 4 from the sync pulse's middle, OSR
  // the line count for every vertical sync.
  pio_sm_put(pio, video_osd_sync_sm, VIDEO_OSD_SYNC_MIDDLE);
  pio_sm_exec(pio, video_osd_sync_sm, pio_encode_pull(false, true));
  pio_sm_exec(pio, video_osd_sync_sm, pio_encode_mov(pio_isr, pio_osr));
  pio_sm_put(pio, video_osd_sync_sm, VIDEO_OSD_LINES);
  pio_sm_exec(pio, video_osd_sync_sm, pio_encode_pull(false, true));

  const uint32_t channel = dma_stream_channel(video_osd_dma);
  dma_channel_config dma = dma_channel_get_default_config(channel);
  channel_config_set_transfer_data_size(&dma, DMA_SIZE_32);
  channel_config_set_dreq(&dma, pio_get_dreq(pio, video_osd_pixel_sm, true));
  channel_config_set_read_increment(&dma, true);
  channel_config_set_write_increment(&dma, false);
  dma_channel_configure(channel, &dma, &pio->txf[video_osd_pixel_sm], video_osd_framebuffer, 0, false);

  memset(video_osd_framebuffer, 0, sizeof(video_osd_framebuffer));

  if (video_osd_gen_sm >= 0) {
    video_osd_gen_build();

    config = video_osd_generate_program_get_default_config(video_osd_gen_offset);
    sm_config_set_set_pins(&config, enable, 1);
    sm_config_set_out_shift(&config, true, true, 32);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&config, clkdiv);
    pio_sm_init(pio, video_osd_gen_sm, video_osd_gen_offset, &config);

    const uint32_t gen_channel = dma_stream_channel(video_osd_gen_dma);
    dma = dma_channel_get_default_config(gen_channel);
    channel_config_set_transfer_data_size(&dma, DMA_SIZE_32);
    channel_config_set_dreq(&dma, pio_get_dreq(pio, video_osd_gen_sm, true));
    channel_config_set_read_increment(&dma, true);
    channel_config_set_write_increment(&dma, false);
    dma_channel_configure(gen_channel, &dma, &pio->txf[video_osd_gen_sm], video_osd_gen_table, 0, false);

    pio_set_irqn_source_enabled(pio, 1, static_cast<pio_interrupt_source_t>(pis_interrupt0 + video_osd_gen_sm), true);
  }

  pio_set_irqn_source_enabled(pio, 1, static_cast<pio_interrupt_source_t>(pis_interrupt0 + video_osd_sync_sm), true);
  interrupt_enable(video_osd_irqs[pio_get_index(pio)], EXTI_PRIORITY);

  pio_sm_set_enabled(pio, video_osd_pixel_sm, true);
  pio_sm_set_enabled(pio, video_osd_sync_sm, true);
  return true;
}

bool video_osd_is_ready() {
  return true;
}

// MAX7456 font bytes hold four pixels, most significant first: 00 black, 10
// white, 01 and 11 transparent. The tables turn one into four pixel codes,
// first pixel in the low bits as the pixel state machine shifts them out.
struct video_osd_font_lut_t {
  uint8_t codes[2][256];

  constexpr video_osd_font_lut_t()
      : codes() {
    for (uint32_t invert = 0; invert < 2; invert++) {
      const uint32_t black = invert ? VIDEO_OSD_WHITE : VIDEO_OSD_BLACK;
      const uint32_t white = invert ? VIDEO_OSD_BLACK : VIDEO_OSD_WHITE;
      for (uint32_t byte = 0; byte < 256; byte++) {
        uint32_t out = 0;
        for (uint32_t p = 0; p < 4; p++) {
          const uint32_t bits = (byte >> (6 - 2 * p)) & 0b11;
          const uint32_t code = bits == 0b00 ? black : bits == 0b10 ? white : VIDEO_OSD_TRANSPARENT;
          out |= code << (2 * p);
        }
        codes[invert][byte] = out;
      }
    }
  }
};

static constexpr video_osd_font_lut_t video_osd_font_lut;

// A glyph row is 24 bits of pixel codes; at its offset in the line it spans
// at most two framebuffer words, written whole so the DMA never reads a half
// updated one.
static void video_osd_draw_char(uint8_t col, uint8_t row, uint8_t c, bool invert) {
  const uint8_t *glyph = video_osd_glyph(c);
  const uint8_t *lut = video_osd_font_lut.codes[invert];

  const uint32_t x0 = VIDEO_OSD_LEFT_PIXELS + col * VIDEO_OSD_CHAR_WIDTH;
  const uint32_t word = x0 / 16;
  const uint32_t shift = (x0 % 16) * 2;
  const uint64_t mask = (uint64_t)0xFFFFFF << shift;
  const bool spans = shift + 24 > 32;

  uint32_t(*line)[VIDEO_OSD_LINE_WORDS] = &video_osd_framebuffer[VIDEO_OSD_TOP_LINES + row * VIDEO_OSD_CHAR_HEIGHT];
  for (uint32_t r = 0; r < VIDEO_OSD_CHAR_HEIGHT; r++, line++, glyph += 3) {
    const uint64_t bits = (uint64_t)(lut[glyph[0]] | lut[glyph[1]] << 8 | lut[glyph[2]] << 16) << shift;
    uint32_t *dst = &(*line)[word];
    dst[0] = (dst[0] & ~(uint32_t)mask) | (uint32_t)bits;
    if (spans)
      dst[1] = (dst[1] & ~(uint32_t)(mask >> 32)) | (uint32_t)(bits >> 32);
  }
}

bool video_osd_clear_async() {
  memset(video_osd_framebuffer, 0, sizeof(video_osd_framebuffer));
  return true;
}

// Called on every OSD thread pass; free-running starts once the camera's sync
// has been missing for the timeout and stops at the first camera field.
osd_system_t video_osd_check_system() {
  const uint32_t last = video_osd_last_vsync;
  const bool camera = last != 0 && time_micros() - last <= VIDEO_OSD_SYNC_TIMEOUT_US;
  if (video_osd_gen_sm >= 0 && camera == video_osd_free_run)
    video_osd_set_free_run(!camera);

  if (video_osd_free_run)
    video_osd_system = OSD_SYS_PAL;
  else if (!camera)
    video_osd_system = OSD_SYS_NONE;
  else
    video_osd_system = video_osd_field_us >= VIDEO_OSD_PAL_MIN_PERIOD_US ? OSD_SYS_PAL : OSD_SYS_NTSC;
  return video_osd_system;
}

// splash screen
void video_osd_intro() {
  uint8_t buffer[24];
  for (uint8_t row = 0; row < 4; row++) {
    const uint8_t start = 160 + row * 24;
    for (uint8_t i = 0; i < 24; i++) {
      buffer[i] = start + i;
    }
    video_osd_push_string(OSD_ATTR_TEXT, 3, row + 5, buffer, 24);
  }
}

uint32_t video_osd_can_fit() {
  return SD_COLS;
}

bool video_osd_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size) {
  // An NTSC field ends after row 12; like the MAX7456 driver, lift the rows
  // below it by two so a layout made for PAL stays on screen.
  if (video_osd_system == OSD_SYS_NTSC && y > 12)
    y -= 2;
  if (y >= SD_ROWS)
    return true;
  for (uint8_t i = 0; i < size && x + i < SD_COLS; i++)
    video_osd_draw_char(x + i, y, data[i], attr & OSD_ATTR_INVERT);
  return true;
}

bool video_osd_flush() {
  return true;
}

void video_osd_read_character(uint8_t addr, uint8_t *out, const uint8_t size) {
  memcpy(out, video_osd_glyph(addr), MIN(size, VIDEO_OSD_CHAR_BYTES));
}

static uint32_t video_osd_flash_offset(const uint8_t *ptr) {
  return (uint32_t)ptr - XIP_BASE;
}

// Flash programs whole 256-byte pages and only clears bits: an erased slot is
// programmed in its page, anything else rewrites its 4 KiB sector. Runs from
// the USB task while disarmed; the SDK flash routines execute from RAM with
// interrupts held off and the second core idle.
void video_osd_write_character(uint8_t addr, const uint8_t *in, const uint8_t size) {
  uint8_t slot[VIDEO_OSD_FONT_SLOT];
  memset(slot, 0xFF, sizeof(slot));
  memcpy(slot, in, MIN(size, VIDEO_OSD_CHAR_BYTES));

  const uint8_t *current = &video_osd_font[addr * VIDEO_OSD_FONT_SLOT];
  if (memcmp(current, slot, sizeof(slot)) == 0)
    return;

  bool erased = true;
  for (uint32_t i = 0; i < VIDEO_OSD_FONT_SLOT; i++)
    erased &= current[i] == 0xFF;

  if (erased) {
    const uint8_t *page = (const uint8_t *)((uint32_t)current & ~(FLASH_PAGE_SIZE - 1));
    uint8_t buffer[FLASH_PAGE_SIZE];
    memcpy(buffer, page, FLASH_PAGE_SIZE);
    memcpy(buffer + (current - page), slot, sizeof(slot));
    ATOMIC_BLOCK_ALL {
      flash_range_program(video_osd_flash_offset(page), buffer, FLASH_PAGE_SIZE);
    }
    video_osd_update_erased(addr);
    return;
  }

  const uint8_t *sector = (const uint8_t *)((uint32_t)current & ~(FLASH_SECTOR_SIZE - 1));
  uint8_t *buffer = (uint8_t *)malloc(FLASH_SECTOR_SIZE);
  if (buffer == NULL)
    return;
  memcpy(buffer, sector, FLASH_SECTOR_SIZE);
  memcpy(buffer + (current - sector), slot, sizeof(slot));
  ATOMIC_BLOCK_ALL {
    flash_range_erase(video_osd_flash_offset(sector), FLASH_SECTOR_SIZE);
    flash_range_program(video_osd_flash_offset(sector), buffer, FLASH_SECTOR_SIZE);
  }
  free(buffer);
  video_osd_update_erased(addr);
}

extern "C" {
void PIO0_IRQ_1_Handler() { video_osd_vsync_isr(); }
void PIO1_IRQ_1_Handler() { video_osd_vsync_isr(); }
void PIO2_IRQ_1_Handler() { video_osd_vsync_isr(); }
}

#endif
