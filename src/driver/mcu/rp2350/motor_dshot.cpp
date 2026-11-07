#include "driver/motor_dshot.h"

#include <hardware/pio.h>

#include "control/control.h"
#include "core/failloop.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"

#ifdef USE_MOTOR_DSHOT

#include "dshot.pio.h"

// Output slot n runs on state machine n % 4 of PIO0 (slots 0-3) or PIO1
// (slots 4-7). PIO2 stays free for other PIO drivers.
static constexpr uint32_t DSHOT_SM_PER_PIO = 4;
static_assert(MOTOR_PIN_MAX <= 2 * DSHOT_SM_PER_PIO);

// Cycles per third of a DShot bit, as laid out in dshot.pio.
static constexpr uint32_t DSHOT_SLOT_CYCLES = 5;

static volatile DMA_RAM uint16_t dshot_reply[MOTOR_PIN_MAX][GCR_DMA_BUFFER_SIZE];

// Claimed on first init and kept: ESC passthrough re-runs motor_init().
static int32_t dshot_program_offset[2] = {-1, -1};
static dma_stream_t dshot_reply_dma[MOTOR_PIN_MAX];
static uint32_t dshot_claimed_mask = 0;

static uint32_t dshot_frame_mask = 0;

static PIO dshot_pio(uint32_t slot) {
  return slot < DSHOT_SM_PER_PIO ? pio0 : pio1;
}

static uint32_t dshot_sm(uint32_t slot) {
  return slot % DSHOT_SM_PER_PIO;
}

// A PIO block reaches 32 consecutive pins, from GPIO0 or GPIO16.
static void dshot_set_gpio_base(PIO pio, uint32_t mask) {
  uint32_t lowest = NUM_BANK0_GPIOS;
  uint32_t highest = 0;
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (!(mask & (1U << i)) || dshot_pio(i) != pio)
      continue;
    const uint32_t index = gpio_pin_defs[target.outputs[i].pin].pin_index;
    lowest = MIN(lowest, index);
    highest = MAX(highest, index);
  }
  if (highest < 32)
    return;
  if (lowest < 16)
    failloop(FAILLOOP_OUTPUT);
  pio_set_gpio_base(pio, 16);
}

static void dshot_init_slot(uint32_t slot) {
  PIO pio = dshot_pio(slot);
  const uint32_t sm = dshot_sm(slot);
  const uint32_t pio_index = pio_get_index(pio);
  const uint32_t pin = gpio_pin_defs[target.outputs[slot].pin].pin_index;
  const bool bidir = profile.motor.dshot_telemetry;

  if (!(dshot_claimed_mask & (1U << slot))) {
    pio_sm_claim(pio, sm);
    // Kept even without bidirectional DShot, which a profile can enable.
    dshot_reply_dma[slot] = dma_claim_unused(DMA_DEVICE_DSHOT_CH1);
    if (dshot_reply_dma[slot] == DMA_STREAM_INVALID)
      failloop(FAILLOOP_DMA);
    dshot_claimed_mask |= 1U << slot;
  }
  if (dshot_program_offset[pio_index] < 0)
    dshot_program_offset[pio_index] = pio_add_program(pio, &dshot_program);
  const uint32_t offset = dshot_program_offset[pio_index];

  pio_sm_set_enabled(pio, sm, false);

  pio_gpio_init(pio, pin);
  gpio_set_pulls(pin, bidir, !bidir);
  gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_outover(pin, bidir ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

  pio_sm_config config = dshot_program_get_default_config(offset);
  sm_config_set_set_pins(&config, pin, 1);
  sm_config_set_out_pins(&config, pin, 1);
  sm_config_set_in_pins(&config, pin);
  // MSB first; the frame ends once the 16 packet bits are out.
  sm_config_set_out_shift(&config, false, false, 16);
  // Every sample is pushed on its own.
  sm_config_set_in_shift(&config, false, true, 1);
  sm_config_set_clkdiv(&config, (float)SYS_CLOCK_FREQ_HZ / (3 * DSHOT_SLOT_CYCLES * DSHOT_TIME * 1000));
  pio_sm_init(pio, sm, offset, &config);

  // Load Y with the reply sample count, then drive the idle level.
  pio_sm_put(pio, sm, bidir ? GCR_DMA_BUFFER_SIZE - 1 : 0);
  pio_sm_exec(pio, sm, pio_encode_pull(false, true));
  pio_sm_exec(pio, sm, pio_encode_mov(pio_y, pio_osr));
  pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 0));
  pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 1));

  const uint32_t channel = dma_stream_channel(dshot_reply_dma[slot]);
  dma_channel_config dma = dma_channel_get_default_config(channel);
  channel_config_set_transfer_data_size(&dma, DMA_SIZE_16);
  channel_config_set_dreq(&dma, pio_get_dreq(pio, sm, false));
  channel_config_set_read_increment(&dma, false);
  channel_config_set_write_increment(&dma, true);
  dma_channel_configure(channel, &dma, dshot_reply[slot], &pio->rxf[sm], GCR_DMA_BUFFER_SIZE, false);

  pio_sm_set_enabled(pio, sm, true);
}

void dshot_init_outputs(uint32_t mask) {
  dshot_set_gpio_base(pio0, mask);
  dshot_set_gpio_base(pio1, mask);
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (mask & (1U << i))
      dshot_init_slot(i);
  }
  dshot_frame_mask = 0;
}

// Replies of the previous frame; cold for the same reason as on the other MCUs.
COLD_CODE static void dshot_read_telemetry(uint32_t active) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (active & dshot_frame_mask & (1U << i))
      state.dshot_rpm[i] = dshot_decode_gcr((uint16_t *)dshot_reply[i], 1);
  }
}

void dshot_send_frame(const uint16_t *packets, uint32_t active) {
  const bool bidir = profile.motor.dshot_telemetry;
  if (bidir)
    dshot_read_telemetry(active);

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (!(active & (1U << i)))
      continue;
    if (bidir) {
      const uint32_t channel = dma_stream_channel(dshot_reply_dma[i]);
      dma_channel_set_write_addr(channel, dshot_reply[i], true);
    }
    pio_sm_put(dshot_pio(i), dshot_sm(i), (uint32_t)packets[i] << 16);
  }
  dshot_frame_mask = active;
}

// Each state machine is back at its pull with nothing queued, and every
// reply sample has reached memory.
bool dshot_frame_done() {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (!(dshot_frame_mask & (1U << i)))
      continue;
    PIO pio = dshot_pio(i);
    const uint32_t sm = dshot_sm(i);
    if (!pio_sm_is_tx_fifo_empty(pio, sm) || pio_sm_get_pc(pio, sm) != dshot_program_offset[pio_get_index(pio)])
      return false;
    if (dma_channel_is_busy(dma_stream_channel(dshot_reply_dma[i])))
      return false;
  }
  return true;
}

#endif
