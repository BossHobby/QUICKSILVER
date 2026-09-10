#include "core/looptime.h"

#include "control/control.h"
#include "core/project.h"
#include "driver/gyro/gyro.h"
#include "driver/time.h"
#include "util/util.h"

uint8_t looptime_warning = 0;

static uint32_t last_loop_cycles;
static bool skip_looptime_update = false;
static float minimum_period;
static float runtime_sum;
static uint16_t runtime_samples;
static bool window_has_headroom = true;

static void looptime_reset_window() {
  runtime_sum = 0;
  runtime_samples = 0;
  window_has_headroom = true;
}

void looptime_init() {
#ifdef USE_GYRO
  float target = gyro_update_period();
#else
  float target = LOOPTIME_MAX;
#endif
  while (target < LOOPTIME_MAX)
    target *= 2.0f;
  state.looptime = target * 1e-6f;
  state.looptime_us = target;
  state.looptime_autodetect = target;
  minimum_period = target;
  looptime_reset_window();
  skip_looptime_update = false;

  last_loop_cycles = time_cycles();
}

void looptime_reset() {
  skip_looptime_update = true;
  looptime_reset_window();
}

static void looptime_auto_detect() {
  if (state.loop_counter < 200) {
    // skip first couple of loops
    return;
  }
  if (skip_looptime_update) {
    skip_looptime_update = false;
    return;
  }

  // max loop 20ms
  if (state.looptime_us > 20000) {
    failloop(FAILLOOP_LOOPTIME);
  }

  // Measure work before the busy-wait. Require a complete 200-loop window
  // with 10 us spare at the faster rate before reducing the period.
  const float faster_period = MAX(minimum_period, state.looptime_autodetect * 0.5f);
  runtime_sum += state.cpu_load;
  window_has_headroom &= state.cpu_load + 10.0f <= faster_period;
  if (++runtime_samples < 200) {
    return;
  }

  const float previous_period = state.looptime_autodetect;
  if (runtime_sum / runtime_samples > previous_period + 5.0f) {
    state.looptime_autodetect = MIN(MAX(500.0f, minimum_period), previous_period * 2.0f);
    looptime_warning++;
  } else if (window_has_headroom) {
    state.looptime_autodetect = faster_period;
  }
  looptime_reset_window();
  if (state.looptime_autodetect != previous_period) {
    control_filter_update(false);
  }
}

#ifdef PIO_UNIT_TESTING
void looptime_test_auto_detect() {
  looptime_auto_detect();
}
#endif

uint32_t looptime_update() {
  state.cpu_load = CYCLES_TO_US(time_cycles() - last_loop_cycles);

  const uint32_t delay = US_TO_CYCLES(state.looptime_autodetect);
  while ((time_cycles() - last_loop_cycles) < delay)
    __NOP();

  state.looptime_us = CYCLES_TO_US(time_cycles() - last_loop_cycles);
  state.looptime = state.looptime_us * 1e-6f;
  // looptime_inverse is the loop frequency (1/looptime)
  if (state.looptime > 0.0f) {
    state.looptime_inverse = 1.0f / state.looptime;
  } else {
    state.looptime_inverse = 0.0f;
  }

  state.loop_counter++;
  last_loop_cycles = time_cycles();

  looptime_auto_detect();

  state.uptime += state.looptime;
  if (flags.arm_state) {
    state.armtime += state.looptime;
  }
  return last_loop_cycles;
}
