#include "core/looptime.h"

#include "core/project.h"
#include "driver/gyro/gyro.h"
#include "driver/time.h"
#include "flight/control.h"
#include "util/util.h"

uint8_t looptime_warning = 0;

static uint32_t last_loop_cycles;
static bool skip_looptime_update = false;

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

  last_loop_cycles = time_cycles();
}

void looptime_reset() {
  skip_looptime_update = true;
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

  static float loop_avg = 0;
  static uint8_t loop_counter = 0;
  if (loop_counter < 200) {
    loop_avg += state.looptime_us;
    loop_counter++;
  }

  if (loop_counter == 200) {
    loop_avg /= 200;
    if (loop_avg > (state.looptime_autodetect + 5.0f)) {
      state.looptime_autodetect = min(500, state.looptime_autodetect * 2.0f);
    } else if (loop_avg < (state.looptime_autodetect * 0.5f)) {
      state.looptime_autodetect = max(LOOPTIME_MAX, state.looptime_autodetect * 0.5f);
    }
    loop_counter++;
  }

  if (loop_counter == 201) {
    static uint8_t blown_loop_counter = 0;

    if (state.cpu_load > state.looptime_autodetect + 5) {
      blown_loop_counter++;
    }

    if (blown_loop_counter > 100) {
      blown_loop_counter = 0;
      loop_counter = 0;
      loop_avg = 0;
      looptime_warning++;
    }
  }
}

void looptime_update() {
  state.cpu_load = CYCLES_TO_US(time_cycles() - last_loop_cycles);

  const uint32_t delay = US_TO_CYCLES(state.looptime_autodetect);
  while ((time_cycles() - last_loop_cycles) < delay)
    __NOP();

  state.looptime_us = CYCLES_TO_US(time_cycles() - last_loop_cycles);
  state.looptime = state.looptime_us * 1e-6f;
  // 0.0032f is there for legacy purposes, should be 0.001f = looptime
  state.timefactor = 0.0032f / state.looptime;
  state.loop_counter++;

  last_loop_cycles = time_cycles();

  looptime_auto_detect();

  state.uptime += state.looptime;
  if (flags.arm_state) {
    state.armtime += state.looptime;
  }
}