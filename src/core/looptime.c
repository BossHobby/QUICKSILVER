#include "core/looptime.h"

#include "core/project.h"
#include "driver/time.h"
#include "flight/control.h"

uint8_t looptime_warning = 0;

static uint32_t lastlooptime;

void looptime_init() {
  // attempt 8k looptime for f405 or 4k looptime for f411
  state.looptime = LOOPTIME * 1e-6;
  state.looptime_autodetect = LOOPTIME;

  looptime_reset();
}

void looptime_reset() {
  lastlooptime = time_micros();
}

static void looptime_auto_detect() {
  // looptime_autodetect sequence
  static uint8_t loop_delay = 0;
  if (loop_delay < 200) {
    loop_delay++;
  }

  static float loop_avg = 0;
  static uint8_t loop_counter = 0;

  if (loop_delay >= 200 && loop_counter < 200) {
    loop_avg += state.looptime_us;
    loop_counter++;
  }

  if (loop_counter == 200) {
    loop_avg /= 200;

    if (loop_avg < 130.f) {
      state.looptime_autodetect = LOOPTIME_8K;
    } else if (loop_avg < 255.f) {
      state.looptime_autodetect = LOOPTIME_4K;
    } else {
      state.looptime_autodetect = LOOPTIME_2K;
    }

    loop_counter++;
  }

  if (loop_counter == 201) {
    static uint8_t blown_loop_counter;

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

void looptime_update(uint32_t time) {
  state.looptime_us = ((uint32_t)(time - lastlooptime));
  lastlooptime = time;

  if (state.looptime_us <= 0) {
    state.looptime_us = 1;
  }
  state.looptime = state.looptime_us * 1e-6f;

  // max loop 20ms
  if (state.looptime_us > 20000) {
    failloop(FAILLOOP_LOOPTIME);
  }

  looptime_auto_detect();

  state.uptime += state.looptime;
  if (flags.arm_state) {
    state.armtime += state.looptime;
  }
}