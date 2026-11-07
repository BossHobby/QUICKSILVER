#include "control/sixaxis.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "control/control.h"
#include "control/sixaxis.h"
#include "core/debug.h"
#include "core/flash.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/gyro/gyro.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "io/blackbox.h"
#include "io/led.h"
#include "util/filter.h"
#include "util/sdft.h"
#include "util/util.h"

#define CAL_INTERVAL 2000           // time between measurements in us
#define CAL_COUNT 500               // count of samples
#define CAL_TRIES (CAL_COUNT * 100) // number of attempts

#define GYRO_BIAS_LIMIT 800
#define ACCEL_BIAS_LIMIT 800

// gyro has +-2000 divided over 16bit.
#define GYRO_RANGE (1.f / (65536.f / 4000.f))
#define ACCEL_RANGE (1.f / 2048.0f)

#ifdef USE_GYRO

static float gyrocal[3];
// Supported orientations are 45 degree yaw steps plus an optional 180 degree roll flip,
// so the board rotation reduces to a planar roll/pitch rotation and a sign.
static float rot_cos = 1.0f;
static float rot_sin = 0.0f;
static float rot_flip = 1.0f;

static filter_t<vec3_t> filter[FILTER_MAX_SLOTS];

static sdft_t gyro_sdft[SDFT_AXES];
static filter_biquad_notch_t notch_filter[SDFT_AXES][SDFT_PEAKS];

void sixaxis_orientation_update() {
  const uint8_t orientation = profile.motor.gyro_orientation;

  // yaw in clockwise 45 degree steps
  int32_t steps = 0;
  if (orientation & GYRO_ROTATE_90_CW)
    steps += 2;
  if (orientation & GYRO_ROTATE_90_CCW)
    steps -= 2;
  if (orientation & GYRO_ROTATE_45_CW)
    steps += 1;
  if (orientation & GYRO_ROTATE_45_CCW)
    steps -= 1;
  if (orientation & GYRO_ROTATE_180)
    steps += 4;
  if (orientation & GYRO_FLIP_180)
    steps += 4;

  static constexpr float H = M_SQRT1_2; // cos(45 deg)
  static constexpr struct {
    float cos, sin;
  } yaw_steps[8] = {
      {1.0f, 0.0f},  // 0
      {H, H},        // 45
      {0.0f, 1.0f},  // 90
      {-H, H},       // 135
      {-1.0f, 0.0f}, // 180
      {-H, -H},      // 225
      {0.0f, -1.0f}, // 270
      {H, -H},       // 315
  };

  rot_cos = yaw_steps[steps & 7].cos;
  rot_sin = yaw_steps[steps & 7].sin;
  rot_flip = (orientation & GYRO_FLIP_180) ? -1.0f : 1.0f;
}

void sixaxis_filter_update(bool reset) {
  for (uint8_t i = 0; i < SDFT_AXES; i++) {
    sdft_update_period(&gyro_sdft[i], state.looptime_autodetect);
  }
  for (uint8_t i = 0; i < FILTER_MAX_SLOTS; i++) {
    if (reset) {
      filter[i].init(profile.filter.gyro[i].type, profile.filter.gyro[i].cutoff_freq, state.looptime_autodetect);
    } else {
      filter[i].coeff(profile.filter.gyro[i].type, profile.filter.gyro[i].cutoff_freq, state.looptime_autodetect);
    }
  }
}

void sixaxis_init() {
  target_info.gyro_id = gyro_init();
  if (target_info.gyro_id == GYRO_TYPE_INVALID) {
    failloop(FAILLOOP_GYRO);
  }
  sixaxis_orientation_update();
  sixaxis_filter_update(true);

  for (uint8_t i = 0; i < SDFT_AXES; i++) {
    for (uint8_t j = 0; j < SDFT_PEAKS; j++) {
      notch_filter[i][j].init(0, state.looptime_autodetect);
    }
  }
}

static vec3_t sixaxis_apply_orientation(vec3_t v) {
  const float pitch = v.pitch * rot_flip;
  return (vec3_t){
      .roll = rot_cos * v.roll + rot_sin * pitch,
      .pitch = rot_cos * pitch - rot_sin * v.roll,
      .yaw = rot_flip * v.yaw,
  };
}

HOT_CODE void sixaxis_read() {
  const gyro_data_t data = gyro_read();

  const vec3_t accel = sixaxis_apply_orientation(data.accel);
  // swap pitch and roll to match gyro
  state.accel_raw.roll = (accel.pitch - flash_storage.accelcal[1]) * ACCEL_RANGE;
  state.accel_raw.pitch = (accel.roll - flash_storage.accelcal[0]) * ACCEL_RANGE;
  state.accel_raw.yaw = (accel.yaw - flash_storage.accelcal[2]) * ACCEL_RANGE;

  state.gyro_raw.roll = data.gyro.roll - gyrocal[0];
  state.gyro_raw.pitch = data.gyro.pitch - gyrocal[1];
  state.gyro_raw.yaw = data.gyro.yaw - gyrocal[2];
  state.gyro_raw = sixaxis_apply_orientation(state.gyro_raw);
  state.gyro.roll = state.gyro_raw.roll = state.gyro_raw.roll * GYRO_RANGE * DEGTORAD;
  state.gyro.pitch = state.gyro_raw.pitch = -state.gyro_raw.pitch * GYRO_RANGE * DEGTORAD;
  state.gyro.yaw = state.gyro_raw.yaw = -state.gyro_raw.yaw * GYRO_RANGE * DEGTORAD;

  state.gyro_temp = data.temp;

  if (profile.filter.gyro_dynamic_notch_enable) {
    // we are updating the sdft state per axis per loop
    // eg. axis 0 step 0, axis 0 step 1 ... axis 2, step 3

    // 3 == idle state
    static uint8_t current_axis = 3;

    for (uint32_t i = 0; i < 3; i++) {
      // push new samples every loop
      if (sdft_push(&gyro_sdft[i], state.gyro.axis[i]) && current_axis == 3) {
        // a batch just finished and we are in idle state
        // kick off a new round of axis updates
        current_axis = 0;
      }
    }

    // if we have a batch ready, start walking the sdft steps for a given axis
    if (current_axis < 3 && sdft_update(&gyro_sdft[current_axis])) {
      // once all sdft update steps are done, we update the filters and continue to the next axis
      for (uint32_t p = 0; p < SDFT_PEAKS; p++) {
        notch_filter[current_axis][p].coeff(gyro_sdft[current_axis].notch_hz[p], state.looptime_autodetect);
        blackbox_set_debug(BBOX_DEBUG_DYN_NOTCH, current_axis * SDFT_PEAKS + p, gyro_sdft[current_axis].notch_hz[p]);
      }
      // on the last axis we increment this to the idle state 3
      current_axis++;
    }
  }

  state.gyro = filter[0].step(state.gyro);
  state.gyro = filter[1].step(state.gyro);

  if (profile.filter.gyro_dynamic_notch_enable) {
    for (uint32_t i = 0; i < 3; i++) {
      for (uint32_t p = 0; p < SDFT_PEAKS; p++) {
        state.gyro.axis[i] = notch_filter[i][p].step(state.gyro.axis[i]);
      }
    }
  }

  state.gyro_delta_angle.roll = state.gyro.roll * state.looptime;
  state.gyro_delta_angle.pitch = state.gyro.pitch * state.looptime;
  state.gyro_delta_angle.yaw = state.gyro.yaw * state.looptime;
}

static bool test_gyro_move(const gyro_data_t *last_data, const gyro_data_t *data) {
  bool did_move = false;
  for (uint8_t i = 0; i < 3; i++) {
    const float delta = fabsf(fabsf(last_data->gyro.axis[i] * GYRO_RANGE) - fabsf(data->gyro.axis[i] * GYRO_RANGE));
    if (delta > 0.5f) {
      did_move = true;
      break;
    }
  }
  return did_move;
}

// returns true if it's already still, i.e. no move since the first loops
static bool sixaxis_wait_for_still(uint32_t timeout) {
  uint8_t move_counter = 15;
  uint32_t loop_counter = 0;

  gyro_data_t last_data;
  memset(&last_data, 0, sizeof(gyro_data_t));

  const uint32_t start = time_micros();
  uint32_t now = start;
  while (now - start < timeout && move_counter > 0) {
    const gyro_data_t data = gyro_read();

    const bool did_move = test_gyro_move(&last_data, &data);
    if (did_move) {
      move_counter = 15;
    } else {
      move_counter--;
    }

    led_pwm(move_counter / 15.f, 1000);

    while ((time_micros() - now) < 1000)
      ;

    now = time_micros();
    last_data = data;
    ++loop_counter;
  }
  return loop_counter < 20;
}

void sixaxis_gyro_cal() {
  for (uint8_t retry = 0; retry < 15; ++retry) {
    if (sixaxis_wait_for_still(CAL_INTERVAL)) {
      // break only if it's already still, otherwise, wait and try again
      break;
    }
    time_delay_ms(100);
  }
  gyro_calibrate();

  gyro_data_t last_data = gyro_read();
  for (uint32_t tries = 0, cal_counter = 0; tries < CAL_TRIES; tries++) {
    const gyro_data_t data = gyro_read();

    led_pwm(((float)(cal_counter) / (float)(CAL_COUNT)), CAL_INTERVAL);

    bool did_move = test_gyro_move(&last_data, &data);
    if (!did_move) { // only cali gyro when it's still
      for (uint8_t i = 0; i < 3; i++) {
        lpf(&gyrocal[i], data.gyro.axis[i], lpfcalc(CAL_INTERVAL, 0.5 * 1e6));
      }
      if (cal_counter++ == CAL_COUNT) {
        break;
      }
    }

    time_delay_us(CAL_INTERVAL);
    last_data = data;
  }
}

void sixaxis_acc_cal() {
  for (uint8_t retry = 0; retry < 15; ++retry) {
    if (sixaxis_wait_for_still(CAL_INTERVAL)) {
      // break only if it's already still, otherwise, wait and try again
      break;
    }
    time_delay_ms(100);
  }

  sixaxis_orientation_update();

  flash_storage.accelcal[0] = 0;
  flash_storage.accelcal[1] = 0;
  flash_storage.accelcal[2] = 2048;

  gyro_data_t last_data = gyro_read();
  for (uint32_t tries = 0, cal_counter = 0; tries < CAL_TRIES; tries++) {
    const gyro_data_t data = gyro_read();

    led_pwm(((float)(cal_counter) / (float)(CAL_COUNT)), CAL_INTERVAL);

    // Skip samples if gyro shows movement
    if (!test_gyro_move(&last_data, &data)) {
      const vec3_t accel = sixaxis_apply_orientation(data.accel);
      for (uint8_t i = 0; i < 3; i++) {
        lpf(&flash_storage.accelcal[i], accel.axis[i], lpfcalc(CAL_INTERVAL, 0.5 * 1e6));
      }

      if (cal_counter++ == CAL_COUNT) {
        break;
      }
    }

    time_delay_us(CAL_INTERVAL);
    last_data = data;
  }
  // Level Z averages 1 g; store only its bias.
  flash_storage.accelcal[2] -= 1.0f / ACCEL_RANGE;

  for (uint8_t i = 0; i < 3; i++) {
    flash_storage.accelcal[i] = constrain(flash_storage.accelcal[i], -ACCEL_BIAS_LIMIT, ACCEL_BIAS_LIMIT);
  }
}

#else

void sixaxis_init() {}
void sixaxis_filter_update(bool reset) {}
void sixaxis_orientation_update() {}
void sixaxis_read() {}

void sixaxis_gyro_cal() {
  time_delay_ms(1500);
}
void sixaxis_acc_cal() {
  time_delay_ms(1500);
}

#endif
