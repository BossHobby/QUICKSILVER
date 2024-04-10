#include "flight/sixaxis.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "core/debug.h"
#include "core/flash.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/serial.h"
#include "driver/spi_gyro.h"
#include "driver/time.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "flight/sdft.h"
#include "flight/sixaxis.h"
#include "io/blackbox.h"
#include "io/led.h"
#include "util/util.h"

#define CAL_TIME 2e6
#define WAIT_TIME 15e6
#define GLOW_TIME 62500

#define GYRO_BIAS_LIMIT 800
#define ACCEL_BIAS_LIMIT 800

// gyro has +-2000 divided over 16bit.
#define GYRO_RANGE (1.f / (65536.f / 4000.f))
#define ACCEL_RANGE (1.f / 2048.0f)

#ifdef USE_GYRO

static float gyrocal[3];
static float rot_mat[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

static filter_t filter[FILTER_MAX_SLOTS];
static filter_state_t filter_state[FILTER_MAX_SLOTS][3];

static sdft_t gyro_sdft[SDFT_AXES];
static filter_biquad_notch_t notch_filter[SDFT_AXES][SDFT_PEAKS];
static filter_biquad_state_t notch_filter_state[SDFT_AXES][SDFT_PEAKS];

bool sixaxis_detect() {
  target_info.gyro_id = gyro_spi_init();
  return target_info.gyro_id != GYRO_TYPE_INVALID;
}

void sixaxis_init() {
  for (uint8_t i = 0; i < FILTER_MAX_SLOTS; i++) {
    filter_init(profile.filter.gyro[i].type, &filter[i], filter_state[i], 3, profile.filter.gyro[i].cutoff_freq);
  }

  for (uint8_t i = 0; i < SDFT_AXES; i++) {
    sdft_init(&gyro_sdft[i]);
    for (uint8_t j = 0; j < SDFT_PEAKS; j++) {
      filter_biquad_notch_init(&notch_filter[i][j], &notch_filter_state[i][j], 1, 0);
    }
  }
}

static void sixaxis_compute_matrix() {
  static uint8_t last_gyro_orientation = GYRO_ROTATE_NONE;
  if (last_gyro_orientation == profile.motor.gyro_orientation) {
    return;
  }

  vec3_t rot = {.roll = 0, .pitch = 0, .yaw = 0};

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CW) {
    rot.yaw += 90.0f * DEGTORAD;
  }
  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CCW) {
    rot.yaw -= 90.0f * DEGTORAD;
  }
  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CW) {
    rot.yaw += 45.0f * DEGTORAD;
  }
  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CCW) {
    rot.yaw -= 45.0f * DEGTORAD;
  }
  if (profile.motor.gyro_orientation & GYRO_ROTATE_180) {
    rot.yaw += 180.0f * DEGTORAD;
  }
  if (profile.motor.gyro_orientation & GYRO_FLIP_180) {
    rot.roll += 180.0f * DEGTORAD;
    rot.yaw += 180.0f * DEGTORAD;
  }

  const float cosx = fastcos(rot.roll);
  const float sinx = fastsin(rot.roll);
  const float cosy = fastcos(rot.pitch);
  const float siny = fastsin(rot.pitch);
  const float cosz = fastcos(rot.yaw);
  const float sinz = fastsin(rot.yaw);

  const float coszcosx = cosz * cosx;
  const float sinzcosx = sinz * cosx;
  const float coszsinx = sinx * cosz;
  const float sinzsinx = sinx * sinz;

  rot_mat[0][0] = cosz * cosy;
  rot_mat[0][1] = -cosy * sinz;
  rot_mat[0][2] = siny;
  rot_mat[1][0] = sinzcosx + (coszsinx * siny);
  rot_mat[1][1] = coszcosx - (sinzsinx * siny);
  rot_mat[1][2] = -sinx * cosy;
  rot_mat[2][0] = (sinzsinx) - (coszcosx * siny);
  rot_mat[2][1] = (coszsinx) + (sinzcosx * siny);
  rot_mat[2][2] = cosy * cosx;

  last_gyro_orientation = profile.motor.gyro_orientation;
}

static vec3_t sixaxis_apply_matrix(vec3_t v) {
  return (vec3_t){
      .roll = (rot_mat[0][0] * v.roll + rot_mat[1][0] * v.pitch + rot_mat[2][0] * v.yaw),
      .pitch = (rot_mat[0][1] * v.roll + rot_mat[1][1] * v.pitch + rot_mat[2][1] * v.yaw),
      .yaw = (rot_mat[0][2] * v.roll + rot_mat[1][2] * v.pitch + rot_mat[2][2] * v.yaw),
  };
}

void sixaxis_read() {
  sixaxis_compute_matrix();

  filter_coeff(profile.filter.gyro[0].type, &filter[0], profile.filter.gyro[0].cutoff_freq);
  filter_coeff(profile.filter.gyro[1].type, &filter[1], profile.filter.gyro[1].cutoff_freq);

  const gyro_data_t data = gyro_spi_read();

  const vec3_t accel = sixaxis_apply_matrix(data.accel);
  // swap pitch and roll to match gyro
  state.accel_raw.roll = (accel.pitch - flash_storage.accelcal[1]) * ACCEL_RANGE;
  state.accel_raw.pitch = (accel.roll - flash_storage.accelcal[0]) * ACCEL_RANGE;
  state.accel_raw.yaw = (accel.yaw - flash_storage.accelcal[2]) * ACCEL_RANGE;

  state.gyro_raw.roll = data.gyro.roll - gyrocal[0];
  state.gyro_raw.pitch = data.gyro.pitch - gyrocal[1];
  state.gyro_raw.yaw = data.gyro.yaw - gyrocal[2];
  state.gyro_raw = sixaxis_apply_matrix(state.gyro_raw);
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
        filter_biquad_notch_coeff(&notch_filter[current_axis][p], gyro_sdft[current_axis].notch_hz[p]);
      }
      // on the last axis we increment this to the idle state 3
      current_axis++;
    }
  }

  for (uint32_t i = 0; i < 3; i++) {
    state.gyro.axis[i] = filter_step(profile.filter.gyro[0].type, &filter[0], &filter_state[0][i], state.gyro.axis[i]);
    state.gyro.axis[i] = filter_step(profile.filter.gyro[1].type, &filter[1], &filter_state[1][i], state.gyro.axis[i]);

    if (profile.filter.gyro_dynamic_notch_enable) {
      for (uint32_t p = 0; p < SDFT_PEAKS; p++) {
        if (p == 1) {
          blackbox_set_debug(i, gyro_sdft[i].notch_hz[p]);
        }
        state.gyro.axis[i] = filter_biquad_notch_step(&notch_filter[i][p], &notch_filter_state[i][p], state.gyro.axis[i]);
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
    if (delta > 0.3f) {
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
    const gyro_data_t data = gyro_spi_read();

    const bool did_move = test_gyro_move(&last_data, &data);
    if (did_move) {
      move_counter = 15;
    } else {
      move_counter--;
    }

    led_pwm(move_counter, 1000);

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
    if (sixaxis_wait_for_still(WAIT_TIME / 15)) {
      // break only if it's already still, otherwise, wait and try again
      break;
    }
    time_delay_ms(200);
  }
  gyro_spi_calibrate();

  uint8_t brightness = 0;
  led_pwm(brightness, 1000);

  gyro_data_t last_data = gyro_spi_read();

  uint32_t start = time_micros();
  uint32_t now = start;
  int32_t cal_counter = CAL_TIME / 1000;
  for (int32_t timeout = WAIT_TIME / 1000; timeout > 0; --timeout) {
    const gyro_data_t data = gyro_spi_read();

    led_pwm(brightness, 1000);
    if ((brightness & 1) ^ ((now - start) % GLOW_TIME > (GLOW_TIME >> 1))) {
      brightness++;
      brightness &= 0xF;
    }

    bool did_move = test_gyro_move(&last_data, &data);
    if (!did_move) { // only cali gyro when it's still
      for (uint8_t i = 0; i < 3; i++) {
        lpf(&gyrocal[i], data.gyro.axis[i], lpfcalc(1000, 0.5 * 1e6));
      }
      if (--cal_counter <= 0) {
        break;
      }
    }

    while ((time_micros() - now) < 1000)
      ;

    now = time_micros();
    last_data = data;
  }
}

void sixaxis_acc_cal() {
  sixaxis_compute_matrix();

  flash_storage.accelcal[2] = 2048;
  for (uint32_t i = 0; i < 500; i++) {
    const gyro_data_t data = gyro_spi_read();
    const vec3_t accel = sixaxis_apply_matrix(data.accel);

    for (uint32_t x = 0; x < 3; x++) {
      lpf(&flash_storage.accelcal[x], accel.axis[x], 0.92);
    }

    time_delay_us(500);
  }
  flash_storage.accelcal[2] -= 2048;

  for (int x = 0; x < 3; x++) {
    flash_storage.accelcal[x] = constrain(flash_storage.accelcal[x], -ACCEL_BIAS_LIMIT, ACCEL_BIAS_LIMIT);
  }
}

#else

bool sixaxis_detect() {
  return true;
}
void sixaxis_init() {}
void sixaxis_read() {}

void sixaxis_gyro_cal() {}
void sixaxis_acc_cal() {}

#endif