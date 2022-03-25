#include "flight/sixaxis.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "debug.h"
#include "drv_serial.h"
#include "drv_spi_gyro.h"
#include "drv_time.h"
#include "flash.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "flight/sixaxis.h"
#include "io/blackbox.h"
#include "io/led.h"
#include "profile.h"
#include "project.h"
#include "sdft.h"
#include "util/util.h"

#define CAL_TIME 2e6
#define GLOW_TIME 62500

// gyro has +-2000 divided over 16bit.
#define GYRO_RANGE (1.f / (65536.f / 4000.f))

// this is the value of both cos 45 and sin 45 = 1/sqrt(2)
#define INVSQRT2 0.707106781f

#define NOTCH_FILTER_CENTER 120

static filter_t filter[FILTER_MAX_SLOTS];
static filter_state_t filter_state[FILTER_MAX_SLOTS][3];

static sdft_t gyro_sdft[3];
static filter_biquad_notch_t notch_filter[SDFT_AXES][SDFT_PEAKS];
static filter_biquad_state_t notch_filter_state[SDFT_AXES][SDFT_PEAKS];

extern profile_t profile;
extern target_info_t target_info;

float gyrocal[3];

bool sixaxis_init() {
  const gyro_types_t id = gyro_spi_init();

  target_info.gyro_id = id;

  for (uint8_t i = 0; i < FILTER_MAX_SLOTS; i++) {
    filter_init(profile.filter.gyro[i].type, &filter[i], filter_state[i], 3, profile.filter.gyro[i].cutoff_freq);
  }

  for (uint8_t i = 0; i < 3; i++) {
    sdft_init(&gyro_sdft[i]);
    gyro_sdft[i].sample_count = i + 1;
    filter_biquad_notch_init(&notch_filter[i][0], &notch_filter_state[i][0], 1, NOTCH_FILTER_CENTER);
    filter_biquad_notch_init(&notch_filter[i][1], &notch_filter_state[i][1], 1, NOTCH_FILTER_CENTER);
  }

  return id != GYRO_TYPE_INVALID;
}

void sixaxis_read() {
  const gyro_data_t data = gyro_spi_read();

  state.accel_raw = data.accel;
  state.gyro_temp = data.temp;
  state.gyro_raw = data.gyro;

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CW) {
    float temp = state.accel_raw.axis[1];
    state.accel_raw.axis[1] = state.accel_raw.axis[0];
    state.accel_raw.axis[0] = -temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CCW) {
    float temp = state.accel_raw.axis[0];
    state.accel_raw.axis[0] = (state.accel_raw.axis[0] * INVSQRT2 + state.accel_raw.axis[1] * INVSQRT2);
    state.accel_raw.axis[1] = -(temp * INVSQRT2 - state.accel_raw.axis[1] * INVSQRT2);
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CW) {
    float temp = state.accel_raw.axis[1];
    state.accel_raw.axis[1] = (state.accel_raw.axis[1] * INVSQRT2 + state.accel_raw.axis[0] * INVSQRT2);
    state.accel_raw.axis[0] = -(temp * INVSQRT2 - state.accel_raw.axis[0] * INVSQRT2);
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CCW) {
    float temp = state.accel_raw.axis[1];
    state.accel_raw.axis[1] = -state.accel_raw.axis[0];
    state.accel_raw.axis[0] = temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_180) {
    state.accel_raw.axis[1] = -state.accel_raw.axis[1];
    state.accel_raw.axis[0] = -state.accel_raw.axis[0];
  }

  if (profile.motor.gyro_orientation & GYRO_FLIP_180) {
    state.accel_raw.axis[2] = -state.accel_raw.axis[2];
    state.accel_raw.axis[0] = -state.accel_raw.axis[0];
  }

  // remove bias and reduce to state.accel_raw in G
  state.accel_raw.axis[0] = (state.accel_raw.axis[0] - flash_storage.accelcal[0]) * (1 / 2048.0f);
  state.accel_raw.axis[1] = (state.accel_raw.axis[1] - flash_storage.accelcal[1]) * (1 / 2048.0f);
  state.accel_raw.axis[2] = (state.accel_raw.axis[2] - flash_storage.accelcal[2]) * (1 / 2048.0f);

  state.gyro_raw.axis[0] = state.gyro_raw.axis[0] - gyrocal[0];
  state.gyro_raw.axis[1] = state.gyro_raw.axis[1] - gyrocal[1];
  state.gyro_raw.axis[2] = state.gyro_raw.axis[2] - gyrocal[2];

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CW) {
    float temp = state.gyro_raw.axis[1];
    state.gyro_raw.axis[1] = -state.gyro_raw.axis[0];
    state.gyro_raw.axis[0] = temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CCW) {
    float temp = state.gyro_raw.axis[1];
    state.gyro_raw.axis[1] = state.gyro_raw.axis[0] * INVSQRT2 + state.gyro_raw.axis[1] * INVSQRT2;
    state.gyro_raw.axis[0] = state.gyro_raw.axis[0] * INVSQRT2 - temp * INVSQRT2;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CW) {
    float temp = state.gyro_raw.axis[0];
    state.gyro_raw.axis[0] = state.gyro_raw.axis[1] * INVSQRT2 + state.gyro_raw.axis[0] * INVSQRT2;
    state.gyro_raw.axis[1] = state.gyro_raw.axis[1] * INVSQRT2 - temp * INVSQRT2;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CCW) {
    float temp = state.gyro_raw.axis[1];
    state.gyro_raw.axis[1] = state.gyro_raw.axis[0];
    state.gyro_raw.axis[0] = -temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_180) {
    state.gyro_raw.axis[1] = -state.gyro_raw.axis[1];
    state.gyro_raw.axis[0] = -state.gyro_raw.axis[0];
  }

  if (profile.motor.gyro_orientation & GYRO_FLIP_180) {
    state.gyro_raw.axis[1] = -state.gyro_raw.axis[1];
    state.gyro_raw.axis[2] = -state.gyro_raw.axis[2];
  }

  state.gyro_raw.axis[0] = state.gyro_raw.axis[0] * GYRO_RANGE * DEGTORAD;
  state.gyro_raw.axis[1] = -state.gyro_raw.axis[1] * GYRO_RANGE * DEGTORAD;
  state.gyro_raw.axis[2] = -state.gyro_raw.axis[2] * GYRO_RANGE * DEGTORAD;

  filter_coeff(profile.filter.gyro[0].type, &filter[0], profile.filter.gyro[0].cutoff_freq);
  filter_coeff(profile.filter.gyro[1].type, &filter[1], profile.filter.gyro[1].cutoff_freq);

  for (uint32_t i = 0; i < 3; i++) {
    state.gyro.axis[i] = state.gyro_raw.axis[i];

    static bool needs_update[3] = {false, false, false};
    if (sdft_push(&gyro_sdft[i], state.gyro.axis[i])) {
      needs_update[i] = true;
    }

    if (needs_update[i] && sdft_update(&gyro_sdft[i])) {
      for (uint32_t p = 0; p < SDFT_PEAKS; p++) {
        filter_biquad_notch_coeff(&notch_filter[i][p], gyro_sdft[i].notch_hz[p]);
      }
      needs_update[i] = false;
    }

    state.gyro.axis[i] = filter_step(profile.filter.gyro[0].type, &filter[0], &filter_state[0][i], state.gyro.axis[i]);
    state.gyro.axis[i] = filter_step(profile.filter.gyro[1].type, &filter[1], &filter_state[1][i], state.gyro.axis[i]);

    for (uint32_t p = 0; p < SDFT_PEAKS; p++) {
      if (i == 0) {
        blackbox_set_debug(p, gyro_sdft[i].notch_hz[p]);
      }
      state.gyro.axis[i] = filter_biquad_notch_step(&notch_filter[i][p], &notch_filter_state[i][p], state.gyro.axis[i]);
    }
  }
}

void sixaxis_gyro_cal() {
  float limit[3];
  uint32_t time = time_micros();
  uint32_t timestart = time;
  uint32_t timemax = time;
  uint32_t lastlooptime = time;

  for (int i = 0; i < 3; i++) {
    limit[i] = gyrocal[i];
  }

  // 2 and 15 seconds
  while (time - timestart < CAL_TIME && time - timemax < 15e6) {

    uint32_t looptime;
    looptime = time - lastlooptime;
    lastlooptime = time;
    if (looptime == 0)
      looptime = 1;

    const gyro_data_t data = gyro_spi_read();

    static int brightness = 0;
    led_pwm(brightness);
    if ((brightness & 1) ^ ((time - timestart) % GLOW_TIME > (GLOW_TIME >> 1))) {
      brightness++;
    }

    brightness &= 0xF;

    for (int i = 0; i < 3; i++) {

      if (data.gyro.axis[i] > limit[i])
        limit[i] += 0.1f; // 100 gyro bias / second change
      if (data.gyro.axis[i] < limit[i])
        limit[i] -= 0.1f;

      limitf(&limit[i], 800);

      if (fabsf(data.gyro.axis[i]) > 100 + fabsf(limit[i])) {
        timestart = time_micros();
        brightness = 1;
      } else {
        lpf(&gyrocal[i], data.gyro.axis[i], lpfcalc((float)looptime, 0.5 * 1e6));
      }
    }

    while ((time_micros() - time) < 1000)
      time_delay_us(10);
    time = time_micros();
  }

  if (time - timestart < CAL_TIME) {
    for (int i = 0; i < 3; i++) {
      gyrocal[i] = 0;
    }
  }
}

void sixaxis_acc_cal() {
  flash_storage.accelcal[2] = 2048;
  for (int y = 0; y < 500; y++) {
    sixaxis_read();
    for (int x = 0; x < 3; x++) {
      lpf(&flash_storage.accelcal[x], state.accel_raw.axis[x], 0.92);
    }
    time_micros(); // if it takes too long time will overflow so we call it here
  }
  flash_storage.accelcal[2] -= 2048;

  for (int x = 0; x < 3; x++) {
    limitf(&flash_storage.accelcal[x], 500);
  }
}
