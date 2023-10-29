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

// this is the value of both cos 45 and sin 45 = 1/sqrt(2)
#define INVSQRT2 0.707106781f

#ifdef USE_GYRO

static float gyrocal[3];

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

void sixaxis_read() {
  const gyro_data_t data = gyro_spi_read();

  // remove bias and reduce to state.accel_raw in G
  state.accel_raw.roll = (data.accel.roll - flash_storage.accelcal[0]) * (1 / 2048.0f);
  state.accel_raw.pitch = (data.accel.pitch - flash_storage.accelcal[1]) * (1 / 2048.0f);
  state.accel_raw.yaw = (data.accel.yaw - flash_storage.accelcal[2]) * (1 / 2048.0f);

  state.gyro_raw.roll = data.gyro.roll - gyrocal[0];
  state.gyro_raw.pitch = data.gyro.pitch - gyrocal[1];
  state.gyro_raw.yaw = data.gyro.yaw - gyrocal[2];

  state.gyro_temp = data.temp;

  float temp = 0;

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CW) {
    temp = state.accel_raw.pitch;
    state.accel_raw.pitch = state.accel_raw.roll;
    state.accel_raw.roll = -temp;

    temp = state.gyro_raw.pitch;
    state.gyro_raw.pitch = -state.gyro_raw.roll;
    state.gyro_raw.roll = temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CCW) {
    temp = state.accel_raw.roll;
    state.accel_raw.roll = (state.accel_raw.roll * INVSQRT2 + state.accel_raw.pitch * INVSQRT2);
    state.accel_raw.pitch = -(temp * INVSQRT2 - state.accel_raw.pitch * INVSQRT2);

    temp = state.gyro_raw.pitch;
    state.gyro_raw.pitch = state.gyro_raw.roll * INVSQRT2 + state.gyro_raw.pitch * INVSQRT2;
    state.gyro_raw.roll = state.gyro_raw.roll * INVSQRT2 - temp * INVSQRT2;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CW) {
    temp = state.accel_raw.pitch;
    state.accel_raw.pitch = (state.accel_raw.pitch * INVSQRT2 + state.accel_raw.roll * INVSQRT2);
    state.accel_raw.roll = -(temp * INVSQRT2 - state.accel_raw.roll * INVSQRT2);

    temp = state.gyro_raw.roll;
    state.gyro_raw.roll = state.gyro_raw.pitch * INVSQRT2 + state.gyro_raw.roll * INVSQRT2;
    state.gyro_raw.pitch = state.gyro_raw.pitch * INVSQRT2 - temp * INVSQRT2;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CCW) {
    temp = state.accel_raw.pitch;
    state.accel_raw.pitch = -state.accel_raw.roll;
    state.accel_raw.roll = temp;

    temp = state.gyro_raw.pitch;
    state.gyro_raw.pitch = state.gyro_raw.roll;
    state.gyro_raw.roll = -temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_180) {
    state.accel_raw.pitch = -state.accel_raw.pitch;
    state.accel_raw.roll = -state.accel_raw.roll;

    state.gyro_raw.pitch = -state.gyro_raw.pitch;
    state.gyro_raw.roll = -state.gyro_raw.roll;
  }

  if (profile.motor.gyro_orientation & GYRO_FLIP_180) {
    state.accel_raw.yaw = -state.accel_raw.yaw;
    state.accel_raw.roll = -state.accel_raw.roll;

    state.gyro_raw.pitch = -state.gyro_raw.pitch;
    state.gyro_raw.yaw = -state.gyro_raw.yaw;
  }

  filter_coeff(profile.filter.gyro[0].type, &filter[0], profile.filter.gyro[0].cutoff_freq);
  filter_coeff(profile.filter.gyro[1].type, &filter[1], profile.filter.gyro[1].cutoff_freq);

  state.gyro.roll = state.gyro_raw.roll = state.gyro_raw.roll * GYRO_RANGE * DEGTORAD;
  state.gyro.pitch = state.gyro_raw.pitch = -state.gyro_raw.pitch * GYRO_RANGE * DEGTORAD;
  state.gyro.yaw = state.gyro_raw.yaw = -state.gyro_raw.yaw * GYRO_RANGE * DEGTORAD;

  for (uint32_t i = 0; i < 3; i++) {
    if (profile.filter.gyro_dynamic_notch_enable) {
      static uint8_t axis_needs_update = 0;
      if (sdft_push(&gyro_sdft[i], state.gyro.axis[i]) && axis_needs_update == 0) {
        axis_needs_update = 3;
      }

      if ((axis_needs_update - 1) == i && sdft_update(&gyro_sdft[i])) {
        for (uint32_t p = 0; p < SDFT_PEAKS; p++) {
          filter_biquad_notch_coeff(&notch_filter[i][p], gyro_sdft[i].notch_hz[p]);
        }
        axis_needs_update--;
      }
    }

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
  flash_storage.accelcal[2] = 2048;
  for (int y = 0; y < 500; y++) {
    sixaxis_read();
    for (int x = 0; x < 3; x++) {
      lpf(&flash_storage.accelcal[x], state.accel_raw.axis[x], 0.92);
    }
  }
  flash_storage.accelcal[2] -= 2048;

  for (int x = 0; x < 3; x++) {
    flash_storage.accelcal[x] = constrain(flash_storage.accelcal[x], -ACCEL_BIAS_LIMIT, ACCEL_BIAS_LIMIT);
  }
}

#else

bool sixaxis_init() {
  return false;
}
void sixaxis_read() {}

void sixaxis_gyro_cal() {}
void sixaxis_acc_cal() {}

#endif