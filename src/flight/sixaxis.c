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
#include "flight/sixaxis.h"
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

static filter_t filter[FILTER_MAX_SLOTS];
static filter_state_t filter_state[FILTER_MAX_SLOTS][3];

float gyrocal[3];

bool sixaxis_init() {
  const gyro_types_t id = gyro_spi_init();

  target_info.gyro_id = id;

  for (uint8_t i = 0; i < FILTER_MAX_SLOTS; i++) {
    filter_init(profile.filter.gyro[i].type, &filter[i], filter_state[i], 3, profile.filter.gyro[i].cutoff_freq);
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

  for (int i = 0; i < 3; i++) {
    state.gyro.axis[i] = state.gyro_raw.axis[i];

    state.gyro.axis[i] = filter_step(profile.filter.gyro[0].type, &filter[0], &filter_state[0][i], state.gyro.axis[i]);
    state.gyro.axis[i] = filter_step(profile.filter.gyro[1].type, &filter[1], &filter_state[1][i], state.gyro.axis[i]);
  }
}

static void sixaxis_wait_for_still() {
  const uint32_t start = time_micros();

  // turn on led
  uint8_t move_counter = 15;

  gyro_data_t last_data;
  memset(&last_data, 0, sizeof(gyro_data_t));

  uint32_t now = start;
  while (now - start < WAIT_TIME && move_counter > 0) {
    const gyro_data_t data = gyro_spi_read();

    bool did_move = false;
    for (uint8_t i = 0; i < 3; i++) {
      const float delta = fabsf(fabsf(last_data.gyro.axis[i] * GYRO_RANGE) - fabsf(data.gyro.axis[i] * GYRO_RANGE));
      if (delta > 0.3f) {
        did_move = true;
        break;
      }
    }

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
  }
}

void sixaxis_gyro_cal() {
  float limit[3];
  for (int i = 0; i < 3; i++) {
    limit[i] = gyrocal[i];
  }

  sixaxis_wait_for_still();
  gyro_spi_calibrate();

  uint8_t brightness = 0;
  led_pwm(brightness, 1000);

  uint32_t start = time_micros();
  uint32_t now = start;
  while (now - start < CAL_TIME) {
    const gyro_data_t data = gyro_spi_read();

    led_pwm(brightness, 1000);
    if ((brightness & 1) ^ ((now - start) % GLOW_TIME > (GLOW_TIME >> 1))) {
      brightness++;
      brightness &= 0xF;
    }

    for (uint8_t i = 0; i < 3; i++) {
      if (data.gyro.axis[i] > limit[i])
        limit[i] += 0.1f; // 100 gyro bias / second change
      if (data.gyro.axis[i] < limit[i])
        limit[i] -= 0.1f;

      limitf(&limit[i], GYRO_BIAS_LIMIT);
      lpf(&gyrocal[i], data.gyro.axis[i], lpfcalc(1000, 0.5 * 1e6));
    }

    while ((time_micros() - now) < 1000)
      ;

    now = time_micros();
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
    limitf(&flash_storage.accelcal[x], ACCEL_BIAS_LIMIT);
  }
}
