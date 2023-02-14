#include "flight/imu.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "driver/time.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "flight/sixaxis.h"
#include "project.h"
#include "util/util.h"
#include "util/vector.h"

// IMU fusion algo, CHOOSE ONE
// #define SILVERWARE_IMU
#define QUICKSILVER_IMU

// filter times in seconds
// time to correct gyro readings using the accelerometer
// 1-4 are generally good
#define FASTFILTER 0.05 // on_ground filter
// #define PREFILTER 0.2   //in_air prefilter (this can be commented out)
#define FILTERTIME 2.0 // in_air fusion filter

#define PT1_FILTER_HZ 10

// accel magnitude limits for drift correction
#define ACC_MIN 0.7f
#define ACC_MAX 1.3f

#ifdef QUICKSILVER_IMU
static filter_lp_pt1 filter;
static filter_state_t filter_pass1[3];
static filter_state_t filter_pass2[3];
#endif

void imu_init() {
  // init the gravity vector with accel values
  for (int xx = 0; xx < 100; xx++) {
    sixaxis_read();

    for (int x = 0; x < 3; x++) {
      lpf(&state.GEstG.axis[x], state.accel_raw.axis[x], 0.85);
    }
    time_delay_us(1000);
  }

#ifdef QUICKSILVER_IMU
  filter_lp_pt1_init(&filter, filter_pass1, 3, PT1_FILTER_HZ);
  filter_lp_pt1_init(&filter, filter_pass2, 3, PT1_FILTER_HZ);
#endif
}

#ifdef SILVERWARE_IMU
void imu_calc() {
  const float gyro_delta_angle[3] = {
      state.gyro.axis[0] * state.looptime,
      state.gyro.axis[1] * state.looptime,
      state.gyro.axis[2] * state.looptime,
  };

  state.GEstG.axis[2] = state.GEstG.axis[2] - (gyro_delta_angle[0]) * state.GEstG.axis[0];
  state.GEstG.axis[0] = (gyro_delta_angle[0]) * state.GEstG.axis[2] + state.GEstG.axis[0];

  state.GEstG.axis[1] = state.GEstG.axis[1] + (gyro_delta_angle[1]) * state.GEstG.axis[2];
  state.GEstG.axis[2] = -(gyro_delta_angle[1]) * state.GEstG.axis[1] + state.GEstG.axis[2];

  state.GEstG.axis[0] = state.GEstG.axis[0] - (gyro_delta_angle[2]) * state.GEstG.axis[1];
  state.GEstG.axis[1] = (gyro_delta_angle[2]) * state.GEstG.axis[0] + state.GEstG.axis[1];

  if (flags.on_ground) { // happyhour bartender - quad is ON GROUND and disarmed
    // calc acc mag
    float accmag = vec3_magnitude(&state.accel_raw);
    if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G)) {
      // normalize acc
      for (int axis = 0; axis < 3; axis++) {
        state.accel_raw.axis[axis] = state.accel_raw.axis[axis] * (ACC_1G / accmag);
      }

      float filtcoeff = lpfcalc_hz(state.looptime, 1.0f / (float)FASTFILTER);
      for (int x = 0; x < 3; x++) {
        lpf(&state.GEstG.axis[x], state.accel_raw.axis[x], filtcoeff);
      }
    }
  } else {
    // lateshift bartender - quad is IN AIR and things are getting wild
    //  hit state.accel_raw.axis[3] with a sledgehammer
#ifdef PREFILTER
    float filtcoeff = lpfcalc_hz(state.looptime, 1.0f / (float)PREFILTER);
    for (int x = 0; x < 3; x++) {
      lpf(&state.accel.axis[x], state.accel_raw.axis[x], filtcoeff);
    }
#else
    for (int x = 0; x < 3; x++) {
      state.accel.axis[x] = state.accel_raw.axis[x];
    }
#endif

    // calc mag of filtered acc
    float accmag = vec3_magnitude(&state.accel);
    if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G)) {
      // normalize acc
      for (int axis = 0; axis < 3; axis++) {
        state.accel.axis[axis] = state.accel.axis[axis] * (ACC_1G / accmag);
      }
      // filter accel on to GEstG
      float filtcoeff = lpfcalc_hz(state.looptime, 1.0f / (float)FILTERTIME);
      for (int x = 0; x < 3; x++) {
        lpf(&state.GEstG.axis[x], state.accel.axis[x], filtcoeff);
      }
    }
    // heal the gravity vector after fusion with accel
    float GEstGmag = vec3_magnitude(&state.GEstG);
    for (int axis = 0; axis < 3; axis++) {
      state.GEstG.axis[axis] = state.GEstG.axis[axis] * (ACC_1G / GEstGmag);
    }
  }

  if (rx_aux_on(AUX_HORIZON)) {
    state.attitude.axis[0] = atan2approx(state.GEstG.axis[0], state.GEstG.axis[2]);
    state.attitude.axis[1] = atan2approx(state.GEstG.axis[1], state.GEstG.axis[2]);
  }
}
#endif

#ifdef QUICKSILVER_IMU
void imu_calc() {
  const float gyro_delta_angle[3] = {
      state.gyro.axis[0] * state.looptime,
      state.gyro.axis[1] * state.looptime,
      state.gyro.axis[2] * state.looptime,
  };

  state.GEstG.axis[2] = state.GEstG.axis[2] - (gyro_delta_angle[0]) * state.GEstG.axis[0];
  state.GEstG.axis[0] = (gyro_delta_angle[0]) * state.GEstG.axis[2] + state.GEstG.axis[0];

  state.GEstG.axis[1] = state.GEstG.axis[1] + (gyro_delta_angle[1]) * state.GEstG.axis[2];
  state.GEstG.axis[2] = -(gyro_delta_angle[1]) * state.GEstG.axis[1] + state.GEstG.axis[2];

  state.GEstG.axis[0] = state.GEstG.axis[0] - (gyro_delta_angle[2]) * state.GEstG.axis[1];
  state.GEstG.axis[1] = (gyro_delta_angle[2]) * state.GEstG.axis[0] + state.GEstG.axis[1];

  filter_lp_pt1_coeff(&filter, PT1_FILTER_HZ);

  state.accel.axis[0] = filter_lp_pt1_step(&filter, &filter_pass1[0], state.accel_raw.axis[0]);
  state.accel.axis[1] = filter_lp_pt1_step(&filter, &filter_pass1[1], state.accel_raw.axis[1]);
  state.accel.axis[2] = filter_lp_pt1_step(&filter, &filter_pass1[2], state.accel_raw.axis[2]);

  state.accel.axis[0] = filter_lp_pt1_step(&filter, &filter_pass2[0], state.accel.axis[0]);
  state.accel.axis[1] = filter_lp_pt1_step(&filter, &filter_pass2[1], state.accel.axis[1]);
  state.accel.axis[2] = filter_lp_pt1_step(&filter, &filter_pass2[2], state.accel.axis[2]);

  const float accmag = vec3_magnitude(&state.accel);
  if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G)) {
    state.accel.axis[0] = state.accel.axis[0] * (ACC_1G / accmag);
    state.accel.axis[1] = state.accel.axis[1] * (ACC_1G / accmag);
    state.accel.axis[2] = state.accel.axis[2] * (ACC_1G / accmag);

    if (flags.on_ground) {
      // happyhour bartender - quad is ON GROUND and disarmed
      const float filtcoeff = lpfcalc_hz(state.looptime, 1.0f / (float)FASTFILTER);
      lpf(&state.GEstG.axis[0], state.accel.axis[0], filtcoeff);
      lpf(&state.GEstG.axis[1], state.accel.axis[1], filtcoeff);
      lpf(&state.GEstG.axis[2], state.accel.axis[2], filtcoeff);
    } else {
      // lateshift bartender - quad is IN AIR and things are getting wild
      const float filtcoeff = lpfcalc_hz(state.looptime, 1.0f / (float)FILTERTIME);
      lpf(&state.GEstG.axis[0], state.accel.axis[0], filtcoeff);
      lpf(&state.GEstG.axis[1], state.accel.axis[1], filtcoeff);
      lpf(&state.GEstG.axis[2], state.accel.axis[2], filtcoeff);
    }
  }

  // heal the gravity vector after fusion with accel
  const float GEstGmag = vec3_magnitude(&state.GEstG);
  state.GEstG.axis[0] = state.GEstG.axis[0] * (ACC_1G / GEstGmag);
  state.GEstG.axis[1] = state.GEstG.axis[1] * (ACC_1G / GEstGmag);
  state.GEstG.axis[2] = state.GEstG.axis[2] * (ACC_1G / GEstGmag);

  if (rx_aux_on(AUX_HORIZON)) {
    state.attitude.axis[0] = atan2approx(state.GEstG.axis[0], state.GEstG.axis[2]);
    state.attitude.axis[1] = atan2approx(state.GEstG.axis[1], state.GEstG.axis[2]);
  }
}
#endif