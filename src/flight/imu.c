#include "flight/imu.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/time.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "flight/sixaxis.h"
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
  const vec3_t rot = {{
      -state.gyro_delta_angle.axis[1],
      state.gyro_delta_angle.axis[0],
      state.gyro_delta_angle.axis[2],
  }};
  state.GEstG = vec3_rotate(state.GEstG, rot);

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
    state.attitude.roll = atan2approx(state.GEstG.roll, state.GEstG.yaw);
    state.attitude.pitch = atan2approx(state.GEstG.pitch, state.GEstG.yaw);
  }
}
#endif

#ifdef QUICKSILVER_IMU
void imu_calc() {
  const vec3_t rot = {{
      -state.gyro_delta_angle.axis[1],
      state.gyro_delta_angle.axis[0],
      state.gyro_delta_angle.axis[2],
  }};
  state.GEstG = vec3_rotate(state.GEstG, rot);

  filter_lp_pt1_coeff(&filter, PT1_FILTER_HZ);

  state.accel.roll = filter_lp_pt1_step(&filter, &filter_pass1[0], state.accel_raw.roll);
  state.accel.pitch = filter_lp_pt1_step(&filter, &filter_pass1[1], state.accel_raw.pitch);
  state.accel.yaw = filter_lp_pt1_step(&filter, &filter_pass1[2], state.accel_raw.yaw);

  state.accel.roll = filter_lp_pt1_step(&filter, &filter_pass2[0], state.accel.roll);
  state.accel.pitch = filter_lp_pt1_step(&filter, &filter_pass2[1], state.accel.pitch);
  state.accel.yaw = filter_lp_pt1_step(&filter, &filter_pass2[2], state.accel.yaw);

  const float accmag = vec3_magnitude(&state.accel);
  if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G)) {
    state.accel.roll = state.accel.roll * (ACC_1G / accmag);
    state.accel.pitch = state.accel.pitch * (ACC_1G / accmag);
    state.accel.yaw = state.accel.yaw * (ACC_1G / accmag);

    if (flags.on_ground) {
      // happyhour bartender - quad is ON GROUND and disarmed
      const float filtcoeff = lpfcalc_hz(state.looptime, 1.0f / (float)FASTFILTER);
      lpf(&state.GEstG.roll, state.accel.roll, filtcoeff);
      lpf(&state.GEstG.pitch, state.accel.pitch, filtcoeff);
      lpf(&state.GEstG.yaw, state.accel.yaw, filtcoeff);
    } else {
      // lateshift bartender - quad is IN AIR and things are getting wild
      const float filtcoeff = lpfcalc_hz(state.looptime, 1.0f / (float)FILTERTIME);
      lpf(&state.GEstG.roll, state.accel.roll, filtcoeff);
      lpf(&state.GEstG.pitch, state.accel.pitch, filtcoeff);
      lpf(&state.GEstG.yaw, state.accel.yaw, filtcoeff);
    }
  }

  // heal the gravity vector after fusion with accel
  const float GEstGmag = vec3_magnitude(&state.GEstG);
  state.GEstG.roll = state.GEstG.roll * (ACC_1G / GEstGmag);
  state.GEstG.pitch = state.GEstG.pitch * (ACC_1G / GEstGmag);
  state.GEstG.yaw = state.GEstG.yaw * (ACC_1G / GEstGmag);

  if (rx_aux_on(AUX_HORIZON)) {
    state.attitude.roll = atan2approx(state.GEstG.roll, state.GEstG.yaw);
    state.attitude.pitch = atan2approx(state.GEstG.pitch, state.GEstG.yaw);
  }
}
#endif