#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "defines.h"
#include "drv_time.h"
#include "filter.h"
#include "sixaxis.h"
#include "util.h"

#define ACC_1G 1.0f

// disable drift correction ( for testing)
#define DISABLE_ACC 0

// IMU fusion algo, CHOOSE ONE
#define BFPV_IMU
//#define SILVERWARE_IMU
//#define QUICKSILVER_IMU

// filter times in seconds
// time to correct gyro readings using the accelerometer
// 1-4 are generally good
#define FASTFILTER 0.05 //onground filter
//#define PREFILTER 0.2   //in_air prefilter (this can be commented out)
#define FILTERTIME 2.0 //in_air fusion filter

#define PT1_FILTER_HZ 10

// accel magnitude limits for drift correction
#define ACC_MIN 0.7f
#define ACC_MAX 1.3f

#define _sinf(val) sinf(val)
#define _cosf(val) cosf(val)

float GEstG[3] = {0, 0, ACC_1G};
float attitude[3];

extern float gyro[3];
extern float accel[3];
extern float accelcal[3];

float accel_filter[3];

extern float looptime;

float calcmagnitude(float vector[3]) {
  float accmag = 0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    accmag += vector[axis] * vector[axis];
  }
  accmag = 1.0f / Q_rsqrt(accmag);
  return accmag;
}

void vectorcopy(float *vector1, float *vector2) {
  for (int axis = 0; axis < 3; axis++) {
    vector1[axis] = vector2[axis];
  }
}

#ifdef BFPV_IMU
static filter_lp2_iir filter[3];
#endif

#ifdef QUICKSILVER_IMU
static filter_lp_pt1 filter;
static filter_state_t filter_pass1[3];
static filter_state_t filter_pass2[3];
#endif

void imu_init(void) {
  // init the gravity vector with accel values
  for (int xx = 0; xx < 100; xx++) {
    sixaxis_read();

    for (int x = 0; x < 3; x++) {
      lpf(&GEstG[x], accel[x] * (1 / 2048.0f), 0.85);
    }
    delay(1000);
  }

#ifdef BFPV_IMU
  filter_lp2_iir_init(&filter[0], IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
  filter_lp2_iir_init(&filter[1], IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
  filter_lp2_iir_init(&filter[2], IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
#endif

#ifdef QUICKSILVER_IMU
  filter_lp_pt1_init(&filter, filter_pass1, 3, PT1_FILTER_HZ);
  filter_lp_pt1_init(&filter, filter_pass1, 3, PT1_FILTER_HZ);
#endif
}

#ifdef BFPV_IMU
void imu_calc(void) {
  // remove bias and reduce to accel in G
  accel[0] = (accel[0] - accelcal[0]) * (1 / 2048.0f);
  accel[1] = (accel[1] - accelcal[1]) * (1 / 2048.0f);
  accel[2] = (accel[2] - accelcal[2]) * (1 / 2048.0f);

  accel_filter[0] = filter_lp2_iir_step(&filter[0], accel[0]);
  accel_filter[1] = filter_lp2_iir_step(&filter[1], accel[1]);
  accel_filter[2] = filter_lp2_iir_step(&filter[2], accel[2]);

  float EstG[3];
  vectorcopy(&EstG[0], &GEstG[0]);

  float gyros[3];
  for (int i = 0; i < 3; i++) {
    gyros[i] = gyro[i] * looptime;
  }

  const float cosx = _cosf(gyros[1]);
  const float sinx = _sinf(gyros[1]);
  const float cosy = _cosf(gyros[0]);
  const float siny = _sinf(-gyros[0]);
  const float cosz = _cosf(gyros[2]);
  const float sinz = _sinf(-gyros[2]);

  const float coszcosx = cosz * cosx;
  const float coszcosy = cosz * cosy;
  const float sinzcosx = sinz * cosx;
  const float coszsinx = sinx * cosz;
  const float sinzsinx = sinx * sinz;

  float mat[3][3];
  mat[0][0] = coszcosy;
  mat[0][1] = -cosy * sinz;
  mat[0][2] = siny;
  mat[1][0] = sinzcosx + (coszsinx * siny);
  mat[1][1] = coszcosx - (sinzsinx * siny);
  mat[1][2] = -sinx * cosy;
  mat[2][0] = (sinzsinx) - (coszcosx * siny);
  mat[2][1] = (coszsinx) + (sinzcosx * siny);
  mat[2][2] = cosy * cosx;

  EstG[0] = GEstG[0] * mat[0][0] + GEstG[1] * mat[1][0] + GEstG[2] * mat[2][0];
  EstG[1] = GEstG[0] * mat[0][1] + GEstG[1] * mat[1][1] + GEstG[2] * mat[2][1];
  EstG[2] = GEstG[0] * mat[0][2] + GEstG[1] * mat[1][2] + GEstG[2] * mat[2][2];

  vectorcopy(&GEstG[0], &EstG[0]);

  extern int onground;
  if (onground) { //happyhour bartender - quad is ON GROUND and disarmed
    // calc acc mag
    float accmag = calcmagnitude(&accel_filter[0]);
    if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G)) {
      // normalize acc
      for (int axis = 0; axis < 3; axis++) {
        accel_filter[axis] = accel_filter[axis] * (ACC_1G / accmag);
      }

      float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)FASTFILTER);
      for (int x = 0; x < 3; x++) {
        lpf(&GEstG[x], accel_filter[x], filtcoeff);
      }
    }
  } else {
    float accmag = calcmagnitude(&accel_filter[0]);
    for (int axis = 0; axis < 3; axis++) {
      accel_filter[axis] = accel_filter[axis] * (ACC_1G / accmag);
    }
    float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)FILTERTIME);
    for (int x = 0; x < 3; x++) {
      lpf(&GEstG[x], accel_filter[x], filtcoeff);
    }

    //heal the gravity vector after fusion with accel
    float GEstGmag = calcmagnitude(&GEstG[0]);
    for (int axis = 0; axis < 3; axis++) {
      GEstG[axis] = GEstG[axis] * (ACC_1G / GEstGmag);
    }
  }

  if (rx_aux_on(AUX_HORIZON)) {
    attitude[0] = atan2approx(GEstG[0], GEstG[2]);
    attitude[1] = atan2approx(GEstG[1], GEstG[2]);
  }
}
#endif

#ifdef SILVERWARE_IMU
void imu_calc(void) {
  // remove bias and reduce to accel in G
  accel[0] = (accel[0] - accelcal[0]) * (1 / 2048.0f);
  accel[1] = (accel[1] - accelcal[1]) * (1 / 2048.0f);
  accel[2] = (accel[2] - accelcal[2]) * (1 / 2048.0f);

  const float gyro_delta_angle[3] = {
      gyro[0] * looptime,
      gyro[1] * looptime,
      gyro[2] * looptime,
  };

  GEstG[2] = GEstG[2] - (gyro_delta_angle[0]) * GEstG[0];
  GEstG[0] = (gyro_delta_angle[0]) * GEstG[2] + GEstG[0];

  GEstG[1] = GEstG[1] + (gyro_delta_angle[1]) * GEstG[2];
  GEstG[2] = -(gyro_delta_angle[1]) * GEstG[1] + GEstG[2];

  GEstG[0] = GEstG[0] - (gyro_delta_angle[2]) * GEstG[1];
  GEstG[1] = (gyro_delta_angle[2]) * GEstG[0] + GEstG[1];

  extern int onground;
  if (onground) { //happyhour bartender - quad is ON GROUND and disarmed
    // calc acc mag
    float accmag = calcmagnitude(&accel[0]);
    if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G)) {
      // normalize acc
      for (int axis = 0; axis < 3; axis++) {
        accel[axis] = accel[axis] * (ACC_1G / accmag);
      }

      float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)FASTFILTER);
      for (int x = 0; x < 3; x++) {
        lpf(&GEstG[x], accel[x], filtcoeff);
      }
    }
  } else {
    //lateshift bartender - quad is IN AIR and things are getting wild
    // hit accel[3] with a sledgehammer
#ifdef PREFILTER
    float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)PREFILTER);
    for (int x = 0; x < 3; x++) {
      lpf(&accel_filter[x], accel[x], filtcoeff);
    }
#else
    for (int x = 0; x < 3; x++) {
      accel_filter[x] = accel[x];
    }
#endif

    // calc mag of filtered acc
    float accmag = calcmagnitude(&accel_filter[0]);
    if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G)) {
      // normalize acc
      for (int axis = 0; axis < 3; axis++) {
        accel_filter[axis] = accel_filter[axis] * (ACC_1G / accmag);
      }
      // filter accel_filter on to GEstG
      float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)FILTERTIME);
      for (int x = 0; x < 3; x++) {
        lpf(&GEstG[x], accel_filter[x], filtcoeff);
      }
    }
    //heal the gravity vector after fusion with accel_filter
    float GEstGmag = calcmagnitude(&GEstG[0]);
    for (int axis = 0; axis < 3; axis++) {
      GEstG[axis] = GEstG[axis] * (ACC_1G / GEstGmag);
    }
  }

  if (rx_aux_on(AUX_HORIZON)) {
    attitude[0] = atan2approx(GEstG[0], GEstG[2]);
    attitude[1] = atan2approx(GEstG[1], GEstG[2]);
  }
}
#endif

#ifdef QUICKSILVER_IMU
void imu_calc(void) {
  const float gyro_delta_angle[3] = {
      gyro[0] * looptime,
      gyro[1] * looptime,
      gyro[2] * looptime,
  };

  GEstG[2] = GEstG[2] - (gyro_delta_angle[0]) * GEstG[0];
  GEstG[0] = (gyro_delta_angle[0]) * GEstG[2] + GEstG[0];

  GEstG[1] = GEstG[1] + (gyro_delta_angle[1]) * GEstG[2];
  GEstG[2] = -(gyro_delta_angle[1]) * GEstG[1] + GEstG[2];

  GEstG[0] = GEstG[0] - (gyro_delta_angle[2]) * GEstG[1];
  GEstG[1] = (gyro_delta_angle[2]) * GEstG[0] + GEstG[1];

  // remove bias and reduce to accel in G
  accel[0] = (accel[0] - accelcal[0]) * (1 / 2048.0f);
  accel[1] = (accel[1] - accelcal[1]) * (1 / 2048.0f);
  accel[2] = (accel[2] - accelcal[2]) * (1 / 2048.0f);

  filter_lp_pt1_coeff(&filter, PT1_FILTER_HZ);

  accel_filter[0] = filter_lp_pt1_step(&filter, &filter_pass1[0], accel[0]);
  accel_filter[1] = filter_lp_pt1_step(&filter, &filter_pass1[1], accel[1]);
  accel_filter[2] = filter_lp_pt1_step(&filter, &filter_pass1[2], accel[2]);

  accel_filter[0] = filter_lp_pt1_step(&filter, &filter_pass2[0], accel_filter[0]);
  accel_filter[1] = filter_lp_pt1_step(&filter, &filter_pass2[1], accel_filter[1]);
  accel_filter[2] = filter_lp_pt1_step(&filter, &filter_pass2[2], accel_filter[2]);

  const float accmag = calcmagnitude(accel_filter);
  if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G)) {
    accel_filter[0] = accel_filter[0] * (ACC_1G / accmag);
    accel_filter[1] = accel_filter[1] * (ACC_1G / accmag);
    accel_filter[2] = accel_filter[2] * (ACC_1G / accmag);

    extern int onground;
    if (onground) {
      //happyhour bartender - quad is ON GROUND and disarmed
      const float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)FASTFILTER);
      lpf(&GEstG[0], accel_filter[0], filtcoeff);
      lpf(&GEstG[1], accel_filter[1], filtcoeff);
      lpf(&GEstG[2], accel_filter[2], filtcoeff);
    } else {
      //lateshift bartender - quad is IN AIR and things are getting wild
      const float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)FILTERTIME);
      lpf(&GEstG[0], accel_filter[0], filtcoeff);
      lpf(&GEstG[1], accel_filter[1], filtcoeff);
      lpf(&GEstG[2], accel_filter[2], filtcoeff);
    }
  }

  //heal the gravity vector after fusion with accel
  const float GEstGmag = calcmagnitude(GEstG);
  GEstG[0] = GEstG[0] * (ACC_1G / GEstGmag);
  GEstG[1] = GEstG[1] * (ACC_1G / GEstGmag);
  GEstG[2] = GEstG[2] * (ACC_1G / GEstGmag);

  if (rx_aux_on(AUX_HORIZON)) {
    attitude[0] = atan2approx(GEstG[0], GEstG[2]);
    attitude[1] = atan2approx(GEstG[1], GEstG[2]);
  }
}
#endif