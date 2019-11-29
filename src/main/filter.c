#include "filter.h"
#include "defines.h"

#ifndef GYRO_FILTER_PASS1
#define SOFT_LPF1_NONE
#endif
#ifndef GYRO_FILTER_PASS2
#define SOFT_LPF2_NONE
#endif

extern float looptime;

#if defined(PT1_GYRO) && defined(GYRO_FILTER_PASS1)
#define SOFT_LPF_1ST_PASS1 GYRO_FILTER_PASS1
typedef struct {
  float lpf_last;
} filter_lpf1;

static float alpha = 0.5;

filter_lpf1 filter1[3];

void filter_lpf1_coeff() {
  alpha = FILTERCALC(looptime, (1.0f / SOFT_LPF_1ST_PASS1));
}

void filter_lpf1_init(filter_lpf1 *filter) {
  filter->lpf_last = 0;
}

float filter_lpf1_step(filter_lpf1 *filter, float in) {
  lpf(&filter->lpf_last, in, alpha);
  return filter->lpf_last;
}
#define filter1_init filter_lpf1_init
#define filter1_step filter_lpf1_step
#endif

#if defined(PT1_GYRO) && defined(GYRO_FILTER_PASS2)
#define SOFT_LPF_1ST_PASS2 GYRO_FILTER_PASS2
typedef struct {
  float lpf_last;
} filter_lpf2;

static float alpha2 = 0.5;

filter_lpf2 filter2[3];

void filter_lpf2_coeff() {
  alpha2 = FILTERCALC(looptime, (1.0f / SOFT_LPF_1ST_PASS2));
}

void filter_lpf2_init(filter_lpf2 *filter) {
  filter->lpf_last = 0;
}

float filter_lpf2_step(filter_lpf2 *filter, float in) {
  lpf(&filter->lpf_last, in, alpha2);
  return filter->lpf_last;
}
#define filter2_init filter_lpf2_init
#define filter2_step filter_lpf2_step
#endif

#if defined(KALMAN_GYRO) && defined(GYRO_FILTER_PASS1)
#define SOFT_KALMAN_GYRO_PASS1 GYRO_FILTER_PASS1
typedef struct {
  float x_est_last;
  float P_last;
  float Q;
  float R;
} filter_kalman;

void filter_kalman_init(filter_kalman *filter) {
  filter->Q = 0.02;
  filter->R = 0.1;

#ifdef SOFT_KALMAN_GYRO_PASS1
  filter->R = filter->Q / (float)SOFT_KALMAN_GYRO_PASS1;
#endif
}

float filter_kalman_step(filter_kalman *filter, float in) {
  //do a prediction
  float x_temp_est = filter->x_est_last;
  float P_temp = filter->P_last + filter->Q;

  float K = P_temp * (1.0f / (P_temp + filter->R));
  float x_est = x_temp_est + K * (in - x_temp_est);
  float P = (1 - K) * P_temp;

  //update our last's
  filter->P_last = P;
  filter->x_est_last = x_est;

  return x_est;
}

filter_kalman filter1[3];
#define filter1_init filter_kalman_init
#define filter1_step filter_kalman_step
#endif

#if defined(KALMAN_GYRO) && defined(GYRO_FILTER_PASS2)
#define SOFT_KALMAN_GYRO_PASS2 GYRO_FILTER_PASS2
typedef struct {
  float x_est_last;
  float P_last;
  float Q;
  float R;
} filter_kalman2;

void filter_kalman2_init(filter_kalman2 *filter) {
  filter->Q = 0.02;
  filter->R = 0.1;

#ifdef SOFT_KALMAN_GYRO_PASS2
  filter->R = filter->Q / (float)SOFT_KALMAN_GYRO_PASS2;
#endif
}

float filter_kalman2_step(filter_kalman2 *filter, float in) {
  //do a prediction
  float x_temp_est = filter->x_est_last;
  float P_temp = filter->P_last + filter->Q;

  float K = P_temp * (1.0f / (P_temp + filter->R));
  float x_est = x_temp_est + K * (in - x_temp_est);
  float P = (1 - K) * P_temp;

  //update our last's
  filter->P_last = P;
  filter->x_est_last = x_est;

  return x_est;
}

filter_kalman2 filter2[3];
#define filter2_init filter_kalman2_init
#define filter2_step filter_kalman2_step
#endif

void filter_init() {
  for (uint8_t i = 0; i < 3; i++) {
#ifndef SOFT_LPF1_NONE
    filter1_init(&filter1[i]);
#endif
#ifndef SOFT_LPF2_NONE
    filter2_init(&filter2[i]);
#endif
  }
}

float lpffilter(float in, int num) {
#ifdef SOFT_LPF1_NONE
  return in;
#else

#ifdef SOFT_LPF_1ST_PASS1
  if (num == 0)
    filter_lpf1_coeff();
#endif

  return filter1_step(&filter1[num], in);
#endif
}

float lpffilter2(float in, int num) {
#ifdef SOFT_LPF2_NONE
  return in;
#else

#ifdef SOFT_LPF_1ST_PASS2
  if (num == 0)
    filter_lpf2_coeff();
#endif

  return filter2_step(&filter2[num], in);
#endif
}

// 16Hz hpf filter for throttle compensation
//High pass bessel filter order=1 alpha1=0.016
typedef struct {
  float v[2];
} filter_be_hp1;

void filter_be_hp1_init(filter_be_hp1 *filter) {
  filter->v[0] = 0.0;
}

float filter_be_hp1_step(filter_be_hp1 *filter, float x) //class II
{
  filter->v[0] = filter->v[1];
  filter->v[1] = (9.521017968695103528e-1f * x) + (0.90420359373902081668f * filter->v[0]);
  return (filter->v[1] - filter->v[0]);
}

filter_be_hp1 throttlehpf1;

float throttlehpf(float in) {
  return filter_be_hp1_step(&throttlehpf1, in);
}

// for TRANSIENT_WINDUP_PROTECTION feature
//Low pass bessel filter order=1 alpha1=0.023
typedef struct {
  float v[2];
} filter_sp;

void filter_sp_init(filter_sp *filter) {
  filter->v[0] = 0.0;
}

float filter_sp_step(filter_sp *filter, float x) { //class II
  filter->v[0] = filter->v[1];
  filter->v[1] = (6.749703162983405891e-2f * x) + (0.86500593674033188218f * filter->v[0]);
  return (filter->v[0] + filter->v[1]);
}

filter_sp spfilter[3];

float splpf(float in, int num) {
  return filter_sp_step(&spfilter[num], in);
}
