#include "filter.h"

#include "defines.h"
#include "math.h"
#include "project.h"

#ifndef GYRO_FILTER_PASS1
#define SOFT_LPF1_NONE
#endif
#ifndef GYRO_FILTER_PASS2
#define SOFT_LPF2_NONE
#endif

extern float looptime;

// calculates the coefficient for lpf filter, times in the same units
float lpfcalc(float sampleperiod, float filtertime) {
  float ga = 1.0f - sampleperiod / filtertime;
  if (ga > 1.0f)
    ga = 1.0f;
  if (ga < 0.0f)
    ga = 0.0f;
  return ga;
}

// calculates the coefficient for lpf filter
float lpfcalc_hz(float sampleperiod, float filterhz) {
  float ga = 1.0f - sampleperiod * filterhz;
  if (ga > 1.0f)
    ga = 1.0f;
  if (ga < 0.0f)
    ga = 0.0f;
  return ga;
}

void lpf(float *out, float in, float coeff) {
  *out = (*out) * coeff + in * (1 - coeff);
}

#if defined(PT1_GYRO) && defined(GYRO_FILTER_PASS1)
#define SOFT_LPF_1ST_PASS1 GYRO_FILTER_PASS1
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

void iir_filter_lpf2_set_freq(iir_filter_lpf2 *filter, float sample_freq, float cutoff_freq) {
  const float fr = sample_freq / cutoff_freq;
  const float ohm = tanf(M_PI_F / fr);
  const float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;

  filter->cutoff_freq = cutoff_freq;

  if (filter->cutoff_freq > 0.0f) {
    filter->b0 = ohm * ohm / c;
    filter->b1 = 2.0f * filter->b0;
    filter->b2 = filter->b0;
    filter->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    filter->a2 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
  }
}

float iir_filter_lpf2_apply(iir_filter_lpf2 *filter, float sample) {
  if (filter->cutoff_freq <= 0.0f) {
    return sample; /* No filtering */
  }

  float delay_element_0 = sample - filter->delay_element_1 * filter->a1 - filter->delay_element_2 * filter->a2;
  /* Do the filtering */
  if (isnan(delay_element_0) || isinf(delay_element_0)) {
    /* Don't allow bad values to propogate via the filter */
    delay_element_0 = sample;
  }

  const float output = delay_element_0 * filter->b0 + filter->delay_element_1 * filter->b1 + filter->delay_element_2 * filter->b2;
  filter->delay_element_2 = filter->delay_element_1;
  filter->delay_element_1 = delay_element_0;

  /* Return the value.  Should be no need to check limits */
  return output;
}

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
