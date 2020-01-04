#include "filter.h"

#include "defines.h"
#include "math.h"
#include "project.h"

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

void filter_lp_pt1_init(filter_lp_pt1 *filter, uint8_t count, float hz) {
  const float alpha = FILTERCALC((LOOPTIME * 1e-6), (1.0f / hz));
  for (uint8_t i = 0; i < count; i++) {
    filter[i].last_out = 0;
    filter[i].alpha = alpha;
  }
}

void filter_lp_pt1_coeff(filter_lp_pt1 *filter, uint8_t count, float hz) {
  const float alpha = FILTERCALC(looptime, (1.0f / hz));
  for (uint8_t i = 0; i < count; i++) {
    filter[i].alpha = alpha;
  }
}

float filter_lp_pt1_step(filter_lp_pt1 *filter, float in) {
  lpf(&filter->last_out, in, filter->alpha);
  return filter->last_out;
}

void filter_lp2_pt1_init(filter_lp2_pt1 *filter, uint8_t count, float hz) {
  const float two_one_minus_alpha = 2 * FILTERCALC((LOOPTIME * 1e-6), (1.0f / hz));
  const float one_minus_alpha_sqr = (FILTERCALC((LOOPTIME * 1e-6), (1.0f / hz))) * (FILTERCALC((LOOPTIME * 1e-6), (1.0f / hz)));
  const float alpha_sqr = (1 - FILTERCALC((LOOPTIME * 1e-6), (1.0f / hz))) * (1 - FILTERCALC((LOOPTIME * 1e-6), (1.0f / hz)));

  for (uint8_t i = 0; i < count; i++) {
    filter[i].last_out = 0;
    filter[i].last_out2 = 0;

    filter[i].two_one_minus_alpha = two_one_minus_alpha;
    filter[i].one_minus_alpha_sqr = one_minus_alpha_sqr;
    filter[i].alpha_sqr = alpha_sqr;
  }
}

void filter_lp2_pt1_coeff(filter_lp2_pt1 *filter, uint8_t count, float hz) {
  const float two_one_minus_alpha = 2 * FILTERCALC(looptime, (1.0f / hz));
  const float one_minus_alpha_sqr = (FILTERCALC(looptime, (1.0f / hz))) * (FILTERCALC(looptime, (1.0f / hz)));
  const float alpha_sqr = (1 - FILTERCALC(looptime, (1.0f / hz))) * (1 - FILTERCALC(looptime, (1.0f / hz)));

  for (uint8_t i = 0; i < count; i++) {
    filter[i].two_one_minus_alpha = two_one_minus_alpha;
    filter[i].one_minus_alpha_sqr = one_minus_alpha_sqr;
    filter[i].alpha_sqr = alpha_sqr;
  }
}

float filter_lp2_pt1_step(filter_lp2_pt1 *filter, float in) {
  const float out = in * filter->alpha_sqr + filter->two_one_minus_alpha * filter->last_out - filter->one_minus_alpha_sqr * filter->last_out2;

  filter->last_out2 = filter->last_out;
  filter->last_out = out;

  return out;
}

#if defined(KALMAN_GYRO)
void filter_kalman_init(filter_kalman *filter, uint8_t count, float coeff) {
  for (uint8_t i = 0; i < count; i++) {
    filter[i].Q = 0.02;
    filter[i].R = 0.1;

    if (coeff > 0) {
      filter[i].R = filter[i].Q / coeff;
    }
  }
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
#endif

// 16Hz hpf filter for throttle compensation
//High pass bessel filter order=1 alpha1=0.016
void filter_hp_be_init(filter_hp_be *filter) {
  filter->v[0] = 0.0;
}

float filter_hp_be_step(filter_hp_be *filter, float x) { //class II
  filter->v[0] = filter->v[1];
  filter->v[1] = (9.521017968695103528e-1f * x) + (0.90420359373902081668f * filter->v[0]);
  return (filter->v[1] - filter->v[0]);
}

// for TRANSIENT_WINDUP_PROTECTION feature
//Low pass bessel filter order=1 alpha1=0.023
void filter_lp_sp_init(filter_lp_sp *filter, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    filter[i].v[0] = 0.0;
  }
}

float filter_lp_sp_step(filter_lp_sp *filter, float x) { //class II
  filter->v[0] = filter->v[1];
  filter->v[1] = (6.749703162983405891e-2f * x) + (0.86500593674033188218f * filter->v[0]);
  return (filter->v[0] + filter->v[1]);
}

void filter_lp2_iir_init(filter_lp2_iir *filter, float sample_freq, float cutoff_freq) {
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

float filter_lp2_iir_step(filter_lp2_iir *filter, float sample) {
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

filter_hp_be throttlehpf1;
float throttlehpf(float in) {
  return filter_hp_be_step(&throttlehpf1, in);
}

filter_lp_sp spfilter[3];
float splpf(float in, int num) {
  return filter_lp_sp_step(&spfilter[num], in);
}

void filter_init() {
  filter_hp_be_init(&throttlehpf1);
  filter_lp_sp_init(spfilter, 3);
}