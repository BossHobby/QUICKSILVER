#include "filter.h"

#include "control.h"
#include "math.h"
#include "project.h"

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

void filter_lp_pt1_init(filter_lp_pt1 *filter, filter_state_t *state, uint8_t count, float hz) {
  filter->alpha = FILTERCALC((LOOPTIME * 1e-6), (1.0f / hz));

  for (uint8_t i = 0; i < count; i++) {
    state[i].delay_element[0] = 0;
    state[i].delay_element[1] = 0;
  }
}

void filter_lp_pt1_coeff(filter_lp_pt1 *filter, float hz) {
  filter->alpha = FILTERCALC(state.looptime, (1.0f / hz));
}

float filter_lp_pt1_step(filter_lp_pt1 *filter, filter_state_t *state, float in) {
  const float out = state->delay_element[0] * filter->alpha + in * (1 - filter->alpha);

  state->delay_element[0] = out;

  return out;
}

void filter_lp2_pt1_init(filter_lp2_pt1 *filter, filter_state_t *state, uint8_t count, float hz) {
  const float alpha = FILTERCALC((LOOPTIME * 1e-6), (1.0f / hz));

  filter->two_one_minus_alpha = 2 * alpha;
  filter->one_minus_alpha_sqr = (alpha) * (alpha);
  filter->alpha_sqr = (1 - alpha) * (1 - alpha);

  for (uint8_t i = 0; i < count; i++) {
    state[i].delay_element[0] = 0;
    state[i].delay_element[1] = 0;
  }
}

void filter_lp2_pt1_coeff(filter_lp2_pt1 *filter, float hz) {
  const float alpha = FILTERCALC(state.looptime, (1.0f / hz));

  filter->two_one_minus_alpha = 2 * alpha;
  filter->one_minus_alpha_sqr = (alpha) * (alpha);
  filter->alpha_sqr = (1 - alpha) * (1 - alpha);
}

float filter_lp2_pt1_step(filter_lp2_pt1 *filter, filter_state_t *state, float in) {
  const float out = in * filter->alpha_sqr + filter->two_one_minus_alpha * state->delay_element[0] - filter->one_minus_alpha_sqr * state->delay_element[1];

  state->delay_element[1] = state->delay_element[0];
  state->delay_element[0] = out;

  return out;
}


// 16Hz hpf filter for throttle compensation
// High pass bessel filter order=1 alpha1=0.016
void filter_hp_be_init(filter_hp_be *filter) {
  filter->v[0] = 0.0;
}

float filter_hp_be_step(filter_hp_be *filter, float x) { //class II
  filter->v[0] = filter->v[1];
  filter->v[1] = (9.521017968695103528e-1f * x) + (0.90420359373902081668f * filter->v[0]);
  return (filter->v[1] - filter->v[0]);
}

// for TRANSIENT_WINDUP_PROTECTION feature
// Low pass bessel filter order=1 alpha1=0.023
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

void filter_global_init() {
  filter_hp_be_init(&throttlehpf1);
  filter_lp_sp_init(spfilter, 3);
}

void filter_init(filter_type_t type, filter_t *filter, filter_state_t *state, uint8_t count, float hz) {
  switch (type) {
  case FILTER_LP_PT1:
    filter_lp_pt1_init(&filter->lp_pt1, state, count, hz);
    break;
  case FILTER_LP2_PT1:
    filter_lp2_pt1_init(&filter->lp2_pt1, state, count, hz);
    break;
  default:
    // no filter, do nothing
    break;
  }
}

void filter_coeff(filter_type_t type, filter_t *filter, float hz) {
  switch (type) {
  case FILTER_LP_PT1:
    filter_lp_pt1_coeff(&filter->lp_pt1, hz);
    break;
  case FILTER_LP2_PT1:
    filter_lp2_pt1_coeff(&filter->lp2_pt1, hz);
    break;
  default:
    // no filter, do nothing
    break;
  }
}

float filter_step(filter_type_t type, filter_t *filter, filter_state_t *state, float in) {
  switch (type) {
  case FILTER_LP_PT1:
    return filter_lp_pt1_step(&filter->lp_pt1, state, in);
  case FILTER_LP2_PT1:
    return filter_lp2_pt1_step(&filter->lp2_pt1, state, in);
  default:
    // no filter at all
    return in;
  }
}
