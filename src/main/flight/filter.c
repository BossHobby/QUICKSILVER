#include "flight/filter.h"

#include <math.h>

#include "flight/control.h"
#include "math.h"
#include "project.h"

#define M_PI_F 3.14159265358979323846f

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
  filter_lp_pt1_coeff(filter, hz);

  for (uint8_t i = 0; i < count; i++) {
    state[i].delay_element[0] = 0;
    state[i].delay_element[1] = 0;
  }
}

void filter_lp_pt1_coeff(filter_lp_pt1 *filter, float hz) {
  const float cutoff = 1 / sqrtf(powf(2, 1.0f / 1.0f) - 1);
  const float rc = 1 / (2 * cutoff * M_PI_F * hz);

  filter->alpha = state.looptime / (rc + state.looptime);
}

float filter_lp_pt1_step(filter_lp_pt1 *filter, filter_state_t *state, float in) {
  state->delay_element[0] = state->delay_element[0] + filter->alpha * (in - state->delay_element[0]);
  return state->delay_element[0];
}

void filter_lp_pt2_init(filter_lp_pt2 *filter, filter_state_t *state, uint8_t count, float hz) {
  filter_lp_pt2_coeff(filter, hz);

  for (uint8_t i = 0; i < count; i++) {
    state[i].delay_element[0] = 0;
    state[i].delay_element[1] = 0;
  }
}

void filter_lp_pt2_coeff(filter_lp_pt2 *filter, float hz) {
  const float cutoff = 1 / sqrtf(powf(2, 1.0f / 2.0f) - 1);
  const float rc = 1 / (2 * cutoff * M_PI_F * hz);

  filter->alpha = state.looptime / (rc + state.looptime);
}

float filter_lp_pt2_step(filter_lp_pt2 *filter, filter_state_t *state, float in) {
  state->delay_element[1] = state->delay_element[1] + filter->alpha * (in - state->delay_element[1]);
  state->delay_element[0] = state->delay_element[0] + filter->alpha * (state->delay_element[1] - state->delay_element[0]);
  return state->delay_element[0];
}

void filter_lp_pt3_init(filter_lp_pt3 *filter, filter_state_t *state, uint8_t count, float hz) {
  filter_lp_pt3_coeff(filter, hz);

  for (uint8_t i = 0; i < count; i++) {
    state[i].delay_element[0] = 0;
    state[i].delay_element[1] = 0;
  }
}

void filter_lp_pt3_coeff(filter_lp_pt3 *filter, float hz) {
  const float cutoff = 1 / sqrtf(powf(2, 1.0f / 3.0f) - 1);
  const float rc = 1 / (2 * cutoff * M_PI_F * hz);

  filter->alpha = state.looptime / (rc + state.looptime);
}

float filter_lp_pt3_step(filter_lp_pt3 *filter, filter_state_t *state, float in) {
  state->delay_element[1] = state->delay_element[1] + filter->alpha * (in - state->delay_element[1]);
  state->delay_element[2] = state->delay_element[2] + filter->alpha * (state->delay_element[1] - state->delay_element[2]);
  state->delay_element[0] = state->delay_element[0] + filter->alpha * (state->delay_element[2] - state->delay_element[0]);
  return state->delay_element[0];
}

void filter_lp2_iir_init(filter_lp2_iir *filter, filter_state_t *state, uint8_t count, float hz) {
  filter_lp2_iir_coeff(filter, hz);

  for (uint8_t i = 0; i < count; i++) {
    state[i].delay_element[0] = 0;
    state[i].delay_element[1] = 0;
  }
}

void filter_lp2_iir_coeff(filter_lp2_iir *filter, float hz) {
  const float fr = (1 / state.looptime) / hz;
  const float ohm = tanf(M_PI_F / fr);
  const float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;

  filter->b0 = ohm * ohm / c;
  filter->b1 = 2.0f * filter->b0;
  filter->b2 = filter->b0;
  filter->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
  filter->a2 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}

float filter_lp2_iir_step(filter_lp2_iir *filter, filter_state_t *state, float sample) {
  const float delay_element_0 = sample - state->delay_element[0] * filter->a1 - state->delay_element[1] * filter->a2;
  const float output = delay_element_0 * filter->b0 + state->delay_element[0] * filter->b1 + state->delay_element[1] * filter->b2;

  state->delay_element[1] = state->delay_element[0];
  state->delay_element[0] = delay_element_0;

  return output;
}

// 16Hz hpf filter for throttle compensation
// High pass bessel filter order=1 alpha1=0.016
void filter_hp_be_init(filter_hp_be *filter) {
  filter->v[0] = 0.0;
}

float filter_hp_be_step(filter_hp_be *filter, float x) { // class II
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

float filter_lp_sp_step(filter_lp_sp *filter, float x) { // class II
  filter->v[0] = filter->v[1];
  filter->v[1] = (6.749703162983405891e-2f * x) + (0.86500593674033188218f * filter->v[0]);
  return (filter->v[0] + filter->v[1]);
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
  case FILTER_LP_PT2:
    filter_lp_pt2_init(&filter->lp_pt2, state, count, hz);
    break;
  case FILTER_LP_PT3:
    filter_lp_pt3_init(&filter->lp_pt3, state, count, hz);
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
  case FILTER_LP_PT2:
    filter_lp_pt2_coeff(&filter->lp_pt2, hz);
    break;
  case FILTER_LP_PT3:
    filter_lp_pt3_coeff(&filter->lp_pt3, hz);
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
  case FILTER_LP_PT2:
    return filter_lp_pt2_step(&filter->lp_pt2, state, in);
  case FILTER_LP_PT3:
    return filter_lp_pt3_step(&filter->lp_pt3, state, in);
  default:
    // no filter at all
    return in;
  }
}
