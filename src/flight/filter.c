#include "flight/filter.h"

#include <math.h>
#include <string.h>

#include "flight/control.h"
#include "project.h"
#include "util/util.h"

// equation is 1 / sqrtf(powf(2, 1.0f / ORDER) - 1);
#define ORDER1_CORRECTION 1
#define ORDER2_CORRECTION 1.55377397403f
#define ORDER3_CORRECTION 1.9614591767f

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

static void filter_init_state(filter_state_t *state, uint8_t count) {
  memset(state, 0, count * sizeof(filter_state_t));
}

void filter_lp_pt1_init(filter_lp_pt1 *filter, filter_state_t *state, uint8_t count, float hz) {
  filter_lp_pt1_coeff(filter, hz);
  filter_init_state(state, count);
}

void filter_lp_pt1_coeff(filter_lp_pt1 *filter, float hz) {
  if (filter->hz == hz && filter->sample_period_us == state.looptime_autodetect) {
    return;
  }
  filter->hz = hz;
  filter->sample_period_us = state.looptime_autodetect;

  const float rc = 1 / (2 * ORDER1_CORRECTION * M_PI_F * hz);
  const float sample_period = state.looptime_autodetect * 1e-6f;

  filter->alpha = sample_period / (rc + sample_period);
}

float filter_lp_pt1_step(filter_lp_pt1 *filter, filter_state_t *state, float in) {
  state->delay_element[0] = state->delay_element[0] + filter->alpha * (in - state->delay_element[0]);
  return state->delay_element[0];
}

void filter_lp_pt2_init(filter_lp_pt2 *filter, filter_state_t *state, uint8_t count, float hz) {
  filter_lp_pt2_coeff(filter, hz);
  filter_init_state(state, count);
}

void filter_lp_pt2_coeff(filter_lp_pt2 *filter, float hz) {
  if (filter->hz == hz && filter->sample_period_us == state.looptime_autodetect) {
    return;
  }
  filter->hz = hz;
  filter->sample_period_us = state.looptime_autodetect;

  const float rc = 1 / (2 * ORDER2_CORRECTION * M_PI_F * hz);
  const float sample_period = state.looptime_autodetect * 1e-6f;

  filter->alpha = sample_period / (rc + sample_period);
}

float filter_lp_pt2_step(filter_lp_pt2 *filter, filter_state_t *state, float in) {
  state->delay_element[1] = state->delay_element[1] + filter->alpha * (in - state->delay_element[1]);
  state->delay_element[0] = state->delay_element[0] + filter->alpha * (state->delay_element[1] - state->delay_element[0]);
  return state->delay_element[0];
}

void filter_lp_pt3_init(filter_lp_pt3 *filter, filter_state_t *state, uint8_t count, float hz) {
  filter_lp_pt3_coeff(filter, hz);
  filter_init_state(state, count);
}

void filter_lp_pt3_coeff(filter_lp_pt3 *filter, float hz) {
  if (filter->hz == hz && filter->sample_period_us == state.looptime_autodetect) {
    return;
  }
  filter->hz = hz;
  filter->sample_period_us = state.looptime_autodetect;

  const float rc = 1 / (2 * ORDER3_CORRECTION * M_PI_F * hz);
  const float sample_period = state.looptime_autodetect * 1e-6f;

  filter->alpha = sample_period / (rc + sample_period);
}

float filter_lp_pt3_step(filter_lp_pt3 *filter, filter_state_t *state, float in) {
  state->delay_element[1] = state->delay_element[1] + filter->alpha * (in - state->delay_element[1]);
  state->delay_element[2] = state->delay_element[2] + filter->alpha * (state->delay_element[1] - state->delay_element[2]);
  state->delay_element[0] = state->delay_element[0] + filter->alpha * (state->delay_element[2] - state->delay_element[0]);
  return state->delay_element[0];
}

void filter_biquad_notch_init(filter_biquad_notch_t *filter, filter_biquad_state_t *state, uint8_t count, float hz) {
  memset(filter, 0, sizeof(filter_biquad_notch_t));
  filter_biquad_notch_coeff(filter, hz);
  memset(state, 0, count * sizeof(filter_biquad_state_t));
}

void filter_biquad_notch_coeff(filter_biquad_notch_t *filter, float hz) {
  if (filter->hz == hz || hz < 0.1f) {
    filter->hz = 0;
    return;
  }

  // from https://webaudio.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
  const float Q = constrainf(hz / 20.0f, 0.0f, 6.0f);

  const float omega = 2.0f * M_PI_F * hz * state.looptime;
  const float cos_omega = fastcos(omega);
  const float alpha = fastsin(omega) / (2.0f * Q);

  const float a0_rcpt = 1.0f / (1.0f + alpha);

  filter->b0 = 1 * a0_rcpt;
  filter->b1 = (-2 * cos_omega) * a0_rcpt;
  filter->b2 = 1 * a0_rcpt;
  filter->a1 = filter->b1;
  filter->a2 = (1 - alpha) * a0_rcpt;

  filter->hz = hz;
}

float filter_biquad_notch_step(filter_biquad_notch_t *filter, filter_biquad_state_t *state, float in) {
  if (filter->hz < 0.1f) {
    return in;
  }

  const float result = filter->b0 * in + filter->b1 * state->x1 + filter->b2 * state->x2 - filter->a1 * state->y1 - filter->a2 * state->y2;

  state->x2 = state->x1;
  state->x1 = in;

  state->y2 = state->y1;
  state->y1 = result;

  return result;
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
