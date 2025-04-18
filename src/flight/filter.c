#include "flight/filter.h"

#include <math.h>
#include <string.h>

#include "core/project.h"
#include "flight/control.h"
#include "util/util.h"

#define NOTCH_Q 3.0f

// equation is 1 / sqrtf(powf(2, 1.0f / ORDER) - 1);
#define ORDER1_CORRECTION 1
#define ORDER2_CORRECTION 1.55377397403f
#define ORDER3_CORRECTION 1.9614591767f

void filter_init_state(filter_state_t *state, uint8_t count) {
  memset(state, 0, count * sizeof(filter_state_t));
}

void filter_lp_pt1_init(filter_lp_pt1 *filter, filter_state_t *state, uint8_t count, float hz, float sample_period_us) {
  filter_lp_pt1_coeff(filter, hz, sample_period_us);
  filter_init_state(state, count);
}

void filter_lp_pt1_coeff(filter_lp_pt1 *filter, float hz, float sample_period_us) {
  if (filter->hz == hz && filter->sample_period_us == sample_period_us) {
    return;
  }
  filter->hz = hz;
  filter->sample_period_us = sample_period_us;

  const float rc = 1 / (2 * ORDER1_CORRECTION * M_PI_F * hz);
  const float sample_period = sample_period_us * 1e-6f;

  filter->alpha = sample_period / (rc + sample_period);
}

float filter_lp_pt1_step(filter_lp_pt1 *filter, filter_state_t *state, float in) {
  state->delay_element[0] = state->delay_element[0] + filter->alpha * (in - state->delay_element[0]);
  return state->delay_element[0];
}

void filter_lp_pt2_init(filter_lp_pt2 *filter, filter_state_t *state, uint8_t count, float hz, float sample_period_us) {
  filter_lp_pt2_coeff(filter, hz, sample_period_us);
  filter_init_state(state, count);
}

void filter_lp_pt2_coeff(filter_lp_pt2 *filter, float hz, float sample_period_us) {
  if (filter->hz == hz && filter->sample_period_us == sample_period_us) {
    return;
  }
  filter->hz = hz;
  filter->sample_period_us = sample_period_us;

  const float rc = 1 / (2 * ORDER2_CORRECTION * M_PI_F * hz);
  const float sample_period = sample_period_us * 1e-6f;

  filter->alpha = sample_period / (rc + sample_period);
}

float filter_lp_pt2_step(filter_lp_pt2 *filter, filter_state_t *state, float in) {
  state->delay_element[1] = state->delay_element[1] + filter->alpha * (in - state->delay_element[1]);
  state->delay_element[0] = state->delay_element[0] + filter->alpha * (state->delay_element[1] - state->delay_element[0]);
  return state->delay_element[0];
}

void filter_lp_pt3_init(filter_lp_pt3 *filter, filter_state_t *state, uint8_t count, float hz, float sample_period_us) {
  filter_lp_pt3_coeff(filter, hz, sample_period_us);
  filter_init_state(state, count);
}

void filter_lp_pt3_coeff(filter_lp_pt3 *filter, float hz, float sample_period_us) {
  if (filter->hz == hz && filter->sample_period_us == sample_period_us) {
    return;
  }
  filter->hz = hz;
  filter->sample_period_us = sample_period_us;

  const float rc = 1 / (2 * ORDER3_CORRECTION * M_PI_F * hz);
  const float sample_period = sample_period_us * 1e-6f;

  filter->alpha = sample_period / (rc + sample_period);
}

float filter_lp_pt3_step(filter_lp_pt3 *filter, filter_state_t *state, float in) {
  state->delay_element[1] = state->delay_element[1] + filter->alpha * (in - state->delay_element[1]);
  state->delay_element[2] = state->delay_element[2] + filter->alpha * (state->delay_element[1] - state->delay_element[2]);
  state->delay_element[0] = state->delay_element[0] + filter->alpha * (state->delay_element[2] - state->delay_element[0]);
  return state->delay_element[0];
}

void filter_biquad_notch_init(filter_biquad_notch_t *filter, filter_biquad_state_t *state, uint8_t count, float hz, float sample_period_us) {
  memset(filter, 0, sizeof(filter_biquad_notch_t));
  filter_biquad_notch_coeff(filter, hz, sample_period_us);
  memset(state, 0, count * sizeof(filter_biquad_state_t));
}

void filter_biquad_notch_coeff(filter_biquad_notch_t *filter, float hz, float sample_period_us) {
  if (filter->hz == hz && filter->sample_period_us == sample_period_us) {
    return;
  }
  if (hz < 0.1f) {
    filter->hz = 0;
    return;
  }

  // from https://webaudio.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
  const float omega = 2.0f * M_PI_F * hz * sample_period_us * 1e-6;
  const float cos_omega = fastcos(omega);
  const float alpha = fastsin(omega) / (2.0f * NOTCH_Q);

  const float a0_rcpt = 1.0f / (1.0f + alpha);

  filter->b0 = 1 * a0_rcpt;
  filter->b1 = (-2 * cos_omega) * a0_rcpt;
  filter->b2 = 1 * a0_rcpt;
  filter->a1 = filter->b1;
  filter->a2 = (1 - alpha) * a0_rcpt;

  filter->hz = hz;
  filter->sample_period_us = sample_period_us;
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

filter_hp_be throttlehpf1;
float throttlehpf(float in) {
  return filter_hp_be_step(&throttlehpf1, in);
}

void filter_global_init() {
  filter_hp_be_init(&throttlehpf1);
}

void filter_init(filter_type_t type, filter_t *filter, filter_state_t *state, uint8_t count, float hz, float sample_period_us) {
  switch (type) {
  case FILTER_LP_PT1:
    filter_lp_pt1_init(&filter->lp_pt1, state, count, hz, sample_period_us);
    break;
  case FILTER_LP_PT2:
    filter_lp_pt2_init(&filter->lp_pt2, state, count, hz, sample_period_us);
    break;
  case FILTER_LP_PT3:
    filter_lp_pt3_init(&filter->lp_pt3, state, count, hz, sample_period_us);
    break;
  default:
    // no filter, do nothing
    break;
  }
}

void filter_coeff(filter_type_t type, filter_t *filter, float hz, float sample_period_us) {
  switch (type) {
  case FILTER_LP_PT1:
    filter_lp_pt1_coeff(&filter->lp_pt1, hz, sample_period_us);
    break;
  case FILTER_LP_PT2:
    filter_lp_pt2_coeff(&filter->lp_pt2, hz, sample_period_us);
    break;
  case FILTER_LP_PT3:
    filter_lp_pt3_coeff(&filter->lp_pt3, hz, sample_period_us);
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
