#include "util/filter.h"

#include <math.h>

#include "control/control.h"
#include "core/project.h"
#include "util/util.h"

constexpr float NOTCH_Q = 3.0f;

// 1 / sqrtf(powf(2, 1.0f / order) - 1): keeps the cascade's -3 dB point at hz.
static constexpr float ORDER_CORRECTION[FILTER_MAX_ORDER + 1] = {0.0f, 1.0f, 1.55377397403f, 1.9614591767f};

void filter_coeff_t::coeff(filter_type_t type, float cutoff_hz, float period_us) {
  // Profiles are external input; unknown types pass through as before.
  const uint8_t poles = type <= FILTER_MAX_ORDER ? type : FILTER_NONE;
  if (order == poles && hz == cutoff_hz && sample_period_us == period_us) {
    return;
  }
  order = poles;
  hz = cutoff_hz;
  sample_period_us = period_us;
  // Pass-through never reads alpha; the dynamic cutoff changes every loop.
  if (poles == FILTER_NONE) {
    return;
  }

  // alpha = dt / (rc + dt) with rc = 1 / (2 pi correction hz)
  const float w = 2.0f * M_PI_F * ORDER_CORRECTION[poles] * cutoff_hz * period_us * 1e-6f;
  alpha = w / (1.0f + w);
}

void filter_biquad_notch_t::init(float cutoff_hz, float period_us) {
  *this = {};
  coeff(cutoff_hz, period_us);
}

void filter_biquad_notch_t::coeff(float cutoff_hz, float period_us) {
  if (hz == cutoff_hz && sample_period_us == period_us) {
    return;
  }
  if (cutoff_hz < 0.1f) {
    hz = 0;
    return;
  }

  // from https://webaudio.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
  const float omega = 2.0f * M_PI_F * cutoff_hz * period_us * 1e-6;
  const float cos_omega = fastcos(omega);
  const float alpha = fastsin(omega) / (2.0f * NOTCH_Q);

  const float a0_rcpt = 1.0f / (1.0f + alpha);

  b0 = 1 * a0_rcpt;
  b1 = (-2 * cos_omega) * a0_rcpt;
  b2 = 1 * a0_rcpt;
  a1 = b1;
  a2 = (1 - alpha) * a0_rcpt;

  hz = cutoff_hz;
  sample_period_us = period_us;
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
