#pragma once

#include <stdint.h>

#include "util/vector.h"

#define FILTER_MAX_SLOTS 2

// Poles of the highest low-pass order, FILTER_LP_PT3.
static constexpr uint32_t FILTER_MAX_ORDER = 3;

typedef enum {
  FILTER_NONE,
  FILTER_LP_PT1,
  FILTER_LP_PT2,
  FILTER_LP_PT3,

  FILTER_MAX
} __attribute__((__packed__)) filter_type_t;

// filter_coeff_t::coeff uses a low-pass type's value as its pole count.
static_assert(FILTER_LP_PT1 == 1 && FILTER_LP_PT2 == 2 && FILTER_LP_PT3 == FILTER_MAX_ORDER, "low-pass type is its order");

// Cutoff and smoothing factor shared by every channel of a low-pass filter.
struct filter_coeff_t {
  float hz;
  float sample_period_us;

  float alpha;
  uint32_t order; // poles; equals the configured filter_type_t

  void coeff(filter_type_t type, float cutoff_hz, float period_us);
};

// Low-pass of 1-3 cascaded poles over a float, vec3_t or vec4_t. FILTER_NONE
// passes values through.
template <typename value_t = float>
struct filter_t : filter_coeff_t {
  value_t pole[FILTER_MAX_ORDER];

  void init(filter_type_t type, float cutoff_hz, float period_us) {
    *this = {};
    coeff(type, cutoff_hz, period_us);
  }

  void reset() {
    for (value_t &p : pole)
      p = {};
  }

  // Specialized in filter.cpp for each value type in use: LTO drops the
  // HOT_CODE section from template instances, not from specializations.
  value_t step(value_t v);
};

template <>
float filter_t<float>::step(float v);
template <>
vec3_t filter_t<vec3_t>::step(vec3_t v);
template <>
vec4_t filter_t<vec4_t>::step(vec4_t v);

// Second-order notch with its own history. Disabled below 0.1 Hz.
struct filter_biquad_notch_t {
  float hz;
  float sample_period_us;

  float b0;
  float b1;
  float b2;
  float a1;
  float a2;

  float x1;
  float x2;
  float y1;
  float y2;

  void init(float cutoff_hz, float period_us);
  void coeff(float cutoff_hz, float period_us);

  float step(float in);
};

typedef struct {
  float v[2];
} filter_hp_be;

//(1 - alpha. filtertime = 1 / filter-cutoff-frequency) as long as filtertime > sampleperiod
#define lpfcalc(sampleperiod, filtertime) ({                                                   \
  const float _sampleperiod = sampleperiod;                                                    \
  const float _filtertime = filtertime;                                                        \
  constrain(1.0f - (6.0f * _sampleperiod) / (3.0f * _sampleperiod + _filtertime), 0.0f, 1.0f); \
})
#define lpfcalc_hz(sampleperiod, filter_hz) lpfcalc(sampleperiod, 1.0f / filter_hz)

#define lpf(out, in, coeff)                                  \
  ({                                                         \
    __typeof__(coeff) _coeff = (coeff);                          \
    __typeof__(in) _in = (in);                                   \
    (*(out) = (*(out)) * (_coeff) + (_in) * (1 - (_coeff))); \
  })

void filter_global_init();

void filter_hp_be_init(filter_hp_be *filter);
float filter_hp_be_step(filter_hp_be *filter, float x);

float throttlehpf(float in);