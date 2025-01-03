#pragma once

#include <stdint.h>

#define FILTER_MAX_SLOTS 2

typedef enum {
  FILTER_NONE,
  FILTER_LP_PT1,
  FILTER_LP_PT2,
  FILTER_LP_PT3,

  FILTER_MAX
} __attribute__((__packed__)) filter_type_t;

typedef struct {
  float delay_element[3];
} filter_state_t;

typedef struct {
  float x1;
  float x2;
  float y1;
  float y2;
} filter_biquad_state_t;

typedef struct {
  float hz;
  float sample_period_us;

  float alpha;
} filter_lp_pt1;

typedef struct {
  float hz;
  float sample_period_us;

  float alpha;
} filter_lp_pt2;

typedef struct {
  float hz;
  float sample_period_us;

  float alpha;
} filter_lp_pt3;

typedef struct {
  float hz;
  uint32_t sample_period_us;

  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
} filter_biquad_notch_t;

typedef union {
  filter_lp_pt1 lp_pt1;
  filter_lp_pt2 lp_pt2;
  filter_lp_pt3 lp_pt3;
} filter_t;

typedef struct {
  float v[2];
} filter_lp_sp;

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
    typeof(coeff) _coeff = (coeff);                          \
    typeof(in) _in = (in);                                   \
    (*(out) = (*(out)) * (_coeff) + (_in) * (1 - (_coeff))); \
  })

void filter_global_init();

void filter_lp_pt1_init(filter_lp_pt1 *filter, filter_state_t *state, uint8_t count, float hz);
void filter_lp_pt1_coeff(filter_lp_pt1 *filter, float hz);
float filter_lp_pt1_step(filter_lp_pt1 *filter, filter_state_t *state, float in);

void filter_lp_pt2_init(filter_lp_pt2 *filter, filter_state_t *state, uint8_t count, float hz);
void filter_lp_pt2_coeff(filter_lp_pt2 *filter, float hz);
float filter_lp_pt2_step(filter_lp_pt2 *filter, filter_state_t *state, float in);

void filter_lp_pt3_init(filter_lp_pt3 *filter, filter_state_t *state, uint8_t count, float hz);
void filter_lp_pt3_coeff(filter_lp_pt3 *filter, float hz);
float filter_lp_pt3_step(filter_lp_pt3 *filter, filter_state_t *state, float in);

void filter_biquad_notch_init(filter_biquad_notch_t *filter, filter_biquad_state_t *state, uint8_t count, float hz);
void filter_biquad_notch_coeff(filter_biquad_notch_t *filter, float hz);
float filter_biquad_notch_step(filter_biquad_notch_t *filter, filter_biquad_state_t *state, float in);

void filter_lp_sp_init(filter_lp_sp *filter, uint8_t count);
float filter_lp_sp_step(filter_lp_sp *filter, float x);

void filter_hp_be_init(filter_hp_be *filter);
float filter_hp_be_step(filter_hp_be *filter, float x);

void filter_init(filter_type_t type, filter_t *filter, filter_state_t *state, uint8_t count, float hz);
void filter_coeff(filter_type_t type, filter_t *filter, float hz);
float filter_step(filter_type_t type, filter_t *filter, filter_state_t *state, float in);

float throttlehpf(float in);