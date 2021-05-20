#pragma once

#include <stdint.h>

#define IMU_SAMPLE_RATE 200.0f
#define IMU_FILTER_CUTOFF_FREQ 30.0f
#define M_PI_F 3.1415926

#define FILTER_MAX_SLOTS 2

typedef enum {
  FILTER_NONE,
  FILTER_LP_PT1,
  FILTER_LP2_PT1,
} filter_type_t;

typedef struct {
  float delay_element[2];
} filter_state_t;

typedef struct {
  float alpha;
} filter_lp_pt1;

typedef struct {
  float two_one_minus_alpha;
  float one_minus_alpha_sqr;
  float alpha_sqr;
} filter_lp2_pt1;

typedef union {
  filter_lp_pt1 lp_pt1;
  filter_lp2_pt1 lp2_pt1;
} filter_t;

typedef struct {
  float v[2];
} filter_lp_sp;

typedef struct {
  float cutoff_freq;
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
  float delay_element_1; /* Buffered sample -1 */
  float delay_element_2; /* Buffered sample -2 */
} filter_lp2_iir;

typedef struct {
  float v[2];
} filter_hp_be;

float lpfcalc(float sampleperiod, float filtertime);
float lpfcalc_hz(float sampleperiod, float filterhz);

void lpf(float *out, float in, float coeff);

void filter_global_init();

void filter_lp_pt1_init(filter_lp_pt1 *filter, filter_state_t *state, uint8_t count, float hz);
void filter_lp_pt1_coeff(filter_lp_pt1 *filter, float hz);
float filter_lp_pt1_step(filter_lp_pt1 *filter, filter_state_t *state, float in);

void filter_lp2_pt1_init(filter_lp2_pt1 *filter, filter_state_t *state, uint8_t count, float hz);
void filter_lp2_pt1_coeff(filter_lp2_pt1 *filter, float hz);
float filter_lp2_pt1_step(filter_lp2_pt1 *filter, filter_state_t *state, float in);

void filter_lp_sp_init(filter_lp_sp *filter, uint8_t count);
float filter_lp_sp_step(filter_lp_sp *filter, float x);

void filter_lp2_iir_init(filter_lp2_iir *filter, float sample_freq, float cutoff_freq);
float filter_lp2_iir_step(filter_lp2_iir *filter, float sample);

void filter_hp_be_init(filter_hp_be *filter);
float filter_hp_be_step(filter_hp_be *filter, float x);

void filter_init(filter_type_t type, filter_t *filter, filter_state_t *state, uint8_t count, float hz);
void filter_coeff(filter_type_t type, filter_t *filter, float hz);
float filter_step(filter_type_t type, filter_t *filter, filter_state_t *state, float in);

float throttlehpf(float in);