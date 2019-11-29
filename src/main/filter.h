#pragma once

#define IMU_SAMPLE_RATE 200.0f
#define IMU_FILTER_CUTOFF_FREQ 30.0f
#define M_PI_F 3.1415926

typedef struct {
  float cutoff_freq;
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
  float delay_element_1; /* Buffered sample -1 */
  float delay_element_2; /* Buffered sample -2 */
} iir_filter_lpf2;

void iir_filter_lpf2_set_freq(iir_filter_lpf2 *filter, float sample_freq, float cutoff_freq);
float iir_filter_lpf2_apply(iir_filter_lpf2 *filter, float sample);

float lpfcalc(float sampleperiod, float filtertime);
float lpfcalc_hz(float sampleperiod, float filterhz);
void lpf(float *out, float in, float coeff);

float lpffilter(float in, int num);
float lpffilter2(float in, int num);
float throttlehpf(float in);