#pragma once

#define M_PI 3.14159265358979323846 /* pi */

void lpf(float *out, float in, float coeff);

float lpfcalc(float sampleperiod, float filtertime);
float lpfcalc_hz(float sampleperiod, float filterhz);

float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void limitf(float *input, const float limit);

int round_num(float num);
float round_dec2(float num);

void TS(void);
unsigned long TE(void);

float atan2approx(float y, float x);
float Q_rsqrt(float number);
int ipow(int base, int exp);
float fastsin(float x);
float fastcos(float x);

void limit180(float *);
