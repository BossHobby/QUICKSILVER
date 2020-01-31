#pragma once

#include <stddef.h>
#include <stdint.h>

#define M_PI 3.14159265358979323846 /* pi */

float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void limitf(float *input, const float limit);
float constrainf(const float in, const float min, const float max);

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

int8_t buf_equal(const uint8_t *str1, size_t len1, const uint8_t *str2, size_t len2);
int8_t buf_equal_string(const uint8_t *str1, size_t len1, const char *str2);

typedef struct {
  uint8_t *const buffer;
  uint32_t head;
  uint32_t tail;
  const uint32_t size;
} circular_buffer_t;

uint8_t circular_buffer_write(volatile circular_buffer_t *c, uint8_t data);
uint8_t circular_buffer_read(volatile circular_buffer_t *c, uint8_t *data);

void reset_looptime();