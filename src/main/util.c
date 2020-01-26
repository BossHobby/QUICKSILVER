/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include <math.h>
#include <string.h>

#include "drv_time.h"
#include "project.h"
#include "util.h"

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;
}

void limitf(float *input, const float limit) {
  if (*input > limit)
    *input = limit;
  if (*input < -limit)
    *input = -limit;
}

float constrainf(const float in, const float min, const float max) {
  if (in > max)
    return max;
  if (in < min)
    return min;
  return in;
}

// timing routines for debugging
static unsigned long timestart;
unsigned long timeend;

// timestart
void TS(void) {
  timestart = gettime();
}
// timeend
unsigned long TE(void) {
  return timeend = (gettime() - timestart);
}

float fastsin(float x) {
  //always wrap input angle to -PI..PI
  while (x < -3.14159265f)
    x += 6.28318531f;

  while (x > 3.14159265f)
    x -= 6.28318531f;
  float sin1;

  //compute sine
  if (x < 0)
    sin1 = (1.27323954f + .405284735f * x) * x;
  else
    sin1 = (1.27323954f - .405284735f * x) * x;

  return sin1;
}

float fastcos(float x) {
  x += 1.57079632f;
  return fastsin(x);
}

int ipow(int base, int exp) {
  int result = 1;
  for (;;) {
    if (exp & 1)
      result *= base;
    exp >>= 1;
    if (!exp)
      break;
    base *= base;
  }

  return result;
}

int round_num(float num) {
  return num < 0 ? num - 0.5 : num + 0.5;
}

#include <inttypes.h>
uint32_t seed = 7;
uint32_t random(void) {
  seed ^= seed << 13;
  seed ^= seed >> 17;
  seed ^= seed << 5;
  return seed;
}

#define OCTANTIFY(_x, _y, _o) \
  do {                        \
    float _t;                 \
    _o = 0;                   \
    if (_y < 0) {             \
      _x = -_x;               \
      _y = -_y;               \
      _o += 4;                \
    }                         \
    if (_x <= 0) {            \
      _t = _x;                \
      _x = _y;                \
      _y = -_t;               \
      _o += 2;                \
    }                         \
    if (_x <= _y) {           \
      _t = _y - _x;           \
      _x = _x + _y;           \
      _y = _t;                \
      _o += 1;                \
    }                         \
  } while (0);

// +-0.09 deg error
float atan2approx(float y, float x) {

  if (x == 0)
    x = 123e-15f;
  float phi = 0;
  float dphi;
  float t;

  OCTANTIFY(x, y, phi);

  t = (y / x);
  // atan function for 0 - 1 interval
  dphi = t * ((M_PI / 4 + 0.2447f) + t * ((-0.2447f + 0.0663f) + t * (-0.0663f)));
  phi *= M_PI / 4;
  dphi = phi + dphi;
  if (dphi > (float)M_PI)
    dphi -= 2 * M_PI;
  return RADTODEG * dphi;
}

//#pragma GCC push_options
//#pragma GCC optimize ("O0")
// from http://en.wikipedia.org/wiki/Fast_inverse_square_root
// originally from quake3 code
float Q_rsqrt(float number) {
  long i;
  float x2, y;
  const float threehalfs = 1.5F;

  x2 = number * 0.5F;
  y = number;
  i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (threehalfs - (x2 * y * y)); // 1st iteration
  y = y * (threehalfs - (x2 * y * y)); // 2nd iteration, this can be removed
                                       //	y  = y * ( threehalfs - ( x2 * y * y ) );   // 3nd iteration, this can be removed

  return y;
}
//#pragma GCC pop_options

// serial print routines
#ifdef SERIAL_ENABLE

extern void buffer_add(int val);
#include <stdlib.h>

// print a 32bit signed int
void print_int(int val) {

#define SP_INT_BUFFERSIZE 12
  char buffer2[SP_INT_BUFFERSIZE];

  if (val < 0) {
    buffer_add((char)'-');
    val = abs(val);
  }

  int power = SP_INT_BUFFERSIZE;

  do {
    power--;
    int quotient = val / (10);
    int remainder = val - quotient * 10;
    val = quotient;
    buffer2[power] = remainder + '0';
  } while ((val) && power >= 0);

  for (; power <= SP_INT_BUFFERSIZE - 1; power++) {
    buffer_add(buffer2[power]);
  }
}

// print float with 2 decimal points
// this does not handle Nans inf and values over 32bit signed int
void print_float(float val) {
  int ival = (int)val;

  if (val < 0 && ival == 0)
    buffer_add((char)'-');

  print_int(ival);

  buffer_add((char)'.');

  val = val - (int)val;

  int decimals = val * 100;

  decimals = abs(decimals);

  if (decimals < 10)
    buffer_add((char)'0');
  print_int(decimals);
}

void print_str(const char *str) {
  int count = 0;
  // a 64 character limit so we don't print the entire flash by mistake
  while (str[count] && !(count >> 6)) {
    buffer_add((char)str[count]);
    count++;
  }
}

#endif

int8_t buf_equal(const uint8_t *str1, size_t len1, const uint8_t *str2, size_t len2) {
  if (len2 != len1) {
    return 0;
  }
  for (size_t i = 0; i < len1; i++) {
    if (str1[i] != str2[i]) {
      return 0;
    }
  }
  return 1;
}

int8_t buf_equal_string(const uint8_t *str1, size_t len1, const char *str2) {
  return buf_equal(str1, len1, (const uint8_t *)str2, strlen(str2));
}

uint8_t circular_buffer_write(volatile circular_buffer_t *c, uint8_t data) {
  uint32_t next = c->head + 1;
  if (next >= c->size)
    next = 0;

  if (next == c->tail)
    return 0;

  c->buffer[c->head] = data;
  c->head = next;
  return 1;
}

uint8_t circular_buffer_read(volatile circular_buffer_t *c, uint8_t *data) {
  if (c->head == c->tail)
    return 0;

  uint32_t next = c->tail + 1;
  if (next >= c->size)
    next = 0;

  *data = c->buffer[c->tail];
  c->tail = next;
  return 1;
}