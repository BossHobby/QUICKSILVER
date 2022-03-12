#include "util/util.h"

#include <math.h>
#include <string.h>

#include "drv_time.h"
#include "project.h"

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

uint32_t min_uint32(uint32_t a, uint32_t b) {
  if (a < b) {
    return a;
  }
  return b;
}

float fastsin(float x) {
  // always wrap input angle to -PI..PI
  while (x < -3.14159265f)
    x += 6.28318531f;

  while (x > 3.14159265f)
    x -= 6.28318531f;
  float sin1;

  // compute sine
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

uint32_t seed = 7;
uint32_t random() {
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

void reset_looptime() {
  extern uint32_t lastlooptime;
  lastlooptime = time_micros();
}