#include "util/util.h"

#include <math.h>
#include <string.h>

#include "driver/time.h"
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

#define sinPolyCoef3 -1.666665710e-1f // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5 8.333017292e-3f  // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9 2.600054768e-6f  // Double:  2.600054767890361277123254766503271638682e-6

float fastsin(float x) {
  const int32_t xint = x;

  if (xint < -32 || xint > 32)
    return 0.0f; // Stop here on error input (5 * 360 Deg)

  while (x > M_PI_F)
    x -= (2.0f * M_PI_F); // always wrap input angle to -PI..PI

  while (x < -M_PI_F)
    x += (2.0f * M_PI_F);

  if (x > (0.5f * M_PI_F))
    x = (0.5f * M_PI_F) - (x - (0.5f * M_PI_F)); // We just pick -90..+90 Degree

  else if (x < -(0.5f * M_PI_F))
    x = -(0.5f * M_PI_F) - ((0.5f * M_PI_F) + x);

  const float x2 = x * x;
  return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float fastcos(float x) {
  return fastsin(x + (0.5f * M_PI_F));
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

uint32_t get_chip_uid() {
  return ((uint32_t *)UID_BASE)[0] ^ ((uint32_t *)UID_BASE)[1] ^ ((uint32_t *)UID_BASE)[1];
}
