#include "sdft.h"

#include <complex.h>
#undef I

#include <math.h>

#include "control.h"
#include "filter.h"
#include "util/util.h"

#define SDFT_AXES 3

#define SDFT_MIN_HZ 80
#define SDFT_MAX_HZ 500 // limit: 500 Hz if SAMPLE_PERIOD is 1 ms.

#define SDFT_SAMPLE_PERIOD 1000                 // us. Sampling every 1 ms is enough for MAX_HZ up to 500.
#define SDFT_SIZE (100000 / SDFT_SAMPLE_PERIOD) // 0.1 seconds time window gives 10 Hz resolution.

#define SDFT_DAMPING_FACTOR 0.999f
#define SDFT_SUBSAMPLES (SDFT_SAMPLE_PERIOD / state.looptime_autodetect)

typedef float complex complex_float;

static uint32_t idx;

static float r_to_N;

static float sample_accu[SDFT_AXES];
static uint32_t sample_count;

static float x[SDFT_AXES][SDFT_SIZE];
static complex_float X[SDFT_AXES][SDFT_SIZE];
static complex_float coeff[SDFT_SIZE];

static int32_t index_1st[SDFT_AXES];
static int32_t index_2nd[SDFT_AXES];

static float value_1st[SDFT_AXES];
static float value_2nd[SDFT_AXES];

static float delta[SDFT_AXES];

static float f_hz_filt[SDFT_AXES][2];
float sdft_notch_hz[SDFT_AXES][2];

void sdft_init() {
  r_to_N = powf(SDFT_DAMPING_FACTOR, SDFT_SIZE);

  for (uint32_t i = 0; i < SDFT_SIZE; i++) {
    const float phi = 2.0f * M_PI_F * (float)i / (float)SDFT_SIZE;
    coeff[i] = SDFT_DAMPING_FACTOR * (cosf(phi) + _Complex_I * sinf(phi));
  }
}

void sdft_step() {
  sample_accu[0] += state.gyro_raw.axis[0];
  sample_accu[1] += state.gyro_raw.axis[1];
  sample_accu[2] += state.gyro_raw.axis[2];
  sample_count++;

  if (sample_count == SDFT_SUBSAMPLES) {
    sample_accu[0] /= sample_count;
    sample_accu[1] /= sample_count;
    sample_accu[2] /= sample_count;
    sample_count = 0;
  }

  for (uint8_t axis = 0; axis < SDFT_AXES; ++axis) {
    const int32_t min_bin_index = (float)SDFT_MIN_HZ * (float)SDFT_SIZE * (SDFT_SAMPLE_PERIOD * 1e-6f) + 0.5f;
    const int32_t max_bin_index = (float)SDFT_MAX_HZ * (float)SDFT_SIZE * (SDFT_SAMPLE_PERIOD * 1e-6f) + 0.5f;

    if (sample_count == 0) {
      // Changing the notch filter frequency is not instantaneous and transitioning to a new frequency
      // takes some time. To avoid long transients for example if the 1st and 2nd filter switch places,
      // we ensure that index_1st is always higher than index_2nd:
      if (index_1st[axis] < index_2nd[axis]) {
        const int temp = index_1st[axis];
        index_1st[axis] = index_2nd[axis];
        index_2nd[axis] = temp;
      }

      const float f_1st_Hz = index_1st[axis] / (SDFT_SAMPLE_PERIOD * 1e-6f) / (float)SDFT_SIZE;
      const float f_2nd_Hz = index_2nd[axis] / (SDFT_SAMPLE_PERIOD * 1e-6f) / (float)SDFT_SIZE;

      lpf(&f_hz_filt[axis][0], f_1st_Hz, FILTERCALC(SDFT_SAMPLE_PERIOD, 1e6f / 20.0f));               // 20 Hz
      lpf(&sdft_notch_hz[axis][0], f_hz_filt[axis][0], FILTERCALC(SDFT_SAMPLE_PERIOD, 1e6f / 20.0f)); // 20 Hz
      lpf(&f_hz_filt[axis][1], f_2nd_Hz, FILTERCALC(SDFT_SAMPLE_PERIOD, 1e6f / 20.0f));               // 20 Hz
      lpf(&sdft_notch_hz[axis][1], f_hz_filt[axis][1], FILTERCALC(SDFT_SAMPLE_PERIOD, 1e6f / 20.0f)); // 20 Hz

      index_1st[axis] = max_bin_index;
      index_2nd[axis] = max_bin_index;
      value_1st[axis] = 0.0f;
      value_2nd[axis] = 0.0f;

      delta[axis] = sample_accu[axis] - r_to_N * x[axis][idx];
      x[axis][idx] = sample_accu[axis];

      sample_accu[axis] = 0.0f;
    }

    int32_t bin_chunk = (max_bin_index - min_bin_index + 1) / SDFT_SUBSAMPLES;
    if (bin_chunk * SDFT_SUBSAMPLES < max_bin_index - min_bin_index + 1) {
      bin_chunk++;
    }

    const int32_t start_bin_index = min_bin_index + bin_chunk * sample_count;
    for (int32_t k = start_bin_index; k < start_bin_index + bin_chunk && k <= max_bin_index; ++k) {
      // Do the actual SDFT calculation.
      X[axis][k] = coeff[k] * (X[axis][k] + delta[axis]);

      // Find the two most dominant peaks:
      const float re = crealf(X[axis][k]);
      const float im = cimagf(X[axis][k]);
      const float mag_squared = re * re + im * im;
      if (mag_squared > value_1st[axis]) {
        value_2nd[axis] = value_1st[axis];
        index_2nd[axis] = index_1st[axis];
        value_1st[axis] = mag_squared;
        index_1st[axis] = k;
      } else if (mag_squared > value_2nd[axis]) {
        value_2nd[axis] = mag_squared;
        index_2nd[axis] = k;
      }
    }
  }

  if (sample_count == 0) {
    idx++;
    if (idx == SDFT_SIZE) {
      idx = 0;
    }
  }
}