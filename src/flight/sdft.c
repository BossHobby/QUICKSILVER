#include "sdft.h"

#include <complex.h>
#undef I

#include <math.h>

#include "control.h"
#include "filter.h"
#include "util/util.h"

#define SDFT_MIN_HZ 80
#define SDFT_MAX_HZ 500 // limit: 500 Hz if SAMPLE_PERIOD is 1 ms.

#define SDFT_SAMPLE_PERIOD 1000                 // us. Sampling every 1 ms is enough for MAX_HZ up to 500.
#define SDFT_SIZE (100000 / SDFT_SAMPLE_PERIOD) // 0.1 seconds time window gives 10 Hz resolution.

#define SDFT_DAMPING_FACTOR 0.999f
#define SDFT_SUBSAMPLES (SDFT_SAMPLE_PERIOD / LOOPTIME) // should be looptime_autodetect

typedef float complex complex_float;

static uint32_t idx;

static float r_to_N;

static float sample_accu[SDFT_AXES];
static uint32_t sample_count;

static float x[SDFT_AXES][SDFT_SIZE];
static complex_float X[SDFT_AXES][SDFT_SIZE];
static complex_float coeff[SDFT_SIZE];

static uint32_t indices[SDFT_AXES][SDFT_PEAKS];
static float values[SDFT_AXES][SDFT_PEAKS];
static float delta[SDFT_AXES];

static float f_hz_filt[SDFT_AXES][SDFT_PEAKS];
float sdft_notch_hz[SDFT_AXES][SDFT_PEAKS];

void sdft_init() {
  r_to_N = powf(SDFT_DAMPING_FACTOR, SDFT_SIZE);

  for (uint32_t i = 0; i < SDFT_SIZE; i++) {
    const float phi = 2.0f * M_PI_F * (float)i / (float)SDFT_SIZE;
    coeff[i] = SDFT_DAMPING_FACTOR * (fastcos(phi) + _Complex_I * fastsin(phi));
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
      for (uint32_t peak = 0; peak < SDFT_PEAKS; peak++) {
        const float f_hz = indices[axis][peak] / (SDFT_SAMPLE_PERIOD * 1e-6f) / (float)SDFT_SIZE;

        lpf(&f_hz_filt[axis][peak], f_hz, FILTERCALC(SDFT_SAMPLE_PERIOD, 1e6f / 20.0f));                      // 20 Hz
        lpf(&sdft_notch_hz[axis][peak], f_hz_filt[axis][peak], FILTERCALC(SDFT_SAMPLE_PERIOD, 1e6f / 20.0f)); // 20 Hz

        indices[axis][peak] = max_bin_index;
        values[axis][peak] = 0.0f;
      }

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
      for (uint32_t peak = 0; peak < SDFT_PEAKS; peak++) {
        if (mag_squared <= values[axis][peak]) {
          continue;
        }

        // push current value down
        for (uint32_t p = SDFT_PEAKS - 1; p > peak; p--) {
          values[axis][p] = values[axis][p - 1];
          indices[axis][p] = indices[axis][p - 1];
        }

        values[axis][peak] = mag_squared;
        indices[axis][peak] = k;
        break;
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