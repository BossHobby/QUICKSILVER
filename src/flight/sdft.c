#include "sdft.h"

#include <math.h>

#include "control.h"
#include "filter.h"
#include "util/util.h"

#define SDFT_FILTER_HZ 4

#define SDFT_DAMPING_FACTOR 0.999f
#define SDFT_SUBSAMPLES (SDFT_SAMPLE_PERIOD / LOOPTIME) // should be looptime_autodetect

#define SWAP(x, y)      \
  {                     \
    typeof(x) temp = x; \
    x = y;              \
    y = temp;           \
  }

static float r_to_N;
static complex_float coeff[SDFT_SAMPLE_SIZE];

static const uint32_t bin_min_index = (float)SDFT_MIN_HZ * (float)SDFT_SAMPLE_SIZE * (SDFT_SAMPLE_PERIOD * 1e-6f) + 0.5f;
static const uint32_t bin_max_index = (float)SDFT_MAX_HZ * (float)SDFT_SAMPLE_SIZE * (SDFT_SAMPLE_PERIOD * 1e-6f) + 0.5f;
static const uint32_t bin_batches = (bin_max_index - bin_min_index) / SDFT_SUBSAMPLES + 1;

void sdft_init(sdft_t *sdft) {
  r_to_N = powf(SDFT_DAMPING_FACTOR, SDFT_SAMPLE_SIZE);

  for (uint32_t i = 0; i < SDFT_SAMPLE_SIZE; i++) {
    const float phi = 2.0f * M_PI_F * (float)i / (float)SDFT_SAMPLE_SIZE;
    coeff[i] = SDFT_DAMPING_FACTOR * (fastcos(phi) + _Complex_I * fastsin(phi));
  }

  sdft->state = SDFT_UPDATE_MAGNITUE;
  sdft->idx = 0;
  sdft->sample_avg = 0;
  sdft->sample_accumulator = 0;
  sdft->sample_count = 0;
  sdft->noise_floor = 0;

  for (uint32_t i = 0; i < SDFT_SAMPLE_SIZE; i++) {
    sdft->samples[i] = 0.0f;
  }

  for (uint32_t i = 0; i < SDFT_BIN_COUNT; i++) {
    sdft->data[i] = 0.0f;
  }

  for (uint32_t peak = 0; peak < SDFT_PEAKS; peak++) {
    sdft->peak_values[peak] = 0;
    sdft->peak_indicies[peak] = 0;
    sdft->notch_hz[peak] = 0;
  }
}

bool sdft_push(sdft_t *sdft, float val) {
  bool batch_finished = false;

  const uint32_t bin_min = bin_batches * sdft->sample_count;
  const uint32_t bin_max = min_uint32(bin_min + bin_batches, bin_max_index);

  const float delta = sdft->sample_avg - r_to_N * sdft->samples[sdft->idx];

  if (sdft->sample_count == SDFT_SUBSAMPLES) {
    sdft->sample_avg = sdft->sample_accumulator / (float)sdft->sample_count;
    sdft->sample_accumulator = 0;
    sdft->sample_count = 0;

    sdft->samples[sdft->idx] = sdft->sample_avg;
    sdft->idx = (sdft->idx + 1) % SDFT_SAMPLE_SIZE;

    batch_finished = true;
  } else {
    sdft->sample_accumulator += val;
    sdft->sample_count++;
  }

  for (uint32_t i = bin_min; i < bin_max; i++) {
    sdft->data[i] = coeff[i] * (sdft->data[i] + delta);
  }

  return batch_finished;
}

bool sdft_update(sdft_t *sdft) {
  bool filters_updated = false;

  switch (sdft->state) {
  case SDFT_UPDATE_MAGNITUE:
    sdft->noise_floor = 0;

    for (uint32_t i = bin_min_index + 1; i < bin_max_index - 1; i++) {
      // Hann window in frequency domain: X[k] = -0.25 * X[k-1] +0.5 * X[k] -0.25 * X[k+1]
      const complex_float val = sdft->data[i] - 0.5f * (sdft->data[i - 1] + sdft->data[i + 1]);
      const float re = crealf(val);
      const float im = cimagf(val);

      sdft->magnitude[i] = re * re + im * im;
      sdft->noise_floor += sdft->magnitude[i];
    }

    sdft->state = SDFT_DETECT_PEAKS;
    break;

  case SDFT_DETECT_PEAKS: {
    for (uint32_t peak = 0; peak < SDFT_PEAKS; peak++) {
      sdft->peak_values[peak] = 0;
      sdft->peak_indicies[peak] = 0;
    }

    for (uint32_t i = bin_min_index + 1; i < bin_max_index - 1; i++) {
      if (sdft->magnitude[i] <= sdft->magnitude[i - 1] || sdft->magnitude[i] <= sdft->magnitude[i + 1]) {
        // neighbours are higher, not a peak
        continue;
      }

      for (uint32_t peak = 0; peak < SDFT_PEAKS; peak++) {
        if (sdft->magnitude[i] <= sdft->peak_values[peak]) {
          continue;
        }

        // push current value down
        for (uint32_t p = SDFT_PEAKS - 1; p > peak; p--) {
          sdft->peak_values[p] = sdft->peak_values[p - 1];
          sdft->peak_indicies[p] = sdft->peak_indicies[p - 1];
        }

        sdft->peak_values[peak] = sdft->magnitude[i];
        sdft->peak_indicies[peak] = i;
        break;
      }

      // next entry cannot be a peak
      i++;
    }

    // sort peaks in ascending order
    for (uint32_t p = SDFT_PEAKS - 1; p > 0; p--) {
      for (uint32_t i = 0; i < p; i++) {
        if (sdft->peak_indicies[i + 1] == 0) {
          // ignore zero peaks
          continue;
        }
        if (sdft->peak_indicies[i] < sdft->peak_indicies[i + 1]) {
          // already sorted
          continue;
        }

        SWAP(sdft->peak_indicies[i], sdft->peak_indicies[i + 1]);
        SWAP(sdft->peak_values[i], sdft->peak_values[i + 1]);
      }
    }

    sdft->state = SDFT_CALC_FREQ;
    break;
  }

  case SDFT_CALC_FREQ: {
    uint32_t peak_count = 0;
    for (uint32_t peak = 0; peak < SDFT_PEAKS; peak++) {
      if (sdft->peak_indicies[peak] == 0) {
        continue;
      }

      sdft->noise_floor -= 0.75f * sdft->magnitude[sdft->peak_indicies[peak] - 1];
      sdft->noise_floor -= sdft->magnitude[sdft->peak_indicies[peak]];
      sdft->noise_floor -= 0.75f * sdft->magnitude[sdft->peak_indicies[peak] + 1];
      peak_count++;
    }
    sdft->noise_floor = (sdft->noise_floor / (bin_max_index - bin_min_index - peak_count - 1)) * 2.0f;

    for (uint32_t peak = 0; peak < SDFT_PEAKS; peak++) {
      if (sdft->peak_indicies[peak] == 0 || sdft->peak_values[peak] <= sdft->noise_floor) {
        continue;
      }

      const float y0 = sdft->magnitude[sdft->peak_indicies[peak] - 1];
      const float y1 = sdft->magnitude[sdft->peak_indicies[peak]];
      const float y2 = sdft->magnitude[sdft->peak_indicies[peak] + 1];

      // Estimate true peak position aka. meanBin (fit parabola y(x) over y0, y1 and y2, solve dy/dx=0 for x)
      float meanBin = sdft->peak_indicies[peak];
      const float denom = 2.0f * (y0 - 2 * y1 + y2);
      if (denom != 0.0f) {
        meanBin += (y0 - y2) / denom;
      }

      const float f_hz = meanBin / (SDFT_SAMPLE_PERIOD * 1e-6f) / (float)SDFT_SAMPLE_SIZE;

      const float filter_multi = constrainf(sdft->peak_values[peak] / sdft->noise_floor, 1.0f, 10.0f);
      lpf(&sdft->notch_hz[peak], f_hz, FILTERCALC(SDFT_SAMPLE_PERIOD, 1e6f / (filter_multi * SDFT_FILTER_HZ)));
    }

    sdft->state = SDFT_UPDATE_FILTERS;
    break;
  }

  case SDFT_UPDATE_FILTERS:
    sdft->state = SDFT_UPDATE_MAGNITUE;
    filters_updated = true;
    break;
  }

  return filters_updated;
}