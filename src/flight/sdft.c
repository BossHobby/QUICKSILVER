#include "sdft.h"

#include <math.h>

#include "control.h"
#include "filter.h"
#include "util/util.h"

#define SDFT_DAMPING_FACTOR 0.999f
#define SDFT_SUBSAMPLES (SDFT_SAMPLE_PERIOD / LOOPTIME) // should be looptime_autodetect

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

  sdft->state = SDFT_WAIT_FOR_SAMPLES;
  sdft->idx = 0;
  sdft->sample_avg = 0;
  sdft->sample_accumulator = 0;
  sdft->sample_count = 0;

  for (uint32_t i = 0; i < SDFT_SAMPLE_SIZE; i++) {
    sdft->samples[i] = 0.0f;
  }

  for (uint32_t i = 0; i < SDFT_BIN_COUNT; i++) {
    sdft->data[i] = 0.0f;
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

bool sdft_update(sdft_t *sdft, float val) {
  bool filters_updated = false;

  if (sdft_push(sdft, val) && sdft->state == SDFT_WAIT_FOR_SAMPLES) {
    sdft->state = SDFT_UPDATE_MAGNITUE;
  }

  switch (sdft->state) {
  case SDFT_WAIT_FOR_SAMPLES:
    break;

  case SDFT_UPDATE_MAGNITUE:
    for (uint32_t i = bin_min_index + 1; i < bin_max_index - 1; i++) {
      // Hann window in frequency domain: X[k] = -0.25 * X[k-1] +0.5 * X[k] -0.25 * X[k+1]
      const complex_float val = sdft->data[i] - 0.5f * (sdft->data[i - 1] + sdft->data[i + 1]);
      const float re = crealf(val);
      const float im = cimagf(val);

      sdft->magnitude[i] = re * re + im * im;
    }
    sdft->state = SDFT_DETECT_PEAKS;
    break;

  case SDFT_DETECT_PEAKS: {
    float peak_values[SDFT_PEAKS];
    uint32_t peak_indicies[SDFT_PEAKS];

    for (uint32_t peak = 0; peak < SDFT_PEAKS; peak++) {
      peak_values[peak] = 0;
      peak_indicies[peak] = 0;
    }

    for (uint32_t i = bin_min_index + 1; i < bin_max_index - 1; i++) {
      if (sdft->magnitude[i] <= sdft->magnitude[i - 1] || sdft->magnitude[i] <= sdft->magnitude[i + 1]) {
        // neighbours are higher, not a peak
        continue;
      }

      for (uint32_t peak = 0; peak < SDFT_PEAKS; peak++) {
        if (sdft->magnitude[i] <= peak_values[peak]) {
          continue;
        }

        // push current value down
        for (uint32_t p = SDFT_PEAKS - 1; p > peak; p--) {
          peak_values[p] = peak_values[p - 1];
          peak_indicies[p] = peak_indicies[p - 1];
        }

        peak_values[peak] = sdft->magnitude[i];
        peak_indicies[peak] = i;
        break;
      }

      // next entry cannot be a peak
      i++;
    }

    for (uint32_t peak = 0; peak < SDFT_PEAKS; peak++) {
      const float f_hz = peak_indicies[peak] / (SDFT_SAMPLE_PERIOD * 1e-6f) / (float)SDFT_SAMPLE_SIZE;
      lpf(&sdft->notch_hz[peak], f_hz, FILTERCALC(SDFT_SAMPLE_PERIOD, 1e6f / 20.0f)); // 20 Hz
    }

    sdft->state = SDFT_UPDATE_FILTERS;
    break;
  }

  case SDFT_UPDATE_FILTERS:
    sdft->state = SDFT_WAIT_FOR_SAMPLES;
    filters_updated = true;
    break;
  }

  return filters_updated;
}