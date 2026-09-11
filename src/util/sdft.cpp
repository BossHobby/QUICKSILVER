#include "sdft.h"

#include <math.h>
#include <string.h>

#include "util/filter.h"
#include "util/util.h"

// from https://www.dsprelated.com/showarticle/776.php
// citing E. Jacobsen and R. Lyons, “The Sliding DFT”
// and E. Jacobsen and R. Lyons, “An Update to the Sliding DFT”

static float r_to_N;
static complex_float twiddle[SDFT_SAMPLE_SIZE];

static uint32_t sub_samples;
static uint32_t resolution_hz;

static uint32_t bin_min_index;
static uint32_t bin_max_index;
static uint32_t bin_batches;

void sdft_init(sdft_t *sdft, float sample_period_us) {
  *sdft = {};
  if (sample_period_us <= 0.0f) {
    return;
  }
  sdft->sample_period_us = sample_period_us;
  const float sample_hz = 1e6f / sample_period_us;
  sub_samples = MAX(1U, (uint32_t)(sample_hz / (2.0f * SDFT_MAX_HZ)));
  resolution_hz = ((sample_hz / (float)sub_samples) / SDFT_SAMPLE_SIZE);

  bin_min_index = (float)SDFT_MIN_HZ / (float)resolution_hz + 0.5f;
  bin_max_index = (float)SDFT_MAX_HZ / (float)resolution_hz + 0.5f;
  bin_batches = (bin_max_index - bin_min_index) / sub_samples + 1;

  r_to_N = powf(SDFT_DAMPING_FACTOR, SDFT_SAMPLE_SIZE);

  const complex_float j = {0.0f, 1.0f};
  for (uint32_t i = 0; i < SDFT_SAMPLE_SIZE; i++) {
    const float factor = 2.0f * M_PI_F * (float)i / (float)SDFT_SAMPLE_SIZE;
    twiddle[i] = __builtin_cexpf(j * factor);
  }

  sdft->state = SDFT_UPDATE_MAGNITUDE;
}

void sdft_update_period(sdft_t *sdft, float sample_period_us) {
  if (sdft->sample_period_us == sample_period_us) {
    return;
  }
  // Restart spectral history at the new rate, retaining applied notch centres.
  float notch_hz[SDFT_PEAKS];
  memcpy(notch_hz, sdft->notch_hz, sizeof(notch_hz));
  sdft_init(sdft, sample_period_us);
  memcpy(sdft->notch_hz, notch_hz, sizeof(notch_hz));
}

bool sdft_push(sdft_t *sdft, float val) {
  if (sdft->sample_period_us <= 0.0f) {
    return false;
  }
  bool batch_finished = false;

  const uint32_t bin_min = bin_batches * sdft->sample_count;
  const uint32_t bin_max = MIN(bin_min + bin_batches, SDFT_BIN_COUNT);

  const float last_sample = r_to_N * sdft->samples[sdft->idx];

  sdft->sample_accumulator += val;
  sdft->sample_count++;

  if (sdft->sample_count >= sub_samples) {
    sdft->sample_avg = sdft->sample_accumulator / (float)sdft->sample_count;
    sdft->sample_accumulator = 0;
    sdft->sample_count = 0;

    sdft->samples[sdft->idx] = sdft->sample_avg;
    sdft->idx = (sdft->idx + 1) % SDFT_SAMPLE_SIZE;

    batch_finished = true;
  }

  const float delta = sdft->sample_avg - last_sample;

  for (uint32_t i = bin_min; i < bin_max; i++) {
    sdft->data[i] = twiddle[i] * (SDFT_DAMPING_FACTOR * sdft->data[i] + delta);
  }

  return batch_finished;
}

bool sdft_update(sdft_t *sdft) {
  if (sdft->sample_period_us <= 0.0f) {
    return false;
  }
  bool filters_updated = false;

  switch (sdft->state) {
  case SDFT_UPDATE_MAGNITUDE:
    sdft->noise_floor = 0;

    for (uint32_t i = bin_min_index + 1; i < bin_max_index - 1; i++) {
      // Hann window in frequency domain: X[k] = -0.25 * X[k-1] +0.5 * X[k] -0.25 * X[k+1]
      const complex_float val = sdft->data[i] - 0.5f * (sdft->data[i - 1] + sdft->data[i + 1]);
      const float re = __real__ val;
      const float im = __imag__ val;

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

      const float y0 = 0.95 * sdft->magnitude[sdft->peak_indicies[peak] - 1];
      const float y1 = sdft->magnitude[sdft->peak_indicies[peak]];
      const float y2 = 1.25f * sdft->magnitude[sdft->peak_indicies[peak] + 1];

      // Estimate true peak position aka. meanBin (fit parabola y(x) over y0, y1 and y2, solve dy/dx=0 for x)
      float meanBin = sdft->peak_indicies[peak];
      const float denom = y0 + y1 + y2;
      if (denom != 0.0f) {
        float lower_ratio = y0 / denom;
        float upper_ratio = y2 / denom;
        meanBin += upper_ratio - lower_ratio;
      }

      const float f_hz = meanBin * (float)resolution_hz;

      const float filter_multi = constrain(sdft->peak_values[peak] / sdft->noise_floor, 1.0f, 10.0f);
      const float filter_period = sdft->sample_period_us * 1e-6f * (float)(SDFT_STEP_COUNT * 3);
      const float gain = filter_period / (1 / (2.0f * M_PI_F * (filter_multi * SDFT_FILTER_HZ)) + filter_period);

      sdft->notch_hz[peak] += gain * (f_hz - sdft->notch_hz[peak]);
    }

    sdft->state = SDFT_UPDATE_FILTERS;
    break;
  }

  case SDFT_UPDATE_FILTERS:
    sdft->state = SDFT_UPDATE_MAGNITUDE;

    filters_updated = true;
    break;

  case SDFT_STEP_COUNT:
    // THIS SHOULD NEVER RUN
    // ONLY ADDED TO REMOVE COMPILER WARNING
    break;
  }

  return filters_updated;
}
