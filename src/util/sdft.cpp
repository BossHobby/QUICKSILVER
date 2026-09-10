#include "sdft.h"

#include <array>
#include <math.h>
#include <string.h>

#include "control/control.h"
#include "util/util.h"

// from https://www.dsprelated.com/showarticle/776.php
// citing E. Jacobsen and R. Lyons, “The Sliding DFT”
// and E. Jacobsen and R. Lyons, “An Update to the Sliding DFT”

// Interpolation needs one magnitude neighbour; Hann needs one more DFT bin.
static constexpr uint32_t MAGNITUDE_HALO = 1;
static constexpr uint32_t SPECTRUM_HALO = MAGNITUDE_HALO + 1;
static constexpr float TRACKING_OMEGA = 2.0f * M_PI_F * SDFT_FILTER_HZ;
static constexpr float MAX_TRACKING_MULTIPLIER = 10.0f;
static constexpr float WINDOW_DAMPING = __builtin_powf(SDFT_DAMPING_FACTOR, SDFT_SAMPLE_SIZE);
static constexpr auto TWIDDLE = [] {
  std::array<complex_float, SDFT_BIN_COUNT> values{};
  for (uint32_t i = 0; i < SDFT_BIN_COUNT; i++) {
    const float angle = 2.0f * M_PI_F * (float)i / SDFT_SAMPLE_SIZE;
    values[i] = {__builtin_cosf(angle), __builtin_sinf(angle)};
  }
  return values;
}();

void sdft_init(sdft_t *sdft) {
  *sdft = {};
  const float sample_period_us = state.looptime_autodetect;
  // Gyro initialization precedes scheduler/looptime initialization.
  if (sample_period_us <= 0.0f) {
    return;
  }

  const float sample_hz = 1e6f / sample_period_us;
  sdft->sample_period_us = sample_period_us;
  sdft->sub_samples = MAX(1U, (uint32_t)(sample_hz / (2.0f * SDFT_MAX_HZ)));
  sdft->resolution_hz = sample_hz / (float)sdft->sub_samples / SDFT_SAMPLE_SIZE;
  constexpr uint32_t LAST_PEAK_BIN = SDFT_BIN_COUNT - SPECTRUM_HALO - 1;
  sdft->bin_min_index = constrain(SDFT_MIN_HZ / sdft->resolution_hz + 0.5f, SPECTRUM_HALO, LAST_PEAK_BIN);
  sdft->bin_max_index = constrain(SDFT_MAX_HZ / sdft->resolution_hz + 0.5f, sdft->bin_min_index, LAST_PEAK_BIN);
  const uint32_t bin_count = sdft->bin_max_index - sdft->bin_min_index + 1 + 2 * SPECTRUM_HALO;
  sdft->bin_batches = (bin_count + sdft->sub_samples - 1) / sdft->sub_samples;
}

bool sdft_push(sdft_t *sdft, float val) {
  if (state.looptime_autodetect <= 0.0f) {
    return false;
  }
  if (sdft->sample_period_us != state.looptime_autodetect) {
    // Rebuild spectral history at the new rate, retaining applied centres.
    float notch_hz[SDFT_PEAKS];
    memcpy(notch_hz, sdft->notch_hz, sizeof(notch_hz));
    sdft_init(sdft);
    memcpy(sdft->notch_hz, notch_hz, sizeof(notch_hz));
  }

  sdft->update_samples++;
  const uint32_t bin_min = sdft->bin_min_index - SPECTRUM_HALO + sdft->bin_batches * sdft->sample_count;
  const uint32_t bin_max = MIN(bin_min + sdft->bin_batches, sdft->bin_max_index + SPECTRUM_HALO + 1);
  for (uint32_t i = bin_min; i < bin_max; i++) {
    sdft->data[i] = TWIDDLE[i] * (SDFT_DAMPING_FACTOR * sdft->data[i] + sdft->sample_delta);
  }

  // Accumulate the next sample while finishing the current spectrum.
  sdft->sample_accumulator += val;
  sdft->sample_count++;

  if (sdft->sample_count >= sdft->sub_samples) {
    const float sample_avg = sdft->sample_accumulator / (float)sdft->sample_count;
    sdft->sample_delta = sample_avg - WINDOW_DAMPING * sdft->samples[sdft->idx];
    sdft->sample_accumulator = 0;
    sdft->sample_count = 0;

    sdft->samples[sdft->idx] = sample_avg;
    sdft->idx = (sdft->idx + 1) % SDFT_SAMPLE_SIZE;
    return true;
  }

  return false;
}

bool sdft_update(sdft_t *sdft) {
  bool filters_updated = false;
  const uint32_t bin_min_index = sdft->bin_min_index;
  const uint32_t bin_max_index = sdft->bin_max_index;
  if (sdft->sample_period_us <= 0.0f) {
    return false;
  }

  switch (sdft->state) {
  case SDFT_UPDATE_MAGNITUDE:
    // Other axes can reach this step halfway through a distributed update.
    if (sdft->sample_count != 0) {
      break;
    }
    sdft->noise_floor = 0;

    for (uint32_t i = bin_min_index - MAGNITUDE_HALO; i <= bin_max_index + MAGNITUDE_HALO; i++) {
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

    for (uint32_t i = bin_min_index; i <= bin_max_index; i++) {
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
    const uint32_t magnitude_bins = bin_max_index - bin_min_index + 1 + 2 * MAGNITUDE_HALO;
    const uint32_t noise_bins = magnitude_bins - peak_count;
    sdft->noise_floor = MAX(0.0f, sdft->noise_floor) / noise_bins * 2.0f;

    const float tracking_step = TRACKING_OMEGA * sdft->update_samples * sdft->sample_period_us * 1e-6f;
    for (uint32_t peak = 0; peak < SDFT_PEAKS; peak++) {
      if (sdft->peak_indicies[peak] == 0 || sdft->peak_values[peak] <= sdft->noise_floor) {
        continue;
      }

      const float y0 = sdft->magnitude[sdft->peak_indicies[peak] - 1];
      const float y1 = sdft->magnitude[sdft->peak_indicies[peak]];
      const float y2 = sdft->magnitude[sdft->peak_indicies[peak] + 1];

      // Parabolic interpolation of the peak and its two neighbours.
      float mean_bin = sdft->peak_indicies[peak];
      const float denom = y0 - 2.0f * y1 + y2;
      if (denom < 0.0f) {
        mean_bin += constrain(0.5f * (y0 - y2) / denom, -0.5f, 0.5f);
      }

      const float f_hz = constrain(mean_bin * sdft->resolution_hz, SDFT_MIN_HZ, SDFT_MAX_HZ);

      const float filter_multi = sdft->noise_floor > 0.0f ? constrain(sdft->peak_values[peak] / sdft->noise_floor, 1.0f, MAX_TRACKING_MULTIPLIER) : MAX_TRACKING_MULTIPLIER;
      const float step = tracking_step * filter_multi;
      const float gain = step / (1.0f + step);

      sdft->notch_hz[peak] += gain * (f_hz - sdft->notch_hz[peak]);
    }

    sdft->update_samples = 0;
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
