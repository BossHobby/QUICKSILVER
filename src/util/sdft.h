#pragma once

#include <stdint.h>

constexpr uint32_t SDFT_AXES = 3;
constexpr uint32_t SDFT_PEAKS = 3;

constexpr float SDFT_FILTER_HZ = 4.0f;

constexpr float SDFT_MIN_HZ = 100.0f;
constexpr float SDFT_MAX_HZ = 600.0f;

constexpr uint32_t SDFT_SAMPLE_SIZE = 62;
constexpr uint32_t SDFT_BIN_COUNT = SDFT_SAMPLE_SIZE / 2;

constexpr float SDFT_DAMPING_FACTOR = 0.9999f;

// Keep the C implementation's complex arithmetic and trivial initialization.
typedef __complex__ float complex_float;

typedef enum {
  SDFT_UPDATE_MAGNITUDE,
  SDFT_DETECT_PEAKS,
  SDFT_CALC_FREQ,
  SDFT_UPDATE_FILTERS,
  SDFT_STEP_COUNT,
} sdft_state_t;

typedef struct {
  sdft_state_t state;

  uint32_t idx;

  float resolution_hz;
  float sample_period_us;

  uint32_t sub_samples;
  uint32_t bin_min_index;
  uint32_t bin_max_index;
  uint32_t bins_per_push;

  float sample_accumulator;
  float sample_delta; // One completed decimated sample delta, shared by every bin batch.

  uint32_t sample_count;
  uint32_t update_samples;

  float samples[SDFT_SAMPLE_SIZE];
  complex_float data[SDFT_BIN_COUNT];

  float noise_floor;
  float magnitude[SDFT_BIN_COUNT];

  float peak_values[SDFT_PEAKS];
  uint32_t peak_indices[SDFT_PEAKS];

  float notch_hz[SDFT_PEAKS];
} sdft_t;

void sdft_init(sdft_t *sdft, float sample_period_us);
void sdft_update_period(sdft_t *sdft, float sample_period_us);
bool sdft_push(sdft_t *sdft, float val);
bool sdft_update(sdft_t *sdft);
