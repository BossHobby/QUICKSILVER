#include "rx/express_lrs.h"

#include "core/project.h"

#if defined(USE_RX_SPI_EXPRESS_LRS)

typedef struct {
  const char *domain;
  uint32_t freq_start;
  uint32_t freq_stop;
  uint32_t freq_count;
} fhss_config_t;

#define SX1280_XTAL_FREQ 52000000.0
#define POW_2_18 262144.0
#define FREQ_STEP ((double)(SX1280_XTAL_FREQ / POW_2_18))

#define FREQ_CORRECTION_MIN (-FREQ_CORRECTION_MAX)
#define FREQ_CORRECTION_MAX ((int32_t)(200000 / FREQ_STEP))

#define FREQ_SPREAD_SCALE 256
#define FREQ_HZ_TO_REG_VAL(freq) ((uint32_t)((double)freq / (double)FREQ_STEP))

extern expresslrs_mod_settings_t *current_air_rate_config();

const fhss_config_t domains[] = {
    {"ISM2G4", FREQ_HZ_TO_REG_VAL(2400400000), FREQ_HZ_TO_REG_VAL(2479400000), 80},
};
const fhss_config_t *config = &domains[0];

uint8_t fhss_index = 0;

static uint8_t fhss_sync_index = 0;

static int32_t freq_correction = 0;

static uint16_t fhss_sequence_count;
static uint32_t freq_spread;

static uint8_t fhss_sequence[256];
static uint32_t fhss_rng_seed = 0;

static uint8_t fhss_rng_max(const uint8_t max) {
  const uint32_t m = 2147483648;
  const uint32_t a = 214013;
  const uint32_t c = 2531011;
  fhss_rng_seed = (a * fhss_rng_seed + c) % m;

  const uint16_t result = fhss_rng_seed >> 16;
  return result % max;
}

void fhss_randomize(int32_t seed) {
  fhss_rng_seed = seed;
  fhss_index = 0;

  fhss_sync_index = (config->freq_count / 2) + 1;
  freq_spread = (config->freq_stop - config->freq_start) * FREQ_SPREAD_SCALE / (config->freq_count - 1);

  fhss_sequence_count = (256 / config->freq_count) * config->freq_count;

  // initialize the sequence array
  for (uint16_t i = 0; i < fhss_sequence_count; i++) {
    if (i % config->freq_count == 0) {
      fhss_sequence[i] = fhss_sync_index;
    } else if (i % config->freq_count == fhss_sync_index) {
      fhss_sequence[i] = 0;
    } else {
      fhss_sequence[i] = i % config->freq_count;
    }
  }

  for (uint16_t i = 0; i < fhss_sequence_count; i++) {
    // if it's not the sync channel
    if (i % config->freq_count != 0) {
      const uint8_t offset = (i / config->freq_count) * config->freq_count; // offset to start of current block
      const uint8_t rand = fhss_rng_max(config->freq_count - 1) + 1;        // random number between 1 and config->freq_count

      // switch this entry and another random entry in the same block
      const uint8_t temp = fhss_sequence[i];
      fhss_sequence[i] = fhss_sequence[offset + rand];
      fhss_sequence[offset + rand] = temp;
    }
  }
}

void fhss_set_index(const uint8_t value) {
  fhss_index = value % fhss_sequence_count;
}

uint8_t fhss_get_index() {
  return fhss_index;
}

static uint32_t fhss_get_freq(const uint8_t index) {
  return config->freq_start + (freq_spread * index / FREQ_SPREAD_SCALE) - freq_correction;
}

uint32_t fhss_get_sync_freq() {
  return fhss_get_freq(fhss_sync_index);
}

uint32_t fhss_next_freq() {
  fhss_index = (fhss_index + 1) % fhss_sequence_count;
  return fhss_get_freq(fhss_sequence[fhss_index]);
}

int32_t fhss_update_freq_correction(bool value) {
  if (value) {
    if (freq_correction > FREQ_CORRECTION_MIN) {
      freq_correction -= 1; // FREQ_STEP units
    }
  } else {
    if (freq_correction < FREQ_CORRECTION_MAX) {
      freq_correction += 1; // FREQ_STEP units
    }
  }
  return freq_correction;
}

void fhss_reset() {
  freq_correction = 0;
}

uint8_t fhss_min_lq_for_chaos() {
  // Determine the most number of CRC-passing packets we could receive on
  // a single channel out of 100 packets that fill the LQcalc span.
  // The LQ must be GREATER THAN this value, not >=
  // The amount of time we coexist on the same channel is
  // 100 divided by the total number of packets in a FHSS loop (rounded up)
  // and there would be 4x packets received each time it passes by so
  // FHSShopInterval * ceil(100 / FHSShopInterval * config->freq_count) or
  // FHSShopInterval * trunc((100 + (FHSShopInterval * config->freq_count) - 1) / (FHSShopInterval * config->freq_count))
  // With a interval of 4 this works out to: 2.4=4, FCC915=4, AU915=8, EU868=8, EU/AU433=36
  const uint8_t interval = current_air_rate_config()->fhss_hop_interval;
  return interval * ((interval * config->freq_count + 99) / (interval * config->freq_count));
}

uint32_t fhss_rf_mode_cycle_interval() {
  expresslrs_mod_settings_t *air_rate = current_air_rate_config();
  return ((uint32_t)11U * config->freq_count * air_rate->fhss_hop_interval * air_rate->interval) / (10U * 1000U);
}

#endif