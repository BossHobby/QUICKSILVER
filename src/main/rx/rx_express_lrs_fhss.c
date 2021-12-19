#include "rx_express_lrs.h"

#if defined(RX_EXPRESS_LRS) && (defined(USE_SX127X) || defined(USE_SX128X))

#ifdef USE_SX127X
#define FREQ_STEP 61.03515625
#endif

#ifdef USE_SX128X
#define SX1280_XTAL_FREQ 52000000.0
#define POW_2_18 262144.0
#define FREQ_STEP ((double)(SX1280_XTAL_FREQ / POW_2_18))
#endif

#define FHSS_FREQ_CNT (sizeof(fhss_freqs) / sizeof(uint32_t))
#define FHSS_SEQUENCE_CNT ((256 / FHSS_FREQ_CNT) * FHSS_FREQ_CNT)

#define FREQ_CORRECTION_MAX ((int32_t)(100000 / FREQ_STEP))
#define FREQ_CORRECTION_MIN ((int32_t)(-100000 / FREQ_STEP))

#define FREQ_HZ_TO_REG_VAL(freq) ((uint32_t)((double)freq / (double)FREQ_STEP))

extern expresslrs_mod_settings_t *current_air_rate_config();

const uint32_t fhss_freqs[] = {
#ifdef USE_SX127X
    FREQ_HZ_TO_REG_VAL(863275000), // band H1, 863 - 865MHz, 0.1% duty cycle or CSMA techniques, 25mW EIRP
    FREQ_HZ_TO_REG_VAL(863800000),
    FREQ_HZ_TO_REG_VAL(864325000),
    FREQ_HZ_TO_REG_VAL(864850000),
    FREQ_HZ_TO_REG_VAL(865375000), // Band H2, 865 - 868.6MHz, 1.0% dutycycle or CSMA, 25mW EIRP
    FREQ_HZ_TO_REG_VAL(865900000),
    FREQ_HZ_TO_REG_VAL(866425000),
    FREQ_HZ_TO_REG_VAL(866950000),
    FREQ_HZ_TO_REG_VAL(867475000),
    FREQ_HZ_TO_REG_VAL(868000000),
    FREQ_HZ_TO_REG_VAL(868525000), // Band H3, 868.7-869.2MHz, 0.1% dutycycle or CSMA, 25mW EIRP
    FREQ_HZ_TO_REG_VAL(869050000),
    FREQ_HZ_TO_REG_VAL(869575000),
#endif

#ifdef USE_SX128X
    FREQ_HZ_TO_REG_VAL(2400400000),
    FREQ_HZ_TO_REG_VAL(2401400000),
    FREQ_HZ_TO_REG_VAL(2402400000),
    FREQ_HZ_TO_REG_VAL(2403400000),
    FREQ_HZ_TO_REG_VAL(2404400000),

    FREQ_HZ_TO_REG_VAL(2405400000),
    FREQ_HZ_TO_REG_VAL(2406400000),
    FREQ_HZ_TO_REG_VAL(2407400000),
    FREQ_HZ_TO_REG_VAL(2408400000),
    FREQ_HZ_TO_REG_VAL(2409400000),

    FREQ_HZ_TO_REG_VAL(2410400000),
    FREQ_HZ_TO_REG_VAL(2411400000),
    FREQ_HZ_TO_REG_VAL(2412400000),
    FREQ_HZ_TO_REG_VAL(2413400000),
    FREQ_HZ_TO_REG_VAL(2414400000),

    FREQ_HZ_TO_REG_VAL(2415400000),
    FREQ_HZ_TO_REG_VAL(2416400000),
    FREQ_HZ_TO_REG_VAL(2417400000),
    FREQ_HZ_TO_REG_VAL(2418400000),
    FREQ_HZ_TO_REG_VAL(2419400000),

    FREQ_HZ_TO_REG_VAL(2420400000),
    FREQ_HZ_TO_REG_VAL(2421400000),
    FREQ_HZ_TO_REG_VAL(2422400000),
    FREQ_HZ_TO_REG_VAL(2423400000),
    FREQ_HZ_TO_REG_VAL(2424400000),

    FREQ_HZ_TO_REG_VAL(2425400000),
    FREQ_HZ_TO_REG_VAL(2426400000),
    FREQ_HZ_TO_REG_VAL(2427400000),
    FREQ_HZ_TO_REG_VAL(2428400000),
    FREQ_HZ_TO_REG_VAL(2429400000),

    FREQ_HZ_TO_REG_VAL(2430400000),
    FREQ_HZ_TO_REG_VAL(2431400000),
    FREQ_HZ_TO_REG_VAL(2432400000),
    FREQ_HZ_TO_REG_VAL(2433400000),
    FREQ_HZ_TO_REG_VAL(2434400000),

    FREQ_HZ_TO_REG_VAL(2435400000),
    FREQ_HZ_TO_REG_VAL(2436400000),
    FREQ_HZ_TO_REG_VAL(2437400000),
    FREQ_HZ_TO_REG_VAL(2438400000),
    FREQ_HZ_TO_REG_VAL(2439400000),

    FREQ_HZ_TO_REG_VAL(2440400000),
    FREQ_HZ_TO_REG_VAL(2441400000),
    FREQ_HZ_TO_REG_VAL(2442400000),
    FREQ_HZ_TO_REG_VAL(2443400000),
    FREQ_HZ_TO_REG_VAL(2444400000),

    FREQ_HZ_TO_REG_VAL(2445400000),
    FREQ_HZ_TO_REG_VAL(2446400000),
    FREQ_HZ_TO_REG_VAL(2447400000),
    FREQ_HZ_TO_REG_VAL(2448400000),
    FREQ_HZ_TO_REG_VAL(2449400000),

    FREQ_HZ_TO_REG_VAL(2450400000),
    FREQ_HZ_TO_REG_VAL(2451400000),
    FREQ_HZ_TO_REG_VAL(2452400000),
    FREQ_HZ_TO_REG_VAL(2453400000),
    FREQ_HZ_TO_REG_VAL(2454400000),

    FREQ_HZ_TO_REG_VAL(2455400000),
    FREQ_HZ_TO_REG_VAL(2456400000),
    FREQ_HZ_TO_REG_VAL(2457400000),
    FREQ_HZ_TO_REG_VAL(2458400000),
    FREQ_HZ_TO_REG_VAL(2459400000),

    FREQ_HZ_TO_REG_VAL(2460400000),
    FREQ_HZ_TO_REG_VAL(2461400000),
    FREQ_HZ_TO_REG_VAL(2462400000),
    FREQ_HZ_TO_REG_VAL(2463400000),
    FREQ_HZ_TO_REG_VAL(2464400000),

    FREQ_HZ_TO_REG_VAL(2465400000),
    FREQ_HZ_TO_REG_VAL(2466400000),
    FREQ_HZ_TO_REG_VAL(2467400000),
    FREQ_HZ_TO_REG_VAL(2468400000),
    FREQ_HZ_TO_REG_VAL(2469400000),

    FREQ_HZ_TO_REG_VAL(2470400000),
    FREQ_HZ_TO_REG_VAL(2471400000),
    FREQ_HZ_TO_REG_VAL(2472400000),
    FREQ_HZ_TO_REG_VAL(2473400000),
    FREQ_HZ_TO_REG_VAL(2474400000),

    FREQ_HZ_TO_REG_VAL(2475400000),
    FREQ_HZ_TO_REG_VAL(2476400000),
    FREQ_HZ_TO_REG_VAL(2477400000),
    FREQ_HZ_TO_REG_VAL(2478400000),
    FREQ_HZ_TO_REG_VAL(2479400000),
#endif
};

uint8_t fhss_index = 0;

static uint8_t fhss_sync_index = 0;

static int32_t freq_correction = 0;

static uint8_t fhss_sequence[FHSS_SEQUENCE_CNT];
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

  fhss_sync_index = FHSS_FREQ_CNT / 2;

  // initialize the sequence array
  for (uint8_t i = 0; i < FHSS_SEQUENCE_CNT; i++) {
    if (i % FHSS_FREQ_CNT == 0) {
      fhss_sequence[i] = fhss_sync_index;
    } else if (i % FHSS_FREQ_CNT == fhss_sync_index) {
      fhss_sequence[i] = 0;
    } else {
      fhss_sequence[i] = i % FHSS_FREQ_CNT;
    }
  }

  for (uint8_t i = 0; i < FHSS_SEQUENCE_CNT; i++) {
    // if it's not the sync channel
    if (i % FHSS_FREQ_CNT != 0) {
      const uint8_t offset = (i / FHSS_FREQ_CNT) * FHSS_FREQ_CNT; // offset to start of current block
      const uint8_t rand = fhss_rng_max(FHSS_FREQ_CNT - 1) + 1;   // random number between 1 and FHSS_FREQ_CNT

      // switch this entry and another random entry in the same block
      const uint8_t temp = fhss_sequence[i];
      fhss_sequence[i] = fhss_sequence[offset + rand];
      fhss_sequence[offset + rand] = temp;
    }
  }
}

void fhss_set_index(const uint8_t value) {
  fhss_index = value % FHSS_SEQUENCE_CNT;
}

uint8_t fhss_get_index() {
  return fhss_index;
}

uint32_t fhss_get_freq(const uint8_t index) {
  return fhss_freqs[index] - freq_correction;
}

uint32_t fhss_get_sync_freq() {
  return fhss_freqs[fhss_sync_index] - freq_correction;
}

uint32_t fhss_next_freq() {
  fhss_index = (fhss_index + 1) % FHSS_SEQUENCE_CNT;
  return fhss_get_freq(fhss_sequence[fhss_index]);
}

int32_t fhss_update_freq_correction(uint8_t value) {
  if (!value) {
    if (freq_correction < FREQ_CORRECTION_MAX) {
      freq_correction += 1; //min freq step is ~ 61hz but don't forget we use FREQ_HZ_TO_REG_VAL so the units here are not hz!
    } else {
      freq_correction = 0; //reset because something went wrong
    }
  } else {
    if (freq_correction > FREQ_CORRECTION_MIN) {
      freq_correction -= 1; //min freq step is ~ 61hz
    } else {
      freq_correction = 0; //reset because something went wrong
    }
  }

  const uint32_t current_freq = fhss_get_freq(fhss_sequence[fhss_index]);
  return (freq_correction * 1e6 / current_freq) * 95 / 100;
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
  // FHSShopInterval * ceil(100 / FHSShopInterval * FHSS_FREQ_CNT) or
  // FHSShopInterval * trunc((100 + (FHSShopInterval * FHSS_FREQ_CNT) - 1) / (FHSShopInterval * FHSS_FREQ_CNT))
  // With a interval of 4 this works out to: 2.4=4, FCC915=4, AU915=8, EU868=8, EU/AU433=36
  const uint8_t interval = current_air_rate_config()->fhss_hop_interval;
  return interval * ((interval * FHSS_FREQ_CNT + 99) / (interval * FHSS_FREQ_CNT));
}

uint32_t fhss_rf_mode_cycle_interval() {
  expresslrs_mod_settings_t *config = current_air_rate_config();
  return ((uint32_t)11U * FHSS_FREQ_CNT * config->fhss_hop_interval * config->interval) / (10U * 1000U);
}

#endif