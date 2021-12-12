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

#define FHSS_RNG_MAX 0x7FFF

#define NR_FHSS_ENTRIES (sizeof(fhss_freqs) / sizeof(uint32_t))
#define NR_SEQUENCE_ENTRIES 256

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

static int32_t freq_correction = 0;

static uint8_t fhss_sequence[NR_SEQUENCE_ENTRIES] = {0};
static uint32_t fhss_rng_seed = 0;

static unsigned int fhss_rng_max(unsigned int max) {
  unsigned long m = 2147483648;
  long a = 214013;
  long c = 2531011;
  fhss_rng_seed = (a * fhss_rng_seed + c) % m;
  unsigned int result = ((fhss_rng_seed >> 16) * max) / FHSS_RNG_MAX;
  return result;
}

static void fhss_reset_is_available(uint8_t *is_available) {
  // channel 0 is the sync channel and is never considered available
  is_available[0] = 0;

  for (uint32_t i = 1; i < NR_FHSS_ENTRIES; i++)
    is_available[i] = 1;
}

void fhss_randomize(int32_t seed) {
  fhss_rng_seed = seed;

  uint8_t is_available[NR_FHSS_ENTRIES];
  fhss_reset_is_available(is_available);

  // Fill the fhss_sequence with channel indices
  // The 0 index is special - the 'sync' channel. The sync channel appears every
  // syncInterval hops. The other channels are randomly distributed between the
  // sync channels
  const int32_t SYNC_INTERVAL = NR_FHSS_ENTRIES;

  int nLeft = NR_FHSS_ENTRIES - 1; // how many channels are left to be allocated. Does not include the sync channel
  unsigned int prev = 0;           // needed to prevent repeats of the same index

  for (int i = 0; i < NR_SEQUENCE_ENTRIES; i++) {
    if (i % SYNC_INTERVAL == 0) {
      // assign sync channel 0
      fhss_sequence[i] = 0;
      prev = 0;
    } else {
      // pick one of the available channels. May need to loop to avoid repeats
      unsigned int index;
      do {
        int c = fhss_rng_max(nLeft); // returnc 0<c<nLeft
        // find the c'th entry in the is_available array
        // can skip 0 as that's the sync channel and is never available for normal allocation
        index = 1;
        int found = 0;
        while (index < NR_FHSS_ENTRIES) {
          if (is_available[index]) {
            if (found == c)
              break;
            found++;
          }
          index++;
        }
        if (index == NR_FHSS_ENTRIES) {
          // This should never happen
          // What to do? We don't want to hang as that will stop us getting to the wifi hotspot
          // Use the sync channel
          index = 0;
          break;
        }
      } while (index == prev); // can't use index if it repeats the previous value

      fhss_sequence[i] = index; // assign the value to the sequence array
      is_available[index] = 0;  // clear the flag
      prev = index;             // remember for next iteration
      nLeft--;                  // reduce the count of available channels
      if (nLeft == 0) {
        // we've assigned all of the channels, so reset for next cycle
        fhss_reset_is_available(is_available);
        nLeft = NR_FHSS_ENTRIES - 1;
      }
    }
  }
}

uint32_t fhss_get_freq(uint16_t index) {
  return fhss_freqs[index] - freq_correction;
}

uint32_t fhss_next_freq() {
  return fhss_get_freq(fhss_sequence[fhss_index++]);
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
  // FHSShopInterval * ceil(100 / FHSShopInterval * NR_FHSS_ENTRIES) or
  // FHSShopInterval * trunc((100 + (FHSShopInterval * NR_FHSS_ENTRIES) - 1) / (FHSShopInterval * NR_FHSS_ENTRIES))
  // With a interval of 4 this works out to: 2.4=4, FCC915=4, AU915=8, EU868=8, EU/AU433=36
  uint8_t interval = current_air_rate_config()->fhss_hop_interval;
  return interval * ((interval * NR_FHSS_ENTRIES + 99) / (interval * NR_FHSS_ENTRIES));
}

uint32_t fhss_rf_mode_cycle_interval() {
  expresslrs_mod_settings_t *config = current_air_rate_config();
  return ((uint32_t)11U * NR_FHSS_ENTRIES * config->fhss_hop_interval * config->interval) / (10U * 1000U);
}

#endif