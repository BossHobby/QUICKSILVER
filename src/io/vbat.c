#include "io/vbat.h"

#include "core/flash.h"
#include "core/profile.h"
#include "driver/adc.h"
#include "flight/control.h"
#include "util/util.h"

// compensation factor for li-ion internal model
// zero to bypass
#define CF1 0.25f

// the lowest vbatt is ever allowed to go
#define VBATTLOW_ABS 2.7f

extern profile_t profile;

void vbat_init() {
  int count = 0;
  while (count < 5000) {
    state.vbat = adc_read(ADC_CHAN_VBAT);
    lpf(&state.vbat_filtered, state.vbat, 0.9968f);
    count++;
  }

  if (profile.voltage.lipo_cell_count == 0) {
    // Lipo count not specified, trigger auto detect
    for (int i = 6; i > 0; i--) {
      float cells = i;
      if (state.vbat_filtered / cells > 3.7f) {
        state.lipo_cell_count = cells;
        break;
      }
    }
  } else {
    state.lipo_cell_count = profile.voltage.lipo_cell_count;
  }

  state.vbat_filtered_decay *= (float)state.lipo_cell_count;
}

void vbat_calc() {
  state.cpu_temp = adc_read(ADC_CHAN_TEMP);

  // read acd and scale based on processor voltage
  state.ibat = adc_read(ADC_CHAN_IBAT);
  lpf(&state.ibat_filtered, state.ibat, FILTERCALC(1000, 5000e3));

  // li-ion battery model compensation time decay ( 18 seconds )
  state.vbat = adc_read(ADC_CHAN_VBAT);
  lpf(&state.vbat_filtered, state.vbat, 0.9968f);
  lpf(&state.vbat_filtered_decay, state.vbat_filtered, FILTERCALC(1000, 18000e3));

  state.vbat_cell_avg = state.vbat_filtered_decay / (float)state.lipo_cell_count;

  // average of all motors
  // filter motorpwm so it has the same delay as the filtered voltage
  // ( or they can use a single filter)
  static float thrfilt = 0;
  lpf(&thrfilt, state.thrsum, 0.9968f); // 0.5 sec at 1.6ms loop time

  float tempvolt = state.vbat_filtered * (1.00f + CF1) - state.vbat_filtered_decay * (CF1);

#ifdef AUTO_VDROP_FACTOR

  static float lastout[12];
  static float lastin[12];
  static float vcomp[12];
  static float score[12];
  static int z = 0;
  static int minindex = 0;
  static int firstrun = 1;

  if (thrfilt > 0.1f) {
    vcomp[z] = tempvolt + (float)z * 0.1f * thrfilt;

    if (firstrun) {
      for (int y = 0; y < 12; y++)
        lastin[y] = vcomp[z];
      firstrun = 0;
    }
    float ans;
    //  y(n) = x(n) - x(n-1) + R * y(n-1)
    //  out = in - lastin + coeff*lastout
    // hpf
    ans = vcomp[z] - lastin[z] + FILTERCALC(1000 * 12, 6000e3) * lastout[z];
    lastin[z] = vcomp[z];
    lastout[z] = ans;
    lpf(&score[z], ans * ans, FILTERCALC(1000 * 12, 60e6));
    z++;

    if (z >= 12) {
      z = 0;
      float min = score[0];
      for (int i = 0; i < 12; i++) {
        if ((score[i]) < min) {
          min = (score[i]);
          minindex = i;
          // add an offset because it seems to be usually early
          minindex++;
        }
      }
    }
  }

  const float vdrop_factor = minindex * 0.1f;
#else
  const float vdrop_factor = VDROP_FACTOR;
#endif

  float hyst;
  if (flags.lowbatt)
    hyst = HYST;
  else
    hyst = 0.0f;

  state.vbat_compensated = tempvolt + vdrop_factor * thrfilt;
  state.vbat_compensated_cell_avg = state.vbat_compensated / (float)state.lipo_cell_count;

  if ((state.vbat_compensated_cell_avg < profile.voltage.vbattlow + hyst) || (state.vbat_cell_avg < VBATTLOW_ABS))
    flags.lowbatt = 1;
  else
    flags.lowbatt = 0;
}

void vbat_lvc_throttle() {
#ifdef LVC_LOWER_THROTTLE
  static float throttle_i = 0.0f;

  if (flash_storage.lvc_lower_throttle == 1) {
    float throttle_p = 0.0f;

    if (state.vbat_filtered < (float)LVC_LOWER_THROTTLE_VOLTAGE_RAW)
      throttle_p = ((float)LVC_LOWER_THROTTLE_VOLTAGE_RAW - state.vbat_filtered) * (float)LVC_LOWER_THROTTLE_KP;

    if (state.vbat_compensated < (float)LVC_LOWER_THROTTLE_VOLTAGE)
      throttle_p = ((float)LVC_LOWER_THROTTLE_VOLTAGE - state.vbat_compensated) * (float)LVC_LOWER_THROTTLE_KP;

    if (throttle_p > 1.0f)
      throttle_p = 1.0f;

    if (throttle_p > 0) {
      throttle_i += throttle_p * 0.0001f; // ki
    } else
      throttle_i -= 0.001f; // ki on release

    if (throttle_i > 0.5f)
      throttle_i = 0.5f;
    if (throttle_i < 0.0f)
      throttle_i = 0.0f;

    state.throttle -= throttle_p + throttle_i;
  }

#endif
}