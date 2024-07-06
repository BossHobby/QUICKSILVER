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
    lpf(&state.vbat_filtered, state.vbat, lpfcalc(1, 500));
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

static float vbat_auto_vdrop(float thrfilt, float tempvolt) {
  static int minindex = 0;

  if (thrfilt <= 0.1f) {
    return minindex * 0.1f;
  }

  static int z = 0;
  static float lastin[12];
  static float lastout[12];

  //  y(n) = x(n) - x(n-1) + R * y(n-1)
  //  out = in - lastin + coeff*lastout
  const float vcomp = tempvolt + (float)z * 0.1f * thrfilt;
  const float ans = vcomp - lastin[z] + lpfcalc(1000 * 12, 6000e3) * lastout[z];
  lastin[z] = vcomp;
  lastout[z] = ans;

  static float score[12];
  lpf(&score[z], ans * ans, lpfcalc(1000 * 12, 60e6));
  z++;

  if (z >= 12) {
    z = 0;
    float min = score[0];
    for (int i = 0; i < 12; i++) {
      if ((score[i]) < min) {
        min = (score[i]);
        // add an offset because it seems to be usually early
        minindex = i + 1;
      }
    }
  }

  return minindex * 0.1f;
}

void vbat_calc() {
  state.cpu_temp = adc_read(ADC_CHAN_TEMP);

  // read acd and scale based on processor voltage
  state.ibat = adc_read(ADC_CHAN_IBAT);
  lpf(&state.ibat_filtered, state.ibat, lpfcalc_hz(0.001, 1));

  // li-ion battery model compensation time decay ( 18 seconds )
  state.vbat = adc_read(ADC_CHAN_VBAT);
  lpf(&state.vbat_filtered, state.vbat, lpfcalc(1, 500));
  lpf(&state.vbat_filtered_decay, state.vbat_filtered, lpfcalc(1, 18000));

  state.vbat_cell_avg = state.vbat_filtered_decay / (float)state.lipo_cell_count;

  // average of all motors
  // filter motorpwm so it has the same delay as the filtered voltage
  // ( or they can use a single filter)
  static float thrfilt = 0;
  lpf(&thrfilt, state.thrsum, lpfcalc(1, 500));

  const float tempvolt = state.vbat_filtered * (1.00f + CF1) - state.vbat_filtered_decay * (CF1);
  const float hyst = flags.lowbatt ? HYST : 0.0f;
  const float vdrop_factor = vbat_auto_vdrop(thrfilt, tempvolt);
  state.vbat_compensated = tempvolt + vdrop_factor * thrfilt;
  state.vbat_compensated_cell_avg = state.vbat_compensated / (float)state.lipo_cell_count;

  if (profile.voltage.use_filtered_voltage_for_warnings) {
    flags.lowbatt = state.vbat_cell_avg < profile.voltage.vbattlow ? 1 : 0;
  } else {
    if ((state.vbat_compensated_cell_avg < profile.voltage.vbattlow + hyst) || (state.vbat_cell_avg < VBATTLOW_ABS))
      flags.lowbatt = 1;
    else
      flags.lowbatt = 0;
  }
}
