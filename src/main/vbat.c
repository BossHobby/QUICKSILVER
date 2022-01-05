#include "vbat.h"

#include "control.h"
#include "drv_adc.h"
#include "flash.h"
#include "profile.h"

// compensation factor for li-ion internal model
// zero to bypass
#define CF1 0.25f

// the lowest vbatt is ever allowed to go
#define VBATTLOW_ABS 2.7f

extern profile_t profile;

void vbat_init() {
  int count = 0;
  while (count < 5000) {
    float bootadc = adc_read(0) * state.vreffilt;
    lpf(&state.vreffilt, adc_read(1), 0.9968f);
    lpf(&state.vbattfilt, bootadc, 0.9968f);
    count++;
  }

  if (profile.voltage.lipo_cell_count == 0) {
    // Lipo count not specified, trigger auto detect
    for (int i = 6; i > 0; i--) {
      float cells = i;
      if (state.vbattfilt / cells > 3.7f) {
        state.lipo_cell_count = (float)cells;
        break;
      }
    }
  } else {
    state.lipo_cell_count = (float)profile.voltage.lipo_cell_count;
  }

  state.vbattfilt_corr *= (float)state.lipo_cell_count;
}

void vbat_calc() {
  // read acd and scale based on processor voltage
  float battadc = adc_read(0) * state.vreffilt;
  // read and filter internal reference
  lpf(&state.vreffilt, adc_read(1), 0.9968f);

  // average of all motors
  static float thrfilt = 0;

  // filter motorpwm so it has the same delay as the filtered voltage
  // ( or they can use a single filter)
  lpf(&thrfilt, state.thrsum, 0.9968f); // 0.5 sec at 1.6ms loop time

  // li-ion battery model compensation time decay ( 18 seconds )
  lpf(&state.vbattfilt_corr, state.vbattfilt, FILTERCALC(1000, 18000e3));

  lpf(&state.vbattfilt, battadc, 0.9968f);

  float tempvolt = state.vbattfilt * (1.00f + CF1) - state.vbattfilt_corr * (CF1);

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

  state.vbatt_comp = tempvolt + vdrop_factor * thrfilt;

  if ((state.vbatt_comp < profile.voltage.vbattlow * state.lipo_cell_count + hyst) || (state.vbattfilt < VBATTLOW_ABS * state.lipo_cell_count))
    flags.lowbatt = 1;
  else
    flags.lowbatt = 0;
}

void vbat_lvc_throttle() {
#ifdef LVC_LOWER_THROTTLE
  static float throttle_i = 0.0f;

  if (flash_storage.lvc_lower_throttle == 1) {
    float throttle_p = 0.0f;

    if (state.vbattfilt < (float)LVC_LOWER_THROTTLE_VOLTAGE_RAW)
      throttle_p = ((float)LVC_LOWER_THROTTLE_VOLTAGE_RAW - state.vbattfilt) * (float)LVC_LOWER_THROTTLE_KP;

    if (state.vbatt_comp < (float)LVC_LOWER_THROTTLE_VOLTAGE)
      throttle_p = ((float)LVC_LOWER_THROTTLE_VOLTAGE - state.vbatt_comp) * (float)LVC_LOWER_THROTTLE_KP;

    if (throttle_p > 1.0f)
      throttle_p = 1.0f;

    if (throttle_p > 0) {
      throttle_i += throttle_p * 0.0001f; //ki
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