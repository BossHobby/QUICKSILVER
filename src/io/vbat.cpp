#include "io/vbat.h"

#include "control/control.h"
#include "core/failloop.h"
#include "core/flash.h"
#include "core/profile.h"
#include "driver/adc.h"
#include "driver/time.h"
#include "util/util.h"

// compensation factor for li-ion internal model
// zero to bypass
#define CF1 0.25f

// the lowest vbatt is ever allowed to go
#define VBATTLOW_ABS 2.7f

#define THRSUM_FILTER_HZ 60

// Each pass publishes the ADC average since the previous one. The period is
// longer than the slowest full scan (~4 ms on G4), so windows are complete.
static constexpr uint32_t VBAT_PERIOD_US = 5000;
static constexpr float US_PER_HOUR = 60.0f * 60.0f * 1000000.0f;
static constexpr float DISPLAY_FILTER_HZ = 2.0f;
static constexpr float SAG_FILTER_HZ = 5.0f;
static constexpr uint32_t ADC_STARTUP_TIMEOUT_US = 100000;

struct battery_filter_state_t {
  filter_state_t display;
  filter_state_t sag;
};

static struct {
  // Both measurements arrive in one ADC window and share filter coefficients.
  filter_lp_pt2 display_filter;
  filter_lp_pt1 sag_filter;
  battery_filter_state_t voltage;
  battery_filter_state_t current;
  filter_lp_pt1 throttle_filter;
  filter_state_t throttle_state;
  float voltage_decay;
  uint32_t last_update_us;
  uint32_t next_update_us;
} battery;

void vbat_init() {
  battery = {};
  filter_lp_pt2_coeff(&battery.display_filter, DISPLAY_FILTER_HZ, VBAT_PERIOD_US);
  filter_lp_pt1_coeff(&battery.sag_filter, SAG_FILTER_HZ, VBAT_PERIOD_US);
  filter_lp_pt1_coeff(&battery.throttle_filter, THRSUM_FILTER_HZ, VBAT_PERIOD_US);

  // Boot runs with interrupts enabled, before IO owns acquisition. Cell count
  // detection needs a complete first window, including its reference voltage.
  // Conversions that never complete leave the ADC interrupt unserviced.
  const uint32_t start_us = time_micros();
  while (!adc_update()) {
    if (time_micros() - start_us > ADC_STARTUP_TIMEOUT_US)
      failloop(FAILLOOP_FAULT);
    time_delay_us(100);
  }
  state.vbat = adc_read(ADC_CHAN_VBAT);
  for (size_t i = 0; i < 5000; i++) {
    state.vbat_filtered = filter_lp_pt2_step(&battery.display_filter, &battery.voltage.display, state.vbat);
    state.vbat_sag_filtered = filter_lp_pt1_step(&battery.sag_filter, &battery.voltage.sag, state.vbat);
  }

  if (profile.voltage.lipo_cell_count == 0) {
    // Lipo count not specified, trigger auto detect
    for (uint32_t i = 8; i > 0; i--) {
      if (state.vbat_filtered / (float)(i) > 3.7f) {
        state.lipo_cell_count = i;
        break;
      }
    }
  } else {
    state.lipo_cell_count = profile.voltage.lipo_cell_count;
  }

  battery.voltage_decay = state.vbat_sag_filtered;
  battery.last_update_us = time_micros();
  battery.next_update_us = battery.last_update_us + VBAT_PERIOD_US;
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
  const float ans = vcomp - lastin[z] + lpfcalc(VBAT_PERIOD_US * 12, 6000e3) * lastout[z];
  lastin[z] = vcomp;
  lastout[z] = ans;

  static float score[12];
  lpf(&score[z], ans * ans, lpfcalc(VBAT_PERIOD_US * 12, 60e6));
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

TickType_t vbat_calc() {
  const uint32_t now = time_micros();
  const int32_t wait_us = (int32_t)(battery.next_update_us - now);
  if (wait_us > 0) {
    return pdMS_TO_TICKS((wait_us + 999) / 1000);
  }
  // Keep the schedule's phase so tick rounding does not stretch the period;
  // after a long worker delay, restart it from now instead of catching up.
  battery.next_update_us += VBAT_PERIOD_US;
  if ((int32_t)(battery.next_update_us - now) <= 0)
    battery.next_update_us = now + VBAT_PERIOD_US;

  const uint32_t elapsed_us = now - battery.last_update_us;
  battery.last_update_us = now;
  // Account for the elapsed interval with the previously held current, before
  // a fresh sample changes the filter output (especially after worker delays).
  state.ibat_drawn += state.ibat_sag_filtered * elapsed_us / US_PER_HOUR;
  if (adc_update()) {
    state.cpu_temp = adc_read(ADC_CHAN_TEMP);
    state.ibat = adc_read(ADC_CHAN_IBAT);
    state.ibat_filtered = filter_lp_pt2_step(&battery.display_filter, &battery.current.display, state.ibat);
    state.ibat_sag_filtered = filter_lp_pt1_step(&battery.sag_filter, &battery.current.sag, state.ibat);
    state.vbat = adc_read(ADC_CHAN_VBAT);
    state.vbat_filtered = filter_lp_pt2_step(&battery.display_filter, &battery.voltage.display, state.vbat);
    state.vbat_sag_filtered = filter_lp_pt1_step(&battery.sag_filter, &battery.voltage.sag, state.vbat);
    // Li-ion compensation model: 18-second voltage decay.
    lpf(&battery.voltage_decay, state.vbat_sag_filtered, lpfcalc(VBAT_PERIOD_US * 1e-6f, 18));
  }

  state.vbat_cell_avg = state.vbat_filtered / (float)state.lipo_cell_count;

  // average of all motors
  // filter motorpwm so it has the same delay as the filtered voltage
  const float thrfilt = filter_lp_pt1_step(&battery.throttle_filter, &battery.throttle_state, state.thrsum);
  // Use sag filtered value for compensation calculations
  const float tempvolt = state.vbat_sag_filtered * (1.00f + CF1) - battery.voltage_decay * (CF1);
  const float hyst = flags.lowbatt ? HYST : 0.0f;
  const float vdrop_factor = vbat_auto_vdrop(thrfilt, tempvolt);
  state.vbat_compensated = tempvolt + vdrop_factor * thrfilt;
  state.vbat_compensated_cell_avg = state.vbat_compensated / (float)state.lipo_cell_count;

  // Use sag filtered (faster) voltage for warnings
  const float vbat_sag_cell_avg = state.vbat_sag_filtered / (float)state.lipo_cell_count;
  
  if (profile.voltage.use_filtered_voltage_for_warnings) {
    flags.lowbatt = vbat_sag_cell_avg < profile.voltage.vbattlow ? 1 : 0;
  } else {
    if ((state.vbat_compensated_cell_avg < profile.voltage.vbattlow + hyst) || (state.vbat_cell_avg < VBATTLOW_ABS))
      flags.lowbatt = 1;
    else
      flags.lowbatt = 0;
  }

  return pdMS_TO_TICKS(((battery.next_update_us - now) + 999) / 1000);
}
