#include "io/vbat.h"

#include <algorithm>

#include "control/control.h"
#include "core/failloop.h"
#include "core/profile.h"
#include "driver/adc.h"
#include "driver/time.h"
#include "util/util.h"

// the lowest vbatt is ever allowed to go
#define VBATTLOW_ABS 2.7f

// Each pass publishes the ADC average since the previous one. The period is
// longer than the slowest full scan (~4 ms on G4), so windows are complete.
static constexpr uint32_t VBAT_PERIOD_US = 5000;
static constexpr float US_PER_HOUR = 60.0f * 60.0f * 1000000.0f;
static constexpr float DISPLAY_FILTER_HZ = 2.0f;
static constexpr float SAG_FILTER_HZ = 5.0f;
static constexpr uint32_t ADC_STARTUP_TIMEOUT_US = 100000;

// Above a fully charged HV cell, so a full pack never counts an extra cell.
static constexpr float CELL_MAX_VOLTAGE = 4.4f;
static constexpr uint8_t CELL_COUNT_MAX = 8;

// Sag model: V = V_open - R * load. Load is the measured current in amps with a
// current sensor, otherwise the average motor output. R is the least-squares
// slope of high-passed voltage against high-passed load: the high-pass removes
// discharge, and the averages remember several seconds of in-flight load steps.
static constexpr float SAG_HIGHPASS_HZ = 0.5f;
static constexpr float SAG_AVERAGE_HZ = 0.02f;
// Load variation (RMS, relative to the mean load) needed to update R.
static constexpr float SAG_MIN_RELATIVE_EXCITATION = 0.1f;

static struct {
  filter_t<> voltage_display;
  filter_t<> current_display;
  // Voltage, current and throttle share the sag cutoff, so compensation does
  // not lead or lag the voltage it corrects.
  filter_t<> voltage_sag;
  filter_t<> current_sag;
  filter_t<> throttle_sag;

  filter_t<> voltage_trend;
  filter_t<> load_trend;
  filter_t<> covariance;
  filter_t<> variance;
  bool current_load;
  float resistance;

  uint32_t last_update_us;
  uint32_t next_update_us;
} battery;

static void vbat_reset_sag_estimate() {
  battery.load_trend.reset();
  battery.covariance.reset();
  battery.variance.reset();
  battery.resistance = 0;
}

static uint8_t vbat_detect_cell_count(float voltage) {
  return std::min<uint8_t>(voltage / CELL_MAX_VOLTAGE + 1, CELL_COUNT_MAX);
}

static void vbat_update_cell_count() {
  if (profile.voltage.lipo_cell_count != 0) {
    state.lipo_cell_count = profile.voltage.lipo_cell_count;
    return;
  }
  // A battery connected after booting on USB raises the count while disarmed.
  // The count never falls, so a pack resting low cannot lose a cell.
  if (!flags.arm_state)
    state.lipo_cell_count = std::max(state.lipo_cell_count, vbat_detect_cell_count(state.vbat_filtered));
}

void vbat_init() {
  battery = {};
  battery.voltage_display.init(FILTER_LP_PT2, DISPLAY_FILTER_HZ, VBAT_PERIOD_US);
  battery.current_display.init(FILTER_LP_PT2, DISPLAY_FILTER_HZ, VBAT_PERIOD_US);
  battery.voltage_sag.init(FILTER_LP_PT1, SAG_FILTER_HZ, VBAT_PERIOD_US);
  battery.current_sag.init(FILTER_LP_PT1, SAG_FILTER_HZ, VBAT_PERIOD_US);
  battery.throttle_sag.init(FILTER_LP_PT1, SAG_FILTER_HZ, VBAT_PERIOD_US);
  battery.voltage_trend.init(FILTER_LP_PT1, SAG_HIGHPASS_HZ, VBAT_PERIOD_US);
  battery.load_trend.init(FILTER_LP_PT1, SAG_HIGHPASS_HZ, VBAT_PERIOD_US);
  battery.covariance.init(FILTER_LP_PT1, SAG_AVERAGE_HZ, VBAT_PERIOD_US);
  battery.variance.init(FILTER_LP_PT1, SAG_AVERAGE_HZ, VBAT_PERIOD_US);

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
    state.vbat_filtered = battery.voltage_display.step(state.vbat);
    state.vbat_sag_filtered = battery.voltage_sag.step(state.vbat);
    battery.voltage_trend.step(state.vbat);
  }

  state.lipo_cell_count = 1;
  vbat_update_cell_count();
  state.vbat_cell_avg = state.vbat_filtered / (float)state.lipo_cell_count;
  state.vbat_compensated = state.vbat_sag_filtered;
  state.vbat_compensated_cell_avg = state.vbat_compensated / (float)state.lipo_cell_count;

  battery.last_update_us = time_micros();
  battery.next_update_us = battery.last_update_us + VBAT_PERIOD_US;
}

static void vbat_update_compensation() {
  const float throttle = battery.throttle_sag.step(state.thrsum);
  // Ground configuration can enable or disable the current sensor; R from the
  // other load unit does not apply.
  const bool current_load = target.ibat != PIN_NONE && profile.voltage.ibat_scale != 0;
  if (current_load != battery.current_load) {
    battery.current_load = current_load;
    vbat_reset_sag_estimate();
  }
  const float load = current_load ? state.ibat_sag_filtered * 0.001f : throttle;

  const float voltage_step = state.vbat_sag_filtered - battery.voltage_trend.step(state.vbat_sag_filtered);
  const float load_mean = battery.load_trend.step(load);
  const float load_step = load - load_mean;

  // Only motor load changes in flight reveal the pack's resistance.
  if (flags.in_air) {
    const float covariance = battery.covariance.step(voltage_step * load_step);
    const float variance = battery.variance.step(load_step * load_step);
    const float min_variance = SAG_MIN_RELATIVE_EXCITATION * SAG_MIN_RELATIVE_EXCITATION * load_mean * load_mean;
    if (variance > 0.0f && variance >= min_variance)
      battery.resistance = std::max(0.0f, -covariance / variance);
  }

  state.vbat_compensated = state.vbat_sag_filtered + battery.resistance * load;
  state.vbat_compensated_cell_avg = state.vbat_compensated / (float)state.lipo_cell_count;
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
    state.ibat_filtered = battery.current_display.step(state.ibat);
    state.ibat_sag_filtered = battery.current_sag.step(state.ibat);
    state.vbat = adc_read(ADC_CHAN_VBAT);
    state.vbat_filtered = battery.voltage_display.step(state.vbat);
    state.vbat_sag_filtered = battery.voltage_sag.step(state.vbat);
    vbat_update_cell_count();
    vbat_update_compensation();
  }

  state.vbat_cell_avg = state.vbat_filtered / (float)state.lipo_cell_count;

  // Keep a warning until the voltage recovers past the threshold plus HYST.
  const float hyst = flags.lowbatt ? HYST : 0.0f;
  const float threshold = profile.voltage.vbattlow + hyst;
  if (profile.voltage.use_filtered_voltage_for_warnings) {
    flags.lowbatt = state.vbat_sag_filtered / (float)state.lipo_cell_count < threshold;
  } else {
    flags.lowbatt = state.vbat_compensated_cell_avg < threshold || state.vbat_cell_avg < VBATTLOW_ABS;
  }

  return pdMS_TO_TICKS(((battery.next_update_us - now) + 999) / 1000);
}
