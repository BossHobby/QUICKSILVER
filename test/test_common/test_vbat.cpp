#include <unity.h>

#include <algorithm>
#include <math.h>

#include "control/control.h"
#include "core/profile.h"
#include "driver/adc.h"
#include "driver/time.h"
#include "io/vbat.h"

extern uint16_t adc_set_raw_value(adc_chan_t chan, uint16_t value);

// The ADC converts continuously; model one native scan per elapsed millisecond.
static void vbat_test_advance_ms(unsigned ms) {
  for (unsigned i = 0; i < ms; i++) {
    time_test_advance_us(1000);
    adc_native_scan();
  }
}

static void vbat_test_init() {
  profile.voltage.lipo_cell_count = 1;
  profile.voltage.ibat_scale = 1000;
  target.ibat = PIN_A2;
  target.vbat = PIN_A1;
  adc_init();
  adc_native_scan();
  vbat_init();
}

void test_vbat_runs_on_fixed_period() {
  const auto saved_state = state;
  const auto saved_flags = flags;
  const auto saved_profile = profile;
  const auto saved_target = target;
  state = {};
  time_test_set_us(UINT32_MAX - 12000);
  vbat_test_init();

  // Early wakes report the remaining time; due passes report a full period.
  for (unsigned ms = 1; ms <= 20; ms++) {
    vbat_test_advance_ms(1);
    const unsigned remaining = 5 - ms % 5;
    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(remaining), vbat_calc());
  }

  // A late pass keeps the schedule's phase instead of stretching the period.
  vbat_test_advance_ms(7);
  TEST_ASSERT_EQUAL(pdMS_TO_TICKS(3), vbat_calc());
  vbat_test_advance_ms(3);
  TEST_ASSERT_EQUAL(pdMS_TO_TICKS(5), vbat_calc());

  // After a long worker delay the schedule restarts rather than catching up.
  vbat_test_advance_ms(30);
  TEST_ASSERT_EQUAL(pdMS_TO_TICKS(5), vbat_calc());
  vbat_test_advance_ms(1);
  TEST_ASSERT_EQUAL(pdMS_TO_TICKS(4), vbat_calc());

  state = saved_state;
  flags = saved_flags;
  profile = saved_profile;
  target = saved_target;
}

void test_vbat_current_filter_step_response() {
  const auto saved_state = state;
  const auto saved_flags = flags;
  const auto saved_profile = profile;
  const auto saved_target = target;
  const uint16_t saved_raw = adc_set_raw_value(ADC_CHAN_IBAT, 1000);
  state = {};
  time_test_set_us(UINT32_MAX - 50000);
  vbat_test_init();

  for (unsigned ms = 5; ms <= 100; ms += 5) {
    vbat_test_advance_ms(5);
    vbat_calc();
  }
  // Continuous-time responses at 100 ms, allowing discrete filter error.
  TEST_ASSERT_FLOAT_WITHIN(0.015f, 0.9568f, state.ibat_sag_filtered / state.ibat);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.581f, state.ibat_filtered / state.ibat);

  adc_set_raw_value(ADC_CHAN_IBAT, saved_raw);
  state = saved_state;
  flags = saved_flags;
  profile = saved_profile;
  target = saved_target;
}

void test_vbat_integrates_current_over_elapsed_time() {
  const auto saved_state = state;
  const auto saved_flags = flags;
  const auto saved_profile = profile;
  const auto saved_target = target;
  const uint16_t saved_raw = adc_set_raw_value(ADC_CHAN_IBAT, 1000);

  state = {};
  time_test_set_us(UINT32_MAX - 1005000);
  vbat_test_init();
  // Settle the current filter at a constant measured load.
  for (unsigned i = 0; i < 600; i++) {
    vbat_test_advance_ms(5);
    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(5), vbat_calc());
    if (i == 0)
      TEST_ASSERT_EQUAL_FLOAT(0, state.ibat_drawn);
  }
  const float current_ma = state.ibat;
  TEST_ASSERT_GREATER_THAN_FLOAT(0, current_ma);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, current_ma, state.ibat_sag_filtered);

  state.ibat_drawn = 0;
  uint32_t elapsed_ms = 0;
  // Includes a delayed service pass and the clock wrap above. Tolerance
  // stays well above float32 ulp at this magnitude.
  const uint32_t intervals_ms[] = {5, 5, 27, 100};
  for (uint32_t interval : intervals_ms) {
    vbat_test_advance_ms(interval);
    elapsed_ms += interval;
    vbat_calc();
    const float expected_mah = current_ma * (elapsed_ms * 1e-3f) / 3600.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_mah, state.ibat_drawn);
  }

  // A new load observed after a delay must not be charged to the interval
  // before it was observed. Missing samples keep the previous filtered value.
  float expected_mah = state.ibat_drawn;
  const struct {
    uint16_t raw_current;
    bool available;
  } samples[] = {{2000, true}, {0, false}, {0, true}};
  for (const auto &sample : samples) {
    adc_set_raw_value(ADC_CHAN_IBAT, sample.raw_current);
    const float held_current = state.ibat_sag_filtered;
    expected_mah += held_current * 0.1f / 3600.0f;
    if (sample.available) {
      vbat_test_advance_ms(100);
    } else {
      time_test_advance_us(100000);
    }
    vbat_calc();
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, expected_mah, state.ibat_drawn);
    if (!sample.available)
      TEST_ASSERT_EQUAL_FLOAT(held_current, state.ibat_sag_filtered);
  }

  adc_set_raw_value(ADC_CHAN_IBAT, saved_raw);
  state = saved_state;
  flags = saved_flags;
  profile = saved_profile;
  target = saved_target;
}

// Native VREF reads its calibration value, so full scale is 3300 mV.
static constexpr float ADC_TEST_MV_PER_COUNT = 3300.0f / 4095.0f;

static void vbat_test_set_battery(float volts, float amps) {
  const float volts_per_count = ADC_TEST_MV_PER_COUNT * profile.voltage.vbat_scale / 10000.0f;
  const float amps_per_count = ADC_TEST_MV_PER_COUNT * 10.0f / profile.voltage.ibat_scale;
  adc_set_raw_value(ADC_CHAN_VBAT, (uint16_t)(volts / volts_per_count + 0.5f));
  adc_set_raw_value(ADC_CHAN_IBAT, (uint16_t)(amps / amps_per_count + 0.5f));
}

static void vbat_test_run_ms(unsigned ms) {
  for (unsigned i = 0; i < ms / 5; i++) {
    vbat_test_advance_ms(5);
    vbat_calc();
  }
}

struct vbat_test_saved_t {
  control_state_t state = ::state;
  control_flags_t flags = ::flags;
  profile_t profile = ::profile;
  target_t target = ::target;
  uint16_t vbat = adc_set_raw_value(ADC_CHAN_VBAT, 0);
  uint16_t ibat = adc_set_raw_value(ADC_CHAN_IBAT, 0);

  ~vbat_test_saved_t() {
    adc_set_raw_value(ADC_CHAN_VBAT, vbat);
    adc_set_raw_value(ADC_CHAN_IBAT, ibat);
    ::state = state;
    ::flags = flags;
    ::profile = profile;
    ::target = target;
  }
};

static void vbat_test_start(bool current_sensor, float volts, uint8_t cells) {
  state = {};
  flags = {};
  profile.voltage.lipo_cell_count = cells;
  profile.voltage.vbat_scale = 110;
  profile.voltage.ibat_scale = current_sensor ? 1000 : 0;
  profile.voltage.actual_battery_voltage = 4.2f;
  profile.voltage.reported_telemetry_voltage = 4.2f;
  profile.voltage.vbattlow = 3.3f;
  profile.voltage.use_filtered_voltage_for_warnings = 0;
  target.vbat = PIN_A1;
  target.ibat = current_sensor ? PIN_A2 : PIN_NONE;
  time_test_set_us(0);
  vbat_test_set_battery(volts, 0);
  adc_init();
  adc_native_scan();
  vbat_init();
}

void test_vbat_sag_compensation_learns_current_resistance() {
  const vbat_test_saved_t saved;
  constexpr float OPEN_VOLTAGE = 16.0f;
  constexpr float RESISTANCE = 0.05f;
  vbat_test_start(true, OPEN_VOLTAGE, 4);
  flags.in_air = 1;

  float max_error = 0;
  for (unsigned step = 0; step < 60; step++) {
    const float amps = step % 2 ? 25.0f : 5.0f;
    vbat_test_set_battery(OPEN_VOLTAGE - RESISTANCE * amps, amps);
    for (unsigned ms = 0; ms < 500; ms += 5) {
      vbat_test_run_ms(5);
      // Voltage and current share one filter, so a learned R holds the
      // compensated voltage through the load steps, not only between them.
      if (step >= 50)
        max_error = std::max(max_error, fabsf(state.vbat_compensated - OPEN_VOLTAGE));
    }
  }
  TEST_ASSERT_LESS_THAN_FLOAT(OPEN_VOLTAGE - 0.5f, state.vbat_sag_filtered);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, max_error);
}

void test_vbat_sag_compensation_uses_throttle_without_current_sensor() {
  const vbat_test_saved_t saved;
  constexpr float OPEN_VOLTAGE = 16.0f;
  constexpr float VOLTS_PER_THROTTLE = 2.0f;
  vbat_test_start(false, OPEN_VOLTAGE, 4);
  flags.in_air = 1;

  float max_error = 0;
  for (unsigned step = 0; step < 60; step++) {
    state.thrsum = step % 2 ? 0.7f : 0.3f;
    vbat_test_set_battery(OPEN_VOLTAGE - VOLTS_PER_THROTTLE * state.thrsum, 0);
    for (unsigned ms = 0; ms < 500; ms += 5) {
      vbat_test_run_ms(5);
      if (step >= 50)
        max_error = std::max(max_error, fabsf(state.vbat_compensated - OPEN_VOLTAGE));
    }
  }
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, max_error);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, OPEN_VOLTAGE / 4, state.vbat_compensated_cell_avg);
}

void test_vbat_sag_compensation_ignores_discharge() {
  const vbat_test_saved_t saved;
  constexpr float RESISTANCE = 0.05f;
  vbat_test_start(true, 16.0f, 4);
  flags.in_air = 1;

  // Discharge under a constant load is not mistaken for sag: with no sag,
  // the falling voltage stays uncompensated.
  for (unsigned ms = 0; ms < 20000; ms += 5) {
    vbat_test_set_battery(16.0f - 0.5f * ms / 20000.0f, 10.0f);
    vbat_test_run_ms(5);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, state.vbat_sag_filtered, state.vbat_compensated);

  // Load steps during the same discharge recover only the resistance.
  float max_error = 0;
  for (unsigned step = 0; step < 60; step++) {
    const float amps = step % 2 ? 25.0f : 5.0f;
    for (unsigned ms = 0; ms < 500; ms += 5) {
      const float open_voltage = 15.5f - 0.5f * (step * 500 + ms) / 30000.0f;
      vbat_test_set_battery(open_voltage - RESISTANCE * amps, amps);
      vbat_test_run_ms(5);
      if (step >= 50)
        max_error = std::max(max_error, fabsf(state.vbat_compensated - open_voltage));
    }
  }
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, max_error);
}

void test_vbat_sag_compensation_learns_only_in_air() {
  const vbat_test_saved_t saved;
  vbat_test_start(true, 16.0f, 4);
  flags.in_air = 0;
  for (unsigned step = 0; step < 20; step++) {
    const float amps = step % 2 ? 25.0f : 5.0f;
    vbat_test_set_battery(16.0f - 0.05f * amps, amps);
    vbat_test_run_ms(500);
    TEST_ASSERT_EQUAL_FLOAT(state.vbat_sag_filtered, state.vbat_compensated);
  }
}

void test_vbat_filtered_warning_has_hysteresis() {
  const vbat_test_saved_t saved;
  vbat_test_start(false, 3.6f, 1);
  profile.voltage.use_filtered_voltage_for_warnings = 1;

  const struct {
    float volts;
    bool lowbatt;
  } steps[] = {{3.35f, false}, {3.25f, true}, {3.35f, true}, {3.38f, true}, {3.45f, false}, {3.35f, false}};
  for (const auto &step : steps) {
    vbat_test_set_battery(step.volts, 0);
    vbat_test_run_ms(2000);
    TEST_ASSERT_EQUAL(step.lowbatt, flags.lowbatt);
  }
}

void test_vbat_detects_cell_count_across_charge() {
  const vbat_test_saved_t saved;
  const struct {
    float volts;
    uint8_t cells;
  } packs[] = {
      {4.35f, 1},  // full HV 1S
      {3.4f, 1},   // depleted 1S
      {8.0f, 2},   // 2S storage
      {13.4f, 4},  // depleted 4S
      {16.8f, 4},  // full 4S
      {17.4f, 4},  // full HV 4S
      {21.0f, 5},  // full 5S
      {25.2f, 6},  // full 6S
  };
  for (const auto &pack : packs) {
    vbat_test_start(false, pack.volts, 0);
    TEST_ASSERT_EQUAL_UINT8(pack.cells, state.lipo_cell_count);
  }
}

void test_vbat_redetects_cells_when_battery_connects() {
  const vbat_test_saved_t saved;
  // Booted on USB power without a battery.
  vbat_test_start(false, 0.5f, 0);
  TEST_ASSERT_EQUAL_UINT8(1, state.lipo_cell_count);

  vbat_test_set_battery(16.4f, 0);
  vbat_test_run_ms(2000);
  TEST_ASSERT_EQUAL_UINT8(4, state.lipo_cell_count);

  // A pack resting below the detection boundary keeps its cell.
  vbat_test_set_battery(13.0f, 0);
  vbat_test_run_ms(2000);
  TEST_ASSERT_EQUAL_UINT8(4, state.lipo_cell_count);

  // Detection never changes the count while armed.
  flags.arm_state = 1;
  vbat_test_set_battery(25.0f, 0);
  vbat_test_run_ms(2000);
  TEST_ASSERT_EQUAL_UINT8(4, state.lipo_cell_count);

  // A configured count applies without a reboot.
  flags.arm_state = 0;
  profile.voltage.lipo_cell_count = 3;
  vbat_test_run_ms(5);
  TEST_ASSERT_EQUAL_UINT8(3, state.lipo_cell_count);
}
