#include <unity.h>

#include "control/control.h"
#include "core/profile.h"
#include "core/target.h"
#include "core/tasks.h"
#include "driver/adc.h"
#include "driver/time.h"
#include "io/vbat.h"

extern uint8_t adc_active_channels;
extern void scheduler_test_run(task_t *task, uint32_t release_cycles);

void test_vbat_integrates_elapsed_time_across_delays_and_wrap() {
  const auto saved_pin = target.ibat;
  const auto saved_scale = profile.voltage.ibat_scale;
  const auto saved_channels = adc_active_channels;
  const task_t saved_task = tasks[TASK_VBAT];
  tasks[TASK_VBAT].has_started = false;
  target.ibat = PIN_A2;
  profile.voltage.ibat_scale = 100;
  adc_init();
  // The native ADC mock does not populate the common channel count.
  adc_active_channels = ADC_CHAN_MAX;
  time_test_set_us(UINT32_MAX - 2000);
  vbat_init();
  state.ibat_drawn = 0;
  scheduler_test_run(&tasks[TASK_VBAT], time_cycles());
  TEST_ASSERT_EQUAL_FLOAT(0, state.ibat_drawn);

  time_test_advance_us(1125);
  scheduler_test_run(&tasks[TASK_VBAT], time_cycles());
  TEST_ASSERT_GREATER_THAN_FLOAT(0, state.ibat_sag_filtered);
  float expected = state.ibat_sag_filtered * 1125.0f / 3600000000.0f;
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, expected, state.ibat_drawn);

  time_test_advance_us(5000); // A delayed execution crossing the microsecond wrap.
  scheduler_test_run(&tasks[TASK_VBAT], time_cycles());
  expected += state.ibat_sag_filtered * 5000.0f / 3600000000.0f;
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, expected, state.ibat_drawn);
  target.ibat = saved_pin;
  profile.voltage.ibat_scale = saved_scale;
  adc_active_channels = saved_channels;
  tasks[TASK_VBAT] = saved_task;
}
