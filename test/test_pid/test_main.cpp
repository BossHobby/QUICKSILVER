#include "driver/time.h"
#include <unity.h>

// PID tests
extern void test_pid_proportional_control(void);
extern void test_pid_integral_accumulation(void);
extern void test_pid_derivative_calculation(void);
extern void test_pid_dterm_setpoint_response(void);
extern void test_pid_dterm_combined_response(void);
extern void test_pid_dterm_stick_weighting(void);
extern void test_pid_voltage_compensation(void);
extern void test_pid_complete_loop(void);
extern void test_angle_pid_uses_legacy_dterm_timefactor(void);

void setUp() { time_test_reset(); }
void tearDown() {}

int main() {
  UNITY_BEGIN();
  // PID tests
  RUN_TEST(test_pid_proportional_control);
  RUN_TEST(test_pid_integral_accumulation);
  RUN_TEST(test_pid_derivative_calculation);
  RUN_TEST(test_pid_dterm_setpoint_response);
  RUN_TEST(test_pid_dterm_combined_response);
  RUN_TEST(test_pid_dterm_stick_weighting);
  RUN_TEST(test_pid_voltage_compensation);
  RUN_TEST(test_pid_complete_loop);
  RUN_TEST(test_angle_pid_uses_legacy_dterm_timefactor);

  return UNITY_END();
}
