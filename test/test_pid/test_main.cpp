#include "driver/time.h"
#include <unity.h>

// PID tests
#ifdef VEHICLE_WING
extern void test_wing_rate_feedforward_tracks_target_while_disarmed(void);
extern void test_wing_rate_feedforward_profile_switch_and_limits(void);
#endif
#ifdef VEHICLE_MULTI
extern void test_horizon_error_matches_setpoint(void);
#endif
extern void test_pid_proportional_control(void);
extern void test_pid_rates_update_preserves_integral(void);
extern void test_pid_filter_update_retains_history_on_period_change(void);
extern void test_pid_integral_accumulation(void);
extern void test_pid_derivative_calculation(void);
extern void test_pid_dterm_setpoint_response(void);
extern void test_pid_dterm_combined_response(void);
extern void test_pid_dterm_stick_weighting(void);
extern void test_pid_voltage_compensation(void);
extern void test_pid_complete_loop(void);
extern void test_angle_pid_uses_legacy_dterm_timefactor(void);
extern void test_angle_pid_rth_damps_rotation_without_target_kick(void);

void setUp() { time_test_reset(); }
void tearDown() {}

int main() {
  UNITY_BEGIN();
  // PID tests
#ifdef VEHICLE_WING
  RUN_TEST(test_wing_rate_feedforward_tracks_target_while_disarmed);
  RUN_TEST(test_wing_rate_feedforward_profile_switch_and_limits);
#endif
#ifdef VEHICLE_MULTI
  RUN_TEST(test_horizon_error_matches_setpoint);
#endif
  RUN_TEST(test_pid_proportional_control);
  RUN_TEST(test_pid_rates_update_preserves_integral);
  RUN_TEST(test_pid_filter_update_retains_history_on_period_change);
  RUN_TEST(test_pid_integral_accumulation);
  RUN_TEST(test_pid_derivative_calculation);
  RUN_TEST(test_pid_dterm_setpoint_response);
  RUN_TEST(test_pid_dterm_combined_response);
  RUN_TEST(test_pid_dterm_stick_weighting);
  RUN_TEST(test_pid_voltage_compensation);
  RUN_TEST(test_pid_complete_loop);
  RUN_TEST(test_angle_pid_uses_legacy_dterm_timefactor);
  RUN_TEST(test_angle_pid_rth_damps_rotation_without_target_kick);

  return UNITY_END();
}
