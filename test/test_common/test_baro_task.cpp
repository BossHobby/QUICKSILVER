#include <math.h>
#include <unity.h>

#include "control/control.h"
#include "control/navigation.h"
#include "core/profile.h"
#include "driver/baro/baro.h"
#include "driver/time.h"

extern void baro_test_sample(float altitude, uint32_t now_ms);

void test_baro_altitude_reference_without_gps() {
  const auto saved_gps = profile.serial.gps;
  const auto saved_arm = flags.arm_state;
  profile.serial.gps = SERIAL_PORT_INVALID;
  time_test_set_us(0);
  baro_init();
  nav_init();
  // Level and unaccelerated, so only the barometer moves the estimate.
  state.GEstG = (vec3_t){{0, 0, 1}};
  state.accel_raw = state.GEstG;
  flags.arm_state = false;
  baro_test_sample(99, 0);
  baro_test_sample(100, 0);
  TEST_ASSERT_FALSE(flags.altitude_valid); // IO only publishes; Flight derives altitude.
  time_test_set_us(10000);
  nav_update();
  TEST_ASSERT_TRUE(flags.altitude_valid);
  TEST_ASSERT_EQUAL_FLOAT(0, state.altitude);

  flags.arm_state = true;
  time_test_set_us(20000);
  nav_update();
  baro_test_sample(101, 50);
  time_test_set_us(50000);
  nav_update();
  TEST_ASSERT_TRUE(state.altitude > 0.0f && state.altitude < 1.0f);
  TEST_ASSERT_GREATER_THAN_FLOAT(0, state.velocity.axis[2]);

  // A non-finite sample is dropped; the altitude lapses once the last one is stale.
  baro_test_sample(NAN, 100);
  time_test_set_us(100000);
  nav_update();
  TEST_ASSERT_TRUE(flags.altitude_valid);
  time_test_set_us((50 + BARO_STALE_MS + 10) * 1000);
  nav_update();
  TEST_ASSERT_FALSE(flags.altitude_valid);
  const float held = state.altitude;
  time_test_advance_us(100000);
  nav_update();
  TEST_ASSERT_EQUAL_FLOAT(held, state.altitude);

  // Reacquisition starts from the measurement.
  baro_test_sample(120, 1000);
  time_test_set_us(1000000);
  nav_update();
  TEST_ASSERT_TRUE(flags.altitude_valid);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 20, state.altitude);

  flags.arm_state = false;
  time_test_advance_us(10000);
  nav_update();
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 20, state.altitude); // still relative to the last launch
  baro_test_sample(110, 1010);
  time_test_set_us(1020000);
  nav_update();
  TEST_ASSERT_LESS_THAN_FLOAT(20, state.altitude); // disarmed altitude follows the baro
  flags.arm_state = true;
  time_test_advance_us(10000);
  nav_update();
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0, state.altitude); // arming re-zeroes the estimate, which then fuses on
  // After an outage, the new launch lies between the last two samples, where
  // the estimate was.
  time_test_set_us(1990000);
  nav_update();
  TEST_ASSERT_FALSE(flags.altitude_valid);
  baro_test_sample(123, 2000);
  time_test_set_us(2000000);
  nav_update();
  TEST_ASSERT_TRUE(state.altitude > 123 - 120 && state.altitude < 123 - 110);
  profile.serial.gps = saved_gps;
  flags.arm_state = saved_arm;
}
