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
  flags.arm_state = false;
  baro_test_sample(99, 0);
  baro_test_sample(100, 0);
  TEST_ASSERT_FALSE(state.baro_valid); // IO only publishes; Flight derives altitude.
  time_test_set_us(10000);
  nav_update();
  TEST_ASSERT_TRUE(state.baro_valid);
  TEST_ASSERT_EQUAL_FLOAT(0, state.altitude);

  flags.arm_state = true;
  time_test_set_us(20000);
  nav_update();
  baro_test_sample(101, 50);
  time_test_set_us(50000);
  nav_update();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f / 3.0f, state.altitude);
  TEST_ASSERT_GREATER_THAN_FLOAT(0, state.baro_vertical_speed);
  TEST_ASSERT_EQUAL_UINT32(50, state.baro_last_update_ms);

  baro_test_sample(NAN, 100);
  time_test_set_us(100000);
  nav_update();
  TEST_ASSERT_FALSE(state.baro_valid);
  TEST_ASSERT_EQUAL_UINT32(50, state.baro_last_update_ms);
  baro_test_sample(120, 1000);
  time_test_set_us(1000000);
  nav_update();
  TEST_ASSERT_TRUE(state.baro_valid);
  TEST_ASSERT_EQUAL_FLOAT(0, state.baro_vertical_speed);
  TEST_ASSERT_EQUAL_FLOAT(20, state.altitude);

  flags.arm_state = false;
  time_test_advance_us(10000);
  nav_update();
  TEST_ASSERT_EQUAL_FLOAT(0, state.altitude);
  flags.arm_state = true;
  time_test_advance_us(10000);
  nav_update();
  TEST_ASSERT_EQUAL_FLOAT(0, state.altitude);
  baro_test_sample(123, 2000);
  time_test_set_us(2000000);
  nav_update();
  TEST_ASSERT_EQUAL_FLOAT(3, state.altitude);
  profile.serial.gps = saved_gps;
  flags.arm_state = saved_arm;
}
