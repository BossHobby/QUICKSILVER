#include <unity.h>

#include "control/gestures.h"
#include "core/project.h"

static uint32_t gesture_time;
static bool menu_mode;

static vec4_t gesture_sticks(char direction) {
  vec4_t sticks = {};
#ifdef VEHICLE_ROVER
  sticks.throttle = 0.5f;
  if (direction == 'L') sticks.throttle = 0.0f;
  if (direction == 'R') sticks.throttle = 1.0f;
  if (direction == 'U') sticks.yaw = -1.0f;
  if (direction == 'D') sticks.yaw = 1.0f;
  if (direction == '?') sticks.yaw = 0.4f;
#else
  if (direction == 'L') sticks.roll = -1.0f;
  if (direction == 'R') sticks.roll = 1.0f;
  if (direction == 'U') sticks.pitch = 1.0f;
  if (direction == 'D') sticks.pitch = -1.0f;
  if (direction == '?') sticks.pitch = 0.4f;
#endif
  return sticks;
}

static gesture_command_t hold_gesture(char direction, uint32_t duration) {
  const auto sticks = gesture_sticks(direction);
  gestures_detect(sticks, true, menu_mode, gesture_time);
  gesture_time += duration;
  return gestures_detect(sticks, true, menu_mode, gesture_time);
}

static void begin_gestures(bool menu, uint32_t start = 0) {
  gesture_time = start;
  menu_mode = menu;
  gestures_detect(gesture_sticks('C'), false, menu, gesture_time);
  TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture('C', menu ? 100000 : 700000));
}

static gesture_command_t tap_gesture(char direction) {
  TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture(direction, 60000));
  return hold_gesture('C', 60000);
}

void test_gestures_shortcuts_and_clock_wrap() {
  const char *sequences[] = {"DDD", "UUU", "RRR", "LRL"};
  const gesture_command_t commands[] = {GESTURE_CALIBRATE_SAVE, GESTURE_TOGGLE_BIND, GESTURE_OPEN_MENU, GESTURE_RESET_OSD};
  const uint32_t starts[] = {0U, UINT32_MAX - 800000U};
  for (uint32_t start : starts) {
    for (uint32_t i = 0; i < 4; i++) {
      begin_gestures(false, start);
      TEST_ASSERT_EQUAL(GESTURE_NONE, tap_gesture(sequences[i][0]));
      TEST_ASSERT_EQUAL(GESTURE_NONE, tap_gesture(sequences[i][1]));
      TEST_ASSERT_EQUAL(commands[i], tap_gesture(sequences[i][2]));
      TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture('C', 60000));
    }
  }
}

void test_gestures_menu_taps_and_hold_repeat() {
  const char directions[] = {'U', 'D', 'L', 'R'};
  const gesture_command_t commands[] = {GESTURE_MENU_UP, GESTURE_MENU_DOWN, GESTURE_MENU_LEFT, GESTURE_MENU_RIGHT};
  for (uint32_t i = 0; i < 4; i++) {
    begin_gestures(true, UINT32_MAX - 200000U);
    TEST_ASSERT_EQUAL(commands[i], tap_gesture(directions[i]));
    TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture('C', 100000));
    TEST_ASSERT_EQUAL(commands[i], hold_gesture(directions[i], 500000));
    TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture(directions[i], 124999));
    TEST_ASSERT_EQUAL(commands[i], hold_gesture(directions[i], 1));
    TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture('C', 60000));
  }
}

void test_gestures_reset_disabled_mode_changes_and_ambiguous_input() {
  for (unsigned interruption = 0; interruption < 3; interruption++) {
    begin_gestures(false);
    TEST_ASSERT_EQUAL(GESTURE_NONE, tap_gesture('R'));
    TEST_ASSERT_EQUAL(GESTURE_NONE, tap_gesture('R'));
    if (interruption == 0) {
      gestures_detect(gesture_sticks('C'), false, false, gesture_time);
    } else if (interruption == 1) {
      gestures_detect(gesture_sticks('C'), true, true, gesture_time);
      gestures_detect(gesture_sticks('C'), true, false, gesture_time);
    } else {
      TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture('?', 60000));
    }
    TEST_ASSERT_EQUAL(GESTURE_NONE, tap_gesture('R'));
    TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture('C', 700000));
    TEST_ASSERT_EQUAL(GESTURE_NONE, tap_gesture('R'));
    TEST_ASSERT_EQUAL(GESTURE_NONE, tap_gesture('R'));
    TEST_ASSERT_EQUAL(GESTURE_OPEN_MENU, tap_gesture('R'));
  }
}

void test_gestures_threshold_crossing_and_long_shortcut_press() {
  begin_gestures(false);
  for (unsigned i = 0; i < 3; i++) {
    TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture('?', 10000));
    TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture('U', 60000));
    TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture('?', 10000));
    TEST_ASSERT_EQUAL(i == 2 ? GESTURE_TOGGLE_BIND : GESTURE_NONE, hold_gesture('C', 60000));
  }
  begin_gestures(false);
  TEST_ASSERT_EQUAL(GESTURE_NONE, tap_gesture('D'));
  TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture('D', 500000));
  TEST_ASSERT_EQUAL(GESTURE_NONE, hold_gesture('C', 60000));
  TEST_ASSERT_EQUAL(GESTURE_NONE, tap_gesture('D'));
}
