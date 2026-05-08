#include "rx/stick_wizard.h"

#include "control/control.h"
#include "core/flash.h"
#include "core/looptime.h"
#include "core/profile.h"
#include "core/project.h"
#include "core/scheduler.h"
#include "driver/time.h"
#include "io/led.h"
#include "util/util.h"

#define OSD_TIMEOUT_MILLIS 5000
#define USB_TIMEOUT_MILLIS 20000
static uint32_t wizard_timeout = OSD_TIMEOUT_MILLIS;

//{max, min}
static float test_buffer[RX_ROLE_MAX][2];

static float role_raw_value(uint8_t role) {
  const rx_role_map_t *map = &profile.receiver.role_map[role];
  if (map->channel >= RX_CHANNEL_MAX) {
    return 0.0f;
  }
  return (float)state.rx_channels[map->channel] / (float)AUX_VALUE_MAX * 2.0f - 1.0f;
}

/*stick calibration wizard sequence notes
1. user selects start stick calibration sequence
2. From time 0s to time 5s - user is instructed to move sticks to all extents
3. From time 5s to time 10s - user is instructed to move sticks to all extents again so that they can be tested
4. If sticks test +/- 1% perfect - calibration passes and profile with scaling data saves.  wizard_phase enum will hold results that indicate STICK_WIZARD_CONFIRMED or STICK_WIZARD_TIMEOUT after the sequence.
*/
static void stick_wizard_capture() {
  for (uint8_t i = 0; i < RX_ROLE_MAX; i++) {
    const float value = role_raw_value(i);
    if (value > profile.receiver.role_map[i].max)
      profile.receiver.role_map[i].max = value;
    if (value < profile.receiver.role_map[i].min)
      profile.receiver.role_map[i].min = value;
  }
}

static void stick_wizard_reset_scale() {
  for (uint8_t i = 0; i < RX_ROLE_MAX; i++) {
    profile.receiver.role_map[i].min = -1.0f;
    profile.receiver.role_map[i].center = 0.0f;
    profile.receiver.role_map[i].max = 1.0f;
  }
}

static void stick_wizard_temp_scale() {
  for (uint8_t i = 0; i < RX_ROLE_MAX; i++) {
    profile.receiver.role_map[i].min = 1.0f;
    profile.receiver.role_map[i].center = role_raw_value(i);
    profile.receiver.role_map[i].max = -1.0f;
  }
}

static void stick_wizard_reset_test_buffer() {
  for (uint8_t i = 0; i < RX_ROLE_MAX; i++) {
    test_buffer[i][0] = -1.0f;
    test_buffer[i][1] = 1.0f;
  }
}

static bool stick_wizard_check_for_perfect_sticks() {
  // listen for the max stick values and update buffer
  for (uint8_t i = 0; i < RX_ROLE_MAX; i++) {
    const float value = role_raw_value(i);
    if (value > test_buffer[i][0])
      test_buffer[i][0] = value;
    if (value < test_buffer[i][1])
      test_buffer[i][1] = value;
  }

  // test the "4 corners key"
  uint8_t sum = 0;
  for (uint8_t i = 0; i < RX_ROLE_MAX; i++) {
    if (test_buffer[i][0] > 0.8f)
      sum += 1; // test the max
    if (test_buffer[i][1] < -0.8f)
      sum += 1; // test the min
  }

  if (sum == RX_ROLE_MAX * 2) {
    return true;
  }
  return false;
}

static void rx_stick_calibration_wizard() {
  static uint32_t wizard_start_millis;

  // take appropriate action based on the wizard phase
  switch (state.stick_calibration_wizard) {
  case STICK_WIZARD_INACTIVE:
  case STICK_WIZARD_SUCCESS:
  case STICK_WIZARD_FAILED:
    // idle. do nothing
    break;

  case STICK_WIZARD_START:
    wizard_start_millis = time_millis();
    flags.gestures_disabled = 1;

    stick_wizard_temp_scale();
    stick_wizard_reset_test_buffer();

    state.stick_calibration_wizard = STICK_WIZARD_CAPTURE_STICKS;
    break;

  case STICK_WIZARD_CAPTURE_STICKS:
    stick_wizard_capture();

    if (time_millis() - wizard_start_millis > wizard_timeout) {
      wizard_start_millis = time_millis();
      state.stick_calibration_wizard = STICK_WIZARD_WAIT_FOR_CONFIRM;
    }

    break;

  case STICK_WIZARD_WAIT_FOR_CONFIRM:
    if (stick_wizard_check_for_perfect_sticks()) {
      state.stick_calibration_wizard = STICK_WIZARD_CONFIRMED;
    }
    if (time_millis() - wizard_start_millis > wizard_timeout) {
      wizard_start_millis = time_millis();
      state.stick_calibration_wizard = STICK_WIZARD_TIMEOUT;
    }
    break;

  case STICK_WIZARD_CONFIRMED:
    led_flash();

    flash_save();
    flash_load();
    task_reset_runtime();

    flags.gestures_disabled = 0;
    state.stick_calibration_wizard = STICK_WIZARD_SUCCESS;
    break;

  case STICK_WIZARD_TIMEOUT:
    stick_wizard_reset_scale();

    flags.gestures_disabled = 0;
    state.stick_calibration_wizard = STICK_WIZARD_FAILED;
    break;
  }
}

void rx_apply_stick_scale() {
  if (state.stick_calibration_wizard >= STICK_WIZARD_START) {
    rx_stick_calibration_wizard();
  }
}

void stick_wizard_start(bool from_usb) {
  state.stick_calibration_wizard = STICK_WIZARD_START;
  wizard_timeout = from_usb ? USB_TIMEOUT_MILLIS : OSD_TIMEOUT_MILLIS;
}
