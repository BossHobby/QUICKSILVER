#include "rx/stick_wizard.h"

#include "core/flash.h"
#include "core/looptime.h"
#include "core/profile.h"
#include "driver/time.h"
#include "flight/control.h"
#include "project.h"
#include "util/util.h"

#define OSD_TIMEOUT_MILLIS 5000
#define USB_TIMEOUT_MILLIS 20000
#define LIMITS profile.receiver.stick_calibration_limits

static uint32_t wizard_timeout = OSD_TIMEOUT_MILLIS;

//{max, min}
static float test_buffer[4][2] = {
    {-1, 1},
    {-1, 1},
    {-1, 1},
    {0, 1},
};

/*stick calibration wizard sequence notes
1. user selects start stick calibration sequence
2. From time 0s to time 5s - user is instructed to move sticks to all extents
3. From time 5s to time 10s - user is instructed to move sticks to all extents again so that they can be tested
4. If sticks test +/- 1% perfect - calibration passes and profile with scaling data saves.  wizard_phase enum will hold results that indicate STICK_WIZARD_CONFIRMED or STICK_WIZARD_TIMEOUT after the sequence.
*/
static void stick_wizard_capture() {
  for (uint8_t i = 0; i < 4; i++) {
    if (state.rx.axis[i] > LIMITS[i].max)
      LIMITS[i].max = state.rx.axis[i]; // record max value during calibration to array
    if (state.rx.axis[i] < LIMITS[i].min)
      LIMITS[i].min = state.rx.axis[i]; // record min value during calibration to array
  }
}

static void stick_wizard_reset_scale() {
  for (uint8_t i = 0; i < 3; i++) {
    LIMITS[i].min = -1;
    LIMITS[i].max = 1;
  }
  LIMITS[3].max = 1;
  LIMITS[3].min = 0;
}

static void stick_wizard_temp_scale() {
  for (uint8_t i = 0; i < 3; i++) {
    LIMITS[i].min = 1;
    LIMITS[i].max = -1;
  }
  LIMITS[3].max = 0;
  LIMITS[3].min = 1;
}

static void stick_wizard_reset_test_buffer() {
  for (uint8_t i = 0; i < 3; i++) {
    test_buffer[i][0] = -1;
    test_buffer[i][1] = 1;
  }
  test_buffer[3][0] = 0;
  test_buffer[3][1] = 1;
}

static bool stick_wizard_check_for_perfect_sticks() {
  // first scale the sticks
  state.rx.axis[0] = mapf(state.rx.axis[0], LIMITS[0].min, LIMITS[0].max, -1.f, 1.f);
  state.rx.axis[1] = mapf(state.rx.axis[1], LIMITS[1].min, LIMITS[1].max, -1.f, 1.f);
  state.rx.axis[2] = mapf(state.rx.axis[2], LIMITS[2].min, LIMITS[2].max, -1.f, 1.f);
  state.rx.axis[3] = mapf(state.rx.axis[3], LIMITS[3].min, LIMITS[3].max, 0.f, 1.f);

  // listen for the max stick values and update buffer
  for (uint8_t i = 0; i < 4; i++) {
    if (state.rx.axis[i] > test_buffer[i][0])
      test_buffer[i][0] = state.rx.axis[i]; // record max value during calibration to array
    if (state.rx.axis[i] < test_buffer[i][1])
      test_buffer[i][1] = state.rx.axis[i]; // record min value during calibration to array
  }

  // test the "4 corners key"
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (test_buffer[i][0] > 0.98f && test_buffer[i][0] < 1.02f)
      sum += 1; // test the max
    if (test_buffer[i][1] < -0.98f && test_buffer[i][1] > -1.02f)
      sum += 1; // test the min - throttle should fail
  }
  if (test_buffer[3][1] < .01 && test_buffer[3][1] > -.01) {
    sum += 1; // yes we tested throttle low twice because it doesnt go negative
  }

  if (sum == 8) {
    return true;
  }
  return false;
}

static void rx_stick_calibration_wizard() {
  extern int ledcommand;
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
    ledcommand = 1;

    flash_save();
    flash_load();
    looptime_reset();

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
  } else {
    state.rx.axis[0] = mapf(state.rx.axis[0], LIMITS[0].min, LIMITS[0].max, -1.f, 1.f);
    state.rx.axis[1] = mapf(state.rx.axis[1], LIMITS[1].min, LIMITS[1].max, -1.f, 1.f);
    state.rx.axis[2] = mapf(state.rx.axis[2], LIMITS[2].min, LIMITS[2].max, -1.f, 1.f);
    state.rx.axis[3] = mapf(state.rx.axis[3], LIMITS[3].min, LIMITS[3].max, 0.f, 1.f);
  }
}

void stick_wizard_start(bool from_usb) {
  state.stick_calibration_wizard = STICK_WIZARD_START;
  wizard_timeout = from_usb ? USB_TIMEOUT_MILLIS : OSD_TIMEOUT_MILLIS;
}
