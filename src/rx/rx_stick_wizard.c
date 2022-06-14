#include "rx_stick_wizard.h"

#include "drv_time.h"
#include "flash.h"
#include "flight/control.h"
#include "profile.h"
#include "project.h"
#include "util/util.h"

//{max, min}
static float stick_calibration_test_buffer[4][2] = {
    {-1, 1},
    {-1, 1},
    {-1, 1},
    {0, 1},
};

/*stick calibration wizard sequence notes
1. user selects start stick calibration sequence
2. From time 0s to time 5s - user is instructed to move sticks to all extents
3. From time 5s to time 10s - user is instructed to move sticks to all extents again so that they can be tested
4. If sticks test +/- 1% perfect - calibration passes and profile with scaling data saves.  wizard_phase enum will hold results that indicate CALIBRATION_CONFIRMED or TIMEOUT after the sequence.
*/
static void rx_capture_stick_range() {
  for (uint8_t i = 0; i < 4; i++) {
    if (state.rx.axis[i] > profile.receiver.stick_calibration_limits[i].max)
      profile.receiver.stick_calibration_limits[i].max = state.rx.axis[i]; // record max value during calibration to array
    if (state.rx.axis[i] < profile.receiver.stick_calibration_limits[i].min)
      profile.receiver.stick_calibration_limits[i].min = state.rx.axis[i]; // record min value during calibration to array
  }
}

static void rx_reset_stick_calibration_scale() {
  for (uint8_t i = 0; i < 3; i++) {
    profile.receiver.stick_calibration_limits[i].min = -1;
    profile.receiver.stick_calibration_limits[i].max = 1;
  }
  profile.receiver.stick_calibration_limits[3].max = 1;
  profile.receiver.stick_calibration_limits[3].min = 0;
}

static void rx_apply_temp_calibration_scale() {
  for (uint8_t i = 0; i < 3; i++) {
    profile.receiver.stick_calibration_limits[i].min = 1;
    profile.receiver.stick_calibration_limits[i].max = -1;
  }
  profile.receiver.stick_calibration_limits[3].max = 0;
  profile.receiver.stick_calibration_limits[3].min = 1;
}

static void reset_stick_calibration_test_buffer() {
  for (uint8_t i = 0; i < 3; i++) {
    stick_calibration_test_buffer[i][0] = -1;
    stick_calibration_test_buffer[i][1] = 1;
  }
  stick_calibration_test_buffer[3][0] = 0;
  stick_calibration_test_buffer[3][1] = 1;
}

static uint8_t check_for_perfect_sticks() {
  // first scale the sticks
  state.rx.axis[0] = mapf(state.rx.axis[0], profile.receiver.stick_calibration_limits[0].min, profile.receiver.stick_calibration_limits[0].max, -1.f, 1.f);
  state.rx.axis[1] = mapf(state.rx.axis[1], profile.receiver.stick_calibration_limits[1].min, profile.receiver.stick_calibration_limits[1].max, -1.f, 1.f);
  state.rx.axis[2] = mapf(state.rx.axis[2], profile.receiver.stick_calibration_limits[2].min, profile.receiver.stick_calibration_limits[2].max, -1.f, 1.f);
  state.rx.axis[3] = mapf(state.rx.axis[3], profile.receiver.stick_calibration_limits[3].min, profile.receiver.stick_calibration_limits[3].max, 0.f, 1.f);
  // listen for the max stick values and update buffer
  for (uint8_t i = 0; i < 4; i++) {
    if (state.rx.axis[i] > stick_calibration_test_buffer[i][0])
      stick_calibration_test_buffer[i][0] = state.rx.axis[i]; // record max value during calibration to array
    if (state.rx.axis[i] < stick_calibration_test_buffer[i][1])
      stick_calibration_test_buffer[i][1] = state.rx.axis[i]; // record min value during calibration to array
  }
  // test the "4 corners key"
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (stick_calibration_test_buffer[i][0] > 0.98f && stick_calibration_test_buffer[i][0] < 1.02f)
      sum += 1; // test the max
    if (stick_calibration_test_buffer[i][1] < -0.98f && stick_calibration_test_buffer[i][1] > -1.02f)
      sum += 1; // test the min - throttle should fail
  }
  if (stick_calibration_test_buffer[3][1] < .01 && stick_calibration_test_buffer[3][1] > -.01)
    sum += 1; // yes we tested throttle low twice because it doesnt go negative
  if (sum == 8)
    return 1;
  // else
  return 0;
}

static void rx_stick_calibration_wizard() {
  extern int ledcommand;
  static uint8_t sequence_is_running = 0;
  static uint32_t first_timestamp;
  // get a timestamp and set the initial conditions
  if (!sequence_is_running) {              // calibration has just been called
    first_timestamp = time_micros();       // so we flag the time
    flags.gestures_disabled = 1;           // and disable gestures
    sequence_is_running = 1;               // just once
    rx_apply_temp_calibration_scale();     // and shove temp values into profile that are the inverse of expected values from sticks
    reset_stick_calibration_test_buffer(); // make sure we test with a fresh comparison buffer
  }
  // sequence the phase of the wizard in automatic 5 second intervals
  if (state.stick_calibration_wizard == CALIBRATION_CONFIRMED) {
    // leave it alone
  } else {
    uint32_t time_now = time_micros();
    if ((time_now - first_timestamp > 5e6) && (time_now - first_timestamp < 10e6))
      state.stick_calibration_wizard = WAIT_FOR_CONFIRM;
    if (time_now - first_timestamp > 10e6)
      state.stick_calibration_wizard = TIMEOUT;
  }
  // take appropriate action based on the wizard phase
  switch (state.stick_calibration_wizard) {
  case INACTIVE:
    // how the fuck did we get here?
    break;
  case CAPTURE_STICKS:
    rx_capture_stick_range();
    break;
  case WAIT_FOR_CONFIRM:
    if (check_for_perfect_sticks()) {
      state.stick_calibration_wizard = CALIBRATION_CONFIRMED;
    }
    break;
  case CALIBRATION_CONFIRMED:
    ledcommand = 1;
    flash_save();
    flash_load();
    reset_looptime();
    sequence_is_running = 0;
    flags.gestures_disabled = 0;
    state.stick_calibration_wizard = CALIBRATION_SUCCESS;
    break;
  case TIMEOUT:
    rx_reset_stick_calibration_scale();
    sequence_is_running = 0;
    flags.gestures_disabled = 0;
    state.stick_calibration_wizard = CALIBRATION_FAILED;
    break;
  case CALIBRATION_SUCCESS:
    // or here?
    break;
  case CALIBRATION_FAILED:
    // here too
    break;
  }
}

void rx_apply_stick_calibration_scale() {
  if (state.stick_calibration_wizard == CAPTURE_STICKS || state.stick_calibration_wizard == WAIT_FOR_CONFIRM || state.stick_calibration_wizard == CALIBRATION_CONFIRMED || state.stick_calibration_wizard == TIMEOUT) {
    rx_stick_calibration_wizard();
  } else {
    state.rx.axis[0] = mapf(state.rx.axis[0], profile.receiver.stick_calibration_limits[0].min, profile.receiver.stick_calibration_limits[0].max, -1.f, 1.f);
    state.rx.axis[1] = mapf(state.rx.axis[1], profile.receiver.stick_calibration_limits[1].min, profile.receiver.stick_calibration_limits[1].max, -1.f, 1.f);
    state.rx.axis[2] = mapf(state.rx.axis[2], profile.receiver.stick_calibration_limits[2].min, profile.receiver.stick_calibration_limits[2].max, -1.f, 1.f);
    state.rx.axis[3] = mapf(state.rx.axis[3], profile.receiver.stick_calibration_limits[3].min, profile.receiver.stick_calibration_limits[3].max, 0.f, 1.f);
  }
}

void request_stick_calibration_wizard() {
  state.stick_calibration_wizard = CAPTURE_STICKS;
}
