#include "rx.h"

#include <math.h>

#include "drv_serial.h"
#include "drv_time.h"
#include "flash.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "profile.h"
#include "project.h"
#include "util/util.h"

extern bool rx_check();
extern profile_t profile;

uint8_t failsafe_siglost = 0;

uint32_t last_frame_time_us = 0;
static uint32_t frame_missed_time_us = 0;
static uint32_t frames_per_second = 0;
static uint32_t frames_missed = 0;
static uint32_t frames_received = 0;

// Select filter cut, Formula is [(1/rx framerate)/2] * 0.9
// 0 will trigger selection via rx_smoothing_cutoff
static const uint16_t RX_SMOOTHING_HZ[RX_PROTOCOL_MAX] = {
    0,   // RX_PROTOCOL_INVALID, wont happen
    0,   // RX_PROTOCOL_UNIFIED_SERIAL, will autodetect following
    25,  // RX_PROTOCOL_SBUS,
    67,  // RX_PROTOCOL_CRSF,
    50,  // RX_PROTOCOL_IBUS, check these
    50,  // RX_PROTOCOL_FPORT, check these
    40,  // RX_PROTOCOL_DSM,
    90,  // RX_PROTOCOL_NRF24_BAYANG_TELEMETRY,
    90,  // RX_PROTOCOL_BAYANG_PROTOCOL_BLE_BEACON,
    90,  // RX_PROTOCOL_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND,
    50,  // RX_PROTOCOL_FRSKY_D8,
    50,  // RX_PROTOCOL_FRSKY_D16,
    225, // RX_PROTOCOL_FRSKY_REDPINE,
    0,   // RX_PROTOCOL_EXPRESS_LRS
};

#ifdef RX_UNIFIED_SERIAL
static const uint16_t SERIAL_PROTO_MAP[] = {
    RX_PROTOCOL_INVALID, // RX_SERIAL_PROTOCOL_INVALID
    RX_PROTOCOL_DSM,     // RX_SERIAL_PROTOCOL_DSM
    RX_PROTOCOL_SBUS,    // RX_SERIAL_PROTOCOL_SBUS
    RX_PROTOCOL_IBUS,    // RX_SERIAL_PROTOCOL_IBUS
    RX_PROTOCOL_FPORT,   // RX_SERIAL_PROTOCOL_FPORT
    RX_PROTOCOL_CRSF,    // RX_SERIAL_PROTOCOL_CRSF
    RX_PROTOCOL_REDPINE, // RX_SERIAL_PROTOCOL_REDPINE
    // No need to filter differently for inverted.
    RX_PROTOCOL_SBUS,    // RX_SERIAL_PROTOCOL_SBUS_INVERTED
    RX_PROTOCOL_FPORT,   // RX_SERIAL_PROTOCOL_FPORT_INVERTED
    RX_PROTOCOL_REDPINE, // RX_SERIAL_PROTOCOL_REDPINE_INVERTED
};

uint16_t rx_smoothing_cutoff() {
  const uint16_t serial_proto = SERIAL_PROTO_MAP[bind_storage.unified.protocol];
  if (serial_proto == RX_PROTOCOL_CRSF) {
    return rx_serial_crsf_smoothing_cutoff();
  }
  if (serial_proto == RX_PROTOCOL_DSM) {
    return rx_serial_dsm_smoothing_cutoff();
  }
  return RX_SMOOTHING_HZ[serial_proto];
}
#else
__weak uint16_t rx_smoothing_cutoff() {
  // default implementation, will be overwritten by non __weak functions
  return 0;
}
#endif

uint8_t rx_aux_on(aux_function_t function) {
  return state.aux[profile.receiver.aux[function]];
}

float rx_smoothing_hz(rx_protocol_t proto) {
  uint16_t cutoff = RX_SMOOTHING_HZ[proto];
  if (cutoff == 0) {
    cutoff = rx_smoothing_cutoff();
  }
  return cutoff;
}

void rx_lqi_lost_packet() {
  frames_missed++;

  if (frame_missed_time_us == 0) {
    frame_missed_time_us = time_micros();
  }

  if (time_micros() - frame_missed_time_us > FAILSAFETIME) {
    failsafe_siglost = 1;
  }
}

void rx_lqi_got_packet() {
  frames_received++;
  last_frame_time_us = time_micros();

  frame_missed_time_us = 0;
  failsafe_siglost = 0;
}

void rx_lqi_update() {
  const uint32_t time = time_micros();

  // link quality & rssi
  static uint32_t last_time = 0;
  if (time - last_time < 1000000) {
    // we only run once per second
    return;
  }

  frames_per_second = frames_received;

  frames_received = 0;
  frames_missed = 0;

  last_time = time;
}

void rx_lqi_update_from_fps(float expected_fps) {
  state.rx_rssi = frames_per_second / expected_fps;
  state.rx_rssi = state.rx_rssi * state.rx_rssi * state.rx_rssi * LQ_EXPO + state.rx_rssi * (1 - LQ_EXPO);
  state.rx_rssi *= 100.0f;

  state.rx_rssi = constrainf(state.rx_rssi, 0.f, 100.f);
}

void rx_lqi_update_direct(float rssi) {
  state.rx_rssi = constrainf(rssi, 0.f, 100.f);
}

static void rx_apply_smoothing() {
  for (int i = 0; i < 4; ++i) {
    if (i == 3) {
      // throttle is 0 - 1.0
      state.rx.axis[i] = constrainf(state.rx.axis[i], 0.0, 1.0);
    } else {
      // other channels are -1.0 - 1.0
      state.rx.axis[i] = constrainf(state.rx.axis[i], -1.0, 1.0);
    }
#ifdef RX_SMOOTHING
    lpf(&state.rx_filtered.axis[i], state.rx.axis[i], FILTERCALC(state.looptime, 1.0f / (float)rx_smoothing_hz(RX_PROTOCOL)));
#endif
    if (i == 3) {
      // throttle is 0 - 1.0
      state.rx_filtered.axis[i] = constrainf(state.rx_filtered.axis[i], 0.0, 1.0);
    } else {
      // other channels are -1.0 - 1.0
      state.rx_filtered.axis[i] = constrainf(state.rx_filtered.axis[i], -1.0, 1.0);
    }
  }
}

static float rx_apply_deadband(float val) {
  if (profile.rate.sticks_deadband <= 0.0f) {
    return val;
  }

  if (fabsf(val) <= profile.rate.sticks_deadband) {
    return 0.0f;
  }

  if (val >= 0) {
    return mapf(val, profile.rate.sticks_deadband, 1, 0, 1);
  } else {
    return mapf(val, -profile.rate.sticks_deadband, -1, 0, -1);
  }
}

void rx_update() {
  if (rx_check()) {
    rx_apply_stick_calibration_scale();

    state.rx.roll = rx_apply_deadband(state.rx.roll);
    state.rx.pitch = rx_apply_deadband(state.rx.pitch);
    state.rx.yaw = rx_apply_deadband(state.rx.yaw);
  }

  rx_apply_smoothing();
}

void rx_capture_stick_range() {
  for (uint8_t i = 0; i < 4; i++) {
    if (state.rx.axis[i] > profile.receiver.stick_calibration_limits[i].max)
      profile.receiver.stick_calibration_limits[i].max = state.rx.axis[i]; // record max value during calibration to array
    if (state.rx.axis[i] < profile.receiver.stick_calibration_limits[i].min)
      profile.receiver.stick_calibration_limits[i].min = state.rx.axis[i]; // record min value during calibration to array
  }
}

void rx_reset_stick_calibration_scale() {
  for (uint8_t i = 0; i < 3; i++) {
    profile.receiver.stick_calibration_limits[i].min = -1;
    profile.receiver.stick_calibration_limits[i].max = 1;
  }
  profile.receiver.stick_calibration_limits[3].max = 1;
  profile.receiver.stick_calibration_limits[3].min = 0;
}

void rx_apply_temp_calibration_scale() {
  for (uint8_t i = 0; i < 3; i++) {
    profile.receiver.stick_calibration_limits[i].min = 1;
    profile.receiver.stick_calibration_limits[i].max = -1;
  }
  profile.receiver.stick_calibration_limits[3].max = 0;
  profile.receiver.stick_calibration_limits[3].min = 1;
}

static float stick_calibration_test_buffer[4][2] = {{-1, 1}, {-1, 1}, {-1, 1}, {0, 1}}; //{max, min}
void reset_stick_calibration_test_buffer() {
  for (uint8_t i = 0; i < 3; i++) {
    stick_calibration_test_buffer[i][0] = -1;
    stick_calibration_test_buffer[i][1] = 1;
  }
  stick_calibration_test_buffer[3][0] = 0;
  stick_calibration_test_buffer[3][1] = 1;
}

uint8_t check_for_perfect_sticks() {
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

void rx_stick_calibration_wizard() {
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

/*stick calibration wizard sequence notes
1. user selects start stick calibration sequence
2. From time 0s to time 5s - user is instructed to move sticks to all extents
3. From time 5s to time 10s - user is instructed to move sticks to all extents again so that they can be tested
4. If sticks test +/- 1% perfect - calibration passes and profile with scaling data saves.  wizard_phase enum will hold results that indicate CALIBRATION_CONFIRMED or TIMEOUT after the sequence.
*/
