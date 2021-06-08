#include "rx.h"

#include <math.h>

#include "control.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "filter.h"
#include "flash.h"
#include "profile.h"
#include "project.h"
#include "util.h"

extern profile_t profile;
//extern control_flags_t flags;

uint8_t rx_aux_on(aux_function_t function) {
  return state.aux[profile.receiver.aux[function]];
}

float rx_expo(float in, float exp) {
  if (exp > 1)
    exp = 1;
  if (exp < -1)
    exp = -1;
  float ans = in * in * in * exp + in * (1 - exp);
  limitf(&ans, 1.0);
  return ans;
}

float rx_smoothing_hz(rx_protocol_t proto) {
#ifdef RX_UNIFIED_SERIAL
  if (proto == RX_PROTOCOL_UNIFIED_SERIAL) {
    return RX_SMOOTHING_HZ[SERIAL_PROTO_MAP[bind_storage.unified.protocol]];
  }
#endif
  return RX_SMOOTHING_HZ[proto];
}

void rx_apply_expo(void) {
  vec3_t angle_expo = {
      .roll = 0,
      .pitch = 0,
      .yaw = 0,
  };
  vec3_t acro_expo = {
      .roll = 0,
      .pitch = 0,
      .yaw = 0,
  };

  if (profile.rate.mode == RATE_MODE_BETAFLIGHT) {
    angle_expo = profile.rate.betaflight.expo;
    acro_expo = profile.rate.betaflight.expo;
  } else {
    angle_expo = profile.rate.silverware.angle_expo;
    acro_expo = profile.rate.silverware.acro_expo;
  }

  vec3_t expo = {
      .roll = 0,
      .pitch = 0,
      .yaw = 0,
  };
  if (rx_aux_on(AUX_LEVELMODE)) {
    if (rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON)) {
      expo.axis[0] = angle_expo.roll;
      expo.axis[1] = acro_expo.pitch;
      expo.axis[2] = angle_expo.yaw;
    } else if (rx_aux_on(AUX_HORIZON)) {
      expo.axis[0] = acro_expo.roll;
      expo.axis[1] = acro_expo.pitch;
      expo.axis[2] = angle_expo.yaw;
    } else {
      expo.axis[0] = angle_expo.roll;
      expo.axis[1] = angle_expo.pitch;
      expo.axis[2] = angle_expo.yaw;
    }
  } else {
    expo.axis[0] = acro_expo.roll;
    expo.axis[1] = acro_expo.pitch;
    expo.axis[2] = acro_expo.yaw;
  }

  if (expo.roll > 0.01) {
    state.rx_filtered.axis[0] = rx_expo(state.rx.axis[0], expo.roll);
  } else {
    state.rx_filtered.axis[0] = state.rx.axis[0];
  }
  if (expo.pitch > 0.01) {
    state.rx_filtered.axis[1] = rx_expo(state.rx.axis[1], expo.pitch);
  } else {
    state.rx_filtered.axis[1] = state.rx.axis[1];
  }
  if (expo.yaw > 0.01) {
    state.rx_filtered.axis[2] = rx_expo(state.rx.axis[2], expo.yaw);
  } else {
    state.rx_filtered.axis[2] = state.rx.axis[2];
  }
}

void rx_apply_smoothing(void) {
  for (int i = 0; i < 4; ++i) {
#ifdef RX_SMOOTHING
    static float rx_temp[4] = {0, 0, 0, 0};
    lpf(&rx_temp[i], state.rx_filtered.axis[i], FILTERCALC(state.looptime, 1.0f / (float)rx_smoothing_hz(RX_PROTOCOL)));
    state.rx_filtered.axis[i] = rx_temp[i];
    limitf(&state.rx_filtered.axis[i], 1.0);
#else
    limitf(&state.rx_filtered.axis[i], 1.0);
#endif
  }
}

void rx_apply_deadband(void) {
  for (int i = 0; i < 3; ++i) {
    if (profile.rate.sticks_deadband > 0.0f) {
      if (fabsf(state.rx_filtered.axis[i]) <= profile.rate.sticks_deadband) {
        state.rx_filtered.axis[i] = 0.0f;
      } else {
        if (state.rx_filtered.axis[i] >= 0) {
          state.rx_filtered.axis[i] = mapf(state.rx_filtered.axis[i], profile.rate.sticks_deadband, 1, 0, 1);
        } else {
          state.rx_filtered.axis[i] = mapf(state.rx_filtered.axis[i], -profile.rate.sticks_deadband, -1, 0, -1);
        }
      }
    }
  }
}

void rx_precalc() {
  state.rx_filtered.throttle = constrainf(state.rx.throttle, 0.f, 1.f); //constrain throttle min and max and copy into next bucket
  rx_apply_expo();                                                      //this also constrains and copies the rest of the sticks into rx_filtered.axis[i]
  rx_apply_smoothing();
  rx_apply_deadband();
}

void rx_capture_stick_range(void) {
  for (uint8_t i = 0; i < 4; i++) {
    if (state.rx.axis[i] > profile.receiver.stick_calibration_limits[i].max)
      profile.receiver.stick_calibration_limits[i].max = state.rx.axis[i]; //record max value during calibration to array
    if (state.rx.axis[i] < profile.receiver.stick_calibration_limits[i].min)
      profile.receiver.stick_calibration_limits[i].min = state.rx.axis[i]; //record min value during calibration to array
  }
}

void rx_reset_stick_calibration_scale(void) {
  for (uint8_t i = 0; i < 3; i++) {
    profile.receiver.stick_calibration_limits[i].min = -1;
    profile.receiver.stick_calibration_limits[i].max = 1;
  }
  profile.receiver.stick_calibration_limits[3].max = 1;
  profile.receiver.stick_calibration_limits[3].min = 0;
}

void rx_apply_temp_calibration_scale(void) {
  for (uint8_t i = 0; i < 3; i++) {
    profile.receiver.stick_calibration_limits[i].min = 1;
    profile.receiver.stick_calibration_limits[i].max = -1;
  }
  profile.receiver.stick_calibration_limits[3].max = 0;
  profile.receiver.stick_calibration_limits[3].min = 1;
}

static float stick_calibration_test_buffer[4][2] = {{-1, 1}, {-1, 1}, {-1, 1}, {0, 1}}; //{max, min}
void reset_stick_calibration_test_buffer(void) {
  for (uint8_t i = 0; i < 3; i++) {
    stick_calibration_test_buffer[i][0] = -1;
    stick_calibration_test_buffer[i][1] = 1;
  }
  stick_calibration_test_buffer[3][0] = 0;
  stick_calibration_test_buffer[3][1] = 1;
}

uint8_t check_for_perfect_sticks(void) {
  //first scale the sticks
  state.rx.axis[0] = mapf(state.rx.axis[0], profile.receiver.stick_calibration_limits[0].min, profile.receiver.stick_calibration_limits[0].max, -1.f, 1.f);
  state.rx.axis[1] = mapf(state.rx.axis[1], profile.receiver.stick_calibration_limits[1].min, profile.receiver.stick_calibration_limits[1].max, -1.f, 1.f);
  state.rx.axis[2] = mapf(state.rx.axis[2], profile.receiver.stick_calibration_limits[2].min, profile.receiver.stick_calibration_limits[2].max, -1.f, 1.f);
  state.rx.axis[3] = mapf(state.rx.axis[3], profile.receiver.stick_calibration_limits[3].min, profile.receiver.stick_calibration_limits[3].max, 0.f, 1.f);
  //listen for the max stick values and update buffer
  for (uint8_t i = 0; i < 4; i++) {
    if (state.rx.axis[i] > stick_calibration_test_buffer[i][0])
      stick_calibration_test_buffer[i][0] = state.rx.axis[i]; //record max value during calibration to array
    if (state.rx.axis[i] < stick_calibration_test_buffer[i][1])
      stick_calibration_test_buffer[i][1] = state.rx.axis[i]; //record min value during calibration to array
  }
  //test the "4 corners key"
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (stick_calibration_test_buffer[i][0] > 0.98f && stick_calibration_test_buffer[i][0] < 1.02f)
      sum += 1; //test the max
    if (stick_calibration_test_buffer[i][1] < -0.98f && stick_calibration_test_buffer[i][1] > -1.02f)
      sum += 1; //test the min - throttle should fail
  }
  if (stick_calibration_test_buffer[3][1] < .01 && stick_calibration_test_buffer[3][1] > -.01)
    sum += 1; // yes we tested throttle low twice because it doesnt go negative
  if (sum == 8)
    return 1;
  //else
  return 0;
}

void rx_stick_calibration_wizard(void) {
  extern int ledcommand;
  static uint8_t sequence_is_running = 0;
  static uint32_t first_timestamp;
  //get a timestamp and set the initial conditions
  if (!sequence_is_running) {                        //calibration has just been called
    first_timestamp = gettime();                     //so we flag the time
    flags.gestures_disabled = 1;                     //and disable gestures
    sequence_is_running = 1;                         //just once
    rx_apply_temp_calibration_scale();               //and shove temp values into profile that are the inverse of expected values from sticks
    reset_stick_calibration_test_buffer();           //make sure we test with a fresh comparison buffer
  }
  //sequence the phase of the wizard in automatic 5 second intervals
  if (state.stick_calibration_wizard == CALIBRATION_CONFIRMED) {
    //leave it alone
  } else {
    uint32_t time_now = gettime();
    if ((time_now - first_timestamp > 5e6) && (time_now - first_timestamp < 10e6))
      state.stick_calibration_wizard = WAIT_FOR_CONFIRM;
    if (time_now - first_timestamp > 10e6)
      state.stick_calibration_wizard = TIMEOUT;
  }
  //take appropriate action based on the wizard phase
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
    //or here?
    break;
  case CALIBRATION_FAILED:
    //here too
    break;
  }
}

void rx_apply_stick_calibration_scale(void) {
  if (state.stick_calibration_wizard == CAPTURE_STICKS || state.stick_calibration_wizard == WAIT_FOR_CONFIRM || state.stick_calibration_wizard == CALIBRATION_CONFIRMED || state.stick_calibration_wizard == TIMEOUT) {
    rx_stick_calibration_wizard();
  } else {
    state.rx.axis[0] = mapf(state.rx.axis[0], profile.receiver.stick_calibration_limits[0].min, profile.receiver.stick_calibration_limits[0].max, -1.f, 1.f);
    state.rx.axis[1] = mapf(state.rx.axis[1], profile.receiver.stick_calibration_limits[1].min, profile.receiver.stick_calibration_limits[1].max, -1.f, 1.f);
    state.rx.axis[2] = mapf(state.rx.axis[2], profile.receiver.stick_calibration_limits[2].min, profile.receiver.stick_calibration_limits[2].max, -1.f, 1.f);
    state.rx.axis[3] = mapf(state.rx.axis[3], profile.receiver.stick_calibration_limits[3].min, profile.receiver.stick_calibration_limits[3].max, 0.f, 1.f);
  }
}

void request_stick_calibration_wizard(void) {
  state.stick_calibration_wizard = CAPTURE_STICKS;
}

/*stick calibration wizard sequence notes
1. user selects start stick calibration sequence
2. From time 0s to time 5s - user is instructed to move sticks to all extents
3. From time 5s to time 10s - user is instructed to move sticks to all extents again so that they can be tested
4. If sticks test +/- 1% perfect - calibration passes and profile with scaling data saves.  wizard_phase enum will hold results that indicate CALIBRATION_CONFIRMED or TIMEOUT after the sequence.
*/
