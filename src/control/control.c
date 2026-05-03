#include "control/control.h"

#include <stdint.h>

#include "control/gestures.h"
#include "control/imu.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "util/cbor_helper.h"

FAST_RAM control_flags_t flags = {
    .arm_state = 0,
    .arming_disabled_flags = ARMING_DISABLED_ARM_SWITCH,

    .in_air = 0,
    .on_ground = 1,

    .failsafe = 1,
    .lowbatt = 1,

    .rx_mode = RXMODE_BIND,
    .rx_ready = 0,

    .controls_override = 0,
    .motortest_override = 0,

    .usb_active = 0,
};

FAST_RAM control_state_t state = {
    .failsafe_time_ms = 0,

    .aux = {0},

    .loop_counter = 0,

    .vbat = 0.0,
    .vbat_filtered = 0.0,
    .vbat_sag_filtered = 0.0,
    .vbat_compensated = 4.2,

    .ibat = 0.0,
    .ibat_filtered = 0.0,
    .ibat_sag_filtered = 0.0,
    .ibat_drawn = 0.0,

    .stick_calibration_wizard = STICK_WIZARD_INACTIVE,

    .rx_filter_hz = 0.0f,

    .rx_rssi = 0,
    .rx_status = 0,

    .lipo_cell_count = 1.0,

    .GEstG = {
        .axis = {0, 0, ACC_1G},
    },
};

#define MEMBER CBOR_ENCODE_MEMBER
#define STR_MEMBER CBOR_ENCODE_STR_MEMBER
#define ARRAY_MEMBER CBOR_ENCODE_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_ENCODE_STR_ARRAY_MEMBER
#define START_STRUCT CBOR_START_STRUCT_ENCODER
#define END_STRUCT CBOR_END_STRUCT_ENCODER

GPS_COORD_MEMBERS
CBOR_START_STRUCT_ENCODER(control_state_t)
STATE_MEMBERS
CBOR_END_STRUCT_ENCODER()

#undef START_STRUCT
#undef END_STRUCT

#undef MEMBER
#undef STR_MEMBER
#undef ARRAY_MEMBER
#undef STR_ARRAY_MEMBER

void control_update_arming() {
  static bool checked_prearm = false;
  static uint32_t arming_disabled_latch = ARMING_DISABLED_ARM_SWITCH;
  bool failsafe_lock = false;

  flags.arm_request = rx_aux_on(AUX_ARMING);
  flags.arming_disabled_flags = ARMING_DISABLED_NONE;

  if (flags.failsafe) {
    const uint32_t now_ms = time_millis();

    // failsafe first occurred, record time
    if (state.failsafe_time_ms == 0) {
      state.failsafe_time_ms = now_ms;
    }

    if ((now_ms - state.failsafe_time_ms) > FAILSAFE_LOCK_TIME_MS) {
      failsafe_lock = true;
    }
  } else {
    state.failsafe_time_ms = 0;
  }

  if (failsafe_lock) {
    flags.arming_disabled_flags |= ARMING_DISABLED_FAILSAFE;
  }

  if (flags.usb_active) {
    flags.arming_disabled_flags |= ARMING_DISABLED_USB;
  }

  if (flags.arm_request && (failsafe_lock || flags.usb_active)) {
    arming_disabled_latch |= ARMING_DISABLED_ARM_SWITCH;
  }

  if (flags.arm_request && !failsafe_lock && !flags.usb_active) {
    // CONDITION: AUX_ARMING is high

    if (!checked_prearm &&
        rx_aux_on(AUX_PREARM) &&
        state.rx_filtered.throttle <= THROTTLE_SAFETY &&
        arming_disabled_latch == ARMING_DISABLED_NONE) {
      // CONDITION:  we have not checked prearm this arm cycle AND AUX_PREARM is high AND throttle is zeroed
      flags.arm_state = 1;

      if (!flags.turtle_ready) {
        motor_set_direction(MOTOR_FORWARD);
      }
    } else if (!flags.arm_state) {
      if (state.rx_filtered.throttle > THROTTLE_SAFETY) {
        // throw up throttle safety if we crossed the threshold
        arming_disabled_latch |= ARMING_DISABLED_THROTTLE;
      } else {
        // throw up arming safety if we didnt manage to arm
        arming_disabled_latch |= ARMING_DISABLED_ARM_SWITCH;
      }
    }

    checked_prearm = true;
  } else {
    flags.arm_state = 0;
    checked_prearm = false;

    if (flags.rx_ready == 1) {
      // rx is bound and has been disarmed so clear binding while armed flag
      arming_disabled_latch &= ~ARMING_DISABLED_ARM_SWITCH;
    }

    // clear the throttle safety flag
    arming_disabled_latch &= ~ARMING_DISABLED_THROTTLE;
  }

  if (flags.turtle && !flags.turtle_ready) {
    arming_disabled_latch |= ARMING_DISABLED_ARM_SWITCH;
  }

  flags.arming_disabled_flags |= arming_disabled_latch;

  if (flags.arming_disabled_flags != ARMING_DISABLED_NONE) {
    // disarm the quad by setting armed state variable to zero
    flags.arm_state = 0;
  }
}
