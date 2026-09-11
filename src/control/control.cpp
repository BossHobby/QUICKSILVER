#include "control/control.h"

#include <math.h>
#include <stdint.h>

#include "control/gestures.h"
#include "control/imu.h"
#include "control/pid.h"
#include "control/sixaxis.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "util/cbor_helper.h"

FAST_RAM control_flags_t flags = {
    .arm_state = 0,

    .in_air = 0,
    .on_ground = 1,

    .failsafe = 1,
    .failsafe_outputs_blocked = 1,
    .failsafe_signal_lost = 1,
    .lowbatt = 1,

    .rx_mode = RXMODE_BIND,
    .rx_ready = 0,

    .controls_override = 0,
    .motortest_override = 0,

    .usb_active = 0,
    .arming_disabled_flags = ARMING_DISABLED_ARM_SWITCH,
};

FAST_RAM control_state_t state = {
    .loop_counter = 0,
    .failsafe_time_ms = 0,
    .failsafe_phase = FAILSAFE_PHASE_IDLE,
    .lipo_cell_count = 1,
    .vbat_compensated = 4.2f,
    .stick_calibration_wizard = STICK_WIZARD_INACTIVE,
    .GEstG = {.axis = {0, 0, ACC_1G}},
};

#define MEMBER CBOR_ENCODE_MEMBER
#define STR_MEMBER CBOR_ENCODE_STR_MEMBER
#define ARRAY_MEMBER CBOR_ENCODE_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_ENCODE_STR_ARRAY_MEMBER
#define START_STRUCT CBOR_START_STRUCT_ENCODER
#define END_STRUCT CBOR_END_STRUCT_ENCODER

GPS_COORD_MEMBERS
STATE_MEMBERS

#undef START_STRUCT
#undef END_STRUCT

#undef MEMBER
#undef STR_MEMBER
#undef ARRAY_MEMBER
#undef STR_ARRAY_MEMBER

static uint32_t failsafe_phase_start_us = 0;
static bool failsafe_rearm_allows_prearm_hold = false;

void control_filter_update(bool reset) {
  sixaxis_filter_update(reset);
  pid_filter_update(reset);
  imu_filter_update();
}

const char *control_flight_mode_name(void) {
#ifdef VEHICLE_ROVER
  if (rx_aux_on(AUX_RATE_THROTTLE))
    return "RATE THR";
  if (rx_aux_on(AUX_RATE_ASSIST))
    return "RATE ASST";
  return "MANUAL";
#elif defined(VEHICLE_WING)
  if (rx_aux_on(AUX_LEVELMODE))
    return "LEVEL";
  if (rx_aux_on(AUX_ACROMODE))
    return "ACRO";
  return "MANUAL";
#else
  if (state.rth_failsafe_active)
    return "FS RTH";
  if (state.rth_active || rx_aux_on(AUX_RETURN_TO_HOME))
    return "RTH";
  if (!rx_aux_on(AUX_LEVELMODE))
    return "ACRO";
  if (rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON))
    return "RM HORIZON";
  if (!rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON))
    return "HORIZON";
  if (rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON))
    return "RACEMODE";
  return "LEVEL";
#endif
}

static void failsafe_set_phase(failsafe_phase_t phase, uint32_t now_us) {
  const auto previous_phase = state.failsafe_phase;

  if (previous_phase != phase) {
    state.failsafe_phase = phase;
    failsafe_phase_start_us = now_us;
  }

  if (phase == FAILSAFE_PHASE_IDLE) {
    state.failsafe_time_ms = 0;
  } else if (previous_phase == FAILSAFE_PHASE_IDLE) {
    state.failsafe_time_ms = time_millis();
  }
}

static void failsafe_clear_stage1_fallback(void) {
  if (state.failsafe_phase == FAILSAFE_PHASE_STAGE1_GUARD) {
#ifdef VEHICLE_MULTI
    if (state.rth_active) {
      return;
    }
#endif
    flags.controls_override = 0;
  }
}

static void failsafe_apply_stage1_fallback(void) {
  state.rx_override.roll = 0.0f;
  state.rx_override.pitch = 0.0f;
  state.rx_override.yaw = 0.0f;
#ifdef VEHICLE_ROVER
  state.rx_override.throttle = 0.5f;
#else
  state.rx_override.throttle = 0.0f;
#endif
  flags.controls_override = 1;
}

static bool control_throttle_safe_for_arming(void) {
#ifdef VEHICLE_ROVER
  const float neutral = 0.5f;
  const float deadband = profile.rover.center_deadband;
  if (profile.rover.reversible) {
    return fabsf(state.rx_filtered.throttle - neutral) <= deadband;
  }
  return state.rx_filtered.throttle <= neutral + deadband;
#elif defined(VEHICLE_MULTI) || defined(VEHICLE_WING)
  return state.rx_filtered.throttle <= THROTTLE_SAFETY;
#else
  return true;
#endif
}

static bool control_prearm_requires_cycle(void) {
  return profile.receiver.aux[AUX_PREARM].channel != RX_CHANNEL_ON;
}

static void failsafe_clear(uint32_t now_us) {
  flags.failsafe = 0;
  flags.failsafe_outputs_blocked = 0;
  failsafe_clear_stage1_fallback();
  failsafe_set_phase(FAILSAFE_PHASE_IDLE, now_us);
}

void control_failsafe_update() {
  const uint32_t now_us = time_micros();
  // Receiver loss can change between navigation updates.
  state.rth_failsafe_active = state.rth_active && profile.navigation.rth_on_failsafe && flags.failsafe_signal_lost;

  if (flags.rx_ready != 1) {
    flags.failsafe = 1;
    flags.failsafe_outputs_blocked = 1;
    failsafe_clear_stage1_fallback();
    failsafe_set_phase(FAILSAFE_PHASE_IDLE, now_us);
    return;
  }

  if (flags.failsafe_signal_lost) {
#ifdef VEHICLE_MULTI
    if (state.rth_failsafe_active) {
      flags.failsafe = 1;
      flags.failsafe_outputs_blocked = 0;
      failsafe_set_phase(FAILSAFE_PHASE_STAGE1_GUARD, now_us);
      return;
    }
#endif

    if (state.failsafe_phase == FAILSAFE_PHASE_STAGE2_DROP ||
        state.failsafe_phase == FAILSAFE_PHASE_RECOVERY) {
      flags.failsafe = 1;
      flags.failsafe_outputs_blocked = 1;
      failsafe_set_phase(FAILSAFE_PHASE_STAGE2_DROP, now_us);
      if (flags.arm_state) {
        failsafe_rearm_allows_prearm_hold = true;
      }
      flags.arm_state = 0;
      return;
    }

    const uint32_t loss_us = now_us - state.last_frame_time_us;
    if (loss_us < FAILSAFE_HOLD_TIME_US) {
      flags.failsafe = 0;
      flags.failsafe_outputs_blocked = 0;
      failsafe_set_phase(FAILSAFE_PHASE_HOLD_LAST, now_us);
      return;
    }

    const uint32_t stage1_us = loss_us - FAILSAFE_HOLD_TIME_US;
    if (stage1_us < FAILSAFE_STAGE2_TIME_US) {
      flags.failsafe = 1;
      flags.failsafe_outputs_blocked = 0;
      failsafe_set_phase(FAILSAFE_PHASE_STAGE1_GUARD, now_us);
      failsafe_apply_stage1_fallback();
      return;
    }

    failsafe_clear_stage1_fallback();
    flags.failsafe = 1;
    flags.failsafe_outputs_blocked = 1;
    failsafe_set_phase(FAILSAFE_PHASE_STAGE2_DROP, now_us);
    if (flags.arm_state) {
      failsafe_rearm_allows_prearm_hold = true;
    }
    flags.arm_state = 0;
    return;
  }

  switch ((failsafe_phase_t)state.failsafe_phase) {
  case FAILSAFE_PHASE_STAGE2_DROP:
    flags.failsafe = 1;
    flags.failsafe_outputs_blocked = 1;
    failsafe_set_phase(FAILSAFE_PHASE_RECOVERY, now_us);
    return;

  case FAILSAFE_PHASE_RECOVERY:
    flags.failsafe = 1;
    flags.failsafe_outputs_blocked = 1;
    if ((now_us - failsafe_phase_start_us) >= FAILSAFE_RECOVERY_TIME_US) {
      failsafe_clear(now_us);
    }
    return;

  case FAILSAFE_PHASE_HOLD_LAST:
  case FAILSAFE_PHASE_STAGE1_GUARD:
  case FAILSAFE_PHASE_IDLE:
  default:
    failsafe_clear(now_us);
    return;
  }
}

void control_update_arming() {
  static bool checked_prearm = false;
  static bool rx_ready_seen = false;
  static uint32_t arming_disabled_latch = ARMING_DISABLED_ARM_SWITCH;
  static bool was_failsafe_active = false;

  flags.arm_request = rx_aux_on(AUX_ARMING);
  flags.arming_disabled_flags = ARMING_DISABLED_NONE;
  const bool failsafe_active = flags.failsafe != 0 || flags.failsafe_signal_lost != 0;
  const bool failsafe_disables_arming = failsafe_active && (!flags.arm_state || flags.failsafe_outputs_blocked);

  static bool prearm_ready = true;
  const bool prearm_active = rx_aux_on(AUX_PREARM);
  const bool prearm_requires_cycle = control_prearm_requires_cycle();
  if (!prearm_requires_cycle || !prearm_active) {
    prearm_ready = true;
  }
  if (!prearm_active) {
    failsafe_rearm_allows_prearm_hold = false;
  }

  if (was_failsafe_active && !failsafe_active && !flags.arm_request) {
    arming_disabled_latch &= ~ARMING_DISABLED_ARM_SWITCH;
  }
  was_failsafe_active = failsafe_active;

  if (flags.rx_ready != 1) {
    rx_ready_seen = false;
  }

  if (failsafe_disables_arming) {
    flags.arming_disabled_flags |= ARMING_DISABLED_FAILSAFE;
  }

  if (flags.usb_active) {
    flags.arming_disabled_flags |= ARMING_DISABLED_USB;
  }

  if (flags.arm_request && ((failsafe_active && !flags.arm_state) || flags.usb_active)) {
    arming_disabled_latch |= ARMING_DISABLED_ARM_SWITCH;
  }

  if (flags.arm_request && !flags.usb_active && (flags.arm_state || !failsafe_active)) {
    bool can_arm = !failsafe_active && !checked_prearm && arming_disabled_latch == ARMING_DISABLED_NONE;

    const bool prearm_allows_arm =
        prearm_active && (!prearm_requires_cycle || prearm_ready || failsafe_rearm_allows_prearm_hold);
    can_arm = can_arm && prearm_allows_arm;
    can_arm = can_arm && control_throttle_safe_for_arming();

    if (can_arm) {
      flags.arm_state = 1;

      if (prearm_requires_cycle) {
        prearm_ready = false;
      }
      failsafe_rearm_allows_prearm_hold = false;

#ifdef VEHICLE_MULTI
      if (!flags.turtle_ready) {
        motor_set_direction(MOTOR_FORWARD);
      }
#endif
    } else if (!flags.arm_state) {
      if (!control_throttle_safe_for_arming()) {
        arming_disabled_latch |= ARMING_DISABLED_THROTTLE;
      } else {
        arming_disabled_latch |= ARMING_DISABLED_ARM_SWITCH;
      }
    }

    checked_prearm = true;
  } else {
    flags.arm_state = 0;
    checked_prearm = false;

    if (flags.rx_ready == 1 && !flags.arm_request) {
      // rx is ready and arm switch has been low for a full control cycle
      if (rx_ready_seen) {
        arming_disabled_latch &= ~ARMING_DISABLED_ARM_SWITCH;
      }
      rx_ready_seen = true;
    }

    arming_disabled_latch &= ~ARMING_DISABLED_THROTTLE;
  }

#ifdef VEHICLE_MULTI
  if (flags.turtle && !flags.turtle_ready) {
    arming_disabled_latch |= ARMING_DISABLED_ARM_SWITCH;
  }
#endif

  flags.arming_disabled_flags |= arming_disabled_latch;

  if (flags.arming_disabled_flags != ARMING_DISABLED_NONE) {
    flags.arm_state = 0;
  }
}
