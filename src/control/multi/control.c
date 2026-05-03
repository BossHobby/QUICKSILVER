#include "control/multi/control.h"

#include <math.h>
#include <stdint.h>

#include "control/control.h"
#include "control/gestures.h"
#include "control/multi/angle_pid.h"
#include "control/multi/motor.h"
#include "control/multi/pid.h"
#include "control/multi/rates.h"
#include "control/multi/turtle_mode.h"
#include "core/profile.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "util/util.h"

#ifndef IDLE_THR
#define IDLE_THR .001f // just enough to override motor stop at 0 throttle
#endif

static void control_flight_mode() {
  const vec3_t rates = input_rates_calc();

  if (rx_aux_on(AUX_LEVELMODE)) {
    state.angle_error = input_stick_vector(state.rx_filtered.axis);

    const vec3_t yaw_error = {
        .roll = state.GEstG.pitch * rates.yaw,
        .pitch = -state.GEstG.roll * rates.yaw,
        .yaw = state.GEstG.yaw * rates.yaw,
    };

    if (rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON)) {
      if (state.GEstG.yaw < 0) {
        state.setpoint.roll = rates.roll;
        state.setpoint.pitch = rates.pitch;

        state.error.roll = rates.roll - state.gyro.roll;
        state.error.pitch = rates.pitch - state.gyro.pitch;
      } else {
        state.setpoint.roll = angle_pid(0) + yaw_error.axis[0];
        state.error.roll = state.setpoint.roll - state.gyro.roll;

        state.setpoint.pitch = rates.pitch;
        state.error.pitch = rates.pitch - state.gyro.pitch;
      }
      state.setpoint.yaw = rates.yaw;
      state.error.yaw = yaw_error.axis[2] - state.gyro.yaw;

    } else if (rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON)) {
      float inclinationRoll = state.attitude.roll;
      float inclinationPitch = state.attitude.pitch;
      float inclinationMax;
      if (fabsf(inclinationRoll) >= fabsf(inclinationPitch)) {
        inclinationMax = fabsf(inclinationRoll);
      } else {
        inclinationMax = fabsf(inclinationPitch);
      }
      float angleFade;
      if (inclinationMax <= HORIZON_ANGLE_TRANSITION) {
        angleFade = inclinationMax / HORIZON_ANGLE_TRANSITION;
      } else {
        angleFade = 1;
      }
      float stickFade;
      float deflection = fabsf(state.rx_filtered.roll);
      if (deflection <= HORIZON_STICK_TRANSITION) {
        stickFade = deflection / HORIZON_STICK_TRANSITION;
      } else {
        stickFade = 1;
      }
      float fade = (stickFade * (1 - HORIZON_SLIDER)) + (HORIZON_SLIDER * angleFade);

      if (state.GEstG.yaw < 0) {
        state.setpoint.roll = rates.roll;
        state.setpoint.pitch = rates.pitch;
        state.error.roll = rates.roll - state.gyro.roll;
        state.error.pitch = rates.pitch - state.gyro.pitch;
      } else {
        state.setpoint.roll = (angle_pid(0) + yaw_error.axis[0]) * (1.0f - fade) + fade * (rates.roll);
        state.error.roll = ((angle_pid(0) + yaw_error.axis[0] - state.gyro.roll) * (1 - fade)) + (fade * (rates.roll - state.gyro.roll));
        state.setpoint.pitch = rates.pitch;
        state.error.pitch = rates.pitch - state.gyro.pitch;
      }

      state.setpoint.yaw = rates.yaw;
      state.error.yaw = yaw_error.axis[2] - state.gyro.yaw;

    } else if (!rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON)) {
      for (int i = 0; i <= 1; i++) {
        float inclinationRoll = state.attitude.roll;
        float inclinationPitch = state.attitude.pitch;
        float inclinationMax;
        if (fabsf(inclinationRoll) >= fabsf(inclinationPitch)) {
          inclinationMax = fabsf(inclinationRoll);
        } else {
          inclinationMax = fabsf(inclinationPitch);
        }
        float angleFade;
        if (inclinationMax <= HORIZON_ANGLE_TRANSITION) {
          angleFade = inclinationMax / HORIZON_ANGLE_TRANSITION;
        } else {
          angleFade = 1;
        }
        float stickFade;
        float deflection = fabsf(state.rx_filtered.axis[i]);
        if (deflection <= HORIZON_STICK_TRANSITION) {
          stickFade = deflection / HORIZON_STICK_TRANSITION;
        } else {
          stickFade = 1;
        }
        float fade = (stickFade * (1 - HORIZON_SLIDER)) + (HORIZON_SLIDER * angleFade);

        if (state.GEstG.yaw < 0) {
          state.setpoint.axis[i] = rates.axis[i];
          state.error.axis[i] = rates.axis[i] - state.gyro.axis[i];
        } else {
          state.setpoint.axis[i] = (angle_pid(i) + yaw_error.axis[i]) * (1.0f - fade) + fade * (rates.axis[i]);
          state.error.axis[i] = ((angle_pid(i) + yaw_error.axis[i] - state.gyro.axis[i]) * (1 - fade)) + (fade * (rates.axis[i] - state.gyro.axis[i]));
        }
      }
      state.setpoint.yaw = rates.yaw;
      state.error.yaw = yaw_error.axis[2] - state.gyro.yaw;

    } else {
      for (uint32_t i = 0; i < 3; i++) {
        state.setpoint.axis[i] = angle_pid(i) + yaw_error.axis[i];
        state.error.axis[i] = state.setpoint.axis[i] - state.gyro.axis[i];
      }
    }
  } else {
    state.setpoint.roll = rates.roll;
    state.setpoint.pitch = rates.pitch;
    state.setpoint.yaw = rates.yaw;

    state.error.roll = state.setpoint.roll - state.gyro.roll;
    state.error.pitch = state.setpoint.pitch - state.gyro.pitch;
    state.error.yaw = state.setpoint.yaw - state.gyro.yaw;
  }
}

motor_test_t motor_test = {
    .active = 0,
    .value = {MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF},
};

void control() {
  if (rx_aux_on(AUX_TURTLE) && !rx_aux_on(AUX_MOTOR_TEST)) {
    turtle_mode_start();
  } else {
    turtle_mode_cancel();
  }

  turtle_mode_update();

  bool motortest_usb = false;
  if (flags.usb_active && motor_test.active) {
    flags.arm_state = 1;
    flags.on_ground = 0;
    flags.motortest_override = 1;
    flags.controls_override = 0;
    motortest_usb = true;
  } else if (flags.arm_state && rx_aux_on(AUX_MOTOR_TEST)) {
    flags.motortest_override = 1;
    flags.controls_override = 0;
  } else if (!flags.turtle) {
    flags.motortest_override = 0;
  }

  if (flags.controls_override) {
    state.rx_filtered = state.rx_override;
  }

  control_flight_mode();
  pid_calc();
  control_update_arming();

  if (flags.arm_state == 0) {
    state.throttle = 0;
    flags.in_air = 0;
  } else {
    if (!rx_aux_on(AUX_IDLE_UP)) {
      if (state.rx_filtered.throttle < 0.05f) {
        state.throttle = 0;
      } else {
        state.throttle = (input_throttle_calc(state.rx_filtered.throttle) - 0.05f) * 1.05623158f;
      }
    } else {
      if (flags.controls_override) {
        state.throttle = state.rx_filtered.throttle;
      } else {
        state.throttle = (float)IDLE_THR + input_throttle_calc(state.rx_filtered.throttle) * (1.0f - (float)IDLE_THR);
      }
    }

    if ((state.rx_filtered.throttle > THROTTLE_SAFETY) && (flags.in_air == 0)) {
      flags.in_air = 1;
    }
  }

  if (flags.motortest_override) {
    motor_test_calc(motortest_usb, state.motor_mix.axis);
    motor_output_calc(state.motor_mix.axis);
  } else if (!flags.arm_state || flags.failsafe || (state.throttle < 0.001f)) {
    flags.on_ground = 1;
    state.throttle = 0;
    state.thrsum = 0;
    motor_set_all(MOTOR_OFF);
  } else {
    flags.on_ground = 0;

    if (!flags.controls_override) {
      if (profile.motor.throttle_boost > 0.0f) {
        state.throttle += (float)(profile.motor.throttle_boost) * throttlehpf(state.throttle);
        state.throttle = constrain(state.throttle, 0.0f, 1.0f);
      }
    }

    motor_mixer_calc(state.motor_mix.axis);
    motor_output_calc(state.motor_mix.axis);
  }

#ifdef MOTOR_BEEPS
  if ((flags.usb_active == 0 && flags.rx_ready && flags.failsafe && (time_millis() - state.failsafe_time_ms) > MOTOR_BEEPS_TIMEOUT) ||
      (flags.on_ground && rx_aux_on(AUX_BUZZER_ENABLE))) {
    motor_beep();
  } else
#endif
  {
    motor_update();
  }
}
