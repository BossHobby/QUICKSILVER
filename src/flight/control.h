#pragma once

#include <stdint.h>

#include "core/failloop.h"
#include "core/project.h"
#include "core/scheduler.h"
#include "rx/rx.h"
#include "util/vector.h"

#define ANGLE_PID_SIZE 2

#define RXMODE_BIND 0
#define RXMODE_NORMAL 1

// THE UN OF STRUCTS
typedef struct {
  uint8_t arm_switch : 1; // arming switch (AUX_ARMING + AUX_PREARM) has been tripped
  uint8_t arm_state : 1;  // armed after all saftey checks have passed
  uint8_t arm_safety : 1;
  uint8_t throttle_safety : 1; // throttle is above safety limit

  uint8_t in_air : 1;    // throttle was raised above THROTTLE_SAFETY (10%), only resets on disarm
  uint8_t on_ground : 1; // armed and we are sending some throttle to the motors

  uint8_t failsafe : 1; // failsafe on / off
  uint8_t lowbatt : 1;  // signal for lowbattery

  uint8_t rx_mode : 1; // bind / normal rx mode
  uint8_t rx_ready : 1;

  uint8_t controls_override : 1;  // will activate rx_override below & will write directly to the motors (motor_test)
  uint8_t motortest_override : 1; // tuns off digital idle in the dshot driver & will write either sticks or usb_motortest values directly to motors
  uint8_t turtle : 1;
  uint8_t turtle_ready : 1;
  uint8_t gestures_disabled : 1;

  uint8_t usb_active : 1;
} control_flags_t;

extern control_flags_t flags;

typedef struct {
  failloop_t failloop;

  float looptime;            // looptime in seconds
  float looptime_us;         // looptime in us
  float looptime_autodetect; // desired looptime in us
  float timefactor;          // timefactor for pid calc
  uint32_t loop_counter;     // number of loops ran

  float uptime;      // running sum of looptimes
  float armtime;     // running sum of looptimes (while armed)
  uint32_t cpu_load; // micros we have had left last loop

  uint32_t failsafe_time_ms; // time the last failsafe occured in ms

  uint8_t lipo_cell_count;

  float cpu_temp;

  float vbat;                      // battery in volts
  float vbat_filtered;             // filtered battery in volts
  float vbat_filtered_decay;       // filtered battery with time decay
  float vbat_cell_avg;             // filtered battery divided by cell count
  float vbat_compensated;          // battery compensated for sag
  float vbat_compensated_cell_avg; // battery compensated for sag divided by cell count

  float ibat;
  float ibat_filtered;

  vec4_t rx;          // holds the raw or calibrated main four channels, roll, pitch, yaw, throttle
  vec4_t rx_filtered; // same as above, but with constraints (just in case), smoothing and deadband applied
  vec4_t rx_override; // override values, activated by controls_override
  float rx_filter_hz;

  stick_wizard_state_t stick_calibration_wizard; // current phase of the calibration wizard

  float rx_rssi;
  uint32_t rx_status;

  float throttle; // input throttle with idle etc applied
  float thrsum;   // average of all 4 motor thrusts

  uint8_t aux[AUX_CHANNEL_MAX]; // digital on / off channels

  vec3_t accel_raw; // raw accel reading with rotation and scaling applied
  vec3_t accel;     // filtered accel readings

  float gyro_temp;         // gyro temparture reading
  vec3_t gyro_raw;         // raw gyro reading with rotation and scaling applied
  vec3_t gyro;             // filtered gyro reading
  vec3_t gyro_delta_angle; // angle covered in  last time interval

  vec3_t GEstG; // gravity vector
  vec3_t attitude;

  vec3_t setpoint; // angular velocity setpoint from stick input
  vec3_t error;    // setpoint - gyro = error in angular velocity

  vec3_t pid_p_term;
  vec3_t pid_i_term;
  vec3_t pid_d_term;
  vec3_t pidoutput; // combinded output of the pid controller

  vec4_t motor_mix;

  float angleerror[ANGLE_PID_SIZE];
} control_state_t;

#define STATE_MEMBERS                         \
  MEMBER(failloop, uint8_t)                   \
  MEMBER(looptime, float)                     \
  MEMBER(looptime_us, float)                  \
  MEMBER(looptime_autodetect, float)          \
  MEMBER(timefactor, float)                   \
  MEMBER(loop_counter, uint32_t)              \
  MEMBER(uptime, float)                       \
  MEMBER(armtime, float)                      \
  MEMBER(cpu_load, uint32_t)                  \
  MEMBER(failsafe_time_ms, uint32_t)          \
  MEMBER(lipo_cell_count, uint8_t)            \
  MEMBER(cpu_temp, float)                     \
  MEMBER(vbat, float)                         \
  MEMBER(vbat_filtered, float)                \
  MEMBER(vbat_filtered_decay, float)          \
  MEMBER(vbat_cell_avg, float)                \
  MEMBER(vbat_compensated, float)             \
  MEMBER(vbat_compensated_cell_avg, float)    \
  MEMBER(ibat, float)                         \
  MEMBER(ibat_filtered, float)                \
  MEMBER(rx, vec4_t)                          \
  MEMBER(rx_filtered, vec4_t)                 \
  MEMBER(rx_override, vec4_t)                 \
  MEMBER(stick_calibration_wizard, uint8_t)   \
  MEMBER(rx_rssi, float)                      \
  MEMBER(rx_status, uint32_t)                 \
  MEMBER(throttle, float)                     \
  MEMBER(thrsum, float)                       \
  ARRAY_MEMBER(aux, AUX_CHANNEL_MAX, uint8_t) \
  MEMBER(accel_raw, vec3_t)                   \
  MEMBER(accel, vec3_t)                       \
  MEMBER(gyro_temp, float)                    \
  MEMBER(gyro_raw, vec3_t)                    \
  MEMBER(gyro, vec3_t)                        \
  MEMBER(gyro_delta_angle, vec3_t)            \
  MEMBER(GEstG, vec3_t)                       \
  MEMBER(attitude, vec3_t)                    \
  MEMBER(setpoint, vec3_t)                    \
  MEMBER(error, vec3_t)                       \
  MEMBER(pid_p_term, vec3_t)                  \
  MEMBER(pid_i_term, vec3_t)                  \
  MEMBER(pid_d_term, vec3_t)                  \
  MEMBER(pidoutput, vec3_t)                   \
  MEMBER(motor_mix, vec4_t)                   \
  ARRAY_MEMBER(angleerror, ANGLE_PID_SIZE, float)

typedef struct {
  uint8_t active;
  float value[MOTOR_PIN_MAX];
} motor_test_t;

extern control_state_t state;
extern motor_test_t motor_test;

cbor_result_t cbor_encode_control_state_t(cbor_value_t *enc, const control_state_t *s);

void control();
