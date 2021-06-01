#pragma once

#include <stdint.h>

#include "defines.h"
#include "rx.h"
#include "util/vector.h"

#define RXMODE_BIND 0
#define RXMODE_NORMAL 1

// THE UN OF STRUCTS
typedef struct {
  uint8_t arm_state : 1;
  uint8_t arm_safety : 1;
  uint8_t throttle_safety : 1; // throttle is above safety limit

  uint8_t in_air : 1;
  uint8_t on_ground : 1;

  uint8_t failsafe : 1; // failsafe on / off
  uint8_t lowbatt : 1;  // signal for lowbattery

  uint8_t rx_mode : 1; // bind / normal rx mode
  uint8_t rx_ready : 1;

  uint8_t controls_override : 1; // will activate rx_override below & will write directly to the motors (motor_test)
  uint8_t acro_override : 1;
  uint8_t motortest_override : 1; // tuns off digital idle in the dshot driver & will write either sticks or usb_motortest values directly to motors
  uint8_t turtle : 1;

  uint8_t usb_active : 1;
} control_flags_t;

extern control_flags_t flags;

typedef struct {
  uint16_t looptime_autodetect;
  float looptime; // looptime in seconds
  float uptime;   // running sum of looptimes
  float cpu_load; // micros we have had left last loop

  float lipo_cell_count;
  float vbattfilt; // filtered battery in volts
  float vbattfilt_corr;
  float vbatt_comp;
  float vreffilt; // voltage reference for vcc compensation

  vec4_t rx;          // holds the raw or calibrated main four channels, roll, pitch, yaw, throttle
  vec4_t rx_filtered; // same as above, but with constraints(just in case), expo applied smoothing applied and deadband applied
  vec4_t rx_override; // override values, activated by controls_override

  float rx_rssi;
  uint32_t rx_status;

  float throttle; // input throttle with idle etc applied
  float thrsum;   // average of all 4 motor thrusts

  uint8_t aux[AUX_CHANNEL_MAX]; // digital on / off channels

  vec3_t accel_raw; // raw accel reading with rotation and scaling applied
  vec3_t accel;     // filtered accel readings

  float gyro_temp; // gyro temparture reading
  vec3_t gyro_raw; // raw gyro reading with rotation and scaling applied
  vec3_t gyro;     // filtered gyro reading

  vec3_t GEstG; // gravity vector
  vec3_t attitude;

  vec3_t setpoint;  // angular velocity setpoint from stick input
  vec3_t error;     // setpoint - gyro = error in angular velocity
  vec3_t errorvect; // error vector between stick vector and quad orientation

  vec3_t pid_p_term;
  vec3_t pid_i_term;
  vec3_t pid_d_term;
  vec3_t pidoutput; // combinded output of the pid controller

  vec4_t motor_mix;

  float angleerror[ANGLE_PID_SIZE];
} control_state_t;

#define STATE_MEMBERS                       \
  MEMBER(looptime_autodetect, uint16)       \
  MEMBER(looptime, float)                   \
  MEMBER(uptime, float)                     \
  MEMBER(cpu_load, float)                   \
  MEMBER(lipo_cell_count, float)            \
  MEMBER(vbattfilt, float)                  \
  MEMBER(vbattfilt_corr, float)             \
  MEMBER(vbatt_comp, float)                 \
  MEMBER(vreffilt, float)                   \
  MEMBER(rx, vec4_t)                        \
  MEMBER(rx_filtered, vec4_t)               \
  MEMBER(rx_override, vec4_t)               \
  MEMBER(rx_rssi, float)                    \
  MEMBER(rx_status, uint32)                 \
  MEMBER(throttle, float)                   \
  MEMBER(thrsum, float)                     \
  ARRAY_MEMBER(aux, AUX_CHANNEL_MAX, uint8) \
  MEMBER(accel_raw, vec3_t)                 \
  MEMBER(accel, vec3_t)                     \
  MEMBER(gyro_temp, float)                  \
  MEMBER(gyro_raw, vec3_t)                  \
  MEMBER(gyro, vec3_t)                      \
  MEMBER(GEstG, vec3_t)                     \
  MEMBER(attitude, vec3_t)                  \
  MEMBER(setpoint, vec3_t)                  \
  MEMBER(error, vec3_t)                     \
  MEMBER(errorvect, vec3_t)                 \
  MEMBER(pidoutput, vec3_t)

extern control_state_t state;

cbor_result_t cbor_encode_control_state_t(cbor_value_t *enc, const control_state_t *s);

void control(void);
