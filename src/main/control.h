#pragma once

#include <stdint.h>

#include "defines.h"
#include "rx.h"
#include "util/vector.h"

#define RXMODE_BIND 0
#define RXMODE_NORMAL 1

// THE UN OF STRUCTS
typedef struct {
  uint8_t armed_state : 1;
  uint8_t in_air : 1;
  uint8_t binding_while_armed : 1; // => arming_safety
  uint8_t onground : 1;
  uint8_t failsafe : 1;        // failsafe on / off
  uint8_t lowbatt : 1;         // signal for lowbattery
  uint8_t throttle_safety : 1; // throttle is above safety limit
  uint8_t usb_active : 1;
  uint8_t rxmode : 1; // bind / normal rx mode
  uint8_t rx_ready : 1;
  uint8_t controls_override : 1;
  uint8_t acro_override : 1;
} control_flags_t;

extern control_flags_t flags;

typedef struct {
  float looptime;      // looptime in seconds
  float osd_totaltime; // running sum of looptimes

  float lipo_cell_count;
  float vbattfilt; // filtered battery in volts
  float vbattfilt_corr;
  float vbatt_comp;
  float vreffilt; // voltage reference for vcc compensation

  vec4_t rx;          // holds the main four channels, roll, pitch, yaw, throttle
  vec4_t rx_filtered; // same as above, but filtered by the rx smoothing
  vec4_t rx_override;

  float throttle;
  float thrsum; // average of all 4 motor thrusts

  uint8_t aux[AUX_CHANNEL_MAX]; // digital on / off channels

  vec3_t accel_raw;
  vec3_t accel;

  vec3_t gyro_raw;
  vec3_t gyro;

  vec3_t GEstG;
  vec3_t attitude;

  vec3_t setpoint;  // angular velocity setpoint from stick input
  vec3_t error;     // setpoint - gyro = error in angular velocity
  vec3_t pidoutput; // combinded output of the pid controller

  float angleerror[ANGLE_PID_SIZE];
} control_state_t;

extern control_state_t state;

void control(void);
