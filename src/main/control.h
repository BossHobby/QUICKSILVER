#pragma once

#include <stdint.h>

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
} control_flags_t;

extern control_flags_t flags;

typedef struct {
  vec4_t rx; // holds the main four channels, roll, pitch , yaw , throttle

  vec3_t accel_raw;
  vec3_t accel;

  vec3_t gyro_raw;
  vec3_t gyro;
} control_state_t;

extern control_state_t state;

void control(void);
