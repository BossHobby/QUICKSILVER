#pragma once

#include <stdint.h>

// THE UN OF STRUCTS
typedef struct {
  uint8_t armed_state : 1;
  uint8_t in_air : 1;
  uint8_t binding_while_armed : 1; // => arming_safety
  uint8_t onground : 1;
  uint8_t failsafe : 1; // failsafe on / off
  uint8_t lowbatt : 1;  // signal for lowbattery
  // usb_active
  // throttle_safety
} control_flags_t;

void control(void);
