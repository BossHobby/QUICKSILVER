#pragma once

#include <stdint.h>

typedef struct {
  uint8_t armed_state : 1;
  uint8_t in_air : 1;
  uint8_t binding_while_armed : 1;
  uint8_t onground : 1;
} control_flags_t;

void control(void);
