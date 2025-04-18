#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  RTH_STATE_INACTIVE,
  RTH_STATE_CLIMB,
  RTH_STATE_NAVIGATE,
  RTH_STATE_DESCEND,
  RTH_STATE_LAND,
  RTH_STATE_LANDED,
  RTH_STATE_HOVER_HOME
} rth_state_t;


void nav_update();
rth_state_t nav_get_rth_state();
bool nav_rth_active();
void nav_rth_start();
void nav_rth_stop();