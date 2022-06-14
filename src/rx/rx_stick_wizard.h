#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  STICK_WIZARD_INACTIVE,
  STICK_WIZARD_SUCCESS,
  STICK_WIZARD_FAILED,
  STICK_WIZARD_START,
  STICK_WIZARD_CAPTURE_STICKS,
  STICK_WIZARD_WAIT_FOR_CONFIRM,
  STICK_WIZARD_CONFIRMED,
  STICK_WIZARD_TIMEOUT,
} stick_wizard_state_t;

void stick_wizard_start(bool from_usb);

void rx_apply_stick_scale();