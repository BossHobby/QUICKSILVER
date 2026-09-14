#include "control/gestures.h"

#include <math.h>

#include "control/control.h"
#include "control/sixaxis.h"
#include "core/flash.h"
#include "core/profile.h"
#include "core/project.h"
#include "core/scheduler.h"
#include "driver/time.h"
#include "io/led.h"
#include "osd/render.h"

enum gesture_direction_t {
  DIRECTION_INVALID,
  DIRECTION_CENTER,
  DIRECTION_UP,
  DIRECTION_DOWN,
  DIRECTION_LEFT,
  DIRECTION_RIGHT,
};

static constexpr uint32_t DEBOUNCE_US = 50000;
static constexpr uint32_t HOLD_US = 500000;
static constexpr uint32_t SHORTCUT_IDLE_US = 700000;
static constexpr uint32_t MENU_IDLE_US = 100000;
static constexpr uint32_t REPEAT_US = 125000;

static struct {
  gesture_direction_t direction;
  gesture_direction_t press;
  gesture_direction_t sequence[3];
  uint8_t count;
  uint32_t since;
  uint32_t repeat;
  bool ready;
  bool menu;
  bool repeated;
} detector;

static gesture_direction_t stick_direction(const vec4_t &sticks) {
#ifdef VEHICLE_ROVER
  const float horizontal = (sticks.throttle - 0.5f) * 2.0f;
  const bool left = horizontal < -0.7f;
  const bool right = horizontal > 0.7f;
  const bool horizontal_center = fabsf(horizontal) < 0.2f;
  const float vertical = -sticks.yaw;
#else
  const bool left = sticks.roll < -0.7f || sticks.yaw < -0.7f;
  const bool right = sticks.roll > 0.7f || sticks.yaw > 0.7f;
  const bool horizontal_center = fabsf(sticks.roll) < 0.2f && fabsf(sticks.yaw) < 0.2f;
  const float vertical = sticks.pitch;
#endif
  const bool vertical_center = fabsf(vertical) < 0.2f;
  if (horizontal_center && vertical_center) return DIRECTION_CENTER;
  if (left && !right && vertical_center) return DIRECTION_LEFT;
  if (right && !left && vertical_center) return DIRECTION_RIGHT;
  if (vertical > 0.7f && horizontal_center) return DIRECTION_UP;
  if (vertical < -0.7f && horizontal_center) return DIRECTION_DOWN;
  return DIRECTION_INVALID;
}

static gesture_command_t menu_command(gesture_direction_t direction) {
  switch (direction) {
  case DIRECTION_UP: return GESTURE_MENU_UP;
  case DIRECTION_DOWN: return GESTURE_MENU_DOWN;
  case DIRECTION_LEFT: return GESTURE_MENU_LEFT;
  case DIRECTION_RIGHT: return GESTURE_MENU_RIGHT;
  default: return GESTURE_NONE;
  }
}

static gesture_command_t shortcut_command() {
  const auto *sequence = detector.sequence;
  if (sequence[0] == DIRECTION_DOWN && sequence[1] == DIRECTION_DOWN && sequence[2] == DIRECTION_DOWN)
    return GESTURE_CALIBRATE_SAVE;
  if (sequence[0] == DIRECTION_UP && sequence[1] == DIRECTION_UP && sequence[2] == DIRECTION_UP)
    return GESTURE_TOGGLE_BIND;
  if (sequence[0] == DIRECTION_RIGHT && sequence[1] == DIRECTION_RIGHT && sequence[2] == DIRECTION_RIGHT)
    return GESTURE_OPEN_MENU;
  if (sequence[0] == DIRECTION_LEFT && sequence[1] == DIRECTION_RIGHT && sequence[2] == DIRECTION_LEFT)
    return GESTURE_RESET_OSD;
  return GESTURE_NONE;
}

gesture_command_t gestures_detect(const vec4_t &sticks, bool enabled, bool menu, uint32_t now_us) {
  if (!enabled || menu != detector.menu) {
    detector = {};
    detector.menu = menu;
    detector.since = now_us;
    return GESTURE_NONE;
  }
  const gesture_direction_t direction = stick_direction(sticks);
  if (direction != detector.direction) {
    detector.direction = direction;
    detector.since = now_us;
    detector.repeat = now_us;
  }
  const uint32_t held = now_us - detector.since;
  const uint32_t neutral_time = menu ? MENU_IDLE_US : SHORTCUT_IDLE_US;

  // Sticks pass through the threshold gap on every press/release. Only a
  // sustained ambiguous position invalidates an otherwise valid sequence.
  if (direction == DIRECTION_INVALID) {
    if (held >= DEBOUNCE_US) {
      detector.ready = false;
      detector.press = DIRECTION_INVALID;
      detector.count = 0;
    }
    return GESTURE_NONE;
  }

  if (direction == DIRECTION_CENTER) {
    if (held >= neutral_time) {
      detector.ready = true;
      detector.count = 0;
    }
    if (held < DEBOUNCE_US || detector.press == DIRECTION_INVALID)
      return GESTURE_NONE;
    const gesture_direction_t press = detector.press;
    detector.press = DIRECTION_INVALID;
    if (menu)
      return detector.repeated ? GESTURE_NONE : menu_command(press);
    detector.sequence[detector.count++] = press;
    if (detector.count < 3)
      return GESTURE_NONE;
    detector.count = 0;
    detector.ready = false;
    return shortcut_command();
  }

  if (!detector.ready || held < DEBOUNCE_US)
    return GESTURE_NONE;
  if (detector.press != direction) {
    if (detector.press != DIRECTION_INVALID) {
      detector.ready = false;
      detector.press = DIRECTION_INVALID;
      detector.count = 0;
      return GESTURE_NONE;
    }
    detector.press = direction;
    detector.repeated = false;
  }
  if (held >= HOLD_US) {
    if (!menu) {
      detector.ready = false;
      detector.press = DIRECTION_INVALID;
      detector.count = 0;
    } else if (now_us - detector.repeat >= REPEAT_US) {
      detector.repeat = now_us;
      detector.repeated = true;
      return menu_command(direction);
    }
  }
  return GESTURE_NONE;
}

void gestures() {
  const bool enabled = !flags.arm_state && !flags.in_air && flags.on_ground &&
                       flags.rx_ready && !flags.failsafe && !flags.gestures_disabled;
  const bool menu = osd_state.screen != OSD_SCREEN_REGULAR && osd_state.screen != OSD_SCREEN_CLEAR;
  const auto command = gestures_detect(state.rx, enabled, menu, time_micros());
  static bool save_bind_only = false;
  switch (command) {
  case GESTURE_CALIBRATE_SAVE:
    if (!save_bind_only) {
      sixaxis_gyro_cal();
      sixaxis_acc_cal();
    } else {
      led_flash();
      save_bind_only = false;
    }
    flash_save();
    task_reset_runtime();
    break;
  case GESTURE_TOGGLE_BIND:
    profile.receiver.bind.bind_saved = !profile.receiver.bind.bind_saved;
    save_bind_only = true;
    led_flash();
    break;
  case GESTURE_OPEN_MENU:
    osd_push_screen(OSD_SCREEN_MAIN_MENU);
    led_flash();
    break;
  case GESTURE_RESET_OSD:
    osd_exit();
    break;
  case GESTURE_MENU_UP: osd_handle_input(OSD_INPUT_UP); break;
  case GESTURE_MENU_DOWN: osd_handle_input(OSD_INPUT_DOWN); break;
  case GESTURE_MENU_LEFT: osd_handle_input(OSD_INPUT_LEFT); break;
  case GESTURE_MENU_RIGHT: osd_handle_input(OSD_INPUT_RIGHT); break;
  case GESTURE_NONE: break;
  }
}
