#include "status.h"

#include <string.h>

#include "core/looptime.h"
#include "core/profile.h"
#include "driver/osd.h"
#include "driver/time.h"
#include "flight/control.h"

#define LABEL_LEN 22

typedef enum {
  STATUS_RX_WAIT,
  STATUS_FAILSAFE,
  STATUS_LOOPTIME,
  STATUS_ARM_SAFETY,
  STATUS_THROTTLE_SAFETY,
  STATUS_ARM,
  STATUS_DISARM,
  STATUS_LOW_BAT,
  STATUS_MOTOR_TEST,
  STATUS_TURTLE,
  STATUS_MAX,
} osd_status_entries_t;

typedef enum {
  MODE_HOLD,
  MODE_TEMP,
} osd_status_mode_t;

typedef enum {
  PRINT_IDLE,
  PRINT_START,
  PRINT_WAIT,
  PRINT_CLEAR,
} osd_print_state_t;

typedef struct {
  osd_status_entries_t entry;
  osd_print_state_t state;
  osd_status_mode_t mode;
  uint8_t label[LABEL_LEN];
} osd_status_t;

static osd_status_t current_status = {
    .entry = STATUS_MAX,
    .state = PRINT_IDLE,
};

const char *default_system_status_labels[STATUS_MAX] = {
    [STATUS_RX_WAIT] = "WAIT FOR RX",
    [STATUS_FAILSAFE] = "**FAILSAFE**",
    [STATUS_LOOPTIME] = "**LOOPTIME**",
    [STATUS_ARM_SAFETY] = "ARMING SAFETY",
    [STATUS_THROTTLE_SAFETY] = "THROTTLE SAFETY",
    [STATUS_ARM] = "**ARMED**",
    [STATUS_DISARM] = "**DISARMED**",
    [STATUS_LOW_BAT] = "**LOW BATTERY**",
    [STATUS_MOTOR_TEST] = "**MOTOR TEST**",
    [STATUS_TURTLE] = "**TURTLE**",
};

const char *guac_system_status_labels[STATUS_MAX] = {
    [STATUS_RX_WAIT] = "RX LOADING",
    [STATUS_FAILSAFE] = "**404 RX NOT FOUND**",
    [STATUS_LOOPTIME] = "**LOOPTIME**",
    [STATUS_ARM_SAFETY] = "**ARMING SAFETY**",
    [STATUS_THROTTLE_SAFETY] = "**DANGER ZONE**",
    [STATUS_ARM] = "**HERE WE GO AGAIN**",
    [STATUS_DISARM] = "**GAME OVER**",
    [STATUS_LOW_BAT] = "CONSTRUCT MORE PYLONS",
    [STATUS_MOTOR_TEST] = "**MOTOR TEST**",
    [STATUS_TURTLE] = "\x60THIS SIDE UP\x60",
};

static void osd_status_show(osd_status_mode_t mode, osd_status_entries_t entry) {
  if (current_status.entry == entry) {
    return;
  }

  const char **labels = profile.osd.guac_mode ? guac_system_status_labels : default_system_status_labels;
  const char *label = labels[entry];

  current_status.entry = entry;
  current_status.state = PRINT_START;
  current_status.mode = mode;
  const uint32_t len = strlen(label);
  if (len >= LABEL_LEN) {
    memcpy(current_status.label, label, LABEL_LEN);
  } else {
    const uint32_t offset = (LABEL_LEN - len) / 2;
    memset(current_status.label, ' ', LABEL_LEN);
    memcpy(current_status.label + offset, label, len);
  }
}

static bool osd_status_print(osd_element_t *el) {
  static const uint8_t empty_label[LABEL_LEN] = {' '};
  static uint32_t start_time;

  switch (current_status.state) {
  case PRINT_START:
    osd_start(osd_attr(el) | OSD_ATTR_BLINK, el->pos_x, el->pos_y);
    osd_write_data(current_status.label, LABEL_LEN);

    if (current_status.mode == MODE_HOLD) {
      current_status.state = PRINT_IDLE;
    } else {
      current_status.state = PRINT_WAIT;
    }
    start_time = time_millis();
    return false;

  case PRINT_WAIT:
    if ((time_millis() - start_time) < 1000) {
      return true;
    }
    current_status.state = PRINT_CLEAR;
    return false;

  case PRINT_CLEAR:
    osd_start(osd_attr(el) | OSD_ATTR_BLINK, el->pos_x, el->pos_y);
    osd_write_data(empty_label, LABEL_LEN);
    current_status.state = PRINT_IDLE;
    return false;

  case PRINT_IDLE:
    return true;
  }

  return true;
}

bool osd_status_update(osd_element_t *el) {
  if (!flags.rx_ready) {
    osd_status_show(MODE_HOLD, STATUS_RX_WAIT);
    return osd_status_print(el);
  }

  if (flags.failsafe && flags.rx_ready) {
    // only show failsafe if rx was ready
    osd_status_show(MODE_HOLD, STATUS_FAILSAFE);
    return osd_status_print(el);
  }

  {
    extern uint8_t looptime_warning;
    if (looptime_warning && (state.looptime_autodetect > 125.0f)) {
      osd_status_show(MODE_HOLD, STATUS_LOOPTIME);
      return osd_status_print(el);
    }
  }

  if (flags.arm_safety) {
    osd_status_show(MODE_HOLD, STATUS_ARM_SAFETY);
    return osd_status_print(el);
  }

  if (flags.throttle_safety) {
    osd_status_show(MODE_HOLD, STATUS_THROTTLE_SAFETY);
    return osd_status_print(el);
  }

  {
    static uint8_t last_arm_state = 0;
    if (flags.arm_state != last_arm_state) {
      if (flags.arm_state) {
        osd_status_show(MODE_TEMP, STATUS_ARM);
      } else {
        osd_status_show(MODE_TEMP, STATUS_DISARM);
      }
      last_arm_state = flags.arm_state;
      return osd_status_print(el);
    }
  }

  if (flags.lowbatt && state.vbat >= 1.0f) {
    // only show lowbat if we detect some voltage at all
    osd_status_show(MODE_HOLD, STATUS_LOW_BAT);
    return osd_status_print(el);
  }

  if (rx_aux_on(AUX_MOTOR_TEST)) {
    osd_status_show(MODE_HOLD, STATUS_MOTOR_TEST);
    return osd_status_print(el);
  }

  if (flags.turtle) {
    osd_status_show(MODE_HOLD, STATUS_TURTLE);
    return osd_status_print(el);
  }

  if (current_status.mode != MODE_TEMP && current_status.entry != STATUS_MAX) {
    current_status.entry = STATUS_MAX;
    current_status.state = PRINT_CLEAR;
  }
  return osd_status_print(el);
}