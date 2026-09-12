#include "status.h"

#include <string.h>

#include "control/control.h"
#include "core/profile.h"
#include "driver/osd/osd.h"
#include "driver/time.h"

#define LABEL_LEN 22
#define LABEL_MAX_LEN 64
#define SCROLL_PADDING 4
#define SCROLL_INTERVAL_MS 250

typedef enum {
  STATUS_RX_WAIT,
  STATUS_FAILSAFE,
  STATUS_LOOPTIME,
  STATUS_USB_SAFETY,
  STATUS_ARM_SAFETY,
  STATUS_THROTTLE_SAFETY,
  STATUS_ARM,
  STATUS_DISARM,
  STATUS_LOW_BAT,
  STATUS_MOTOR_TEST,
  STATUS_TURTLE,
  STATUS_AUTOLAUNCH,
  STATUS_AUTOTRIM,
  STATUS_RTH,
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
  uint8_t label[LABEL_MAX_LEN + 1];
  uint8_t label_len;
  uint8_t scroll_offset;
  uint32_t last_scroll_ms;
} osd_status_t;

static osd_status_t current_status = {
    .entry = STATUS_MAX,
    .state = PRINT_IDLE,
};

const char *default_system_status_labels[STATUS_MAX] = {
    [STATUS_RX_WAIT] = "WAIT FOR RX",
    [STATUS_FAILSAFE] = "**FAILSAFE**",
    [STATUS_LOOPTIME] = "**LOOPTIME**",
    [STATUS_USB_SAFETY] = "USB SAFETY",
    [STATUS_ARM_SAFETY] = "ARMING SAFETY",
    [STATUS_THROTTLE_SAFETY] = "THROTTLE SAFETY",
    [STATUS_ARM] = "**ARMED**",
    [STATUS_DISARM] = "**DISARMED**",
    [STATUS_LOW_BAT] = "**LOW BATTERY**",
    [STATUS_MOTOR_TEST] = "**MOTOR TEST**",
    [STATUS_TURTLE] = "**TURTLE**",
    [STATUS_AUTOLAUNCH] = "AUTO LAUNCH",
    [STATUS_AUTOTRIM] = "AUTOTRIM",
    [STATUS_RTH] = "RTH",
};

const char *guac_system_status_labels[STATUS_MAX] = {
    [STATUS_RX_WAIT] = "RX LOADING",
    [STATUS_FAILSAFE] = "**404 RX NOT FOUND**",
    [STATUS_LOOPTIME] = "**LOOPTIME**",
    [STATUS_USB_SAFETY] = "**USB SAFETY**",
    [STATUS_ARM_SAFETY] = "**ARMING SAFETY**",
    [STATUS_THROTTLE_SAFETY] = "**DANGER ZONE**",
    [STATUS_ARM] = "**HERE WE GO AGAIN**",
    [STATUS_DISARM] = "**GAME OVER**",
    [STATUS_LOW_BAT] = "CONSTRUCT MORE PYLONS",
    [STATUS_MOTOR_TEST] = "**MOTOR TEST**",
    [STATUS_TURTLE] = "\x60THIS SIDE UP\x60",
    [STATUS_AUTOLAUNCH] = "AUTO LAUNCH",
    [STATUS_AUTOTRIM] = "AUTOTRIM",
    [STATUS_RTH] = "RTH",
};

static void osd_status_write_label(osd_element_t *el) {
  osd_start(osd_attr(el) | OSD_ATTR_BLINK, pos_x(el), pos_y(el));
  if (current_status.label_len <= LABEL_LEN) {
    const uint32_t offset = (LABEL_LEN - current_status.label_len) / 2;
    for (uint32_t i = 0; i < offset; i++) {
      osd_write_char(' ');
    }
    osd_write_data(current_status.label, current_status.label_len);
    for (uint32_t i = offset + current_status.label_len; i < LABEL_LEN; i++) {
      osd_write_char(' ');
    }
    return;
  }

  const uint8_t scroll_len = current_status.label_len + SCROLL_PADDING;
  for (uint32_t i = 0; i < LABEL_LEN; i++) {
    const uint8_t pos = (current_status.scroll_offset + i) % scroll_len;
    osd_write_char(pos < current_status.label_len ? current_status.label[pos] : ' ');
  }
}

static void osd_status_show_text(osd_status_mode_t mode, osd_status_entries_t entry, const char *label) {
  if (current_status.entry == entry && strcmp((const char *)current_status.label, label) == 0) {
    return;
  }

  current_status.entry = entry;
  current_status.state = PRINT_START;
  current_status.mode = mode;
  current_status.scroll_offset = 0;
  current_status.last_scroll_ms = 0;

  const uint32_t len = strlen(label);
  current_status.label_len = len > LABEL_MAX_LEN ? LABEL_MAX_LEN : len;
  memset(current_status.label, 0, sizeof(current_status.label));
  memcpy(current_status.label, label, current_status.label_len);
}

static void osd_status_show(osd_status_mode_t mode, osd_status_entries_t entry) {
  const char **labels = profile.osd.guac_mode ? guac_system_status_labels : default_system_status_labels;
  osd_status_show_text(mode, entry, labels[entry]);
}

#ifdef VEHICLE_WING
static const char *osd_wing_autolaunch_message(void) {
  if (!rx_aux_on(AUX_AUTOLAUNCH)) {
    return NULL;
  }
  if (!flags.arm_state) {
    return "AUTO LAUNCH: ARM FIRST";
  }
  if (!state.wing_launch_available) {
    return "AUTO LAUNCH: ENABLE BEFORE ARMING";
  }
  if (state.rx_filtered.throttle < THROTTLE_SAFETY) {
    return "AUTO LAUNCH: RAISE THROTTLE";
  }

  switch (state.wing_launch_state) {
  case WING_LAUNCH_IDLE:
  case WING_LAUNCH_IDLE_DELAY:
    return "AUTO LAUNCH: PREPARING";

  case WING_LAUNCH_WAIT:
    return "AUTO LAUNCH: THROW NOW";

  case WING_LAUNCH_DETECTED:
    return "AUTO LAUNCH: DETECTED";

  case WING_LAUNCH_MOTOR_DELAY:
  case WING_LAUNCH_SPINUP:
  case WING_LAUNCH_ACTIVE:
    return "AUTO LAUNCH: MOVE STICKS TO ABORT";

  case WING_LAUNCH_FINISH:
    return "AUTO LAUNCH: TAKE CONTROL";

  case WING_LAUNCH_DONE:
    return "AUTO LAUNCH DONE";

  case WING_LAUNCH_ABORTED:
    return "AUTO LAUNCH ABORTED";
  }
  return NULL;
}

static bool osd_wing_autolaunch_message_temp(void) {
  return state.wing_launch_state == WING_LAUNCH_DONE || state.wing_launch_state == WING_LAUNCH_ABORTED;
}

static const char *osd_wing_autotrim_message(void) {
  if (state.wing_autotrim_state == WING_AUTOTRIM_SAVED) {
    return "AUTOTRIM DONE";
  }
  if (!rx_aux_on(AUX_AUTOTRIM)) {
    return NULL;
  }
  if (!flags.arm_state) {
    return "AUTOTRIM: ARM FIRST";
  }
  if (flags.failsafe) {
    return "AUTOTRIM: FAILSAFE";
  }

  switch (state.wing_autotrim_state) {
  case WING_AUTOTRIM_SAVE_PENDING:
    return "AUTOTRIM: DISARM TO SAVE / AUX OFF CANCEL";

  case WING_AUTOTRIM_ACTIVE:
    return "AUTOTRIM: HOLD LEVEL";

  case WING_AUTOTRIM_IDLE:
    return "AUTOTRIM: CYCLE AUX";

  case WING_AUTOTRIM_SAVED:
    return "AUTOTRIM DONE";
  }
  return NULL;
}

static bool osd_wing_autotrim_message_temp(void) {
  return state.wing_autotrim_state == WING_AUTOTRIM_SAVED;
}
#endif

#ifdef VEHICLE_MULTI
static const char *osd_rth_message() {
  if (state.rth_state == RTH_STATE_ABORTED) {
    return "ABORTED";
  }

  if (state.rth_active) {
    switch (state.rth_state) {
    case RTH_STATE_CLIMB:
      return "CLIMB";
    case RTH_STATE_ACQUIRE_HEADING:
      return "ACQUIRING HEADING";
    case RTH_STATE_TURN:
      return "TURN HOME";
    case RTH_STATE_NAVIGATE:
      return "RETURN HOME";
    case RTH_STATE_HOVER_HOME:
      return "HOLD HOME";
    case RTH_STATE_HEADING_FAILED:
      return "HEADING FAILED";
    }
  }

  if (rx_aux_on(AUX_RETURN_TO_HOME)) {
    if (!flags.arm_state) return "ARM FIRST";
    return "UNAVAILABLE";
  }
  return NULL;
}
#endif

static bool osd_status_print(osd_element_t *el) {
  static uint32_t start_time;

  switch (current_status.state) {
  case PRINT_START:
    osd_status_write_label(el);

    if (current_status.mode == MODE_HOLD) {
      current_status.state = PRINT_IDLE;
    } else {
      current_status.state = PRINT_WAIT;
    }
    start_time = time_millis();
    current_status.last_scroll_ms = start_time;
    return false;

  case PRINT_WAIT:
    if ((time_millis() - start_time) < 1000) {
      return true;
    }
    current_status.state = PRINT_CLEAR;
    return false;

  case PRINT_CLEAR:
    osd_start(osd_attr(el) | OSD_ATTR_BLINK, pos_x(el), pos_y(el));
    for (uint32_t i = 0; i < LABEL_LEN; i++) {
      osd_write_char(' ');
    }
    current_status.state = PRINT_IDLE;
    return false;

  case PRINT_IDLE:
    if (current_status.mode == MODE_HOLD && current_status.label_len > LABEL_LEN) {
      const uint32_t now_ms = time_millis();
      if (now_ms - current_status.last_scroll_ms >= SCROLL_INTERVAL_MS) {
        current_status.last_scroll_ms = now_ms;
        current_status.scroll_offset = (current_status.scroll_offset + 1) % (current_status.label_len + SCROLL_PADDING);
        osd_status_write_label(el);
        return false;
      }
    }
    return true;
  }

  return true;
}

void osd_status_reset() {
  current_status.entry = STATUS_MAX;
  current_status.state = PRINT_IDLE;
}

bool osd_status_update(osd_element_t *el) {
  if (!flags.rx_ready) {
    osd_status_show(MODE_HOLD, STATUS_RX_WAIT);
    return osd_status_print(el);
  }

  if (flags.failsafe && flags.rx_ready) {
    // only show failsafe if rx was ready
#ifdef VEHICLE_MULTI
    const char *rth_message = osd_rth_message();
    if (rth_message && (state.rth_failsafe_active || state.rth_state == RTH_STATE_ABORTED)) {
      osd_status_show_text(MODE_HOLD, STATUS_RTH, rth_message);
      return osd_status_print(el);
    }
#endif
    osd_status_show(MODE_HOLD, STATUS_FAILSAFE);
    return osd_status_print(el);
  }

  if (state.looptime_warning && state.looptime_autodetect > 125.0f) {
    osd_status_show(MODE_HOLD, STATUS_LOOPTIME);
    return osd_status_print(el);
  }

  if (flags.arming_disabled_flags & ARMING_DISABLED_USB) {
    osd_status_show(MODE_HOLD, STATUS_USB_SAFETY);
    return osd_status_print(el);
  }

  if (flags.arming_disabled_flags & ARMING_DISABLED_ARM_SWITCH) {
    osd_status_show(MODE_HOLD, STATUS_ARM_SAFETY);
    return osd_status_print(el);
  }

  if (flags.arming_disabled_flags & ARMING_DISABLED_THROTTLE) {
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

#ifdef VEHICLE_WING
  const char *autolaunch_message = osd_wing_autolaunch_message();
  if (autolaunch_message) {
    osd_status_show_text(osd_wing_autolaunch_message_temp() ? MODE_TEMP : MODE_HOLD, STATUS_AUTOLAUNCH, autolaunch_message);
    return osd_status_print(el);
  }

  const char *autotrim_message = osd_wing_autotrim_message();
  if (autotrim_message) {
    osd_status_show_text(osd_wing_autotrim_message_temp() ? MODE_TEMP : MODE_HOLD, STATUS_AUTOTRIM, autotrim_message);
    return osd_status_print(el);
  }
#endif

#if !defined(VEHICLE_ROVER) && !defined(VEHICLE_WING)
  if (rx_aux_on(AUX_MOTOR_TEST)) {
    osd_status_show(MODE_HOLD, STATUS_MOTOR_TEST);
    return osd_status_print(el);
  }
#endif

  if (flags.turtle) {
    osd_status_show(MODE_HOLD, STATUS_TURTLE);
    return osd_status_print(el);
  }

#ifdef VEHICLE_MULTI
  const char *rth_message = osd_rth_message();
  if (rth_message) {
    osd_status_show_text(MODE_HOLD, STATUS_RTH, rth_message);
    return osd_status_print(el);
  }
#endif

  if (current_status.mode != MODE_TEMP && current_status.entry != STATUS_MAX) {
    current_status.entry = STATUS_MAX;
    current_status.state = PRINT_CLEAR;
  }
  return osd_status_print(el);
}
