#include "flight/gestures.h"

#include "core/flash.h"
#include "core/looptime.h"
#include "core/profile.h"
#include "core/scheduler.h"
#include "flight/control.h"
#include "flight/pid.h"
#include "flight/sixaxis.h"
#include "io/led.h"
#include "rx/rx.h"
#include "util/util.h"

void gestures() {
  if (!flags.on_ground || flags.gestures_disabled) {
    return;
  }

  static bool skip_calib = false;

  const int32_t command = gestures_detect();
  if (command == GESTURE_NONE) {
    return;
  }

  if (osd_state.screen != OSD_SCREEN_REGULAR) {
    return osd_handle_input(command);
  }

  switch (command) {
  case GESTURE_DDD: {
    // skip accel calibration if pid gestures used
    if (!skip_calib) {
      sixaxis_gyro_cal(); // for flashing lights
      sixaxis_acc_cal();
    } else {
      led_flash();
      skip_calib = false;
    }

    flash_save();
    flash_load();

    // reset loop time
    task_reset_runtime();
    break;
  }
  case GESTURE_UUU: {
    bind_storage.bind_saved = !bind_storage.bind_saved;
    skip_calib = true;
    led_flash();
    break;
  }
  case GESTURE_RRR: {
    osd_push_screen(OSD_SCREEN_MAIN_MENU);
    led_flash();
    break;
  }
  case GESTURE_RRD: {
    state.aux[AUX_CHANNEL_GESTURE] = 1;
    led_flash();
    break;
  }
  case GESTURE_LLD: {
    state.aux[AUX_CHANNEL_GESTURE] = 0;
    led_flash();
    break;
  }
  case GESTURE_LRL: {
    while (osd_pop_screen() != OSD_SCREEN_CLEAR)
      ;
    break;
  }
  }
}
