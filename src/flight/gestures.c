#include "flight/gestures.h"

#include "core/flash.h"
#include "core/looptime.h"
#include "core/profile.h"
#include "flight/control.h"
#include "flight/pid.h"
#include "flight/sixaxis.h"
#include "rx/rx.h"
#include "util/util.h"

extern int ledcommand;
extern int ledblink;
extern profile_t profile;

bool pid_gestures_used = false;

void gestures() {
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
    if (!pid_gestures_used) {
      sixaxis_gyro_cal(); // for flashing lights
      sixaxis_acc_cal();
    } else {
      ledcommand = 1;
      pid_gestures_used = false;
    }

    flash_save();
    flash_load();

    // reset flash numbers
    extern int number_of_increments[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        number_of_increments[i][j] = 0;

    // reset loop time
    looptime_reset();
    break;
  }
  case GESTURE_UUU: {
    bind_storage.bind_saved = !bind_storage.bind_saved;
    ledblink = 2 - bind_storage.bind_saved;
    pid_gestures_used = true;
    break;
  }
  case GESTURE_RRR: {
    ledblink = 2 - osd_push_screen(OSD_SCREEN_MAIN_MENU);
    break;
  }
  case GESTURE_RRD: {
    state.aux[AUX_CHANNEL_GESTURE] = 1;
    ledcommand = 1;
    break;
  }
  case GESTURE_LLD: {
    state.aux[AUX_CHANNEL_GESTURE] = 0;
    ledcommand = 1;
    break;
  }
  case GESTURE_LRL: {
    while (osd_pop_screen() != OSD_SCREEN_CLEAR)
      ;
    break;
  }
#ifdef PID_GESTURE_TUNING
  case GESTURE_UDU: {
    // Cycle to next pid term (P I D)
    ledblink = next_pid_term();
    pid_gestures_used = true;
    break;
  }
  case GESTURE_UDD: {
    // Cycle to next axis (Roll Pitch Yaw)
    ledblink = next_pid_axis();
    pid_gestures_used = true;
    break;
  }
  case GESTURE_UDR: {
    // Increase by 10%
    ledblink = increase_pid();
    pid_gestures_used = true;
    break;
  }
  case GESTURE_UDL: {
    // Descrease by 10%
    ledblink = decrease_pid();
    pid_gestures_used = true;
    break;
  }
#endif
  }

  // flash long on zero
  if (pid_gestures_used && ledblink == 0)
    ledcommand = 1;
}
