#include "flight/gestures.h"

#include "core/flash.h"
#include "core/looptime.h"
#include "core/profile.h"
#include "flight/control.h"
#include "flight/pid.h"
#include "flight/sixaxis.h"
#include "osd/render.h"
#include "rx/rx.h"
#include "util/util.h"

extern int ledcommand;
extern int ledblink;
extern profile_t profile;

int pid_gestures_used = 0;

void gestures() {
  int command = gestures2();

  if (command != GESTURE_NONE) {
    if (command == GESTURE_DDD) {

      // skip accel calibration if pid gestures used
      if (!pid_gestures_used) {
        sixaxis_gyro_cal(); // for flashing lights
        sixaxis_acc_cal();
      } else {
        ledcommand = 1;
        pid_gestures_used = 0;
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
    }

    if (command == GESTURE_DUD) {
      profile.motor.invert_yaw = !profile.motor.invert_yaw;
      ledblink = 2 - profile.motor.invert_yaw;
      pid_gestures_used = 1;
    }

    if (command == GESTURE_UUU) {
      bind_storage.bind_saved = !bind_storage.bind_saved;
      ledblink = 2 - bind_storage.bind_saved;
      pid_gestures_used = 1;
    }

    if (command == GESTURE_RRR) {
      ledblink = 2 - osd_push_screen(OSD_SCREEN_MAIN_MENU);
    }

    if (command == GESTURE_LLL) {
      flash_storage.flash_feature_1 = !flash_storage.flash_feature_1;
      ledblink = 1 - flash_storage.flash_feature_1;
      pid_gestures_used = 1;

#ifdef LVC_LOWER_THROTTLE
      flash_storage.lvc_lower_throttle = !flash_storage.lvc_lower_throttle;
      ledblink = 2 - flash_storage.lvc_lower_throttle;
      pid_gestures_used = 1;
#endif
    }

    if (command == GESTURE_RRD) {
      state.aux[AUX_CHANNEL_GESTURE] = 1;
      ledcommand = 1;
    }
    if (command == GESTURE_LLD) {
      ledcommand = 1;
      state.aux[AUX_CHANNEL_GESTURE] = 0;
    }

    if (command == GESTURE_LRL) {
      while (osd_pop_screen() != OSD_SCREEN_CLEAR)
        ;
    }

    if (command == GESTURE_OSD_UP) {
      osd_handle_input(OSD_INPUT_UP);
    }

    if (command == GESTURE_OSD_DOWN) {
      osd_handle_input(OSD_INPUT_DOWN);
    }

    if (command == GESTURE_OSD_RIGHT) {
      osd_handle_input(OSD_INPUT_RIGHT);
    }

    if (command == GESTURE_OSD_LEFT) {
      osd_handle_input(OSD_INPUT_LEFT);
    }

#ifdef PID_GESTURE_TUNING
    if (command >= GESTURE_UDR)
      pid_gestures_used = 1;

    if (command == GESTURE_UDU) {
      // Cycle to next pid term (P I D)
      ledblink = next_pid_term();
    }
    if (command == GESTURE_UDD) {
      // Cycle to next axis (Roll Pitch Yaw)
      ledblink = next_pid_axis();
    }
    if (command == GESTURE_UDR) {
      // Increase by 10%
      ledblink = increase_pid();
    }
    if (command == GESTURE_UDL) {
      // Descrease by 10%
      ledblink = decrease_pid();
    }
    // flash long on zero
    if (pid_gestures_used && ledblink == 0)
      ledcommand = 1;

      // U D U - Next PID term
      // U D D - Next PID Axis
      // U D R - Increase value
      // U D L - Descrease value
      // ledblink = blink; //Will cause led logic to blink the number of times ledblink has stored in it.
#endif
  }
}
