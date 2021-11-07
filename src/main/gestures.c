#include "gestures.h"

#include "control.h"
#include "flash.h"
#include "osd_adjust.h"
#include "pid.h"
#include "profile.h"
#include "rx.h"
#include "sixaxis.h"
#include "util.h"

extern int ledcommand;
extern int ledblink;
extern profile_t profile;

int pid_gestures_used = 0;

void gestures(void) {
  int command = gestures2();

  if (command != GESTURE_NONE) {
    if (command == GESTURE_DDD) {

      //skip accel calibration if pid gestures used
      if (!pid_gestures_used) {
        gyro_cal(); // for flashing lights
        acc_cal();
      } else {
        //#ifdef ENABLE_OSD
        //extern uint8_t osd_display_phase;
        //osd_display_phase = 0;						//Turn off menu
        //flash_storage.flash_feature_1= 0;							//set flash status for setup wizard to OFF
        //#endif
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
      reset_looptime();
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
#ifdef ENABLE_OSD
      extern uint8_t osd_display_phase;
      osd_display_phase--;
      ledblink = 2 - osd_display_phase;
//pid_gestures_used = 1;
#endif
    }

    if (command == GESTURE_LLL) {
      flash_storage.flash_feature_1 = !flash_storage.flash_feature_1;
      ledblink = 1 - flash_storage.flash_feature_1;
      pid_gestures_used = 1;
#ifdef SWITCHABLE_FEATURE_2
      flash_storage.flash_feature_2 = !flash_storage.flash_feature_2;
      ledblink = 2 - flash_storage.flash_feature_2;
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

#ifdef ENABLE_OSD
    if (command == GESTURE_OSD_UP) {
      extern uint8_t osd_menu_phase;
      extern uint8_t osd_cursor;
      extern uint8_t osd_select;
      if (osd_select) {
        extern uint8_t increase_osd_value;
        increase_osd_value = 1;
      } else {
        osd_cursor--;
        osd_menu_phase = 1;
        ledblink = 1;
      }
    }

    if (command == GESTURE_OSD_DOWN) {
      extern uint8_t osd_menu_phase;
      extern uint8_t osd_cursor;
      extern uint8_t osd_select;
      if (osd_select) {
        extern uint8_t decrease_osd_value;
        decrease_osd_value = 1;
      } else {
        osd_menu_phase = 1;
        osd_cursor++;
        ledblink = 1;
      }
    }

    if (command == GESTURE_OSD_RIGHT) {
      extern uint8_t osd_select;
      extern uint8_t osd_menu_phase;
      osd_select++;
      osd_menu_phase = 1;
      ledblink = 2;
    }

    if (command == GESTURE_OSD_LEFT) {
      extern uint8_t osd_cursor;
      extern uint8_t osd_display_phase;
      extern uint8_t osd_menu_phase;
      extern uint8_t osd_select;
      extern uint8_t last_display_phase;
      if (osd_select) {
        osd_select--;
        osd_menu_phase = 1;
      } else {
        osd_cursor = last_cursor_array_stuffer(osd_cursor, RETURN_VALUE); //this tracks like last display phase
        if (osd_display_phase > 2) {
          osd_display_phase = last_display_phase;
          osd_menu_phase = 0;
        } else {
          osd_display_phase--;
        }
        ledblink = 2 - osd_display_phase;
        pid_gestures_used = 0;
      }
    }
#endif

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
