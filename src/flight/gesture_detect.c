#include "flight/gestures.h"

#include <math.h>
#include <stdint.h>

#include "core/project.h"
#include "driver/time.h"
#include "flight/control.h"
#include "osd/render.h"

#define STICKMAX 0.7f
#define STICKCENTER 0.2f

#define GMACRO_LEFT (state.rx.axis[0] < -STICKMAX || state.rx.axis[2] < -STICKMAX)
#define GMACRO_RIGHT (state.rx.axis[0] > STICKMAX || state.rx.axis[2] > STICKMAX)
#define GMACRO_XCENTER (fabsf(state.rx.axis[0]) < STICKCENTER && fabsf(state.rx.axis[2]) < STICKCENTER)

#define GMACRO_DOWN (state.rx.axis[1] < -STICKMAX)
#define GMACRO_UP (state.rx.axis[1] > STICKMAX)

#define GMACRO_PITCHCENTER (fabsf(state.rx.axis[1]) < STICKCENTER)

#define GESTURE_CENTER 0
#define GESTURE_CENTER_IDLE 12
#define GESTURE_LEFT 1
#define GESTURE_RIGHT 2
#define GESTURE_DOWN 3
#define GESTURE_UP 4
#define GESTURE_OTHER 127
#define GESTURE_LONG 255

#define GESTURETIME_MIN 50e3
#define GESTURETIME_MAX 500e3
#define GESTURETIME_IDLE 700e3
#define GESTURETIME_IDLE_OSD 100e3

#define GSIZE 7
#define OSD_GSIZE 3

// L L D
const uint8_t command1[GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_LEFT, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER};

// R R D
const uint8_t command2[GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER};

// D D D
const uint8_t command3[GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER};

// GESTURES FOR PID TUNING

#ifdef PID_GESTURE_TUNING
// U D U - Next PID term
const uint8_t command4[GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER};

// U D D - Next PID Axis
const uint8_t command5[GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER};

// U D R - Increase value
const uint8_t command6[GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER};

// U D L - Decrease value
const uint8_t command7[GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER};
#endif

// NFE ADDED GESTURES

// U U U
const uint8_t command8[GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER};

// R R R
const uint8_t command9[GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER};

// L L L
const uint8_t command10[GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_LEFT, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER};

// D U D
const uint8_t command11[GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER};

//  OSD GESRURES

// OSD-UP
const uint8_t command12[OSD_GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER};

// OSD-DOWN
const uint8_t command13[OSD_GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER};

// OSD-RIGHT
const uint8_t command14[OSD_GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_RIGHT, GESTURE_CENTER};

// OSD-LEFT
const uint8_t command15[OSD_GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_LEFT, GESTURE_CENTER};

const uint8_t command16[GSIZE] = {GESTURE_CENTER_IDLE, GESTURE_LEFT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER};

int gesture_start;
int lastgesture;
int setgesture;
static unsigned gesturetime;

int gestures2() {
  if (flags.on_ground) {
    if (GMACRO_XCENTER && GMACRO_PITCHCENTER) {
      gesture_start = GESTURE_CENTER;
    } else if (GMACRO_LEFT && GMACRO_PITCHCENTER) {
      gesture_start = GESTURE_LEFT;
    } else if (GMACRO_RIGHT && GMACRO_PITCHCENTER) {
      gesture_start = GESTURE_RIGHT;
    } else if (GMACRO_DOWN && GMACRO_XCENTER) {
      gesture_start = GESTURE_DOWN;
    } else if (GMACRO_UP && GMACRO_XCENTER) {
      gesture_start = GESTURE_UP;
    } else {
      //      gesture_start = GESTURE_OTHER;
    }

    uint32_t time = time_micros();

    if (gesture_start != lastgesture) {
      gesturetime = time;
    }

    if (time - gesturetime > GESTURETIME_MIN) {
      int gesturetime_idle;

      if (osd_state.screen != OSD_SCREEN_REGULAR)
        gesturetime_idle = GESTURETIME_IDLE_OSD;
      else
        gesturetime_idle = GESTURETIME_IDLE;

      if ((gesture_start == GESTURE_CENTER) && (time - gesturetime > gesturetime_idle)) {
        setgesture = GESTURE_CENTER_IDLE;
      } else if (time - gesturetime > GESTURETIME_MAX) {
        if ((gesture_start != GESTURE_OTHER))
          setgesture = GESTURE_LONG;
      } else {
        setgesture = gesture_start;
      }
    }

    lastgesture = gesture_start;

    return gesture_sequence(setgesture);

  } else {
    setgesture = GESTURE_OTHER;
    lastgesture = GESTURE_OTHER;
  }

  return 0;
}

//  LEAVING OFF HERE  - need a way to change gbuffer size @ runtime
uint8_t gbuffer[GSIZE];

uint8_t check_command(uint8_t buffer1[], const uint8_t command[]) {
  if (osd_state.screen != OSD_SCREEN_REGULAR) {
    for (int i = 0; i < OSD_GSIZE; i++) {
      if (buffer1[i] != command[OSD_GSIZE - i - 1])
        return 0;
    }
  } else {
    for (int i = 0; i < GSIZE; i++) {
      if (buffer1[i] != command[GSIZE - i - 1])
        return 0;
    }
  }
  return 1;
}

int gesture_sequence(int currentgesture) {

  if (currentgesture != gbuffer[0]) { // add to queue
    if (osd_state.screen != OSD_SCREEN_REGULAR) {
      for (int i = OSD_GSIZE - 1; i >= 1; i--) {
        gbuffer[i] = gbuffer[i - 1];
      }
    } else {
      for (int i = GSIZE - 1; i >= 1; i--) {
        gbuffer[i] = gbuffer[i - 1];
      }
    }
    gbuffer[0] = currentgesture;

    // check commands
    if (osd_state.screen != OSD_SCREEN_REGULAR) {
      if (check_command(&gbuffer[0], &command12[0])) {
        // command 12

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_OSD_UP;
      }

      if (check_command(&gbuffer[0], &command13[0])) {
        // command 13

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_OSD_DOWN;
      }

      if (check_command(&gbuffer[0], &command14[0])) {
        // command 14

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_OSD_RIGHT;
      }

      if (check_command(&gbuffer[0], &command15[0])) {
        // command 15

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_OSD_LEFT;
      }

    } else {
      if (check_command(&gbuffer[0], &command16[0])) {
        // command 16

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_LRL;
      }

      if (check_command(&gbuffer[0], &command1[0])) {
        // command 1

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_LLD;
      }

      if (check_command(&gbuffer[0], &command2[0])) {
        // command 2

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_RRD;
      }

      if (check_command(&gbuffer[0], &command3[0])) {
        // command 3

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_DDD;
      }
      if (check_command(&gbuffer[0], &command8[0])) {
        // command 8

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_UUU;
      }

      if (check_command(&gbuffer[0], &command9[0])) {
        // command 9

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_RRR;
      }

      if (check_command(&gbuffer[0], &command10[0])) {
        // command 10

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_LLL;
      }

      if (check_command(&gbuffer[0], &command11[0])) {
        // command 11

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_DUD;
      }

#ifdef PID_GESTURE_TUNING
      if (check_command(&gbuffer[0], &command4[0])) {
        // command 4

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_UDU;
      }

      if (check_command(&gbuffer[0], &command5[0])) {
        // command 5

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_UDD;
      }

      if (check_command(&gbuffer[0], &command6[0])) {
        // command 6

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_UDR;
      }
      if (check_command(&gbuffer[0], &command7[0])) {
        // command 7

        // change buffer so it does not trigger again
        gbuffer[1] = GESTURE_OTHER;
        return GESTURE_UDL;
      }
#endif
    }
  }

  return GESTURE_NONE;
}
