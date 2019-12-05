#include "flip_sequencer.h"

#include <math.h>

#include "defines.h"
#include "drv_time.h"
#include "profile.h"

int NFE;

#define STANDARD_TURTLE
#define TURTLE_TIMEOUT 1e6	//todo:  1 second timeout for auto turtle


#define THROTTLE_UP 1.0		//todo:  needs to be adjustable
#define THROTTLE_HOVER 0.5
#define THROTTLE_EXIT 0.6

#define FLIP_RATE 1500
//time to up throttle in beginning stage
// 200ms
#define THROTTLE_UP_TIME 200e3
// end time in level mode
#define LEVEL_MODE_TIME 200e3
#define LEVEL_MODE_ANGLE -0

#define FLIP_TIMEOUT_TOTAL 1500e3
#define FLIP_TIMEOUT_STAGE1 500e3

// don't change below

#define STAGE_FLIP_NONE 0
#define STAGE_FLIP_START 1
#define STAGE_FLIP_THROTTLEUP 2
#define STAGE_FLIP_ROTATING 3
#define STAGE_FLIP_ROTATING_INVERTED 4
#define STAGE_FLIP_LEVELMODE 5
#define STAGE_FLIP_EXIT 6

int motortest_override;
int acro_override = 0;
int level_override = 0;
int controls_override = 0;
unsigned long fliptime;
int readytoflip = 0;
int flipstage = STAGE_FLIP_NONE;
unsigned int levelmodetime;
int flipindex = 0;
int flipdir = 0;

extern profile_t profile;
extern int onground;
extern float GEstG[3];
extern float rx[];
extern profile_t profile;
extern int pwmdir;

float rx_override[4];


void start_flip() {
#ifdef STANDARD_TURTLE
	if (!readytoflip && onground) {  //if not currently queued up for a turtle sequence and disarmed
	    readytoflip = 1;					//queue up for a turtle event
	    flipstage = STAGE_FLIP_NONE;
	}
#endif

#ifdef AUTOMATED_FLIP		//depreciated - needs a proper going through if it is to be restored
  if (readytoflip == 0 && !onground) {
    readytoflip = 1;
    fliptime = gettime();
    flipstage = STAGE_FLIP_START;
  }

  flipindex = 0;
  flipdir = 0;
  if (fabsf(rx[0]) < fabsf(rx[1])) {
    flipindex = 1;
    if (rx[1] > 0)
      flipdir = 1;
  } else if (rx[0] > 0)
    flipdir = 1;
#endif
}

void flip_sequencer() {
#ifdef STANDARD_TURTLE
	if (!readytoflip){						//turtle can't be initiated without the all clear flag - hold control variables at 0 state
		if (flipstage != STAGE_FLIP_NONE)
			pwmdir = FORWARD;				//forward pwmdir only once as its last state may be unknown from previously interrupted turtle event
		flipstage = STAGE_FLIP_NONE;
		controls_override = 0;
		motortest_override = 0;
		return;								//turtle mode off or flying away from a successful turtle will return here
	}										// a disarmed quad with turtle mode on will continue past


//  track the change of onground and flag a potential turtle trigger event only on disarmed to armed event.
	int turtle_trigger = 0;
	static int last_onground = 0;
	if (onground != last_onground){
		last_onground = onground;
		if (!onground)						//quad was just armed - set the turtle_trigger flag to ready
			turtle_trigger = 1;				//trigger will reinit to 0 next go round
		NFE++;
	}


	if((GEstG[2] < 0) && turtle_trigger){	//begin the turtle sequence only once and with turtle_trigger flag ready and while upside down.
		flipstage = STAGE_FLIP_START;
	}

	if(GEstG[2] > 0.5f){					//exit the sequence if you failed to turtle, picked up the quad, and flipped it over your damn self
		flipstage = STAGE_FLIP_EXIT;
	}

	switch (flipstage) {
	  case STAGE_FLIP_NONE:
	    break;

	  case STAGE_FLIP_START:
		controls_override = 1;
		rx_override[0] = 0;
		rx_override[1] = 0;
		rx_override[2] = 0;
		rx_override[3] = 0;
		motortest_override = 1;
		pwmdir = REVERSE;
		flipindex = 0;
		flipdir = 0;
		if (fabsf(rx[0]) > 0.5f || fabsf(rx[1]) > 0.5f){
			if (fabsf(rx[0]) < fabsf(rx[1])) {
				flipindex = 1;
				if (rx[1] > 0)
					flipdir = 1;
				flipstage = STAGE_FLIP_ROTATING;
				fliptime = gettime();
			}else{
				if (rx[0] > 0)
					flipdir = 1;
				flipstage = STAGE_FLIP_ROTATING;
				fliptime = gettime();
			}
		}
		break;

	  case STAGE_FLIP_ROTATING:
		rx_override[3] = profile.motor.turtle_throttle_percent/100.0f;
	    if (flipdir){
	        rx_override[flipindex] = 1.0f;
	    }else{
	        rx_override[flipindex] = -1.0f;
	    }
		if (gettime() - fliptime > TURTLE_TIMEOUT)
			flipstage = STAGE_FLIP_START;
		if (GEstG[2] > 0.50f)
			flipstage = STAGE_FLIP_EXIT;
		break;

	  case STAGE_FLIP_EXIT:
		readytoflip = 0;
		flipstage = STAGE_FLIP_NONE;
		controls_override = 0;
		motortest_override = 0;
		pwmdir = FORWARD;
		break;
	}
#endif


#ifdef AUTOMATED_FLIP
  if (!readytoflip)
    return;

  if (onground)
    flipstage = STAGE_FLIP_EXIT;

  if (readytoflip && gettime() - fliptime > FLIP_TIMEOUT_TOTAL) {
    // abort after timeout second
    flipstage = STAGE_FLIP_EXIT;
  }

  float max_rate = 860.0f;
  if (profile.rate.mode == RATE_MODE_BETAFLIGHT) {
    max_rate = 200 * profile.rate.betaflight.rc_rate.roll * (1 / 1 - (float)profile.rate.betaflight.super_rate.roll);
  } else {
    max_rate = (profile.rate.silverware.max_rate.roll + profile.rate.silverware.max_rate.pitch) / 2;
  }

  switch (flipstage) {
  case STAGE_FLIP_NONE:

    break;

  case STAGE_FLIP_START:
    //
    acro_override = 1;
    controls_override = 1;

    rx_override[1] = 0;
    rx_override[2] = 0;
    rx_override[3] = THROTTLE_UP;

    if (GEstG[2] < 0) {
      // flip initiated inverted
      if (flipdir)
        rx_override[flipindex] = (float)FLIP_RATE / max_rate;
      else
        rx_override[flipindex] = (float)-FLIP_RATE / max_rate;
      rx_override[3] = THROTTLE_HOVER;
      flipstage = STAGE_FLIP_ROTATING_INVERTED;
    }
    flipstage = STAGE_FLIP_THROTTLEUP;

    break;

  case STAGE_FLIP_THROTTLEUP:

    if (gettime() - fliptime > THROTTLE_UP_TIME) {
      if (flipdir)
        rx_override[flipindex] = (float)FLIP_RATE / max_rate;
      else
        rx_override[flipindex] = (float)-FLIP_RATE / max_rate;
      rx_override[3] = THROTTLE_UP;
      flipstage = STAGE_FLIP_ROTATING;
    }

    break;

  case STAGE_FLIP_ROTATING:
    if (gettime() - fliptime > FLIP_TIMEOUT_STAGE1 + THROTTLE_UP_TIME) {
      // abort
      flipstage = STAGE_FLIP_EXIT;
    }
    if (GEstG[2] < 0) {
      //we are inverted
      rx_override[3] = THROTTLE_HOVER;
      flipstage = STAGE_FLIP_ROTATING_INVERTED;
    }
    break;

  case STAGE_FLIP_ROTATING_INVERTED:
    if (GEstG[2] > 0) {

      //we no longer inverted
      levelmodetime = gettime();

      rx_override[3] = THROTTLE_EXIT;
      acro_override = 0;
      level_override = 1;
      flipstage = STAGE_FLIP_LEVELMODE;
    }
    break;

  case STAGE_FLIP_LEVELMODE:
    // allow control in other axis at this point
    rx_override[0] = rx[0];
    rx_override[1] = rx[1];
    rx_override[2] = rx[2];

    if (flipdir)
      rx_override[flipindex] = (float)LEVEL_MODE_ANGLE / profile.rate.level_max_angle;
    else
      rx_override[flipindex] = (float)-LEVEL_MODE_ANGLE / profile.rate.level_max_angle;
    if (gettime() - levelmodetime > LEVEL_MODE_TIME)
      flipstage = STAGE_FLIP_EXIT;
    break;

  case STAGE_FLIP_EXIT:
    readytoflip = 0;
    flipstage = STAGE_FLIP_NONE;
    acro_override = 0;
    level_override = 0;
    controls_override = 0;
    break;

  default:
    flipstage = STAGE_FLIP_EXIT;
    break;
  }
#endif
}
