#include "drv_time.h"
#include "config.h"
#include "flip_sequencer.h"
#include "defines.h"


#define THROTTLE_UP 0.8
#define THROTTLE_HOVER 0.5
#define THROTTLE_EXIT 0.6

#define FLIP_RATE 1500
//time to up throttle in beggining stage
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



#include <math.h>



int acro_override = 0;
int level_override = 0;
int controls_override = 0;
unsigned long fliptime;
int isflipping = 0;
int flipstage = STAGE_FLIP_NONE;
unsigned int levelmodetime;
int flipindex = 0;
int flipdir = 0;

extern int onground;
extern float GEstG[3];
extern float rx[];

float rx_override[4];

void start_flip()
{

	if ( isflipping == 0 && !onground)
	{
		isflipping = 1;
		fliptime = gettime();
		flipstage = STAGE_FLIP_START;
		
	}
	
flipindex = 0;
flipdir = 0;	
	if ( fabsf(rx[0]) < fabsf(rx[1]) ) 
	{
		flipindex = 1;
		if ( rx[1] > 0 ) flipdir = 1; 
	}
	else if ( rx[0] > 0 ) flipdir = 1; 
}


void flip_sequencer()
{
	if ( !isflipping) return;
	
	if (onground) flipstage = STAGE_FLIP_EXIT;
	
	if ( isflipping && gettime() - fliptime > FLIP_TIMEOUT_TOTAL )
			{
				// abort after timeout second
				flipstage = STAGE_FLIP_EXIT;
			}
	
	switch (flipstage )
	{
		case STAGE_FLIP_NONE:
			
		break;
		
		case STAGE_FLIP_START:
		// 		
			acro_override = 1;
			controls_override = 1;
			
			rx_override[1] = 0;
			rx_override[2] = 0;
		  rx_override[3] = THROTTLE_UP;
		
			if ( GEstG[2] < 0 ) 
			{
				// flip initiated inverted
				if ( flipdir) 
						rx_override[flipindex] = (float) FLIP_RATE / (float) MAX_RATE;
				else 
					  rx_override[flipindex] = (float)- FLIP_RATE / (float) MAX_RATE;
				rx_override[3] = THROTTLE_HOVER;
				flipstage = STAGE_FLIP_ROTATING_INVERTED;
			}
			flipstage = STAGE_FLIP_THROTTLEUP;
	
		break;
		
		case STAGE_FLIP_THROTTLEUP:
		 
			if ( gettime() - fliptime > THROTTLE_UP_TIME )
			{
				if ( flipdir) 
						rx_override[flipindex] = (float) FLIP_RATE / (float) MAX_RATE;
				else 
					  rx_override[flipindex] = (float)- FLIP_RATE / (float) MAX_RATE;
				rx_override[3] = THROTTLE_UP;
				flipstage = STAGE_FLIP_ROTATING;
			}
			
		break;
		
		case STAGE_FLIP_ROTATING:
			if ( gettime() - fliptime > FLIP_TIMEOUT_STAGE1 + THROTTLE_UP_TIME )
			{
				// abort
				flipstage = STAGE_FLIP_EXIT;
			}
			if ( GEstG[2] < 0 ) 
			{
				//we are inverted
				rx_override[3] = THROTTLE_HOVER;
				flipstage = STAGE_FLIP_ROTATING_INVERTED;
			}
		break;
			
		case STAGE_FLIP_ROTATING_INVERTED:
			if ( GEstG[2] > 0 ) 
			{

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
			
			if ( flipdir )		
			rx_override[flipindex] = (float) LEVEL_MODE_ANGLE / (float) LEVEL_MAX_ANGLE;
		  else
			rx_override[flipindex] = (float) - LEVEL_MODE_ANGLE / (float) LEVEL_MAX_ANGLE;
			if( gettime() - levelmodetime > LEVEL_MODE_TIME )
					flipstage = STAGE_FLIP_EXIT;
		break;
			
			
		case STAGE_FLIP_EXIT:
			isflipping = 0;
			flipstage = STAGE_FLIP_NONE;
			acro_override = 0;
		  level_override = 0;
			controls_override = 0;
		break;
	
	
		default:
		  flipstage = STAGE_FLIP_EXIT;	
		break;
	
	}
	
}


