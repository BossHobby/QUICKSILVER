#include <inttypes.h>
#include <math.h>
#include "drv_time.h"
#include "gestures.h"
#include "defines.h"

#define STICKMAX 0.7f
#define STICKCENTER 0.2f


#define GMACRO_LEFT (rx[0] < - STICKMAX || rx[2] < - STICKMAX)
#define GMACRO_RIGHT (rx[0] >  STICKMAX || rx[2] >  STICKMAX)
#define GMACRO_XCENTER (fabsf(rx[0]) < STICKCENTER && fabsf(rx[2]) < STICKCENTER  )


#define GMACRO_DOWN (rx[1] < - STICKMAX)
#define GMACRO_UP (rx[1] >  STICKMAX)

#define GMACRO_PITCHCENTER (fabsf(rx[1]) < STICKCENTER)


#define GESTURE_CENTER 0
#define GESTURE_CENTER_IDLE 12
#define GESTURE_LEFT 1
#define GESTURE_RIGHT 2
#define GESTURE_DOWN 3
#define GESTURE_UP 4
#define GESTURE_OTHER 127
#define GESTURE_LONG 255

#define GESTURETIME_MIN 100e3
#define GESTURETIME_MAX 500e3
#define GESTURETIME_IDLE 700e3


#define GSIZE 7

// L L L
const uint8_t command11[GSIZE] = {
//	GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER
	GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER
};

// L L L
const uint8_t command10[GSIZE] = {
//	GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER
	GESTURE_CENTER_IDLE, GESTURE_LEFT, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER
};

// R R R
const uint8_t command9[GSIZE] = {
//	GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER
	GESTURE_CENTER_IDLE, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER
};


// L L D
const uint8_t command1[GSIZE] = {
	GESTURE_CENTER_IDLE, GESTURE_LEFT, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER
};

// R R D
const uint8_t command2[GSIZE] = {
	GESTURE_CENTER_IDLE, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER
};

// D D D
const uint8_t command3[GSIZE] = {
	GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER
};


// U U U
const uint8_t command8[GSIZE] = {
	GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER
};

#ifdef PID_GESTURE_TUNING
// U D U - Next PID term
const uint8_t command4[GSIZE] = {
	GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER
};

// U D D - Next PID Axis
const uint8_t command5[GSIZE] = {
	GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER
};

// U D R - Increase value
const uint8_t command6[GSIZE] = {
	GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER
};

// U D L - Descrease value
const uint8_t command7[GSIZE] = {
	GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER
};
#endif



int gesture_start;
int lastgesture;
int setgesture;
static unsigned gesturetime;

extern int onground;
extern float rx[];


int gestures2()
{
	if (onground)
	  {
		  if (GMACRO_XCENTER && GMACRO_PITCHCENTER)
		    {
			    gesture_start = GESTURE_CENTER;
		    }
		  else if (GMACRO_LEFT && GMACRO_PITCHCENTER)
		    {
			    gesture_start = GESTURE_LEFT;
		    }
		  else if (GMACRO_RIGHT && GMACRO_PITCHCENTER)
		    {
			    gesture_start = GESTURE_RIGHT;
		    }
		  else if (GMACRO_DOWN && GMACRO_XCENTER)
		    {
			    gesture_start = GESTURE_DOWN;
		    }
		  else if (GMACRO_UP && GMACRO_XCENTER)
		    {
			    gesture_start = GESTURE_UP;
		    }
		  else
		    {
			    //      gesture_start = GESTURE_OTHER;  
		    }

		  unsigned long time = gettime();

		  if (gesture_start != lastgesture)
		    {
			    gesturetime = time;
		    }


		  if (time - gesturetime > GESTURETIME_MIN)
		    {
			    if ((gesture_start == GESTURE_CENTER) && (time - gesturetime > GESTURETIME_IDLE))
			      {
				      setgesture = GESTURE_CENTER_IDLE;
			      }
			    else if (time - gesturetime > GESTURETIME_MAX)
			      {
				      if ((gesture_start != GESTURE_OTHER))
					      setgesture = GESTURE_LONG;
			      }

			    else
				    setgesture = gesture_start;

		    }


		  lastgesture = gesture_start;



		  return gesture_sequence(setgesture);

	  }
	else
	  {
		  setgesture = GESTURE_OTHER;
		  lastgesture = GESTURE_OTHER;
	  }

	return 0;
}



uint8_t gbuffer[GSIZE];


uint8_t check_command( uint8_t  buffer1[] , const uint8_t  command[]  )
{
    for (int i = 0; i < GSIZE; i++)
            {
                if( buffer1[i] != command[GSIZE - i - 1])
                return 0;
            }     
return 1;            
}

int gesture_sequence(int currentgesture)
{

	if (currentgesture != gbuffer[0])
	  {			// add to queue

		  for (int i = GSIZE - 1; i >= 1; i--)
		    {
			    gbuffer[i] = gbuffer[i - 1];

		    }
		  gbuffer[0] = currentgesture;


// check commands

		  if (check_command ( &gbuffer[0] , &command1[0] ) )
		    {
			    // command 1

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return GESTURE_LLD;
		    }

		
		  if (check_command ( &gbuffer[0] , &command2[0] ))
		    {
			    // command 2

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return GESTURE_RRD;
		    }

		
		  if (check_command ( &gbuffer[0] , &command3[0] ))
		    {
			    // command 3

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return GESTURE_DDD;
		    }
            if (check_command ( &gbuffer[0] , &command8[0] ))
		    {
			    // command 8

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return GESTURE_UUU;
		    }

            if (check_command ( &gbuffer[0] , &command9[0] ))
		    {
			    // command 9

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return GESTURE_RRR;
		    }				
				
				    if (check_command ( &gbuffer[0] , &command10[0] ))
		    {
			    // command 10

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return GESTURE_LLL;
		    }	
				
				    if (check_command ( &gbuffer[0] , &command11[0] ))
		    {
			    // command 11

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return GESTURE_DUD;
		    }					
				
				
				
				
			#ifdef PID_GESTURE_TUNING
			if (check_command ( &gbuffer[0] , &command4[0] ))
		    {
			    // command 4

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return GESTURE_UDU;
		    }
				
			if (check_command ( &gbuffer[0] , &command5[0] ))
		    {
			    // command 5

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return GESTURE_UDD;
		    }
				
			if (check_command ( &gbuffer[0] , &command6[0] ))
		    {
			    // command 6

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return GESTURE_UDR;
		    }
			if (check_command ( &gbuffer[0] , &command7[0] ))
		    {
			    // command 7

			    //change buffer so it does not trigger again
			    gbuffer[1] = GESTURE_OTHER;
			    return GESTURE_UDL;
		    }
			#endif


	  }

	return GESTURE_NONE;
}
