

#include <stdbool.h>
#include "pid.h"
#include "util.h"
#include "config.h"
#include "defines.h"

#define APIDNUMBER 2

//**************************** ANGLE PIDS - used in level mode to set leveling strength


//HIGH LEVELING STRENGTH angle settings                       
//float apidkp[APIDNUMBER] = { 13.00 };  // Kp ROLL + PITCH 
//float apidkd[APIDNUMBER] = { 5.0 };    // Kd ROLL + PITCH 

//NotFastEnuf personal settings                       
float apidkp[APIDNUMBER] = { 5.00 };  // Kp ROLL + PITCH 
float apidkd[APIDNUMBER] = { 0.0 };    // Kd ROLL + PITCH







// code variables below

// rate limit
#define OUTLIMIT_FLOAT (float)LEVEL_MAX_RATE

extern int onground;
extern float looptime;
extern float gyro[3];

float apidoutput[APIDNUMBER];
float angleerror[APIDNUMBER];

float lasterror[APIDNUMBER];

float apid(int x)
{
// int index = x;


	// P term
	apidoutput[x] = angleerror[x] * apidkp[0];


extern float timefactor;
      
    apidoutput[x] = apidoutput[x] + (angleerror[x] - lasterror[x]) * apidkd[0] * timefactor;
    lasterror[x] = angleerror[x];

	limitf(&apidoutput[x], OUTLIMIT_FLOAT);


	return apidoutput[x];
}
