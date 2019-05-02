
#include <stdbool.h>
#include "pid.h"
#include "util.h"
#include "config.h"
#include "defines.h"
#include "math.h"

#define APIDNUMBER 2

//**************************** ANGLE PIDS - used in level mode to set leveling strength




//NotFastEnuf dynamic angle mode test settings 

// Leveling algorithm coefficients for small errors  (normal flying)
float apidkp1[APIDNUMBER] = { 10.00 };  // P TERM GAIN ROLL + PITCH 
float apidkd1[APIDNUMBER] = { 3.0 };    // D TERM GAIN ROLL + PITCH

// Leveling algorithm coefficients for large errors  (stick banging or collisions)
float apidkp2[APIDNUMBER] = { 5.00 };   // P TERM GAIN ROLL + PITCH 
float apidkd2[APIDNUMBER] = { 0.0 };    // D TERM GAIN ROLL + PITCH




// code variables below

#define OUTLIMIT_FLOAT (apidkp1[0]+apidkp2[0])   //set angle pid output limit to sum of both P terms just in case

float apidoutput1[APIDNUMBER];
float apidoutput2[APIDNUMBER];
float angleerror[APIDNUMBER];
float lasterror[APIDNUMBER];
float apidoutput[APIDNUMBER];


float apid(int x)
{

	// P term 1 weighted
	apidoutput1[x] = (1-fabsf(angleerror[x])) * angleerror[x] * apidkp1[0] ;
	
	// P term 2 weighted
	apidoutput2[x] = fabsf(angleerror[x]) * angleerror[x] * apidkp2[0];  
	
extern float timefactor;
	// D term 1 weighted + P term 1 weighted
	apidoutput1[x] += (angleerror[x] - lasterror[x]) * apidkd1[0] * (1-fabsf(angleerror[x])) * timefactor;
	
	// D term 2 weighted + P term 2 weighted
	apidoutput2[x] += ((angleerror[x] - lasterror[x]) * apidkd2[0] * fabsf(angleerror[x]) * timefactor);
	
  // apidoutput sum
	apidoutput[x] = apidoutput1[x] + apidoutput2[x];
	
  lasterror[x] = angleerror[x];
	limitf(&apidoutput[x], OUTLIMIT_FLOAT);	
	
	return apidoutput[x];
}
