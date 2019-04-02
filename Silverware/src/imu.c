
// library headers
#include <stdbool.h>
#include <inttypes.h>

//#define _USE_MATH_DEFINES
#include <math.h>
#include "drv_time.h"

#include "util.h"
#include "sixaxis.h"
#include "config.h"

#include <stdlib.h>


#ifdef DEBUG
#include "debug.h"
extern debug_type debug;
#endif

#define ACC_1G 1.0f

// disable drift correction ( for testing)
#define DISABLE_ACC 0

// filter time in seconds
// time to correct gyro readings using the accelerometer
// 1-4 are generally good
#define FILTERTIME 2.0

// accel magnitude limits for drift correction
#define ACC_MIN 0.7f
#define ACC_MAX 1.3f


float GEstG[3] = { 0, 0, ACC_1G };

float attitude[3];

extern float gyro[3];
extern float accel[3];
extern float accelcal[3];


void imu_init(void)
{
	// init the gravity vector with accel values
	for (int xx = 0; xx < 100; xx++)
	  {
		  sixaxis_read();

		  for (int x = 0; x < 3; x++)
		    {
			    lpf(&GEstG[x], accel[x]* ( 1/ 2048.0f) , 0.85);
		    }
		  delay(1000);


	  }
}

// from http://en.wikipedia.org/wiki/Fast_inverse_square_root
// originally from quake3 code

float Q_rsqrt( float number )
{

	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                       
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed
//	y  = y * ( threehalfs - ( x2 * y * y ) );   // 3nd iteration, this can be removed
	
	return y;
}


void vectorcopy(float *vector1, float *vector2);


float atan2approx(float y, float x);

float calcmagnitude(float vector[3])
{
	float accmag = 0;
	for (uint8_t axis = 0; axis < 3; axis++)
	  {
		  accmag += vector[axis] * vector[axis];
	  }
	accmag = 1.0f/Q_rsqrt(accmag);
	return accmag;
}


void vectorcopy(float *vector1, float *vector2)
{
	for (int axis = 0; axis < 3; axis++)
	  {
		  vector1[axis] = vector2[axis];
	  }
}

extern float looptime;


void imu_calc(void)
{



// remove bias
    accel[0] = accel[0] - accelcal[0];
    accel[1] = accel[1] - accelcal[1];


// reduce to accel in G
    for (int i = 0; i < 3; i++)
	  {
		  accel[i] *= ( 1/ 2048.0f);
	  }
  
      
	float deltaGyroAngle[3];

	for ( int i = 0 ; i < 3 ; i++)
    {
        deltaGyroAngle[i] = (gyro[i]) * looptime;
    }
	
	
	GEstG[2] = GEstG[2] - (deltaGyroAngle[0]) * GEstG[0];
	GEstG[0] = (deltaGyroAngle[0]) * GEstG[2] +  GEstG[0];


	GEstG[1] =  GEstG[1] + (deltaGyroAngle[1]) * GEstG[2];
	GEstG[2] = -(deltaGyroAngle[1]) * GEstG[1] +  GEstG[2];


	GEstG[0] = GEstG[0] - (deltaGyroAngle[2]) * GEstG[1];
	GEstG[1] = (deltaGyroAngle[2]) * GEstG[0] +  GEstG[1];


// calc acc mag
	float accmag;

	accmag = calcmagnitude(&accel[0]);


	if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G) && !DISABLE_ACC)
	  {			 
        // normalize acc
        for (int axis = 0; axis < 3; axis++)
        {
            accel[axis] = accel[axis] * ( ACC_1G / accmag);
        }       
        float filtcoeff = lpfcalc_hz( looptime, 1.0f/(float)FILTERTIME);
        for (int x = 0; x < 3; x++)
          {
              lpf(&GEstG[x], accel[x], filtcoeff);
          }
	  }


	attitude[0] = atan2approx(GEstG[0], GEstG[2]) ;

	attitude[1] = atan2approx(GEstG[1], GEstG[2])  ;

}



#define M_PI  3.14159265358979323846	/* pi */


#define OCTANTIFY(_x, _y, _o)   do {                            \
    float _t;                                                   \
    _o= 0;                                                \
    if(_y<  0)  {            _x= -_x;   _y= -_y; _o += 4; }     \
    if(_x<= 0)  { _t= _x;    _x=  _y;   _y= -_t; _o += 2; }     \
    if(_x<=_y)  { _t= _y-_x; _x= _x+_y; _y=  _t; _o += 1; }     \
} while(0);

// +-0.09 deg error
float atan2approx(float y, float x)
{

	if (x == 0)
		x = 123e-15f;
	float phi = 0;
	float dphi;
	float t;

	OCTANTIFY(x, y, phi);

	t = (y / x);
	// atan function for 0 - 1 interval
	dphi = t*( ( M_PI/4 + 0.2447f ) + t *( ( -0.2447f + 0.0663f ) + t*( - 0.0663f)) );
	phi *= M_PI / 4;
	dphi = phi + dphi;
	if (dphi > (float) M_PI)
		dphi -= 2 * M_PI;
	return RADTODEG * dphi;
}

