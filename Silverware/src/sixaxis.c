/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/



#include "binary.h"
#include "sixaxis.h"
#include "drv_time.h"
#include "util.h"
#include "config.h"
#include "led.h"
#include "drv_serial.h"

#include "drv_i2c.h"


#include <math.h>
#include <stdio.h>
#include <inttypes.h>


#include "debug.h"



// this works only on newer boards (non mpu-6050)
// on older boards the hw gyro setting controls the acc as well
#define ACC_LOW_PASS_FILTER 5

extern debug_type debug;

// temporary fix for compatibility between versions
#ifndef GYRO_ID_1 
#define GYRO_ID_1 0x68 
#endif
#ifndef GYRO_ID_2
#define GYRO_ID_2 0x98
#endif
#ifndef GYRO_ID_3
#define GYRO_ID_3 0x7D
#endif
#ifndef GYRO_ID_4
#define GYRO_ID_4 0x72
#endif

void sixaxis_init( void)
{
// gyro soft reset
	
	
	i2c_writereg(  107 , 128);
	 
 delay(40000);
	

// set pll to 1, clear sleep bit old type gyro (mpu-6050)	
	i2c_writereg(  107 , 1);
	
	int newboard = !(0x68 == i2c_readreg(117) );

    delay(100);
	
	i2c_writereg(  28, B00011000);	// 16G scale

    
// acc lpf for the new gyro type
//       0-6 ( same as gyro)
	if (newboard) i2c_writereg( 29, ACC_LOW_PASS_FILTER);
	
// gyro scale 2000 deg (FS =3)

	i2c_writereg( 27 , 24);
	
// Gyro DLPF low pass filter

	i2c_writereg( 26 , GYRO_LOW_PASS_FILTER);
}


int sixaxis_check( void)
{
	#ifndef DISABLE_GYRO_CHECK
	// read "who am I" register
	int id = i2c_readreg( 117 );

	#ifdef DEBUG
	debug.gyroid = id;
	#endif
	
	return (GYRO_ID_1==id||GYRO_ID_2==id||GYRO_ID_3==id||GYRO_ID_4==id );
	#else
	return 1;
	#endif
}





float accel[3];
float gyro[3];

float accelcal[3];
float gyrocal[3];


float lpffilter(float in, int num);
float lpffilter2(float in, int num);

void sixaxis_read(void)
{
	int data[16];
	float gyronew[3];
	
	i2c_readdata( 59 , data , 14 );
		
#ifdef SENSOR_ROTATE_90_CW	         
        accel[0] = (int16_t) ((data[2] << 8) + data[3]);
        accel[1] = -(int16_t) ((data[0] << 8) + data[1]);
        accel[2] = (int16_t) ((data[4] << 8) + data[5]);
#else
        
	accel[0] = -(int16_t) ((data[0] << 8) + data[1]);
	accel[1] = -(int16_t) ((data[2] << 8) + data[3]);
	accel[2] = (int16_t) ((data[4] << 8) + data[5]);  
        
#endif
  

#ifdef SENSOR_ROTATE_90_CW_deleted
		{//
		float temp = accel[1];
		accel[1] = accel[0];
		accel[0] = -temp;	
		}
#endif       
        
// this is the value of both cos 45 and sin 45 = 1/sqrt(2)
#define INVSQRT2 0.707106781f
		
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = accel[0];
		accel[0] = (accel[0] * INVSQRT2 + accel[1] * INVSQRT2);
		accel[1] = -(temp * INVSQRT2 - accel[1] * INVSQRT2);	
		}
#endif
        
#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = accel[1];
		accel[1] = (accel[1] * INVSQRT2 + accel[0] * INVSQRT2);
		accel[0] = -(temp * INVSQRT2 - accel[0] * INVSQRT2);	
		}
#endif
	
		
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = accel[1];
		accel[1] = -accel[0];
		accel[0] = temp;	
		}
#endif
				
#ifdef SENSOR_ROTATE_180
		{
		accel[1] = -accel[1];
		accel[0] = -accel[0];	
		}
#endif		
		
#ifdef SENSOR_FLIP_180
		{
		accel[2] = -accel[2];
		accel[0] = -accel[0];	
		}
#endif	
//order
	gyronew[1] = (int16_t) ((data[8] << 8) + data[9]);
	gyronew[0] = (int16_t) ((data[10] << 8) + data[11]);
	gyronew[2] = (int16_t) ((data[12] << 8) + data[13]);


gyronew[0] = gyronew[0] - gyrocal[0];
gyronew[1] = gyronew[1] - gyrocal[1];
gyronew[2] = gyronew[2] - gyrocal[2];

#ifdef SENSOR_ROTATE_90_CW
		{
		float temp = gyronew[1];
		gyronew[1] = -gyronew[0];
		gyronew[0] = temp;	
		}
#endif
        
	
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0] * INVSQRT2 + gyronew[1] * INVSQRT2;
		gyronew[0] = gyronew[0] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif

#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = gyronew[0];
		gyronew[0] = gyronew[1] * INVSQRT2 + gyronew[0] * INVSQRT2;
		gyronew[1] = gyronew[1] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif	        
		
		
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0];
		gyronew[0] = -temp;	
		}
#endif
		
					
#ifdef SENSOR_ROTATE_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[0] = -gyronew[0];	
		}
#endif		
		
#ifdef SENSOR_FLIP_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[2] = -gyronew[2];	
		}
#endif	

//gyronew[0] = - gyronew[0];
gyronew[1] = - gyronew[1];
gyronew[2] = - gyronew[2];

	for (int i = 0; i < 3; i++)
	  {
		  gyronew[i] = gyronew[i] * 0.061035156f * 0.017453292f;
#ifndef SOFT_LPF_NONE
			
		#if defined (GYRO_FILTER_PASS2) && defined (GYRO_FILTER_PASS1)
			gyro[i] = lpffilter(gyronew[i], i);
			gyro[i] = lpffilter2(gyro[i], i);
		#endif
			
		#if defined (GYRO_FILTER_PASS1) && !defined(GYRO_FILTER_PASS2)
			gyro[i] = lpffilter(gyronew[i], i);
		#endif
			
		#if defined (GYRO_FILTER_PASS2) && !defined(GYRO_FILTER_PASS1)
			gyro[i] = lpffilter2(gyronew[i], i);
		#endif
#else
		  gyro[i] = gyronew[i];
#endif
	  }


}


void gyro_read( void)
{
int data[6];
	
i2c_readdata( 67 , data , 6 );
	
float gyronew[3];
	// order
gyronew[1] = (int16_t) ((data[0]<<8) + data[1]);
gyronew[0] = (int16_t) ((data[2]<<8) + data[3]);
gyronew[2] = (int16_t) ((data[4]<<8) + data[5]);

		
gyronew[0] = gyronew[0] - gyrocal[0];
gyronew[1] = gyronew[1] - gyrocal[1];
gyronew[2] = gyronew[2] - gyrocal[2];
	
	
		
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0] * INVSQRT2 + gyronew[1] * INVSQRT2;
		gyronew[0] = gyronew[0] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif
        
#ifdef SENSOR_ROTATE_45_CW
		{
		float temp = gyronew[0];
		gyronew[0] = gyronew[1] * INVSQRT2 + gyronew[0] * INVSQRT2;
		gyronew[1] = gyronew[1] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif
			
#ifdef SENSOR_ROTATE_90_CW
		{
		float temp = gyronew[1];
		gyronew[1] = -gyronew[0];
		gyronew[0] = temp;	
		}
#endif

				
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0];
		gyronew[0] = -temp;	
		}
#endif
	
					
#ifdef SENSOR_ROTATE_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[0] = -gyronew[0];	
		}
#endif		
		
							
#ifdef SENSOR_FLIP_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[2] = -gyronew[2];	
		}
#endif		
	


		
//gyronew[0] = - gyronew[0];
gyronew[1] = - gyronew[1];
gyronew[2] = - gyronew[2];
	
	
for (int i = 0; i < 3; i++)
	  {
		  gyronew[i] = gyronew[i] * 0.061035156f * 0.017453292f;
#ifndef SOFT_LPF_NONE
			
		#if defined (GYRO_FILTER_PASS2) && defined (GYRO_FILTER_PASS1)
			gyro[i] = lpffilter(gyronew[i], i);
			gyro[i] = lpffilter2(gyro[i], i);
		#endif
			
		#if defined (GYRO_FILTER_PASS1) && !defined(GYRO_FILTER_PASS2)
			gyro[i] = lpffilter(gyronew[i], i);
		#endif
			
		#if defined (GYRO_FILTER_PASS2) && !defined(GYRO_FILTER_PASS1)
			gyro[i] = lpffilter2(gyronew[i], i);
		#endif
#else
		  gyro[i] = gyronew[i];
#endif
	  }

}
 


#define CAL_TIME 2e6

void gyro_cal(void)
{
int data[6];
float limit[3];	
unsigned long time = gettime();
unsigned long timestart = time;
unsigned long timemax = time;
unsigned long lastlooptime = time;

float gyro[3];	
	
 for ( int i = 0 ; i < 3 ; i++)
			{
			limit[i] = gyrocal[i];
			}

// 2 and 15 seconds
while ( time - timestart < CAL_TIME  &&  time - timemax < 15e6 )
	{	
		
		unsigned long looptime; 
		looptime = time - lastlooptime;
		lastlooptime = time;
		if ( looptime == 0 ) looptime = 1;

	i2c_readdata(  67 , data , 6 );	

			
	gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
	gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
	gyro[2] = (int16_t) ((data[4]<<8) + data[5]);
		

/*		
if ( (time - timestart)%200000 > 100000) 
{
	ledon(B00000101);
	ledoff(B00001010);
}
else 
{
	ledon(B00001010);
	ledoff(B00000101);
}
*/
#define GLOW_TIME 62500 
static int brightness = 0;
led_pwm( brightness);
if ((brightness&1)^((time - timestart)%GLOW_TIME > (GLOW_TIME>>1) ))
{
brightness++;
}

brightness&=0xF;

		 for ( int i = 0 ; i < 3 ; i++)
			{

					if ( gyro[i] > limit[i] )  limit[i] += 0.1f; // 100 gyro bias / second change
					if ( gyro[i] < limit[i] )  limit[i] -= 0.1f;
				
					limitf( &limit[i] , 800);
				
					if ( fabsf(gyro[i]) > 100+ fabsf(limit[i]) ) 
					{										
						timestart = gettime();
						brightness = 1;
					}
					else
					{						
					lpf( &gyrocal[i] , gyro[i], lpfcalc( (float) looptime , 0.5 * 1e6) );
				
					}

			}

while ( (gettime() - time) < 1000 ) delay(10); 				
time = gettime();

	}

	

if ( time - timestart < CAL_TIME )
{
	for ( int i = 0 ; i < 3; i++)
	{
	gyrocal[i] = 0;

	}
	
}


	
}


void acc_cal(void)
{
	accelcal[2] = 2048;
	for (int y = 0; y < 500; y++)
	  {
		  sixaxis_read();
		  for (int x = 0; x < 3; x++)
		    {
			    lpf(&accelcal[x], accel[x], 0.92);
		    }
		  gettime();	// if it takes too long time will overflow so we call it here

	  }
	accelcal[2] -= 2048;
		

#ifdef FLASH_SAVE2
	for (int x = 0; x < 3; x++)
	  {
		  limitf(&accelcal[x], 127);
	  }
#endif
      
#ifdef FLASH_SAVE1
	for (int x = 0; x < 3; x++)
	  {
		  limitf(&accelcal[x], 500);
	  }
#endif
		
}







