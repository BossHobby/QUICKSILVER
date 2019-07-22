#include "defines.h"

#ifdef BOLDCLASH_716MM_8K

float motormap(float input)
{
	// this is a thrust to pwm function
	//  float 0 to 1 input and output
	// output can go negative slightly
    // boldclash f03 with 716 motors ant 4 blade prop
	// a*x^2 + b*x + c

	if (input > 1)
		input = 1;
	if (input < 0)
		input = 0;

	input = input * input * 0.51f + input * (0.44f);
	input += 0.05f;

	return input;
}
#endif

#ifdef BOLDCLASH_716MM_24K

float motormap(float input)
{
	// this is a thrust to pwm function
	//  float 0 to 1 input and output
	// output can go negative slightly
    // boldclash f03 with 716 motors ant 4 blade prop
	// a*x^2 + b*x + c

	if (input > 1)
		input = 1;
	if (input < 0)
		input = 0;

	input = input * input * 0.149f + input * (0.846f);
	input += 0.005f;

	return input;
}
#endif

#ifdef MOTOR_CURVE_6MM_490HZ
// the old map for 490Hz
float motormap(float input)
{
	// this is a thrust to pwm function
	//  float 0 to 1 input and output
	// output can go negative slightly
	// measured eachine motors and prop, stock battery
	// a*x^2 + b*x + c
	// a = 0.262 , b = 0.771 , c = -0.0258

	if (input > 1)
		input = 1;
	if (input < 0)
		input = 0;

	input = input * input * 0.262f + input * (0.771f);
	input += -0.0258f;

	return input;
}
#endif


#ifdef MOTOR_CURVE_6MM_H101_490HZ
float motormap( float input)
{ 

	// H101 thrust curve for normal thrust direction
	// a*x^2 + b*x + c

if (input > 1.0f) input = 1.0f;
if (input < 0) input = 0;

input = input*input*0.277f  + input*(0.715f);
input += 0.0102f;

return input;   
}
#endif

// 8k pwm is where the motor thrust is relatively linear for the H8 6mm motors
// it's due to the motor inductance cancelling the nonlinearities.
#ifdef MOTOR_CURVE_NONE
float motormap(float input)
{
	return input;
}
#endif


#ifdef MOTOR_CURVE_85MM_8KHZ
// Hubsan 8.5mm 8khz pwm motor map
float motormap(float input)
{
//      Hubsan 8.5mm motors and props 

	if (input > 1)
		input = 1;
	if (input < 0)
		input = 0;

	input = input * input * 0.683f + input * (0.262f);
	input += 0.06f;

	return input;
}
#endif


#ifdef MOTOR_CURVE_85MM_8KHZ_OLD
// Hubsan 8.5mm 8khz pwm motor map
float motormap(float input)
{
//      Hubsan 8.5mm motors and props 

	if (input > 1)
		input = 1;
	if (input < 0)
		input = 0;

	input = input * input * 0.789f + input * (0.172f);
	input += 0.04f;

	return input;
}
#endif


#ifdef MOTOR_CURVE_85MM_32KHZ
// Hubsan 8.5mm 32khz pwm motor map
float motormap(float input)
{
//      Hubsan 8.5mm motors and props 

	if (input > 1)
		input = 1;
	if (input < 0)
		input = 0;

	input = input * input * 0.197f + input * (0.74f);
	input += 0.067f;

	return input;
}
#endif

