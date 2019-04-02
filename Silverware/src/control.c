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

#include <inttypes.h>
#include <math.h>

#include "pid.h"
#include "config.h"
#include "util.h"
#include "drv_pwm.h"
#include "control.h"
#include "defines.h"
#include "drv_time.h"
#include "sixaxis.h"
#include "drv_fmc2.h"
#include "drv_fmc.h"
#include "flip_sequencer.h"
#include "gestures.h"
#include "defines.h"
#include "led.h"



float	throttle;
int idle_state;
extern int armed_state;
extern int in_air;
extern int arming_release;
extern int binding_while_armed;
extern int rx_ready;

extern float rx[];
extern float gyro[3];
extern int failsafe;
extern float pidoutput[PIDNUMBER];
extern float setpoint[3];

extern float angleerror[];
extern float attitude[];

int onground = 1;
int onground_long = 1;

float thrsum;

float error[PIDNUMBER];
float motormap( float input);

float yawangle;

extern float looptime;

extern char auxchange[AUXNUMBER];
extern char aux[AUXNUMBER];

extern int ledcommand;
extern int ledblink;

extern float apid(int x);

#ifdef NOMOTORS
// to maintain timing or it will be optimized away
float tempx[4];
#endif

#ifdef STOCK_TX_AUTOCENTER
float autocenter[3];
float lastrx[3];
unsigned int consecutive[3];
#endif

unsigned long timecommand = 0;

extern int controls_override;
extern float rx_override[];
extern int acro_override;

float overthrottlefilt = 0;
float underthrottlefilt = 0;

float rxcopy[4];

#ifdef BETAFLIGHT_RATES
#define SETPOINT_RATE_LIMIT 1998.0f
#define RC_RATE_INCREMENTAL 14.54f

static inline float constrainf(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

static float calcBFRatesRad(int axis)
{
    float rcRate, superExpo;
    if (axis == ROLL) {
        rcRate = (float) BF_RC_RATE_ROLL;
        superExpo = (float) BF_SUPER_RATE_ROLL;
    } else if (axis == PITCH) {
        rcRate = (float) BF_RC_RATE_PITCH;
        superExpo = (float) BF_SUPER_RATE_PITCH;
	} else {
        rcRate = (float) BF_RC_RATE_YAW;
        superExpo = (float) BF_SUPER_RATE_YAW;
    }
    if (rcRate > 2.0f) {
        rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
    }
    const float rcCommandfAbs = rxcopy[axis] > 0 ? rxcopy[axis] : -rxcopy[axis];
    float angleRate = 200.0f * rcRate * rxcopy[axis];
    if (superExpo) {
        const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * superExpo), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
    }
    return constrainf(angleRate, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT) * (float) DEGTORAD;
}
#endif

void control( void)
{	

// high-low rates switch 
float rate_multiplier = 1.0;
	
	if ( aux[RATES]  )
	{		
		
	}
	else
	{
		rate_multiplier = LOW_RATES_MULTI;
	}
	// make local copy
	
	
#ifdef INVERTED_ENABLE	
    extern int pwmdir;
	if ( aux[FN_INVERTED]  )		
        pwmdir = REVERSE;
    else
        pwmdir = FORWARD;    
#endif	
	
	for ( int i = 0 ; i < 3 ; i++)
	{
		#ifdef STOCK_TX_AUTOCENTER
		rxcopy[i] = (rx[i] - autocenter[i]);
		#else
		rxcopy[i] = rx[i];
		#endif
		#ifdef STICKS_DEADBAND
		if ( fabsf( rxcopy[ i ] ) <= STICKS_DEADBAND ) {
			rxcopy[ i ] = 0.0f;
		} else {
			if ( rxcopy[ i ] >= 0 ) {
				rxcopy[ i ] = mapf( rxcopy[ i ], STICKS_DEADBAND, 1, 0, 1 );
			} else {
				rxcopy[ i ] = mapf( rxcopy[ i ], -STICKS_DEADBAND, -1, 0, -1 );
			}
		}
		#endif
	 }

#ifndef DISABLE_FLIP_SEQUENCER	
  flip_sequencer();
	
	if ( controls_override)
	{
		for ( int i = 0 ; i < 3 ; i++)
		{
			rxcopy[i] = rx_override[i];
		}
	}

	if ( auxchange[STARTFLIP]&&!aux[STARTFLIP] )
	{// only on high -> low transition
		start_flip();		
	}
#endif	
	



pid_precalc();	


	// flight control

	float rates[3];

#ifndef BETAFLIGHT_RATES
    rates[0] = rate_multiplier * rxcopy[0] * (float) MAX_RATE * DEGTORAD;
    rates[1] = rate_multiplier * rxcopy[1] * (float) MAX_RATE * DEGTORAD;
    rates[2] = rate_multiplier * rxcopy[2] * (float) MAX_RATEYAW * DEGTORAD;
#else
    rates[0] = rate_multiplier * calcBFRatesRad(0);
    rates[1] = rate_multiplier * calcBFRatesRad(1);
    rates[2] = rate_multiplier * calcBFRatesRad(2);
#endif
        
if (aux[LEVELMODE]&&!acro_override){
	extern void stick_vector( float rx_input[] , float maxangle);
	extern float errorvect[]; // level mode angle error calculated by stick_vector.c	
	extern float GEstG[3]; // gravity vector for yaw feedforward
	float yawerror[3] = {0}; // yaw rotation vector
	// calculate roll / pitch error
	stick_vector( rxcopy , 0 );
	// apply yaw from the top of the quad 
	yawerror[0] = GEstG[1] * rates[2];
	yawerror[1] = - GEstG[0] * rates[2];
	yawerror[2] = GEstG[2] * rates[2];
	
	
	// *************************************************************************
	//horizon modes tuning variables
	// *************************************************************************
	// 1.0 is pure angle based transition, 0.0 is pure stick defelction based transition, values inbetween are a mix of both.  Adjust from 0 to 1
	float HORIZON_SLIDER = 0.3f;
	//leveling transitions into acro below this angle - above this angle is all acro.  DO NOT SET ABOVE 85 DEGREES!
	float HORIZON_ANGLE_TRANSITION = 55.0f;
	//leveling transitions into acro below this stick position - beyond this stick position is all acro. Adjust from 0 to 1
	float HORIZON_STICK_TRANSITION = 0.95f;
	// *************************************************************************
	// *************************************************************************
	
	
	if (aux[RACEMODE] && !aux[HORIZON]){ //racemode with angle behavior on roll ais
			if (GEstG[2] < 0 ){ // acro on roll and pitch when inverted
					error[0] = rates[0] - gyro[0];
					error[1] = rates[1] - gyro[1];
			}else{
					//roll is leveled to max angle limit
					angleerror[0] = errorvect[0] ; 
					error[0] = apid(0) + yawerror[0] - gyro[0];
					//pitch is acro 
					error[1] = rates[1] - gyro[1];}
			// yaw
			error[2] = yawerror[2] - gyro[2];
		
	}else if(aux[RACEMODE] && aux[HORIZON]){	//racemode with horizon behavior on roll axis	
			float inclinationRoll	= attitude[0];
			float inclinationPitch = attitude[1];
			float inclinationMax;
			if (fabsf(inclinationRoll) >= fabsf(inclinationPitch)){
					inclinationMax = fabsf(inclinationRoll);
			}else{
					inclinationMax = fabsf(inclinationPitch);}
			float angleFade;
			// constrains acroFade variable between 0 and 1
			if (inclinationMax <= HORIZON_ANGLE_TRANSITION){
					angleFade = inclinationMax/HORIZON_ANGLE_TRANSITION;
			}else{
					angleFade = 1;}
			float stickFade;
			float deflection = fabsf(rxcopy[0]);
			if (deflection <= HORIZON_STICK_TRANSITION){
					stickFade = deflection/HORIZON_STICK_TRANSITION;
			}else{
					stickFade = 1;}
			float fade = (stickFade *(1-HORIZON_SLIDER))+(HORIZON_SLIDER * angleFade);
			// apply acro to roll for inverted behavior
			if (GEstG[2] < 0 ){
					error[0] = rates[0] - gyro[0];
					error[1] = rates[1] - gyro[1];
			}else{ // apply a transitioning mix of acro and level behavior inside of stick HORIZON_TRANSITION point and full acro beyond stick HORIZON_TRANSITION point					
					angleerror[0] = errorvect[0] ;
					// roll angle strength fades out as sticks approach HORIZON_TRANSITION while acro stength fades in according to value of acroFade factor
					error[0] = ((apid(0) + yawerror[0] - gyro[0]) * (1 - fade)) + (fade * (rates[0] - gyro[0]));
					//pitch is acro
					error[1] = rates[1] - gyro[1];
			}
	
			// yaw
			error[2] = yawerror[2]  - gyro[2];  
		
	}else if(!aux[RACEMODE] && aux[HORIZON]){ //horizon overrites standard level behavior	
			//pitch and roll
			for ( int i = 0 ; i <=1; i++){	
			  	float inclinationRoll	= attitude[0];
					float inclinationPitch = attitude[1];
					float inclinationMax;
					if (fabsf(inclinationRoll) >= fabsf(inclinationPitch)){
						inclinationMax = fabsf(inclinationRoll);
					}else{
						inclinationMax = fabsf(inclinationPitch);}
					float angleFade;
					// constrains acroFade variable between 0 and 1
					if (inclinationMax <= HORIZON_ANGLE_TRANSITION){
						angleFade = inclinationMax/HORIZON_ANGLE_TRANSITION;
					}else{
						angleFade = 1;
					}
					float stickFade;
					float deflection = fabsf(rxcopy[i]);
					if (deflection <= HORIZON_STICK_TRANSITION){
						stickFade = deflection/HORIZON_STICK_TRANSITION;
					}else{
						stickFade = 1;
					}
					float fade = (stickFade *(1-HORIZON_SLIDER))+(HORIZON_SLIDER * angleFade);
					// apply acro to roll and pitch sticks for inverted behavior
					if (GEstG[2] < 0 ){
						error[i] = rates[i] - gyro[i];
					}else{ // apply a transitioning mix of acro and level behavior inside of stick HORIZON_TRANSITION point and full acro beyond stick HORIZON_TRANSITION point					
						angleerror[i] = errorvect[i] ;
						//  angle strength fades out as sticks approach HORIZON_TRANSITION while acro stength fades in according to value of acroFade factor
						error[i] = ((apid(i) + yawerror[i] - gyro[i]) * (1 - fade)) + (fade * (rates[i] - gyro[i]));
					}
			}
			// yaw
			error[2] = yawerror[2]  - gyro[2];  
			
	}else{ //standard level mode
	    // pitch and roll
			for ( int i = 0 ; i <=1; i++){
					angleerror[i] = errorvect[i] ;    
					error[i] = apid(i) + yawerror[i] - gyro[i];
			}
      // yaw
			error[2] = yawerror[2]  - gyro[2];
		} 
}else{	// rate mode

    setpoint[0] = rates[0];
    setpoint[1] = rates[1];
    setpoint[2] = rates[2];
          
	for ( int i = 0; i < 3; i++ ) {
		error[i] = setpoint[i] - gyro[i];
	}
}


 #ifdef YAW_FIX
{	
	rotateErrors();
	pid(0);
	pid(1);
	pid(2);		}
 #else 	
{
  pid(0);
	pid(1);
	pid(2);	

}

#endif


#ifndef THROTTLE_SAFETY
	#define THROTTLE_SAFETY .15f
#endif

		
#ifndef ARMING
 armed_state = 1;																							 									 // if arming feature is disabled - quad is always armed
#else																												  											// CONDITION: arming feature is enabled
	if (!aux[ARMING]){																					 										  // 						CONDITION: switch is DISARMED
		armed_state = 0;																															  // 												disarm the quad by setting armed state variable to zero
		if (rx_ready ==1)	binding_while_armed = 0;																			//                        rx is bound and has been disarmed so clear binding while armed flag
	}else{ 																				   						  										// 						CONDITION: switch is ARMED
		if (((rx[3] > THROTTLE_SAFETY) && (arming_release == 0)) || (binding_while_armed == 1)){ 		//				   CONDITION: (throttle is above safety limit and ARMING RELEASE FLAG IS NOT CLEARED) OR (bind just took place with transmitter armed)		
			armed_state = 0;																				 										  //                         	 				override to disarmed state and rapid blink the leds
		  ledcommand = 1;
		}else{																									  										  //            					 CONDITION: quad is being armed in a safe state 																		
			armed_state = 1;                                        										  //                      					  arm the quad by setting armed state variable to 1
		  arming_release = 1;																														//                       						clear the arming release flag - the arming release flag being cleared
		}																													 										  //											 						is what stops the quad from automatically disarming again the next time
	}																																									//											 						throttle is raised above the safety limit
#endif

#ifndef IDLE_UP
 idle_state = 0;
#else
	if (!aux[IDLE_UP]){
		idle_state = 0;
	}else{ idle_state = 1;}
#endif
	
#ifndef IDLE_THR
	#define IDLE_THR .05f
#endif

	if (armed_state == 0){                                     												// CONDITION: armed state variable is 0 so quad is DISARMED					
		throttle = 0;																																		//						override throttle to 0
		in_air = 0;																																			//						flag in air variable as NOT IN THE AIR for mix throttle increase safety
		arming_release = 0;																															//						arming release flag is set to not cleared to reactivate the throttle safety limit for the next arming event
	
	}else{                                                    	  										// CONDITION: armed state variable is 1 so quad is ARMED							 
			if (idle_state == 0){                                     										//            CONDITION: idle up is turned OFF				
				if ( rx[3] < 0.05f ){
					throttle = 0;                      																				//   											set a small dead zone where throttle is zero and
				  in_air = 0;																																//												deactivate mix increase 3 since throttle is off
				}else{ 
					throttle = (rx[3] - 0.05f)*1.05623158f;            												//                        map the remainder of the the active throttle region to 100%
					in_air = 1;}																															//												activate mix increase since throttle is on
			}else{ 																																				//						CONDITION: idle up is turned ON												
				throttle =  (float) IDLE_THR + rx[3] * (1.0f - (float) IDLE_THR);						//            						throttle range is mapped from idle throttle value to 100%							  
				if ((rx[3] > THROTTLE_SAFETY) && (in_air == 0)) in_air = 1; 			  				//            						change the state of in air flag when first crossing the throttle 
			}																																							//            						safety value to indicate craft has taken off for mix increase safety
	}

#ifdef STICK_TRAVEL_CHECK																				//This feature completely disables throttle and allows visual feedback if control inputs reach full throws
//Stick endpoints check tied to aux channel stick gesture
if (aux[CH_AUX1]){
	throttle = 0;
	if ((rx[0]<= -0.99f) || (rx[0] >= 0.99f) || (rx[1] <= -0.99f) || (rx[1] >= 0.99f) || (rx[2] <= -0.99f) || (rx[2] >= 0.99f) || (rx[3] <= 0.0f) || (rx[3] >= 0.99f)){
		ledcommand = 1;}
}
#endif



// turn motors off if throttle is off and pitch / roll sticks are centered
	if ( failsafe || (throttle < 0.001f && (!ENABLESTIX || !onground_long || aux[LEVELMODE] || (fabsf(rx[ROLL]) < (float) ENABLESTIX_TRESHOLD && fabsf(rx[PITCH]) < (float) ENABLESTIX_TRESHOLD && fabsf(rx[YAW]) < (float) ENABLESTIX_TRESHOLD ) ) ) ) 
	{	// motors off

		if ( onground_long )
		{
			if ( gettime() - onground_long > ENABLESTIX_TIMEOUT)
			{
				onground_long = 0;
			}
		}	
		
		for ( int i = 0 ; i <= 3 ; i++)
		{
			pwm_set( i , 0 );	
			#ifdef MOTOR_FILTER	
			// reset the motor filter
			motorfilter( 0 , i);
			#endif
		}	
		
		#ifdef MOTOR_BEEPS
		extern void motorbeep( void);
		motorbeep();
		#endif

		#ifdef MIX_LOWER_THROTTLE
		// reset the overthrottle filter
		lpf(&overthrottlefilt, 0.0f, 0.72f);	// 50hz 1khz sample rate
		lpf(&underthrottlefilt, 0.0f, 0.72f);	// 50hz 1khz sample rate
		#endif				
		
		#ifdef STOCK_TX_AUTOCENTER
		for( int i = 0 ; i <3;i++)
			{
				if ( rx[i] == lastrx[i] )
					{
						consecutive[i]++;
						
					}
				else consecutive[i] = 0;
				lastrx[i] = rx[i];
				if ( consecutive[i] > 1000 && fabsf( rx[i]) < 0.1f )
					{
						autocenter[i] = rx[i];
					}
			}
		#endif				
		
	
		throttle = 0;										//zero out throttle so it does not come back on as idle up value if enabled			
		onground = 1;
		thrsum = 0;
		
	}
	else
	{// motors on - normal flight
		
		onground = 0;
		onground_long = gettime();
		
		float mix[4];	

#ifdef 	THROTTLE_TRANSIENT_COMPENSATION
        
#ifndef THROTTLE_TRANSIENT_COMPENSATION_FACTOR 
 #define THROTTLE_TRANSIENT_COMPENSATION_FACTOR 7.0 
#endif        
extern float throttlehpf( float in );
        
		  throttle += (float) (THROTTLE_TRANSIENT_COMPENSATION_FACTOR) * throttlehpf(throttle);
		  if (throttle < 0)
			  throttle = 0;
		  if (throttle > 1.0f)
			  throttle = 1.0f;
#endif
		
	if ( controls_override)
	{// change throttle in flip mode
		throttle = rx_override[3];
	}
		
	
		  // throttle angle compensation
#ifdef AUTO_THROTTLE
		  if (aux[LEVELMODE])
		    {
			    //float autothrottle = fastcos(attitude[0] * DEGTORAD) * fastcos(attitude[1] * DEGTORAD);
			    extern float GEstG[];
				float autothrottle = GEstG[2];
				float old_throttle = throttle;
			    if (autothrottle <= 0.5f)
				    autothrottle = 0.5f;
			    throttle = throttle / autothrottle;
			    // limit to 90%
			    if (old_throttle < 0.9f)
				    if (throttle > 0.9f)
					    throttle = 0.9f;

			    if (throttle > 1.0f)
				    throttle = 1.0f;

		    }
#endif
	           
            
#ifdef LVC_LOWER_THROTTLE
	
	#ifdef SWITCHABLE_FEATURE_2
	extern float vbatt_comp;
	extern float vbattfilt;
	extern int flash_feature_2;
	static float throttle_i = 0.0f;
	float throttle_p = 0.0f;
		if (flash_feature_2 == 1){
			// can be made into a function
			if (vbattfilt < (float) LVC_LOWER_THROTTLE_VOLTAGE_RAW ) 
			throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE_RAW - vbattfilt) *(float) LVC_LOWER_THROTTLE_KP;
			// can be made into a function
			if (vbatt_comp < (float) LVC_LOWER_THROTTLE_VOLTAGE) 
			throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE - vbatt_comp) *(float) LVC_LOWER_THROTTLE_KP;	

			if ( throttle_p > 1.0f ) throttle_p = 1.0f;	

			if ( throttle_p > 0 ) 
			{
				throttle_i += throttle_p * 0.0001f; //ki
			}
			else throttle_i -= 0.001f;// ki on release

			if ( throttle_i > 0.5f) throttle_i = 0.5f;
			if ( throttle_i < 0.0f) throttle_i = 0.0f;

			throttle -= throttle_p + throttle_i;
		}else{
			//do nothing - feature is disabled via stick gesture
		}
	#else 
	extern float vbatt_comp;
	extern float vbattfilt;
	static float throttle_i = 0.0f;
	float throttle_p = 0.0f;
		// can be made into a function
		if (vbattfilt < (float) LVC_LOWER_THROTTLE_VOLTAGE_RAW ) 
		throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE_RAW - vbattfilt) *(float) LVC_LOWER_THROTTLE_KP;
		// can be made into a function
		if (vbatt_comp < (float) LVC_LOWER_THROTTLE_VOLTAGE) 
		throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE - vbatt_comp) *(float) LVC_LOWER_THROTTLE_KP;	

		if ( throttle_p > 1.0f ) throttle_p = 1.0f;

		if ( throttle_p > 0 ) 
		{
			throttle_i += throttle_p * 0.0001f; //ki
		}
		else throttle_i -= 0.001f;// ki on release

		if ( throttle_i > 0.5f) throttle_i = 0.5f;
		if ( throttle_i < 0.0f) throttle_i = 0.0f;

		throttle -= throttle_p + throttle_i;		
	#endif
#endif


#ifdef INVERT_YAW_PID
	#ifdef SWITCHABLE_FEATURE_3
	extern int flash_feature_3;
		if (flash_feature_3 == 0){
			pidoutput[2] = -pidoutput[2];
		}else{
			//do nothing
		}
	#else
		pidoutput[2] = -pidoutput[2];
	#endif
#endif
	
#ifdef INVERTED_ENABLE
if (pwmdir == REVERSE)
		{
			// inverted flight
		
		mix[MOTOR_FR] = throttle + pidoutput[ROLL] + pidoutput[PITCH] - pidoutput[YAW];		// FR
		mix[MOTOR_FL] = throttle - pidoutput[ROLL] + pidoutput[PITCH] + pidoutput[YAW];		// FL	
		mix[MOTOR_BR] = throttle + pidoutput[ROLL] - pidoutput[PITCH] + pidoutput[YAW];		// BR
		mix[MOTOR_BL] = throttle - pidoutput[ROLL] - pidoutput[PITCH] - pidoutput[YAW];		// BL	
		

		}	
else
#endif    
{
    // normal mixer
		mix[MOTOR_FR] = throttle - pidoutput[ROLL] - pidoutput[PITCH] + pidoutput[YAW];		// FR
		mix[MOTOR_FL] = throttle + pidoutput[ROLL] - pidoutput[PITCH] - pidoutput[YAW];		// FL	
		mix[MOTOR_BR] = throttle - pidoutput[ROLL] + pidoutput[PITCH] - pidoutput[YAW];		// BR
		mix[MOTOR_BL] = throttle + pidoutput[ROLL] + pidoutput[PITCH] + pidoutput[YAW];		// BL	
}

#ifdef INVERT_YAW_PID
// we invert again cause it's used by the pid internally (for limit)
	#ifdef SWITCHABLE_FEATURE_3
	extern int flash_feature_3;
		if (flash_feature_3 == 0){
			pidoutput[2] = -pidoutput[2];
		}else{
			//do nothing
		}
	#else
		pidoutput[2] = -pidoutput[2];
	#endif		
#endif

		for ( int i = 0 ; i <= 3 ; i++)
		{			
		#ifdef MOTOR_FILTER		
		mix[i] = motorfilter(  mix[i] , i);
		#endif	
		
       		#ifdef MOTOR_FILTER2_ALPHA	
       		 float motorlpf( float in , int x) ;           
		mix[i] = motorlpf(  mix[i] , i);
		#endif
			
		#ifdef MOTOR_KAL
      		float motor_kalman( float in , int x);
       		mix[i] = motor_kalman(  mix[i] , i);  
       		#endif 
			
		#ifdef TORQUE_BOOST
       		float motord( float in , int x);           
		mix[i] = motord(  mix[i] , i);
		#endif
       		}


#if ( defined MIX_LOWER_THROTTLE || defined MIX_INCREASE_THROTTLE)

//#define MIX_INCREASE_THROTTLE

// options for mix throttle lowering if enabled
// 0 - 100 range ( 100 = full reduction / 0 = no reduction )
#ifndef MIX_THROTTLE_REDUCTION_PERCENT
#define MIX_THROTTLE_REDUCTION_PERCENT 100
#endif
// lpf (exponential) shape if on, othewise linear
//#define MIX_THROTTLE_FILTER_LPF

// limit reduction and increase to this amount ( 0.0 - 1.0)
// 0.0 = no action 
// 0.5 = reduce up to 1/2 throttle      
//1.0 = reduce all the way to zero 
#ifndef MIX_THROTTLE_REDUCTION_MAX
#define MIX_THROTTLE_REDUCTION_MAX 0.5
#endif

#ifndef MIX_MOTOR_MAX
#define MIX_MOTOR_MAX 1.0f
#endif


		  float overthrottle = 0;
		  float underthrottle = 0.001f;
		
		  for (int i = 0; i < 4; i++)
		    {
			    if (mix[i] > overthrottle)
				    overthrottle = mix[i];
					if (mix[i] < underthrottle)
						underthrottle = mix[i];
		    }

#ifdef MIX_LOWER_THROTTLE
            
		  overthrottle -= MIX_MOTOR_MAX ;

		  if (overthrottle > (float)MIX_THROTTLE_REDUCTION_MAX)
			  overthrottle = (float)MIX_THROTTLE_REDUCTION_MAX;

#ifdef MIX_THROTTLE_FILTER_LPF
		  if (overthrottle > overthrottlefilt)
			  lpf(&overthrottlefilt, overthrottle, 0.82);	// 20hz 1khz sample rate
		  else
			  lpf(&overthrottlefilt, overthrottle, 0.72);	// 50hz 1khz sample rate
#else
		  if (overthrottle > overthrottlefilt)
			  overthrottlefilt += 0.005f;
		  else
			  overthrottlefilt -= 0.01f;
#endif
#else
overthrottle = 0.0f;        
#endif
          
#ifdef MIX_INCREASE_THROTTLE
// under			
			
		  if (underthrottle < -(float)MIX_THROTTLE_REDUCTION_MAX)
			  underthrottle = -(float)MIX_THROTTLE_REDUCTION_MAX;
			
#ifdef MIX_THROTTLE_FILTER_LPF
		  if (underthrottle < underthrottlefilt)
			  lpf(&underthrottlefilt, underthrottle, 0.82);	// 20hz 1khz sample rate
		  else
			  lpf(&underthrottlefilt, underthrottle, 0.72);	// 50hz 1khz sample rate
#else
		  if (underthrottle < underthrottlefilt)
			  underthrottlefilt -= 0.005f;
		  else
			  underthrottlefilt += 0.01f;
#endif
// under
			if (underthrottlefilt < - (float)MIX_THROTTLE_REDUCTION_MAX)
			  underthrottlefilt = - (float)MIX_THROTTLE_REDUCTION_MAX;
		  if (underthrottlefilt > 0.1f)
			  underthrottlefilt = 0.1;

			underthrottle = underthrottlefilt;
					
			if (underthrottle > 0.0f)
			  underthrottle = 0.0001f;

			underthrottle *= ((float)MIX_THROTTLE_REDUCTION_PERCENT / 100.0f);
#else
  underthrottle = 0.001f;			
#endif			
// over			
		  if (overthrottlefilt > (float)MIX_THROTTLE_REDUCTION_MAX)
			  overthrottlefilt = (float)MIX_THROTTLE_REDUCTION_MAX;
		  if (overthrottlefilt < -0.1f)
			  overthrottlefilt = -0.1;


		  overthrottle = overthrottlefilt;

			
		  if (overthrottle < 0.0f)
			  overthrottle = -0.0001f;

			
			// reduce by a percentage only, so we get an inbetween performance
			overthrottle *= ((float)MIX_THROTTLE_REDUCTION_PERCENT / 100.0f);

			
			
		  if (overthrottle > 0 || underthrottle < 0 )
		    {		// exceeding max motor thrust
					float temp = overthrottle + underthrottle;
			    for (int i = 0; i < 4; i++)
			      {
				      mix[i] -= temp;
			      }
		    }
#endif				


#ifdef MIX_LOWER_THROTTLE_3
{
#ifndef MIX_THROTTLE_REDUCTION_MAX
#define MIX_THROTTLE_REDUCTION_MAX 0.5f
#endif

float overthrottle = 0;

for (int i = 0; i < 4; i++)
		    {
			    if (mix[i] > overthrottle)
				    overthrottle = mix[i];
            }


overthrottle -=1.0f;
// limit to half throttle max reduction
if ( overthrottle > (float) MIX_THROTTLE_REDUCTION_MAX)  overthrottle = (float) MIX_THROTTLE_REDUCTION_MAX;

if ( overthrottle > 0.0f)
{
    for ( int i = 0 ; i < 4 ; i++)
        mix[i] -= overthrottle;
}
#ifdef MIX_THROTTLE_FLASHLED
if ( overthrottle > 0.1f) ledcommand = 1;
#endif
}
#endif


#ifdef MIX_INCREASE_THROTTLE_3
{
#ifndef MIX_THROTTLE_INCREASE_MAX
#define MIX_THROTTLE_INCREASE_MAX 0.2f
#endif
	if (in_air == 1){
		float underthrottle = 0;

		for (int i = 0; i < 4; i++)
			{
					if (mix[i] < underthrottle)
							underthrottle = mix[i];
			}


		// limit to half throttle max reduction
		if ( underthrottle < -(float) MIX_THROTTLE_INCREASE_MAX)  underthrottle = -(float) MIX_THROTTLE_INCREASE_MAX;

		if ( underthrottle < 0.0f)
			{
					for ( int i = 0 ; i < 4 ; i++)
            mix[i] -= underthrottle;
			}
		#ifdef MIX_THROTTLE_FLASHLED
			if ( underthrottle < -0.01f) ledcommand = 1;
		#endif
	}
}
#endif

            
            
            
thrsum = 0;		
				
		for ( int i = 0 ; i <= 3 ; i++)
		{			
		           
		#ifdef CLIP_FF
		mix[i] = clip_ff(mix[i], i);
		#endif

		#if defined(MOTORS_TO_THROTTLE) || defined(MOTORS_TO_THROTTLE_MODE)
		#if defined(MOTORS_TO_THROTTLE_MODE) && !defined(MOTORS_TO_THROTTLE)
		if(aux[MOTORS_TO_THROTTLE_MODE])
		{
		#endif
		mix[i] = throttle;
		if ( i == MOTOR_FL && ( rx[ROLL] > 0.5f || rx[PITCH] < -0.5f ) ) { mix[i] = 0; }
		if ( i == MOTOR_BL && ( rx[ROLL] > 0.5f || rx[PITCH] > 0.5f ) ) { mix[i] = 0; }
		if ( i == MOTOR_FR && ( rx[ROLL] < -0.5f || rx[PITCH] < -0.5f ) ) { mix[i] = 0; }
		if ( i == MOTOR_BR && ( rx[ROLL] < -0.5f || rx[PITCH] > 0.5f ) ) { mix[i] = 0; }
		#if defined(MOTORS_TO_THROTTLE_MODE) && !defined(MOTORS_TO_THROTTLE)
		}
		#endif

		// flash leds in valid throttle range
		#ifdef MOTORS_TO_THROTTLE
		ledcommand = 1;
		#warning "MOTORS TEST MODE"
		#endif
		#endif

		#ifdef MOTOR_MIN_ENABLE
		if (mix[i] < (float) MOTOR_MIN_VALUE)
		{
			mix[i] = (float) MOTOR_MIN_VALUE;
		}
		#endif
		
			
		#ifndef NOMOTORS
		#ifndef MOTORS_TO_THROTTLE
		//normal mode
		pwm_set( i ,motormap( mix[i] ) );
		#else
		// throttle test mode
		ledcommand = 1;
		pwm_set( i , mix[i] );
		#endif
		#else
		// no motors mode ( anti-optimization)
		#warning "NO MOTORS"
		tempx[i] = motormap( mix[i] );
		#endif
		
		if ( mix[i] < 0 ) mix[i] = 0;
		if ( mix[i] > 1 ) mix[i] = 1;
		thrsum+= mix[i];
		}	
		thrsum = thrsum / 4;
		
	}// end motors on
	
}


#ifndef MOTOR_FILTER2_ALPHA
#define MOTOR_FILTER2_ALPHA 0.3
#endif


float motor_filt[4];
#if defined(MOTOR_FILTER2_ALPHA) && defined(SWITCHABLE_MOTOR_FILTER2_ALPHA) && defined(SWITCHABLE_FEATURE_1)

float motorlpf( float in , int x)
{ 
		extern int flash_feature_1;
		if (flash_feature_1 == 0){
    lpf(&motor_filt[x] , in , 1 - MOTOR_FILTER2_ALPHA);
    }else{
		lpf(&motor_filt[x] , in , 1 - SWITCHABLE_MOTOR_FILTER2_ALPHA);		
		}
    return motor_filt[x];	
}
#else
float motorlpf( float in , int x)
{ 
	lpf(&motor_filt[x] , in , 1 - MOTOR_FILTER2_ALPHA);
  return motor_filt[x];
}
#endif

float hann_lastsample[4];
float hann_lastsample2[4];

// hanning 3 sample filter
float motorfilter( float motorin ,int number)
{
 	float ans = motorin*0.25f + hann_lastsample[number] * 0.5f +   hann_lastsample2[number] * 0.25f ;
	
	hann_lastsample2[number] = hann_lastsample[number];
	hann_lastsample[number] = motorin;
	
	return ans;
}


    //initial values for the kalman filter 
    float x_est_last[4] ;
    float P_last[4] ; 
    //the noise in the system 
    const float Q = 0.02;
//the noise in the system ( variance -  squared )
   
    #ifdef MOTOR_KAL
    const float R = Q/(float)MOTOR_KAL;
    #else
    float R = 0.1;
    #endif

float  motor_kalman( float in , int x)   
{    


    
        //do a prediction 
       float x_temp_est = x_est_last[x]; 
       float P_temp = P_last[x] + Q; 

       float K = P_temp * (1.0f/(P_temp + R));
       float x_est = x_temp_est + K * (in - x_temp_est);  
       float P = (1- K) * P_temp; 
       
        //update our last's 
        P_last[x] = P; 
        x_est_last[x] = x_est; 
//

return x_est;
}	
	
	
float clip_feedforward[4];
// clip feedforward adds the amount of thrust exceeding 1.0 ( max) 
// to the next iteration(s) of the loop
// so samples 0.5 , 1.5 , 0.4 would transform into 0.5 , 1.0 , 0.9;

float clip_ff(float motorin, int number)
{

	if (motorin > 1.0f)
	  {
		  clip_feedforward[number] += (motorin - 1.0f);
		  //cap feedforward to prevent windup 
		  if (clip_feedforward[number] > .5f)
			  clip_feedforward[number] = .5f;
	  }
	else if (clip_feedforward[number] > 0)
	  {
		  float difference = 1.0f - motorin;
		  motorin = motorin + clip_feedforward[number];
		  if (motorin > 1.0f)
		    {
			    clip_feedforward[number] -= difference;
			    if (clip_feedforward[number] < 0)
				    clip_feedforward[number] = 0;
		    }
		  else
			  clip_feedforward[number] = 0;

	  }
	return motorin;
}

#ifndef TORQUE_BOOST
    #define TORQUE_BOOST   0.0
#endif

 float motord( float in , int x)
 {
   float factor = TORQUE_BOOST;
   static float lastratexx[4][4];
     
        float out  =  ( + 0.125f *in + 0.250f * lastratexx[x][0]
                    - 0.250f * lastratexx[x][2] - ( 0.125f) * lastratexx[x][3]) * factor; 						;

        lastratexx[x][3] = lastratexx[x][2];
        lastratexx[x][2] = lastratexx[x][1];
        lastratexx[x][1] = lastratexx[x][0];
        lastratexx[x][0] = in;
        
    return in + out;
 }

