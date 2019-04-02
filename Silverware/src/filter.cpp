
#include "config.h"
#include "defines.h"

#ifndef GYRO_FILTER_PASS1
  #define SOFT_LPF1_NONE
#endif
#ifndef GYRO_FILTER_PASS2
  #define SOFT_LPF2_NONE
#endif


#if defined PT1_GYRO && defined GYRO_FILTER_PASS1
	#define SOFT_LPF_1ST_PASS1 GYRO_FILTER_PASS1
extern "C" float lpfcalc( float sampleperiod , float filtertime);
extern "C" float lpfcalc_hz(float sampleperiod, float filterhz);
extern "C" void lpf( float *out, float in , float coeff);

static float alpha = 0.5;
extern float looptime;

void lpf_coeff()
{
 alpha = FILTERCALC( looptime , (1.0f/SOFT_LPF_1ST_PASS1) );       
}


class  filter_lpf1
{
    private:
		float lpf_last;
    public:
        filter_lpf1()
    {
      lpf_last = 0;   
    }
     float step( float in)
     {
       lpf ( &lpf_last , in , alpha); 
         
       return lpf_last;
     }
};

filter_lpf1 filter[3];
#endif


#if defined PT1_GYRO && defined GYRO_FILTER_PASS2
	#define SOFT_LPF_1ST_PASS2 GYRO_FILTER_PASS2
extern "C" float lpfcalc( float sampleperiod , float filtertime);
extern "C" float lpfcalc_hz(float sampleperiod, float filterhz);
extern "C" void lpf( float *out, float in , float coeff);

static float alpha2 = 0.5;
extern float looptime;

void lpf_coeff_pass2()
{
 alpha2 = FILTERCALC( looptime , (1.0f/SOFT_LPF_1ST_PASS2) );       
}


class  filter_lpf2
{
    private:
		float lpf_last;
    public:
        filter_lpf2()
    {
      lpf_last = 0;   
    }
     float step( float in)
     {
       lpf ( &lpf_last , in , alpha2); 
         
       return lpf_last;
     }
};

filter_lpf2 filter2[3];
#endif



#if defined KALMAN_GYRO && defined GYRO_FILTER_PASS1
 #define SOFT_KALMAN_GYRO_PASS1 GYRO_FILTER_PASS1
class  filter_kalman
{
    private:
        float x_est_last ;
        float P_last ; 
        float Q;
        float R;
    public:
        filter_kalman()
        {
            Q = 0.02; 
            R = 0.1;

            #ifdef SOFT_KALMAN_GYRO_PASS1
            R = Q/(float)SOFT_KALMAN_GYRO_PASS1;
            #endif
					
        }
        float  step( float in )   
        {    

            //do a prediction 
            float x_temp_est = x_est_last; 
            float P_temp = P_last + Q; 

            float K = P_temp * (1.0f/(P_temp + R));
            float x_est = x_temp_est + K * (in - x_temp_est);  
            float P = (1- K) * P_temp; 
           
            //update our last's 
            P_last= P; 
            x_est_last = x_est; 

            return x_est;
        }
};       
filter_kalman filter[3];       
#endif

#if defined KALMAN_GYRO && defined GYRO_FILTER_PASS2
	#define SOFT_KALMAN_GYRO_PASS2 GYRO_FILTER_PASS2
class  filter_kalman2
{
    private:
        float x_est_last ;
        float P_last ; 
        float Q;
        float R;
    public:
        filter_kalman2()
        {
            Q = 0.02; 
            R = 0.1;

						#ifdef SOFT_KALMAN_GYRO_PASS2
					  R = Q/(float)SOFT_KALMAN_GYRO_PASS2;
            #endif
        }
        float  step( float in )   
        {    

            //do a prediction 
            float x_temp_est = x_est_last; 
            float P_temp = P_last + Q; 

            float K = P_temp * (1.0f/(P_temp + R));
            float x_est = x_temp_est + K * (in - x_temp_est);  
            float P = (1- K) * P_temp; 
           
            //update our last's 
            P_last= P; 
            x_est_last = x_est; 

            return x_est;
        }
};       
filter_kalman2 filter2[3];       
#endif


extern "C" float lpffilter( float in,int num )
{
	#ifdef SOFT_LPF1_NONE
	return in;
	#else
    
    #ifdef SOFT_LPF_1ST_PASS1
    if ( num == 0 ) lpf_coeff();
    #endif
       
	return filter[num].step(in );   
	#endif
	
}



 extern "C" float lpffilter2( float in,int num )
{
	#ifdef SOFT_LPF2_NONE
	return in;
	#else
    
    #ifdef SOFT_LPF_1ST_PASS2
    if ( num == 0 ) lpf_coeff_pass2();
    #endif
      
	return filter2[num].step(in );   
	#endif
	
} 
// 16Hz hpf filter for throttle compensation
//High pass bessel filter order=1 alpha1=0.016 
class  FilterBeHp1
{
	public:
		FilterBeHp1()
		{
			v[0]=0.0;
		}
	private:
		float v[2];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = (9.521017968695103528e-1f * x)
				 + (0.90420359373902081668f * v[0]);
			return 
				 (v[1] - v[0]);
		}
};

FilterBeHp1 throttlehpf1;

extern "C" float throttlehpf( float in )
{
	return throttlehpf1.step(in );
}



// for TRANSIENT_WINDUP_PROTECTION feature
//Low pass bessel filter order=1 alpha1=0.023
class  FilterSP
{
	public:
		FilterSP()
		{
			v[0]=0.0;
		}
	private:
		float v[2];
	public:
		float step(float x) //class II
		{
			v[0] = v[1];
			v[1] = (6.749703162983405891e-2f * x)
				 + (0.86500593674033188218f * v[0]);
			return
				 (v[0] + v[1]);
		}
};

FilterSP spfilter[3];

extern "C" float splpf( float in,int num )
{

	return spfilter[num].step(in );
}

