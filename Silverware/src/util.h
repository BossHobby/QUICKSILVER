


float lpfcalc( float sampleperiod , float filtertime);
float lpfcalc_hz(float sampleperiod, float filterhz);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void lpf( float *out, float in , float coeff);

float rcexpo ( float x , float exp );

void limitf ( float *input , const float limit);

void TS( void);
void TE( void);

float fastsin( float x );
float fastcos( float x );


void limit180(float *);
