

float lpfcalc(float sampleperiod, float filtertime);
float lpfcalc_hz(float sampleperiod, float filterhz);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void lpf(float *out, float in, float coeff);
void limitf(float *input, const float limit);
int ipow(int base, int exp);
int round_num(float num);
float round_dec2(float num);
void TS(void);
unsigned long TE(void);

float fastsin(float x);
float fastcos(float x);

void limit180(float *);
