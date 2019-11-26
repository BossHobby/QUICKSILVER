
#define IMU_SAMPLE_RATE         200.0f

#define IMU_FILTER_CUTOFF_FREQ	30.0f

#define M_PI_F                               3.1415926


extern void IIRFilter_Init(void);

extern void LPF2pSetCutOffFreq_1(float sample_freq, float cutoff_freq);
extern void LPF2pSetCutOffFreq_2(float sample_freq, float cutoff_freq);
extern void LPF2pSetCutOffFreq_3(float sample_freq, float cutoff_freq);
extern void LPF2pSetCutOffFreq_4(float sample_freq, float cutoff_freq);
extern void LPF2pSetCutOffFreq_5(float sample_freq, float cutoff_freq);
extern void LPF2pSetCutOffFreq_6(float sample_freq, float cutoff_freq);

extern float LPF2pApply_1(float sample);
extern float LPF2pApply_2(float sample);
extern float LPF2pApply_3(float sample);
extern float LPF2pApply_4(float sample);
extern float LPF2pApply_5(float sample);
extern float LPF2pApply_6(float sample);



