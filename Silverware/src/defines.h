
// defines for things that do not normally need changing

#define MOTOR_BL 0
#define MOTOR_FL 1
#define MOTOR_FR 3
#define MOTOR_BR 2

#define PIDNUMBER 3

#define DEGTORAD 0.017453292f
#define RADTODEG 57.29577951f

#define AUXNUMBER 16

#define ROLL 0
#define PITCH 1
#define YAW 2

// this should be precalculated by the compiler as it's a constant
#define FILTERCALC( sampleperiod, filtertime) (1.0f - ( 6.0f*(float)sampleperiod) / ( 3.0f *(float)sampleperiod + (float)filtertime))


#define RXMODE_BIND 0
#define RXMODE_NORMAL (!RXMODE_BIND)

/*
#define CH_ON 4
#define CH_OFF 5
#define CH_FLIP 0
#define CH_EXPERT 1
#define CH_HEADFREE 2
#define CH_RTH 3
*/

// defines for bayang protocol radio
#define CH_ON (AUXNUMBER - 2)
#define CH_OFF (AUXNUMBER - 1)
#define CH_FLIP 0
#define CH_EXPERT 1
#define CH_HEADFREE 2
#define CH_RTH 3
#define CH_AUX1 4
#define CH_AUX2 5
#define CH_EMG 10
#define CH_TO 11
// trims numbers have to be sequential, start at CH_PIT_TRIM
#define CH_PIT_TRIM 6
#define CH_RLL_TRIM 7
#define CH_THR_TRIM 8
#define CH_YAW_TRIM 9
// next 3 channels only when *not* using USE_STOCK_TX
#define CH_INV 6
#define CH_VID 7
#define CH_PIC 8

// defines for cg023 protocol
#define CH_CG023_LED 3
#define CH_CG023_FLIP 0
#define CH_CG023_STILL 2
#define CH_CG023_VIDEO 1

#define CH_H7_FLIP 0
#define CH_H7_VIDEO 1
#define CH_H7_FS 2

#define CH_CX10_CH0 0
#define CH_CX10_CH2 2

#define CH_AUX3 CH_OFF
#define CH_AUX4 CH_OFF

#ifdef USE_DEVO
// devo tx channel mapping
// also for nr24multipro
#define CHAN_5 CH_INV
#define CHAN_6 CH_FLIP
#define CHAN_7 CH_PIC
#define CHAN_8 CH_VID
#define CHAN_9 CH_HEADFREE
#define CHAN_10 CH_RTH
#define CHAN_ON CH_ON
#define CHAN_OFF CH_OFF
#endif

#ifdef USE_MULTI
// multimodule mapping ( taranis )
#define CHAN_5 CH_FLIP
#define CHAN_6 CH_RTH
#define CHAN_7 CH_PIC
#define CHAN_8 CH_VID
#define CHAN_9 CH_HEADFREE
#define CHAN_10 CH_INV
#define CHAN_ON CH_ON
#define CHAN_OFF CH_OFF
#endif

#ifdef USE_STOCK_TX
#define CHAN_5 CH_EXPERT
#define CHAN_6 CH_AUX1
#define CHAN_7 CH_HEADFREE
#define CHAN_8 CH_RLL_TRIM
#define CHAN_9 CH_PIT_TRIM
#define CHAN_10 CH_OFF
#define CHAN_ON CH_ON
#define CHAN_OFF CH_OFF
#endif


#ifdef BETAFLIGHT_RATES
#define ACRO_EXPO_ROLL BF_EXPO_ROLL
#define ACRO_EXPO_PITCH BF_EXPO_PITCH
#define ACRO_EXPO_YAW BF_EXPO_YAW
#define ANGLE_EXPO_ROLL BF_EXPO_ROLL
#define ANGLE_EXPO_PITCH BF_EXPO_PITCH
#define ANGLE_EXPO_YAW BF_EXPO_YAW
#define MAX_RATE 200*(float)BF_RC_RATE_ROLL*(1/1-(float)BF_SUPER_RATE_ROLL)     // roll max rate used for flip sequencer when bf rates selected
#endif


// used for the pwm driver
#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3


#define int32 int_fast32_t
#define int16 int_fast16_t
#define int8 int_fast8_t

#define uint32 uint_fast32_t
#define uint16 uint_fast16_t
#define uint8 uint_fast8_t

// for h-bridge states
#define FREE 2
#define BRAKE 3
#define DIR1 1
#define DIR2 0

// for inverted flight motor direction
#define FORWARD DIR2
#define REVERSE DIR1


#ifdef KALMAN_GYRO
// kalman Q/R ratio for Q = 0.02
// loop time 1000Hz
#define	HZ_10	0.004078
#define	HZ_20	0.015952
#define	HZ_30	0.035546
#define	HZ_40	0.062984
#define	HZ_50	0.097857
#define	HZ_60	0.139957
#define	HZ_70	0.190992
#define	HZ_80	0.249072
#define	HZ_90	0.308894
#define	HZ_100	0.397188
#define	HZ_120	0.542488
#define	HZ_140	0.719026
#define	HZ_160	0.928746
#define	HZ_180	1.144837
#define	HZ_200	1.354386
#define	HZ_220	1.611742
#define	HZ_240	1.87532
#define	HZ_260	2.123421
#define	HZ_280	2.377006
#define	HZ_300	2.595641
#define	HZ_320	2.864404
#define	HZ_340	3.052077
#define	HZ_360	3.272997
#define	HZ_380	3.44942
#define	HZ_400	3.679173
#define	HZ_420	3.721861
#define	HZ_440	3.880844
#define	HZ_460	3.908564
#define	HZ_480	3.984022
#define	HZ_500	4.100000
#endif

#ifdef PT1_GYRO
#define	HZ_10	10
#define	HZ_20	20
#define	HZ_30	30
#define	HZ_40	40
#define	HZ_50	50
#define	HZ_60	60
#define	HZ_70	70
#define	HZ_80	80
#define	HZ_90	90
#define	HZ_100	100
#define	HZ_120	120
#define	HZ_140	140
#define	HZ_160	160
#define	HZ_180	180
#define	HZ_200	200
#define	HZ_220	220
#define	HZ_240	240
#define	HZ_260	260
#define	HZ_280	280
#define	HZ_300	300
#define	HZ_320	320
#define	HZ_340	340
#define	HZ_360	360
#define	HZ_380	380
#define	HZ_400	400
#define	HZ_420	420
#define	HZ_440	440
#define	HZ_460	460
#define	HZ_480	480
#define	HZ_500	500
#endif
// 1st order lpf alpha
// for 1000Hz loop frequency
#define	MFILT1_HZ_10	0.056677
#define	MFILT1_HZ_20	0.109243
#define	MFILT1_HZ_30	0.15976
#define	MFILT1_HZ_40	0.207311
#define	MFILT1_HZ_50	0.250878
#define	MFILT1_HZ_60	0.292612
#define	MFILT1_HZ_70	0.331242
#define	MFILT1_HZ_80	0.366444
#define	MFILT1_HZ_90	0.406108
#define	MFILT1_HZ_100	0.434536
#define	MFILT1_HZ_120	0.49997
#define	MFILT1_HZ_140	0.543307
#define	MFILT1_HZ_160	0.582436
#define	MFILT1_HZ_180	0.631047
#define	MFILT1_HZ_200	0.67169
#define	MFILT1_HZ_220	0.697849
#define	MFILT1_HZ_240	0.714375
#define	MFILT1_HZ_260	0.725199
#define	MFILT1_HZ_280	0.740312
#define	MFILT1_HZ_300	0.758612
#define	MFILT1_HZ_320	0.773861
#define	MFILT1_HZ_340	0.79364
#define	MFILT1_HZ_360	0.803003
#define	MFILT1_HZ_380	0.809752
#define	MFILT1_HZ_400	0.817944
#define	MFILT1_HZ_420	0.81943
#define	MFILT1_HZ_440	0.824737
#define	MFILT1_HZ_460	0.825618
#define	MFILT1_HZ_480	0.827956
#define	MFILT1_HZ_500	0.836544












