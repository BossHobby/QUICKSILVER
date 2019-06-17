#include "hardware.h"

// defines for things that do not normally need changing

#pragma diag_warning 1035 , 177 , 4017
#pragma diag_error 260

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

// this should be precalculated by the compiler when parameters are constant
//(1 - alpha. filtertime = 1 / filter-cutoff-frequency) as long as filtertime > sampleperiod
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


#ifdef ALIENWHOOP_ZERO_FILTERING
#define KALMAN_GYRO
#define GYRO_FILTER_PASS1 HZ_90
#define  DTERM_LPF_2ND_HZ 100
#define MOTOR_FILTER2_ALPHA MFILT1_HZ_50
#define SWITCHABLE_MOTOR_FILTER2_ALPHA MFILT1_HZ_90
#define SWITCHABLE_FEATURE_1
#endif

#ifdef WEAK_FILTERING
#define KALMAN_GYRO
#define GYRO_FILTER_PASS1 HZ_90
#define  DTERM_LPF_2ND_HZ 100
#define MOTOR_FILTER2_ALPHA MFILT1_HZ_90
#endif

#ifdef STRONG_FILTERING
#define KALMAN_GYRO
#define GYRO_FILTER_PASS1 HZ_80
#define  DTERM_LPF_2ND_HZ 90
#define MOTOR_FILTER2_ALPHA MFILT1_HZ_80
#endif

#ifdef VERY_STRONG_FILTERING
#define KALMAN_GYRO
#define GYRO_FILTER_PASS1 HZ_70
#define  DTERM_LPF_2ND_HZ 80
#define MOTOR_FILTER2_ALPHA MFILT1_HZ_70
#endif

#ifdef BETA_FILTERING
	#if (!defined(KALMAN_GYRO) && !defined(PT1_GYRO)) || (!defined(GYRO_FILTER_PASS1) && !defined(GYRO_FILTER_PASS2))
		#define SOFT_LPF_NONE
	#endif
#endif

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


// *************0 - 7 - power for bayang telemetry
#define TX_POWER 7

// *************led brightness in-flight ( solid lights only)
// *************0- 15 range
#define LED_BRIGHTNESS 15

// *************Comment out to disable pid tuning gestures
#define PID_GESTURE_TUNING
#define COMBINE_PITCH_ROLL_PID_TUNING

// *************Comment out for original relative proportional inc/dec steps for PID values
#define PID_TUNING_INCDEC_FACTOR 0.5f //fixed inc/dec values for PID tuning - by SilverAG
//Default value is 0.5f which means that PID values will be increased or decreased by constant index of 0.5
//For example, if PID value is 10.0 it will be incremented by 0.5 to 10.5 or decremented by 0.5 to 9.5
//Feel free to change 0.5f value to your liking

// *************flash save method
// *************flash_save 1: pids + accel calibration
// *************flash_save 2: accel calibration to option bytes
#define FLASH_SAVE1
//#define FLASH_SAVE2

// flash saving features
//#define DISABLE_GESTURES2

//enables use of stick accelerator and stick transition for d term lpf 1 & 2
#define ADVANCED_PID_CONTROLLER

//Throttle must drop below this value if arming feature is enabled for arming to take place.  MIX_INCREASE_THROTTLE_3 if enabled
//will also not activate on the ground untill this threshold is passed during takeoff for safety and better staging behavior.
#define THROTTLE_SAFETY .10f

// level mode "manual" trims ( in degrees)
// pitch positive forward
// roll positive right
#define TRIM_PITCH 0.0
#define TRIM_ROLL 0.0


#ifdef LVC_LOWER_THROTTLE
#define SWITCHABLE_FEATURE_2
#endif

#ifdef INVERT_YAW_PID
#define SWITCHABLE_FEATURE_3
#endif

// for the ble beacon to work after in-flight reset
#ifdef RX_BAYANG_PROTOCOL_BLE_BEACON
#undef STOP_LOWBATTERY
#endif

#if defined(RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND) || defined(RX_NRF24_BAYANG_TELEMETRY)
#undef UART_1
#undef UART_2
#undef UART_3
#undef UART_4
#undef UART_6
#endif

#ifdef SOFTSPI_NONE
#undef RADIO_XN297L
#undef RADIO_XN297
#undef SOFTSPI_3WIRE	
#undef SOFTSPI_4WIRE	
#endif

#ifdef BUZZER_INVERT
#define PIN_ON( port , pin ) GPIO_ResetBits( port , pin)
#define PIN_OFF( port , pin ) GPIO_SetBits( port , pin)
#else
#define PIN_OFF( port , pin ) GPIO_ResetBits( port , pin)
#define PIN_ON( port , pin ) GPIO_SetBits( port , pin)
#endif
// x (micro)seconds after loss of tx or low bat before buzzer starts
#define BUZZER_DELAY     30e6 




