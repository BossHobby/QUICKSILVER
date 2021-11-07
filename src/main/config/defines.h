#pragma once

#include "config.h"
#include "config_helper.h"
#include "hardware.h"

// defines for things that do not normally need changing
typedef enum {
  LOOPTIME_2K = 500,
  LOOPTIME_4K = 250,
  LOOPTIME_8K = 125,
} looptime_autodetect_t;

#define PID_SIZE 3
#define ANGLE_PID_SIZE 2

#define ROLL 0
#define PITCH 1
#define YAW 2

// this should be precalculated by the compiler when parameters are constant
//(1 - alpha. filtertime = 1 / filter-cutoff-frequency) as long as filtertime > sampleperiod
#define FILTERCALC(sampleperiod, filtertime) (1.0f - (6.0f * (float)sampleperiod) / (3.0f * (float)sampleperiod + (float)filtertime))

// for inverted flight motor direction
#define FORWARD 0
#define REVERSE 1

//Throttle must drop below this value if arming feature is enabled for arming to take place.  MIX_INCREASE_THROTTLE_3 if enabled
//will also not activate on the ground untill this threshold is passed during takeoff for safety and better staging behavior.
#define THROTTLE_SAFETY .10f

#ifdef LVC_LOWER_THROTTLE
#define SWITCHABLE_FEATURE_2
#endif

#if defined(BUZZER_ENABLE) && !defined(BUZZER_PIN)
#undef BUZZER_ENABLE
#endif

// x (micro)seconds after loss of tx or low bat before buzzer starts
#define BUZZER_DELAY 30e6

#define OSD_NUMBER_ELEMENTS 32
#define SWITCHABLE_FEATURE_1 //CONFIGURATION WIZARD

#ifdef RX_SBUS
#define RX_UNIFIED_SERIAL
#endif
#ifdef RX_CRSF
#define RX_UNIFIED_SERIAL
#endif
#ifdef RX_IBUS
#define RX_UNIFIED_SERIAL
#endif
#ifdef RX_FPORT
#define RX_UNIFIED_SERIAL
#endif
#ifdef RX_DSMX_2048
#define RX_UNIFIED_SERIAL
#endif
#ifdef RX_DSM2_1024
#define RX_UNIFIED_SERIAL
#endif
