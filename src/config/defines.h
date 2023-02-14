#pragma once

#include "config.h"
#include "hardware.h"

// defines for things that do not normally need changing
#define PID_SIZE 3
#define ANGLE_PID_SIZE 2

#define ROLL 0
#define PITCH 1
#define YAW 2

// this should be precalculated by the compiler when parameters are constant
//(1 - alpha. filtertime = 1 / filter-cutoff-frequency) as long as filtertime > sampleperiod
#define FILTERCALC(sampleperiod, filtertime) (1.0f - (6.0f * (float)(sampleperiod)) / (3.0f * (float)(sampleperiod) + (float)(filtertime)))

#define MHZ_TO_HZ(mhz) (mhz * 1000000)

#define MAKE_SEMVER(major, minor, patch) ((major << 16) | (minor << 8) | patch)

// Throttle must drop below this value if arming feature is enabled for arming to take place.  MIX_INCREASE_THROTTLE_3 if enabled
// will also not activate on the ground untill this threshold is passed during takeoff for safety and better staging behavior.
#define THROTTLE_SAFETY .10f

#if defined(BUZZER_ENABLE) && !defined(BUZZER_PIN)
#undef BUZZER_ENABLE
#endif

// x (micro)seconds after loss of tx or low bat before buzzer starts
#define BUZZER_DELAY 30e6

#ifndef MOTOR_BEEPS_TIMEOUT
#define MOTOR_BEEPS_TIMEOUT 30e3
#endif

#define OSD_NUMBER_ELEMENTS 32
#define SWITCHABLE_FEATURE_1 // CONFIGURATION WIZARD
