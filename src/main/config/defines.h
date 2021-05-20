#pragma once

#include "hardware.h"

#include "config_helper.h"
#include "rx.h"

// defines for things that do not normally need changing
typedef enum {
  LOOPTIME_2K = 500,
  LOOPTIME_4K = 250,
  LOOPTIME_8K = 125,
} looptime_autodetect_t;

#define ACC_1G 1.0f

#define PID_SIZE 3
#define ANGLE_PID_SIZE 2

static const float pid_scales[PID_SIZE][PID_SIZE] = {
    // roll, pitch, yaw
    {628.0f, 628.0f, 314.0f}, //kp
    {50.0f, 50.0f, 50.0f},    //ki
    {120.0f, 120.0f, 120.0f}, //kd
};

#define DEGTORAD 0.017453292f
#define RADTODEG 57.29577951f

#define ROLL 0
#define PITCH 1
#define YAW 2

// this should be precalculated by the compiler when parameters are constant
//(1 - alpha. filtertime = 1 / filter-cutoff-frequency) as long as filtertime > sampleperiod
#define FILTERCALC(sampleperiod, filtertime) (1.0f - (6.0f * (float)sampleperiod) / (3.0f * (float)sampleperiod + (float)filtertime))

#ifdef BETAFLIGHT_RATES
#define ACRO_EXPO_ROLL BF_EXPO_ROLL
#define ACRO_EXPO_PITCH BF_EXPO_PITCH
#define ACRO_EXPO_YAW BF_EXPO_YAW
#define ANGLE_EXPO_ROLL BF_EXPO_ROLL
#define ANGLE_EXPO_PITCH BF_EXPO_PITCH
#define ANGLE_EXPO_YAW BF_EXPO_YAW
#define MAX_RATE 200 * (float)BF_RC_RATE_ROLL *(1 / 1 - (float)BF_SUPER_RATE_ROLL) // roll max rate used for flip sequencer when bf rates selected
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

// *************0 - 7 - power for bayang telemetry
#define TX_POWER 7

// *************led brightness in-flight ( solid lights only)
// *************0- 15 range
#define LED_BRIGHTNESS 15

// *************Comment out to disable pid tuning gestures - originally created by SilverAG
#define PID_GESTURE_TUNING
// *************Comment out to adjust each axis individually - otherwise they move at the same time
#define COMBINE_PITCH_ROLL_PID_TUNING
// *************Feel free to change 1.0 value to your liking
#define PID_TUNING_ADJUST_AMOUNT 1.0 //fixed inc/dec values for PID tuning

// flash saving features  - best left alone or many things might break
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

#if defined(BUZZER_ENABLE) && !defined(BUZZER_PIN)
#undef BUZZER_ENABLE
#endif

// x (micro)seconds after loss of tx or low bat before buzzer starts
#define BUZZER_DELAY 30e6

#define OSD_NUMBER_ELEMENTS 32
#define SWITCHABLE_FEATURE_1 //CONFIGURATION WIZARD

// IDLE_OFFSET is added to the throttle. Adjust its value so that the motors
// still spin at minimum throttle.
#ifndef DIGITAL_IDLE
#define DIGITAL_IDLE 4
#endif

//#define DISABLE_FLIP_SEQUENCER //****************turtle / crashflip recovery available by default

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

#ifdef RX_UNIFIED_SERIAL
#define RX_PROTOCOL RX_PROTOCOL_UNIFIED_SERIAL
#endif
#ifdef RX_NRF24_BAYANG_TELEMETRY
#define RX_PROTOCOL RX_PROTOCOL_NRF24_BAYANG_TELEMETRY
#endif
#ifdef RX_BAYANG_PROTOCOL_BLE_BEACON
#define RX_PROTOCOL RX_PROTOCOL_BAYANG_PROTOCOL_BLE_BEACON
#endif
#ifdef RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND
#define RX_PROTOCOL RX_PROTOCOL_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND
#endif
#ifdef RX_FRSKY_D8
#define RX_PROTOCOL RX_PROTOCOL_FRSKY_D8
#endif
#if defined(RX_FRSKY_D16_LBT) || defined(RX_FRSKY_D16_FCC)
#define RX_PROTOCOL RX_PROTOCOL_FRSKY_D16
#endif
#ifdef RX_REDPINE
#define RX_PROTOCOL RX_PROTOCOL_REDPINE
#endif

// Select filter cut 25hz for SBUS, 67hz for CRSF, 40hz for DSMX, 20hz for DSM2, 90hz for bayang, 45hz for frsky   Formula is [(1/rx framerate)/2] * 0.9
static const uint8_t RX_SMOOTHING_HZ[RX_PROTOCOL_MAX] = {
    0,  //RX_PROTOCOL_INVALID, wont happen
    0,  //RX_PROTOCOL_UNIFIED_SERIAL, will autodetect following
    25, //RX_PROTOCOL_SBUS,
    67, //RX_PROTOCOL_CRSF,
    50, //RX_PROTOCOL_IBUS, check these
    50, //RX_PROTOCOL_FPORT, check these
    40, //RX_PROTOCOL_DSMX_2048,
    20, //RX_PROTOCOL_DSM2_1024,
    90, //RX_PROTOCOL_NRF24_BAYANG_TELEMETRY,
    90, //RX_PROTOCOL_BAYANG_PROTOCOL_BLE_BEACON,
    90, //RX_PROTOCOL_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND,
    50, //RX_PROTOCOL_FRSKY_D8,
    50, //RX_PROTOCOL_FRSKY_D16,
    75, //RX_PROTOCOL_FRSKY_REDPINE,
};

#ifdef RX_UNIFIED_SERIAL
static const uint8_t SERIAL_PROTO_MAP[] = {
    RX_PROTOCOL_INVALID,   //RX_SERIAL_PROTOCOL_INVALID
    RX_PROTOCOL_DSMX_2048, //RX_SERIAL_PROTOCOL_DSM
    RX_PROTOCOL_SBUS,      //RX_SERIAL_PROTOCOL_SBUS
    RX_PROTOCOL_IBUS,      //RX_SERIAL_PROTOCOL_IBUS
    RX_PROTOCOL_FPORT,     //RX_SERIAL_PROTOCOL_FPORT
    RX_PROTOCOL_CRSF,      //RX_SERIAL_PROTOCOL_CRSF
    RX_PROTOCOL_REDPINE,   //RX_SERIAL_PROTOCOL_REDPINE
    // No need to filter differently for inverted.
    RX_PROTOCOL_SBUS,    //RX_SERIAL_PROTOCOL_SBUS_INVERTED
    RX_PROTOCOL_FPORT,   //RX_SERIAL_PROTOCOL_FPORT_INVERTED
    RX_PROTOCOL_REDPINE, //RX_SERIAL_PROTOCOL_REDPINE_INVERTED
};
#endif
