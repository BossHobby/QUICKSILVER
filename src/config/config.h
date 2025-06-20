// ================================================================================================
// RATES & EXPO SETTINGS
// ================================================================================================

// Select your preferred rate calculation format (define only one)
#define SILVERWARE_RATES
// #define BETAFLIGHT_RATES

// ---- SILVERWARE RATES ----
// Rate in deg/sec for acro mode
#define MAX_RATE 860.0     // roll/pitch axis
#define MAX_RATEYAW 500.0  // yaw axis (used in acro and leveling modes)

// Expo from 0.00 to 1.00 (0 = no expo, positive = less sensitive near center)
#define ACRO_EXPO_ROLL 0.80
#define ACRO_EXPO_PITCH 0.80
#define ACRO_EXPO_YAW 0.60

#define ANGLE_EXPO_ROLL 0.55
#define ANGLE_EXPO_PITCH 0.0
#define ANGLE_EXPO_YAW 0.55

// ---- BETAFLIGHT RATES ----
#define BF_RC_RATE_ROLL 1.30
#define BF_RC_RATE_PITCH 1.30
#define BF_RC_RATE_YAW 1.30

#define BF_SUPER_RATE_ROLL 0.70
#define BF_SUPER_RATE_PITCH 0.70
#define BF_SUPER_RATE_YAW 0.70

#define BF_EXPO_ROLL 0.40
#define BF_EXPO_PITCH 0.40
#define BF_EXPO_YAW 0.40

// ---- ACTUAL RATES ----
#define ACTUAL_CENTER_SENS_ROLL 70
#define ACTUAL_CENTER_SENS_PITCH 70
#define ACTUAL_CENTER_SENS_YAW 70

#define ACTUAL_MAX_RATE_ROLL 670.0
#define ACTUAL_MAX_RATE_PITCH 670.0
#define ACTUAL_MAX_RATE_YAW 670.0

#define ACTUAL_EXPO_ROLL 0.0
#define ACTUAL_EXPO_PITCH 0.0
#define ACTUAL_EXPO_YAW 0.0

// ---- STICK SETTINGS ----
#define LEVEL_MAX_ANGLE 65.0f       // max angle for level mode
#define STICKS_DEADBAND .01f        // 1% of stick range - comment out to disable

#define THROTTLE_MID 0.5f           // throttle value at 0.5 of stick range
#define THROTTLE_EXPO 0.0f          // throttle expo (zero crossing at THROTTLE_MID)

// Throttle D-term attenuation
#define THROTTLE_D_ATTENUATION
#define TDA_BREAKPOINT 0.35f
#define TDA_PERCENT 0.80f

// ================================================================================================
// RECEIVER SETTINGS
// ================================================================================================

// ---- RECEIVER PROTOCOL ----
// #define RX_SBUS
// #define RX_CRSF
// #define RX_IBUS
// #define RX_FPORT
// #define RX_DSM

// ---- SERIAL PORT CONFIGURATION ----
// Serial receiver UART selection (uncomment for default from target or if using SPI receiver)
// #define RX_USART SERIAL_PORT1
// #define RX_USART SERIAL_PORT2
// #define RX_USART SERIAL_PORT3
// #define RX_USART SERIAL_PORT4
// #define RX_USART SERIAL_PORT5
// #define RX_USART SERIAL_PORT6
// #define RX_USART SERIAL_PORT7  // STM32F7/H7/AT32F4 only
// #define RX_USART SERIAL_PORT8  // STM32F7/H7/AT32F4 only

// Serial receiver inversion (normally true for SBUS and FPORT)
// #define INVERT_UART

// SmartAudio VTX control UART (comment out to disable)
// #define SMART_AUDIO_USART SERIAL_PORT1
// #define SMART_AUDIO_USART SERIAL_PORT2
// #define SMART_AUDIO_USART SERIAL_PORT3
// #define SMART_AUDIO_USART SERIAL_PORT4
// #define SMART_AUDIO_USART SERIAL_PORT5
// #define SMART_AUDIO_USART SERIAL_PORT6
// #define SMART_AUDIO_USART SERIAL_PORT7  // STM32F7/H7/AT32F4 only
// #define SMART_AUDIO_USART SERIAL_PORT8  // STM32F7/H7/AT32F4 only

// Digital VTX OSD UART - DisplayPort (HDZero, Walksnail, DJI, etc.)
// #define DISPLAYPORT_USART SERIAL_PORT1
// #define DISPLAYPORT_USART SERIAL_PORT2
// #define DISPLAYPORT_USART SERIAL_PORT3
// #define DISPLAYPORT_USART SERIAL_PORT4
// #define DISPLAYPORT_USART SERIAL_PORT5
// #define DISPLAYPORT_USART SERIAL_PORT6
// #define DISPLAYPORT_USART SERIAL_PORT7  // STM32F7/H7/AT32F4 only
// #define DISPLAYPORT_USART SERIAL_PORT8  // STM32F7/H7/AT32F4 only

// ---- ELRS CONFIGURATION ----
// #define EXPRESS_LRS_UID 0, 0, 0, 0, 0, 0

// ---- CHANNEL MAPPING ----
// #define CHANNEL_MAPPING RX_MAPPING_AETR  // options: RX_MAPPING_AETR (default), RX_MAPPING_TAER

// ---- SWITCH ASSIGNMENTS ----
#define ARMING AUX_CHANNEL_0
#define PREARM AUX_CHANNEL_ON
#define IDLE_UP AUX_CHANNEL_0
#define LEVELMODE AUX_CHANNEL_OFF
#define RACEMODE AUX_CHANNEL_OFF
#define HORIZON AUX_CHANNEL_OFF
#define STICK_BOOST_PROFILE AUX_CHANNEL_4
#define TURTLE AUX_CHANNEL_OFF
#define BUZZER_ENABLE AUX_CHANNEL_OFF
#define MOTORS_TO_THROTTLE_MODE AUX_CHANNEL_OFF
#define RSSI AUX_CHANNEL_OFF
#define FPV_SWITCH AUX_CHANNEL_ON

// ---- FAILSAFE ----
#define FAILSAFE_TIME_US 1000000      // 1 second
#define FAILSAFE_LOCK_TIME_MS 5000    // 5 seconds

// ================================================================================================
// VOLTAGE & BATTERY
// ================================================================================================

// ---- BATTERY CONFIGURATION ----
// #define LIPO_CELL_COUNT 1  // uncomment to override auto cell count detection

// ---- PID VOLTAGE COMPENSATION ----
// Raises PIDs automatically as battery voltage drops
// CRITICAL: ensure voltage is calibrated before use!
#define PID_VOLTAGE_COMPENSATION
#define LEVELMODE_PID_ATTENUATION 0.90f  // prevents oscillations in angle modes

// ---- LOW VOLTAGE WARNINGS ----
#define VBATTLOW 3.6  // voltage per cell to start warning
#define HYST 0.10     // voltage hysteresis

// ---- VOLTAGE CALIBRATION ----
// Adjust if voltage telemetry is inaccurate
#define ACTUAL_BATTERY_VOLTAGE 4.20
#define REPORTED_TELEMETRY_VOLTAGE 4.20

// Use filtered voltage for warnings instead of fuel gauge
#define USE_FILTERED_VOLTAGE_FOR_WARNINGS 0

// ---- CURRENT SENSOR ----
// #define IBAT_SCALE 100  // scale factor for current sensor (comment out to disable)

// ================================================================================================
// FILTER SETTINGS
// ================================================================================================

// ---- GYRO FILTERS ----
#define GYRO_PASS1_TYPE FILTER_LP_PT2
#define GYRO_PASS1_FREQ 100

#define GYRO_PASS2_TYPE FILTER_NONE
#define GYRO_PASS2_FREQ 0

// Dynamic notch filter
// #define GYRO_DYNAMIC_NOTCH

// ---- D-TERM FILTERS ----
// Dynamic D-term filter - PT1 with parabolic throttle response
// Reduces D-term latency at higher throttle to combat propwash
// Motor noise has a quadratic relationship with throttle, but we use a faster parabolic curve
// to get the filter out of the way quickly for better performance and handling
#define DTERM_DYNAMIC_TYPE FILTER_LP_PT1
#define DTERM_DYNAMIC_EXPO 1.0f
#define DTERM_DYNAMIC_FREQ_MIN 70
#define DTERM_DYNAMIC_FREQ_MAX 260

// Fixed D-term filters
#define DTERM_PASS1_TYPE FILTER_LP_PT1
#define DTERM_PASS1_FREQ 260

#define DTERM_PASS2_TYPE FILTER_NONE
#define DTERM_PASS2_FREQ 150

// ================================================================================================
// MOTOR OUTPUT
// ================================================================================================

// ---- MOTOR CONFIGURATION ----
#define DIGITAL_IDLE 4.5   // minimum throttle for motors to spin
#define MOTOR_LIMIT 100.0  // maximum motor output percentage
#define INVERT_YAW_PID     // for "props out" configuration

// ---- MOTOR BOOST FEATURES ----
// #define THROTTLE_BOOST 7.0  // can cause thrust imbalances if too high
// #define TORQUE_BOOST 1.0    // EXPERIMENTAL - can damage motors!

// ---- BRUSHED MOTOR MIXER ----
// #define MIX_THROTTLE_REDUCTION_PERCENT 10
// #define MIX_THROTTLE_INCREASE_MAX 0.2f
// #define MIX_THROTTLE_REDUCTION_MAX 0.5f


// ================================================================================================
// PID CONTROLLER
// ================================================================================================

// ---- I-TERM RELAX ----
// Removes bounce back after flips
#define ITERM_RELAX
#define RELAX_FACTOR_DEG 5.7
#define RELAX_FREQUENCY_HZ 20

#define ITERM_RELAX_YAW
#define RELAX_FACTOR_YAW_DEG 5.7
#define RELAX_FREQUENCY_HZ_YAW 25

// ---- HORIZON MODE ----
#define HORIZON_SLIDER 0.3f             // 0.0 = stick based, 1.0 = angle based
#define HORIZON_ANGLE_TRANSITION 55.0f  // degrees (max 85)
#define HORIZON_STICK_TRANSITION 0.95f  // stick position

// ================================================================================================
// ADDITIONAL FEATURES
// ================================================================================================

// ---- LOST MODEL RECOVERY ----
#define MOTOR_BEEPS
#define MOTOR_BEEPS_TIMEOUT 30e3  // 30 seconds

// ---- LED SETTINGS ----
#define LED_BRIGHTNESS 15  // 0-15 range

// ---- BUZZER ----
#define BUZZER_DELAY 30e6  // 30 seconds delay after failsafe/low battery

// ---- COMPATIBILITY FIXES ----
// #define AIRBOT_OSD_PATCH       // for AB7456 OSD chip blink issues
// #define USE_AKK_SA_WORKAROUND  // for AKK VTX SmartAudio issues

// ---- DEFAULT PRESETS ----
// #define DEFAULT_PID_RATE_PRESET 0
// #define DEFAULT_BLACKBOX_PRESET 0

// ================================================================================================
// DEBUG & DEVELOPMENT
// ================================================================================================

// ---- DEBUG FEATURES ----
// #define DEBUG
// #define DEBUG_LOGGING
// #define RESET_ON_FAULT
// #define BLACKBOX_DEBUG_FLAGS BBOX_DEBUG_DYN_NOTCH

// ---- DEVELOPMENT OPTIONS ----
// #define ALLOW_USB_ARMING          // allow arming while connected to USB
// #define NOMOTORS                  // disable motor output for bench testing
// #define MOTOR_PLUS_CONFIGURATION  // use + motor layout instead of X