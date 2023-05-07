

// Universal pids are already loaded for 5" brushless by default.  Adjust pids in pid.c file for your build.

//**********************************************************************************************************************
//***********************************************HARDWARE SELECTION*****************************************************

//**********************************************************************************************************************
//***********************************************NEW STUFF TO PLAY WITH*****************************************************

#define THROTTLE_D_ATTENUATION
#define TDA_BREAKPOINT 0.35f
#define TDA_PERCENT 0.70f

//**********************************************************************************************************************
//***********************************************RATES & EXPO SETTINGS**************************************************

// *************Select your preffered rate calculation format (define only one)
#define SILVERWARE_RATES
// #define BETAFLIGHT_RATES

// ******************** SILVERWARE_RATES ********************
// *************rate in deg/sec
// *************for acro mode
#define MAX_RATE 860.0    // Roll & Pitch axis
#define MAX_RATEYAW 500.0 // Yaw axis (used in acro and leveling modes)

// *************EXPO from 0.00 to 1.00 , 0 = no exp
// *************positive = less sensitive near center
#define ACRO_EXPO_ROLL 0.80
#define ACRO_EXPO_PITCH 0.80
#define ACRO_EXPO_YAW 0.60

#define ANGLE_EXPO_ROLL 0.55
#define ANGLE_EXPO_PITCH 0.0
#define ANGLE_EXPO_YAW 0.55

// ******************** BETAFLIGHT_RATES ********************
#define BF_RC_RATE_ROLL 1.30
#define BF_RC_RATE_PITCH 1.30
#define BF_RC_RATE_YAW 1.30

#define BF_SUPER_RATE_ROLL 0.70
#define BF_SUPER_RATE_PITCH 0.70
#define BF_SUPER_RATE_YAW 0.70

#define BF_EXPO_ROLL 0.40
#define BF_EXPO_PITCH 0.40
#define BF_EXPO_YAW 0.40

// ******************** ACTUAL_RATES ********************
#define ACTUAL_CENTER_SENS_ROLL 70
#define ACTUAL_CENTER_SENS_PITCH 70
#define ACTUAL_CENTER_SENS_YAW 70

#define ACTUAL_MAX_RATE_ROLL 670.0
#define ACTUAL_MAX_RATE_PITCH 670.0
#define ACTUAL_MAX_RATE_YAW 670.0

#define ACTUAL_EXPO_ROLL 0.0
#define ACTUAL_EXPO_PITCH 0.0
#define ACTUAL_EXPO_YAW 0.0

// *************max angle for level mode
#define LEVEL_MAX_ANGLE 65.0f

// *************transmitter stick adjustable deadband for roll/pitch/yaw
// *************.01f = 1% of stick range - comment out to disable
#define STICKS_DEADBAND .01f

// ************* throttle value at 0.5 of the stick range
#define THROTTLE_MID 0.5f

// ************* expo for throttle with the zero crossing at THROTTLE_MID
#define THROTTLE_EXPO 0.0f

#define RX_SMOOTHING

//**********************************************************************************************************************
//***********************************************RECEIVER SETTINGS******************************************************

// *************Receiver protocol selection

// #define RX_SBUS
// #define RX_CRSF
// #define RX_IBUS
// #define RX_FPORT
// #define RX_DSMX
// #define RX_DSM2
// #define RX_BAYANG_PROTOCOL_BLE_BEACON
// #define RX_BAYANG_BLE_APP
// #define RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND
// #define RX_NRF24_BAYANG_TELEMETRY

// *************Serial Receiver UART Selection (uncomment all for default from target or if using spi receiver)																		//todo:  Many missing usart AF setups, So plenty more to do here
// #define RX_USART USART_PORT1
// #define RX_USART USART_PORT2
// #define RX_USART USART_PORT3
// #define RX_USART USART_PORT4
// #define RX_USART USART_PORT6

// *************Serial Receiver Inversion Selection  (Normally true for SBUS and FPORT)
// #define INVERT_UART

// *************Bayang Specific Settings
// #define USE_DEVO
// #define USE_MULTI

// #define RADIO_XN297L
// #define RADIO_XN297

// #define USE_SOFT_SPI_3WIRE
// #define USE_SOFT_SPI_4WIRE

// *************ELRS Specific Settings
// #define EXPRESS_LRS_UID 0, 0, 0, 0, 0, 0

// *******************************SWITCH SELECTION*****************************
#define ARMING AUX_CHANNEL_0
#define PREARM AUX_CHANNEL_ON
#define IDLE_UP AUX_CHANNEL_0
#define LEVELMODE AUX_CHANNEL_OFF
#define RACEMODE AUX_CHANNEL_OFF
#define HORIZON AUX_CHANNEL_OFF
#define STICK_BOOST_PROFILE AUX_CHANNEL_4
#define TURTLE AUX_CHANNEL_OFF //****************turtle mode
// *************enable buzzer functionality
// *************change channel assignment from AUX_CHANNEL_OFF to a numbered aux switch if you want switch control
// *************if no channel is assigned but buzzer is set to AUX_CHANNEL_ON - buzzer will activate on LVC and FAILSAFE.
#define BUZZER_ENABLE AUX_CHANNEL_OFF
#define MOTORS_TO_THROTTLE_MODE AUX_CHANNEL_OFF
#define RSSI AUX_CHANNEL_OFF
// *************switch for fpv / other, requires fet
// *************comment out to disable
#define FPV_SWITCH AUX_CHANNEL_ON

// *************RRD/LLD stick gesture aux start up state.  Gesture aux is AUX_CHANNEL_GESTURE
// #define GESTURE_AUX_START_ON

//**********************************************************************************************************************
//***********************************************VOLTAGE SETTINGS*******************************************************

// ************* Set your lipo cell count to override auto cell count detect logic
// #define LIPO_CELL_COUNT 1

// ************* Raises pids automatically as battery voltage drops in flight.  **CRITICAL** Ensure voltage is calibrated before use.
#define PID_VOLTAGE_COMPENSATION
#define LEVELMODE_PID_ATTENUATION 0.90f // used to prevent oscillations in angle modes with pid_voltage_compensation enabled due to high pids

// *************compensation for battery voltage vs throttle drop
#define VDROP_FACTOR 0.7
// *************calculate above factor automatically
#define AUTO_VDROP_FACTOR

// *************lower throttle when battery below threshold - forced landing low voltage cutoff
// *************THIS FEATURE WILL BE OFF BY DEFAULT EVEN WHEN DEFINED - USE STICK GESTURE LEFT-LEFT-LEFT TO ACTIVATE THEN DOWN-DOWN-DOWN TO SAVE AS ON
// *************Led light will blink once when LVC forced landing is turned on, blink twice when turned off, and will blink multiple times upon save command
// #define LVC_LOWER_THROTTLE
#define LVC_LOWER_THROTTLE_VOLTAGE 3.30
#define LVC_LOWER_THROTTLE_VOLTAGE_RAW 2.70
#define LVC_LOWER_THROTTLE_KP 3.0

// *************voltage/cell to start warning led blinking
#define VBATTLOW 3.3

// *************voltage hysteresis in volts
#define HYST 0.10

// *************automatic voltage telemetry correction/calibration factor - change the values below if voltage telemetry is inaccurate
// *************Corrects for an offset error in the telemetry measurement (same offset across the battery voltage range)
// *************Enter values in total battery volts.  This is factor is used in all voltage related calculations - ensure your transmitter is not mucking with telemetry scale before adjusting
#define ACTUAL_BATTERY_VOLTAGE 4.20
#define REPORTED_TELEMETRY_VOLTAGE 4.20

//**********************************************************************************************************************
//***********************************************FILTER SETTINGS********************************************************

// Gyro Filters
// GYRO FILTER PASS 1 - FILTER TYPE  - define only one or none to disable this pass
#define GYRO_FILTER_PASS1_PT1
// #define GYRO_FILTER_PASS1_PT2
//  GYRO FILTER PASS 2 - FILTER TYPE  - define only one or none to disable this pass
#define GYRO_FILTER_PASS2_PT1
// #define GYRO_FILTER_PASS2_PT2

// GYRO FILTER PASS 1 CUTOFF FREQUENCY
#define GYRO_FREQ_PASS1 90
// GYRO FILTER PASS 2 CUTOFF FREQUENCY
#define GYRO_FREQ_PASS2 90

// Dynamic D term filter
// a pt1 filter that moves up in cut hz with a parabolic relationship to applied throttle.  The theory here is
// that propwash is most likely to occur as throttle is applied in dirty air - and propwash is most significantly
//  caused by latency in the D term filtering.  Therefore, the approach is to reduce latency in the lowest frequency
// range of d term filtering which is responsible for the most phase delay as increasing throttle is applied.  Noise pass-through
// will obviously increase with this approach, but when used in combination with throttle_dterm_attenuation - that gains on D will
// also be lowered with increasing throttle thereby mitigating much of the danger from reduced filtering while allowing D term to be more effective
// at eliminating propwash.  Motor noise related to rpm is known to have a quadratic relationship with increasing throttle.  While a quadratic curve
// could have been selected for this feature, a faster moving parabolic one was selected in its place as the goal is not to follow motor noise, but
// to get the filter out of the way as fast as possible in the interest of better performance and handling through reduced D filter latency when you need it most.
#define DTERM_DYNAMIC_LPF
#define DYNAMIC_FREQ_MIN 70
#define DYNAMIC_FREQ_MAX 260

// Fixed D-Term Filters
// D-Term FILTER PASS 1 - FILTER TYPE  - define only one or none to disable this pass
#define DTERM_FILTER_PASS1_PT1
// #define DTERM_FILTER_PASS1_PT2
//  D-Term FILTER PASS 2 - FILTER TYPE  - define only one or none to disable this pass
// #define DTERM_FILTER_PASS2_PT1
// #define DTERM_FILTER_PASS2_PT2

// D-Term FILTER PASS 1 CUTOFF FREQUENCY
#define DTERM_FREQ_PASS1 260
// D-Term FILTER PASS 2 CUTOFF FREQUENCY
#define DTERM_FREQ_PASS2 150

//**********************************************************************************************************************
//***********************************************MOTOR OUTPUT SETTINGS**************************************************

// *************brushed motor minimum idle percent / dshot digital idle
// IDLE_OFFSET is added to the throttle. Adjust its value so that the motors
// still spin at minimum throttle.
#define DIGITAL_IDLE 4.5

// *************limits the maxium power applied to the motors
#define MOTOR_LIMIT 100.0

// *************invert yaw pid for "PROPS OUT" configuration - This feature is switchable to "PROPS IN" when active with stick gesture DOWN-UP-DOWN, Save selection with DOWN-DOWN-DOWN
#define INVERT_YAW_PID

// *************throttle boost - can intensify small throttle imbalances visible in FPV if factor is set too high on brushed or actually rob performance on brushless due to thrust imbalances
// #define THROTTLE_BOOST 7.0

// *************torque boost is a highly eperimental feature and can smoke brushless motors fast.  it is a lpf D term on motor outputs that will accelerate the response
// *************of the motors when the command to the motors is changing by increasing or decreasing the voltage thats sent.  It differs
// *************from throttle transient compensation in that it acts on all motor commands - not just throttle changes.  this feature
// *************is very noise sensative so D term specifically has to be lowered and gyro/d filtering may need to be increased.
// *************reccomendation right now is to leave boost at or below 2, drop your p gains a few points, then cut your D in half and
// *************retune it back up to where it feels good.  I'm finding about 60 to 65% of my previous D value seems to work.
// #define TORQUE_BOOST 1.0

// *************pwm frequency for motor control
// *************a higher frequency makes the motors more linear
// *************in Hz
#define PWMFREQ 32000

// *************throttle angle compensation in level mode
// #define AUTO_THROTTLE

// *************BRUSHED TARGET MIXER SETTINGS
// *************MIX_THROTTLE_REDUCTION_PERCENT reduces thrust imbalances by reducing throttle proportionally to the adjustable reduction percent to the limit set by MIX_THROTTLE_REDUCTION_MAX
// *************MIX_THROTTLE_INCREASE_MAX increases the authority of the pid controller at lowest throttle values like airmode when combined with idle up
// *************BRUSHLESS_MIX_SCALING define disables the default brushed mixer and applies the default brushless mixer
// #define MIX_THROTTLE_REDUCTION_PERCENT 10
// #define MIX_THROTTLE_INCREASE_MAX 0.2f
// #define MIX_THROTTLE_REDUCTION_MAX 0.5f
// #define BRUSHLESS_MIX_SCALING

// *************BRUSHLESS TARGET MIXER SETTINGS
// *************AIRMODE_STRENGTH is the amount of authority AIRMODE is given over throttle
// *************Decrease CLIPPING_LIMIT to force clipping in place of mix scaling to
// *************restore lost power on "promblem quads" by allowing some thrust imbalances to pass through the mixer
//	#define AIRMODE_STRENGTH  1.0f
//	#define CLIPPING_LIMIT  1.0f

//**************I-term relax.  Removes roll and pitch bounce back after flips
#define ITERM_RELAX
#define RELAX_FACTOR_DEG 5.7
#define RELAX_FREQUENCY_HZ 20 // from my experience the frequency is better to tune than the factor

#define ITERM_RELAX_YAW // adds iterm relax to yaw with its own values as it responds much different
#define RELAX_FACTOR_YAW_DEG 5.7
#define RELAX_FREQUENCY_HZ_YAW 25

// *************************************************************************
// horizon modes tuning variables
// *************************************************************************
// 1.0 is pure angle based transition, 0.0 is pure stick defelction based transition, values inbetween are a mix of both.  Adjust from 0 to 1
#define HORIZON_SLIDER 0.3f
// leveling transitions into acro below this angle - above this angle is all acro.  DO NOT SET ABOVE 85 DEGREES!
#define HORIZON_ANGLE_TRANSITION 55.0f
// leveling transitions into acro below this stick position - beyond this stick position is all acro. Adjust from 0 to 1
#define HORIZON_STICK_TRANSITION 0.95f

//**********************************************************************************************************************
//***********************************************ADDITIONAL FEATURES****************************************************

// *************lost quad beeps using motors (30 sec timeout in ms) - pulses motors after timeout period to help find a lost model
#define MOTOR_BEEPS
#define MOTOR_BEEPS_TIMEOUT 30e3

// *************led brightness in-flight ( solid lights only)
// *************0- 15 range
#define LED_BRIGHTNESS 15

// *************Comment out to disable pid tuning gestures - originally created by SilverAG
#define PID_GESTURE_TUNING
// *************Comment out to adjust each axis individually - otherwise they move at the same time
#define COMBINE_PITCH_ROLL_PID_TUNING
// *************Feel free to change 1.0 value to your liking
#define PID_TUNING_ADJUST_AMOUNT 1.0 // fixed inc/dec values for PID tuning

// x (micro)seconds after loss of tx or low bat before buzzer starts
#define BUZZER_DELAY 30e6

// #############################################################################################################################
// #############################################################################################################################
//  debug / other things
//  this should not be usually changed or still need work
// #############################################################################################################################
// #############################################################################################################################

// The airbot made ab7456 osd chip will not support blink commands
// #define AIRBOT_OSD_PATCH

// failsafe time in uS
#define FAILSAFETIME 1000000 // one second

// debug things ( debug struct and other)
// #define DEBUG
// #define DEBUG_LOGGING
// #define RESET_ON_FAULT

// allow transmitter aux to arm motors while connected to usb gui
// #define ALLOW_USB_ARMING

// disable motors for testing
// #define NOMOTORS

// change mixer to a plus configuration
// #define MOTOR_PLUS_CONFIGURATION