#include "defines.h"
#include "hardware.h"

//Universal pids are already loaded for 6mm and 7mm whoops by default.  Adjust pids in pid.c file for any non whoop builds.  

//**********************************************************************************************************************
//***********************************************HARDWARE SELECTION*****************************************************

// *************DEFINE FLIGHT CONTROLLER HARDWARE
// *************SELECT QUICKSILVER F4 FROM TARGET DROP DOWN BOX FOR F4 TARGETS AND NFE_Silverware FROM TARGET DROP DOWN BOX FOR F0 TARGETS
//#define BWHOOP
//#define E011
//#define H8mini_blue_board
//#define Alienwhoop_ZERO  
//#define CC3D_REVO_F4
//#define OmnibusF4SD
#define OmnibusF4
//#define Alienwhoop_V2
//#define LuxF4osd
//#define CLRacing_F4
//#define Raceflight_Revolt

// *************DEFINE FLIGHT CONTROLLER MOTOR OUTPUT - *****warning*****  GETTING THIS WRONG CAN SMOKE YOUR BOARD :)
#define BRUSHLESS_TARGET
//#define BRUSHED_TARGET


//**********************************************************************************************************************
//***********************************************RATES & EXPO SETTINGS**************************************************

// *************Select your preffered rate calculation format (define only one)
#define SILVERWARE_RATES
//#define BETAFLIGHT_RATES

#ifdef SILVERWARE_RATES
// *************rate in deg/sec
// *************for acro mode
#define MAX_RATE 860.0          //Roll & Pitch axis
#define MAX_RATEYAW 500.0       //Yaw axis (used in acro and leveling modes)

// *************EXPO from 0.00 to 1.00 , 0 = no exp
// *************positive = less sensitive near center 
#define ACRO_EXPO_ROLL 0.80
#define ACRO_EXPO_PITCH 0.80
#define ACRO_EXPO_YAW 0.60

#define ANGLE_EXPO_ROLL 0.55
#define ANGLE_EXPO_PITCH 0.0
#define ANGLE_EXPO_YAW 0.55
#endif

#ifdef BETAFLIGHT_RATES
#define BF_RC_RATE_ROLL 1.00
#define BF_RC_RATE_PITCH 1.00
#define BF_RC_RATE_YAW 1.00
#define BF_SUPER_RATE_ROLL 0.70
#define BF_SUPER_RATE_PITCH 0.70
#define BF_SUPER_RATE_YAW 0.70
#define BF_EXPO_ROLL 0.00
#define BF_EXPO_PITCH 0.00
#define BF_EXPO_YAW 0.00
#endif

// *************max angle for level mode
#define LEVEL_MAX_ANGLE 65.0f

// ************* low rates multiplier if rates are assigned to a channel
#define LOW_RATES_MULTI 0.5f

// *************transmitter stick adjustable deadband for roll/pitch/yaw
// *************.01f = 1% of stick range - comment out to disable
#define STICKS_DEADBAND .01f




//**********************************************************************************************************************
//***********************************************RECEIVER SETTINGS******************************************************

// *************Receiver protocol selection									//todo:  add missing radio protocols from bobnova and make them all jive with new rx_init function in drv_rx_serial.c
#define RX_SBUS
//#define RX_CRSF                                           //Requires tbs firmware v2.88 or newer for failsafe to operate properly
//#define RX_IBUS
//#define RX_FPORT
//#define RX_DSMX_2048																				//  Only sbus, ibus, and dsm protocols are working on F4 right now
//#define RX_DSM2_1024
//#define RX_NRF24_BAYANG_TELEMETRY
//#define RX_BAYANG_PROTOCOL_BLE_BEACON
//#define RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND

// *************Serial Receiver UART Selection																		//todo:  Many missing usart AF setups, So plenty more to do here
#define UART_1
//#define UART_2
//#define UART_3
//#define UART_4
//#define UART_6

// *************Serial Receiver Inversion Selection
#define INVERT_UART					    //Normally true for SBUS and FPORT																											

// *************Transmitter Type Selection																				//todo:  drop toy tx support - clarify that remaining options are just for bayang protocol
//#define USE_STOCK_TX
//#define USE_DEVO
#define USE_MULTI

// *******************************SWITCH SELECTION*****************************
#define ARMING CHAN_5
#define IDLE_UP CHAN_5																															//todo:  sort out a better brushless plan for airmode, idle up, and min throttle enable
#define IDLE_THR 0.04f                   //This designates an idle throttle of 5%
#define LEVELMODE CHAN_OFF
#define RACEMODE  CHAN_OFF
#define HORIZON   CHAN_OFF
#define PIDPROFILE CHAN_9                //For switching stickAccelerator & stickTransition profiles on pid.c page
#define RATES CHAN_ON
#define LEDS_ON CHAN_OFF

// *************switch for fpv / other, requires fet
// *************comment out to disable
//#define FPV_ON CHAN_ON

// *************enable buzzer functionality
// *************external buzzer requires pin assignment in hardware.h before defining below
// *************change channel assignment from CHAN_OFF to a numbered aux switch if you want switch control
// *************if no channel is assigned but buzzer is set to CHAN_ON - buzzer will activate on LVC and FAILSAFE.
//#define BUZZER_ENABLE CHAN_OFF

// *************start in level mode for toy tx.
//#define AUX1_START_ON

// *************automatically remove center bias in toy tx ( needs throttle off for 1 second )
//#define STOCK_TX_AUTOCENTER




//**********************************************************************************************************************
//***********************************************VOLTAGE SETTINGS*******************************************************

// ************* Set your lipo cell count to override auto cell count detect logic
//#define LIPO_CELL_COUNT 1

// ************* Raises pids automatically as battery voltage drops in flight.   Ensure voltage is calibrated before use.
#define PID_VOLTAGE_COMPENSATION
#define LEVELMODE_PID_ATTENUATION 0.90f  //used to prevent oscillations in angle modes with pid_voltage_compensation enabled due to high pids

// *************compensation for battery voltage vs throttle drop
#define VDROP_FACTOR 0.7
// *************calculate above factor automatically
#define AUTO_VDROP_FACTOR

// *************lower throttle when battery below threshold - forced landing low voltage cutoff
// *************THIS FEATURE WILL BE OFF BY DEFAULT EVEN WHEN DEFINED - USE STICK GESTURE LEFT-LEFT-LEFT TO ACTIVATE THEN DOWN-DOWN-DOWN TO SAVE AS ON
// *************Led light will blink once when LVC forced landing is turned on, blink twice when turned off, and will blink multiple times upon save command
#define LVC_LOWER_THROTTLE
#define LVC_LOWER_THROTTLE_VOLTAGE 3.30
#define LVC_LOWER_THROTTLE_VOLTAGE_RAW 2.70
#define LVC_LOWER_THROTTLE_KP 3.0

// *************do not start software if battery is too low (below 3.3v)  - only works on 1s lipos
// *************flashes 2 times repeatedly at startup
//#define STOP_LOWBATTERY

// *************voltage to start warning led blinking
#define VBATTLOW 3.5

// *************voltage hysteresis in volts
#define HYST 0.10

// *************automatic voltage telemetry correction/calibration factor - change the values below if voltage telemetry is inaccurate
// *************Corrects for an offset error in the telemetry measurement (same offset across the battery voltage range)
// *************Enter values in total battery volts.  This is factor is used in all voltage related calculations - ensure your transmitter is not mucking with telemetry scale before adjusting 
#define ACTUAL_BATTERY_VOLTAGE 4.20
#define REPORTED_TELEMETRY_VOLTAGE 4.20




//**********************************************************************************************************************
//***********************************************FILTER SETTINGS********************************************************

//#define WEAK_FILTERING
//#define STRONG_FILTERING
//#define VERY_STRONG_FILTERING
//#define ALIENWHOOP_ZERO_FILTERING
#define BETA_FILTERING

#ifdef BETA_FILTERING  //*** ABOVE 100 ADJUST IN INCRIMENTS OF 20, BELOW 100 ADJUST IN INCRIMENTS OF 10, nothing coded beyond 500hz

//Select Gyro Filter Type *** Select Only One type
//#define KALMAN_GYRO
#define PT1_GYRO

//Select Gyro Filter Cut Frequency
#define GYRO_FILTER_PASS1 HZ_90
#define GYRO_FILTER_PASS2 HZ_140

//Select D Term Filter Cut Frequency *** Select Only one
//#define  DTERM_LPF_2ND_HZ 100
#define DTERM_LPF_1ST_HZ 70

//Select Motor Filter Type  (I am no longer using this)
//#define MOTOR_FILTER2_ALPHA MFILT1_HZ_90

#endif




//**********************************************************************************************************************
//***********************************************MOTOR OUTPUT SETTINGS**************************************************

// *************invert yaw pid for "PROPS OUT" configuration - This feature is switchable to "PROPS IN" when active with stick gesture DOWN-UP-DOWN, Save selection with DOWN-DOWN-DOWN
#define INVERT_YAW_PID

// *************pwm frequency for motor control
// *************a higher frequency makes the motors more linear
// *************in Hz
#define PWMFREQ 16000

// *************clip feedforward attempts to resolve issues that occur near full throttle by adding any clipped motor commands to the next loop output
//#define CLIP_FF

// *************torque boost is a highly eperimental feature.  it is a lpf D term on motor outputs that will accelerate the response
// *************of the motors when the command to the motors is changing by increasing or decreasing the voltage thats sent.  It differs
// *************from throttle transient compensation in that it acts on all motor commands - not just throttle changes.  this feature
// *************is very noise sensative so D term specifically has to be lowered and gyro/d filtering may need to be increased.
// *************reccomendation right now is to leave boost at or below 2, drop your p gains a few points, then cut your D in half and 
// *************retune it back up to where it feels good.  I'm finding about 60 to 65% of my previous D value seems to work.
//#define TORQUE_BOOST 1.0

// *************makes throttle feel more poppy - can intensify small throttle imbalances visible in FPV if factor is set too high
//#define THROTTLE_TRANSIENT_COMPENSATION 
//#define THROTTLE_TRANSIENT_COMPENSATION_FACTOR 4.0 
 
// *************throttle angle compensation in level mode
//#define AUTO_THROTTLE

// *************mix lower throttle reduces thrust imbalances by reducing throttle proportionally to the adjustable reduction percent
// *************mix increase throttle increases the authority of the pid controller at lowest throttle values like airmode when combined with idle up
// *************mix3 has a stronger effect and works better with brushless
//#define MIX_LOWER_THROTTLE
//#define MIX_THROTTLE_REDUCTION_PERCENT 10
//#define MIX_INCREASE_THROTTLE

//#define MIX_LOWER_THROTTLE_3
//#define MIX_INCREASE_THROTTLE_3
//#define MIX_THROTTLE_INCREASE_MAX 0.8f
//#define MIX_THROTTLE_REDUCTION_MAX 0.8f
#define BRUSHLESS_MIX_SCALING

//**************joelucid's yaw fix
#define YAW_FIX

//**************joelucid's transient windup protection.  Removes roll and pitch bounce back after flips
#define TRANSIENT_WINDUP_PROTECTION




//**********************************************************************************************************************
//***********************************************ADDITIONAL FEATURES****************************************************

// *************lost quad beeps using motors (30 sec timeout) - pulses motors after timeout period to help find a lost model
//#define MOTOR_BEEPS

// *************0 - 7 - power for telemetry
#define TX_POWER 7

// *************led brightness in-flight ( solid lights only)
// *************0- 15 range
#define LED_BRIGHTNESS 15

// *************Comment out to disable pid tuning gestures
#define PID_GESTURE_TUNING
#define COMBINE_PITCH_ROLL_PID_TUNING

// *************flash save method
// *************flash_save 1: pids + accel calibration
// *************flash_save 2: accel calibration to option bytes
#define FLASH_SAVE1
//#define FLASH_SAVE2

// *************enable inverted flight code ( brushless only )
//#define INVERTED_ENABLE
//#define FN_INVERTED CH_OFF //for brushless only

// *************SPECIAL TEST MODE TO CHECK TRANSMITTER STICK THROWS
// *************This define will allow you to check if your radio is reaching 100% throws entering <RIGHT-RIGHT-DOWN> gesture
// ************* will disable throttle and will rapid blink the led when sticks are moved to 100% throws
// *************entering <LEFT-LEFT-DOWN> will return the quad to normal operation.
#define STICK_TRAVEL_CHECK





//#############################################################################################################################
//#############################################################################################################################
// debug / other things
// this should not be usually changed
//#############################################################################################################################
//#############################################################################################################################

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

// limit minimum motor output to a value (0.0 - 1.0)
//#define MOTOR_MIN_ENABLE
//#define MOTOR_MIN_VALUE 0.05

// flash saving features
//#define DISABLE_GESTURES2

#ifdef LVC_LOWER_THROTTLE
#define SWITCHABLE_FEATURE_2
#endif

#ifdef INVERT_YAW_PID
#define SWITCHABLE_FEATURE_3
#endif

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

#define GYRO_LOW_PASS_FILTER 0

#define DISABLE_FLIP_SEQUENCER
#define STARTFLIP CHAN_OFF

// disable motors for testing
//#define NOMOTORS

// throttle direct to motors for thrust measure

//#define MOTORS_TO_THROTTLE

// throttle direct to motors for thrust measure as a flight mode
//#define MOTORS_TO_THROTTLE_MODE MULTI_CHAN_8

// *************motor curve to use - select one
// *************the pwm frequency has to be set independently
#define MOTOR_CURVE_NONE

// loop time in uS
// this affects soft gyro lpf frequency if used
#define LOOPTIME 1000

// failsafe time in uS
#define FAILSAFETIME 1000000  // one second

// debug things ( debug struct and other)
#define DEBUG

// rxdebug structure
//#define RXDEBUG

// enable motors if pitch / roll controls off center (at zero throttle)
// possible values: 0 / 1
// use in acro build only
#define ENABLESTIX 0
#define ENABLESTIX_TRESHOLD 0.3
#define ENABLESTIX_TIMEOUT 1e6





#pragma diag_warning 1035 , 177 , 4017
#pragma diag_error 260

//--fpmode=fast


#ifdef MOTOR_BEEPS
#ifdef USE_ESC_DRIVER
#warning "MOTOR BEEPS_WORKS WITH BRUSHED MOTORS ONLY"
#endif
#endif

// for the ble beacon to work after in-flight reset
#ifdef RX_BAYANG_PROTOCOL_BLE_BEACON
#undef STOP_LOWBATTERY
#endif

// gcc warnings in main.c

//TODO:  MOVE TARGET DEFINES TO THEIR OWN FILE AND REORDER THE .H HIERCHARCHY TO config.h->targets.h->defines.h   Correct all relavent #includes related to this change

#ifdef BWHOOP
#define F0
#define ENABLE_OVERCLOCK
#ifdef ENABLE_OVERCLOCK
#define SYS_CLOCK_FREQ_HZ 64000000
#define PWM_CLOCK_FREQ_HZ 64000000
#define TICK_CLOCK_FREQ_HZ 8000000
#else
#define SYS_CLOCK_FREQ_HZ 48000000
#define PWM_CLOCK_FREQ_HZ 48000000
#define TICK_CLOCK_FREQ_HZ 6000000
#endif

//LEDS
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_2
#define LED1PORT GPIOA
#define LED2PIN GPIO_Pin_3
#define LED2PORT GPIOA
#define LED1_INVERT
#define LED2_INVERT

//I2C & GYRO
#define USE_HARDWARE_I2C
#define HW_I2C_SPEED_FAST2
#define HW_I2C_PINS_PA910
#define I2C_SDAPIN GPIO_Pin_10
#define I2C_SDAPORT GPIOA
#define I2C_SCLPIN GPIO_Pin_9
#define I2C_SCLPORT GPIOA
#define I2C_GYRO_ADDRESS 0x68
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x98 // new id
#define GYRO_ID_3 0x7D
#define GYRO_ID_4 0x72
#define SENSOR_ROTATE_90_CW

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART1_SDA
#define F0_USART_PINSWAP
#define SOFTSPI_NONE
#else
#define SOFTSPI_3WIRE
#define SPI_MOSI_PIN GPIO_Pin_0
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_1
#define SPI_CLK_PORT GPIOF
#define SPI_SS_PIN GPIO_Pin_0
#define SPI_SS_PORT GPIOF
#define RADIO_XN297L
#define RADIO_CHECK
#endif

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_5
#define BATTERYPORT GPIOA
#define BATTERY_ADC_CHANNEL ADC_Channel_5
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 10000
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 10000
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 2.8
#endif

// MOTOR PINS
#define MOTOR0_PIN_PB1 // motor 0 back-left
#define MOTOR1_PIN_PA4 // motor 1 front-left
#define MOTOR2_PIN_PA6 // motor 2 back-right
#define MOTOR3_PIN_PA7 // motor 3 front-right

// pwm pin initialization
#define USE_PWM_DRIVER
#define PWM_PA4
#define PWM_PA6
#define PWM_PA7
#define PWM_PB1
#endif

#ifdef E011
#define F0
#define ENABLE_OVERCLOCK
#ifdef ENABLE_OVERCLOCK
#define SYS_CLOCK_FREQ_HZ 64000000
#define PWM_CLOCK_FREQ_HZ 64000000
#define TICK_CLOCK_FREQ_HZ 8000000
#else
#define SYS_CLOCK_FREQ_HZ 48000000
#define PWM_CLOCK_FREQ_HZ 48000000
#define TICK_CLOCK_FREQ_HZ 6000000
#endif

//LEDS
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_2
#define LED1PORT GPIOA
#define LED2PIN GPIO_Pin_3
#define LED2PORT GPIOA
#define LED1_INVERT
#define LED2_INVERT

//I2C & GYRO
#define USE_SOFTWARE_I2C
#define SOFTI2C_SPEED_FAST
#define SOFTI2C_PUSHPULL_CLK
#define I2C_SDAPIN GPIO_Pin_10
#define I2C_SDAPORT GPIOA
#define I2C_SCLPIN GPIO_Pin_9
#define I2C_SCLPORT GPIOA
#define I2C_GYRO_ADDRESS 0x68
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x98 // new id
#define GYRO_ID_3 0x7D
#define GYRO_ID_4 0x72
#define SENSOR_ROTATE_90_CW

// SPI RX PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS)
#define USART1_SDA
#define F0_USART_PINSWAP
#define SOFTSPI_NONE
#else
#define SOFTSPI_3WIRE
#define SPI_MOSI_PIN GPIO_Pin_0
#define SPI_MOSI_PORT GPIOF
#define SPI_CLK_PIN GPIO_Pin_1
#define SPI_CLK_PORT GPIOF
#define SPI_SS_PIN GPIO_Pin_0
#define SPI_SS_PORT GPIOA
#define RADIO_XN297L
#define RADIO_CHECK
#endif

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_5
#define BATTERYPORT GPIOA
#define BATTERY_ADC_CHANNEL ADC_Channel_5
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 10000
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 10000
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 2.8
#endif

// Assingment of pin to motor
#define MOTOR0_PIN_PA6 // motor 0 back-left
#define MOTOR1_PIN_PA4 // motor 1 front-left
#define MOTOR2_PIN_PB1 // motor 2 back-right
#define MOTOR3_PIN_PA7 // motor 3 front-right

// pwm pin initialization
#define USE_PWM_DRIVER
#define PWM_PA4
#define PWM_PA6
#define PWM_PA7
#define PWM_PB1
#endif

#ifdef H8mini_blue_board
#define F0
#define ENABLE_OVERCLOCK
#ifdef ENABLE_OVERCLOCK
#define SYS_CLOCK_FREQ_HZ 64000000
#define PWM_CLOCK_FREQ_HZ 64000000
#define TICK_CLOCK_FREQ_HZ 8000000
#else
#define SYS_CLOCK_FREQ_HZ 48000000
#define PWM_CLOCK_FREQ_HZ 48000000
#define TICK_CLOCK_FREQ_HZ 6000000
#endif

//LEDS
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_1
#define LED1PORT GPIOF
#define LED2PIN GPIO_Pin_3
#define LED2PORT GPIOA

//I2C & GYRO
#define USE_SOFTWARE_I2C
#define SOFTI2C_SPEED_FAST
#define SOFTI2C_PUSHPULL_CLK
#define I2C_SDAPIN GPIO_Pin_10
#define I2C_SDAPORT GPIOA
#define I2C_SCLPIN GPIO_Pin_9
#define I2C_SCLPORT GPIOA
#define I2C_GYRO_ADDRESS 0x68
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x78 // common h8 gyro
#define GYRO_ID_3 0x7D
#define GYRO_ID_4 0x72
#define SENSOR_ROTATE_180

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART1_SDA
#define F0_USART_PINSWAP
#define SOFTSPI_NONE
#else
#define SOFTSPI_3WIRE
#define SPI_MOSI_PIN GPIO_Pin_1
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_2
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_3
#define SPI_SS_PORT GPIOA
#define RADIO_XN297L
#define RADIO_CHECK
#endif

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_5
#define BATTERYPORT GPIOA
#define BATTERY_ADC_CHANNEL ADC_Channel_5
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 10000
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 10000
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 2.8
#endif

// Assingment of pin to motor
#define MOTOR0_PIN_PA6 // motor 0 back-left
#define MOTOR1_PIN_PA4 // motor 1 front-left
#define MOTOR2_PIN_PB1 // motor 2 back-right
#define MOTOR3_PIN_PA7 // motor 3 front-right

// pwm pin initialization
#define USE_PWM_DRIVER
#define PWM_PA4
#define PWM_PA6
#define PWM_PA7
#define PWM_PB1
#endif

#ifdef Alienwhoop_ZERO

#define F0
#define ENABLE_OVERCLOCK
#ifdef ENABLE_OVERCLOCK
#define SYS_CLOCK_FREQ_HZ 64000000
#define PWM_CLOCK_FREQ_HZ 64000000
#define TICK_CLOCK_FREQ_HZ 8000000
#else
#define SYS_CLOCK_FREQ_HZ 48000000
#define PWM_CLOCK_FREQ_HZ 48000000
#define TICK_CLOCK_FREQ_HZ 6000000
#endif

//LEDS
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_0
#define LED1PORT GPIOF
#define LED2PIN GPIO_Pin_0
#define LED2PORT GPIOA

//I2C & GYRO
#define USE_HARDWARE_I2C
#define HW_I2C_SPEED_FAST2
#define HW_I2C_PINS_PA910
#define I2C_SDAPIN GPIO_Pin_10
#define I2C_SDAPORT GPIOA
#define I2C_SCLPIN GPIO_Pin_9
#define I2C_SCLPORT GPIOA
#define I2C_GYRO_ADDRESS 0x68
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x98 // new id
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x72 
#define SENSOR_ROTATE_90_CCW

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#if defined(RX_FPORT)
	#define F0_USART_PINSWAP
#endif
#define USART1_PA3PA2
#define SOFTSPI_NONE
#else
#define SOFTSPI_3WIRE
#define SPI_MOSI_PIN GPIO_Pin_3
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_2
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_1
#define SPI_SS_PORT GPIOA
#define RADIO_CHECK
#define RADIO_XN297L
#endif

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_5
#define BATTERYPORT GPIOA
#define BATTERY_ADC_CHANNEL ADC_Channel_5
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 2000
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 1000
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
#define MOTOR0_PIN_PA7
//#define MOTOR1_PIN_PA4  //2nd Draft prototype patch
//#define MOTOR2_PIN_PB1  //2nd Draft prototype patch
#define MOTOR1_PIN_PB1
#define MOTOR2_PIN_PA4
#define MOTOR3_PIN_PA6

// pwm pin initialization
//#define USE_PWM_DRIVER
#define USE_DSHOT_DMA_DRIVER
#define PWM_PA4
#define PWM_PA6
#define PWM_PA7
#define PWM_PB1
#endif

#if defined(CC3D_REVO_F4) || defined(OmnibusF4SD) || defined(OmnibusF4) || defined(LuxF4osd)
#define F405
#define SYS_CLOCK_FREQ_HZ 168000000
#define PWM_CLOCK_FREQ_HZ 84000000
#define TICK_CLOCK_FREQ_HZ 21000000

//LEDS
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_5
#define LED1PORT GPIOB
#define LED2PIN GPIO_Pin_4
#define LED2PORT GPIOB
#define LED1_INVERT
//#define LED2_INVERT

//SPI, I2C & GYRO
#define MPU6XXX_SPI1


#define USE_DUMMY_I2C									//todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
//#define I2C_SDAPIN GPIO_Pin_10
//#define I2C_SDAPORT GPIOA
//#define I2C_SCLPIN GPIO_Pin_9
//#define I2C_SCLPORT GPIOA
//#define I2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69
#define GYRO_ID_1 0x68
//#define GYRO_ID_2 0x73
//#define GYRO_ID_3 0x78
//#define GYRO_ID_4 0x72
//#define SENSOR_ROTATE_90_CCW
#if defined(LuxF4osd)
#define SENSOR_ROTATE_90_CCW
#else
#define SENSOR_FLIP_180
#endif

//#define DISABLE_GYRO_CHECK


// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
//   //not created yet
//#define UART3_INVERTER_PIN PC9 // Omnibus F4 Pro Corner
#ifdef OmnibusF4SD
#define USART6_PC7PC6
#define USART_INVERTER_PIN GPIO_Pin_8
#define USART_INVERTER_PORT GPIOC
#define USART1_PA10PA9
#endif
#if defined(CC3D_REVO_F4) || defined(OmnibusF4) || defined(LuxF4osd)
#define USART1_PA10PA9
#define USART_INVERTER_PIN GPIO_Pin_0
#define USART_INVERTER_PORT GPIOC
#define USART3_PB11PB10
#define USART4_PA1PA0
#endif
#define SOFTSPI_NONE
#else
#define SOFTSPI_3WIRE							//todo:port spi receiver and soft spi driver to f4
#define SPI_MOSI_PIN GPIO_Pin_3
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_2
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_1
#define SPI_SS_PORT GPIOA
#define RADIO_CHECK
#define RADIO_XN297L
#endif

//VOLTAGE DIVIDER
#define DISABLE_LVC
#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_12
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 4700
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 2200
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
#define MOTOR0_PIN_PA3
#define MOTOR1_PIN_PA2
#define MOTOR2_PIN_PB0
#define MOTOR3_PIN_PB1

//pyroflip
//#define MOTOR0_PIN_PA8
//#define MOTOR1_PIN_PC9			//DSHOT not coded for port C yet
//#define MOTOR2_PIN_PB1
//#define MOTOR3_PIN_PB0

// pwm pin initialization
//#define USE_PWM_DRIVER
//#define USE_ESC_DRIVER       //todo:  evaluate need for this to stay if focused on quadcopters
#define USE_DSHOT_DMA_DRIVER  
//#define USE_DSHOT_DRIVER_BETA  //todo:  probably eliminate this completely

//#define PWM_PA0
//#define PWM_PA1
#define PWM_PA2
#define PWM_PA3
//#define PWM_PA4
//#define PWM_PA5
//#define PWM_PA6
//#define PWM_PA7
//#define PWM_PA8
//#define PWM_PA9
//#define PWM_PA10
//#define PWM_PA11
#define PWM_PB0
#define PWM_PB1
//#define PWM_PC9
#endif

#ifdef Alienwhoop_V2
#define F405
#define SYS_CLOCK_FREQ_HZ 168000000
#define PWM_CLOCK_FREQ_HZ 84000000
#define TICK_CLOCK_FREQ_HZ 21000000

//LEDS
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_12
#define LED1PORT GPIOC
#define LED2PIN GPIO_Pin_2
#define LED2PORT GPIOD
//#define LED1_INVERT
#define LED2_INVERT

//SPI, I2C & GYRO
#define MPU6XXX_SPI1


#define USE_DUMMY_I2C									//todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
//#define I2C_SDAPIN GPIO_Pin_10
//#define I2C_SDAPORT GPIOA
//#define I2C_SCLPIN GPIO_Pin_9
//#define I2C_SCLPORT GPIOA
//#define I2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69
#define GYRO_ID_1 0x70
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x71
//#define GYRO_ID_4 0x72 
#define SENSOR_ROTATE_90_CW
//#define DISABLE_GYRO_CHECK


// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART2_PA3PA2
#define USART3_PC11PC10

#define SOFTSPI_NONE
#else
#define SOFTSPI_3WIRE							//todo:port spi receiver and soft spi driver to f4
#define SPI_MOSI_PIN GPIO_Pin_3
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_2
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_1
#define SPI_SS_PORT GPIOA
#define RADIO_CHECK
#define RADIO_XN297L
#endif

//VOLTAGE DIVIDER
#define DISABLE_LVC
/*#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_12
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 4700
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 2200
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif */

// MOTOR PINS
#define MOTOR0_PIN_PC6
#define MOTOR1_PIN_PC7
#define MOTOR2_PIN_PC8
#define MOTOR3_PIN_PC9

// pwm pin initialization
#define USE_PWM_DRIVER
//#define USE_ESC_DRIVER       //todo:  evaluate need for this to stay if focused on quadcopters
//#define USE_DSHOT_DMA_DRIVER  
//#define USE_DSHOT_DRIVER_BETA  //todo:  probably eliminate this completely

//#define PWM_PA0
//#define PWM_PA1
//#define PWM_PA2
//#define PWM_PA3
//#define PWM_PA4
//#define PWM_PA5
//#define PWM_PA6
//#define PWM_PA7
//#define PWM_PA8
//#define PWM_PA9
//#define PWM_PA10
//#define PWM_PA11
//#define PWM_PB0
//#define PWM_PB1
#define PWM_PC6
#define PWM_PC7
#define PWM_PC8
#define PWM_PC9
#endif


#ifdef CLRacing_F4
#define F405
#define SYS_CLOCK_FREQ_HZ 168000000
#define PWM_CLOCK_FREQ_HZ 84000000
#define TICK_CLOCK_FREQ_HZ 21000000

//LEDS
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_5
#define LED1PORT GPIOB
#define LED2PIN GPIO_Pin_5
#define LED2PORT GPIOB
#define LED1_INVERT
//#define LED2_INVERT

//SPI, I2C & GYRO
#define MPU6XXX_SPI1

#define USE_DUMMY_I2C									//todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
//#define I2C_SDAPIN GPIO_Pin_10
//#define I2C_SDAPORT GPIOA
//#define I2C_SCLPIN GPIO_Pin_9
//#define I2C_SCLPORT GPIOA
//#define I2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69
#define GYRO_ID_1 0x68
//#define GYRO_ID_2 0x73
//#define GYRO_ID_3 0x78
//#define GYRO_ID_4 0x72
//#define SENSOR_ROTATE_90_CCW
#define SENSOR_ROTATE_90_CW
//#define DISABLE_GYRO_CHECK

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART1_PA10PA9
#define USART_INVERTER_PIN GPIO_Pin_0
#define USART_INVERTER_PORT GPIOC
#define USART3_PB11PB10
#define USART4_PA1PA0
#define USART6_PC7PC6
#define SOFTSPI_NONE
#else
#define SOFTSPI_3WIRE							//todo:port spi receiver and soft spi driver to f4
#define SPI_MOSI_PIN GPIO_Pin_3
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_2
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_1
#define SPI_SS_PORT GPIOA
#define RADIO_CHECK
#define RADIO_XN297L
#endif

//VOLTAGE DIVIDER
#define DISABLE_LVC
#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_12
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 4700
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 2200
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
#define MOTOR0_PIN_PA3
#define MOTOR1_PIN_PA2
#define MOTOR2_PIN_PB0
#define MOTOR3_PIN_PB1

// pwm pin initialization
//#define USE_PWM_DRIVER
//#define USE_ESC_DRIVER       //todo:  evaluate need for this to stay if focused on quadcopters
#define USE_DSHOT_DMA_DRIVER  
//#define USE_DSHOT_DRIVER_BETA  //todo:  probably eliminate this completely
#endif


#ifdef Raceflight_Revolt
#define F405
#define SYS_CLOCK_FREQ_HZ 168000000
#define PWM_CLOCK_FREQ_HZ 84000000
#define TICK_CLOCK_FREQ_HZ 21000000

//LEDS
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_5
#define LED1PORT GPIOB
#define LED2PIN GPIO_Pin_5
#define LED2PORT GPIOB
#define LED1_INVERT
//#define LED2_INVERT

//SPI, I2C & GYRO
#define ICM20601_SPI1

#define USE_DUMMY_I2C									//todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
//#define I2C_SDAPIN GPIO_Pin_10
//#define I2C_SDAPORT GPIOA
//#define I2C_SCLPIN GPIO_Pin_9
//#define I2C_SCLPORT GPIOA
//#define I2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69
#define GYRO_ID_1 0x68
//#define GYRO_ID_2 0x73
//#define GYRO_ID_3 0x78
//#define GYRO_ID_4 0x72
#define SENSOR_ROTATE_90_CCW
//#define SENSOR_FLIP_180
//#define SENSOR_ROTATE_90_CW
//#define DISABLE_GYRO_CHECK

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART1_PA10PA9
#define USART_INVERTER_PIN GPIO_Pin_0
#define USART_INVERTER_PORT GPIOC
#define USART3_PB11PB10
#define USART4_PA1PA0
#define USART6_PC7PC6
#define SOFTSPI_NONE
#else
#define SOFTSPI_3WIRE							//todo:port spi receiver and soft spi driver to f4
#define SPI_MOSI_PIN GPIO_Pin_3
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_2
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_1
#define SPI_SS_PORT GPIOA
#define RADIO_CHECK
#define RADIO_XN297L
#endif

//VOLTAGE DIVIDER
#define DISABLE_LVC
#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_12
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 4700
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 2200
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
#define MOTOR0_PIN_PB0
#define MOTOR1_PIN_PB1
#define MOTOR2_PIN_PA3
#define MOTOR3_PIN_PA2

// pwm pin initialization
//#define USE_PWM_DRIVER
//#define USE_ESC_DRIVER       //todo:  evaluate need for this to stay if focused on quadcopters
#define USE_DSHOT_DMA_DRIVER  
//#define USE_DSHOT_DRIVER_BETA  //todo:  probably eliminate this completely
#endif

//more f4 todo:  rgb led port, dma dshot port, code spi dma driver for f4 gyro, setup more uarts in driver
//softserial not throwing errors anymore but needs review for proper configuration, port of buzzer function has not been investigated, reorganize control() in control.c to call functions from new files intuitively named for said functions
//add all recent updates from NFE_Silverware
//add various omnibus f4 targets, raceflight revolt target, airbot target

