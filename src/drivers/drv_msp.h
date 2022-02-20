#pragma once

#define MSP_API_VERSION 1 // out message
#define MSP_FC_VARIANT 2  // out message
#define MSP_FC_VERSION 3  // out message
#define MSP_BOARD_INFO 4  // out message
#define MSP_BUILD_INFO 5  // out message

#define MSP_FEATURE_CONFIG 36

#define MSP_BATTERY_STATE 130 // out message         Connected/Disconnected, Voltage, Current Used

#define MSP_UID 160   // out message         Unique device ID
#define MSP_MOTOR 104 // out message         motors

#define MSP_DISPLAYPORT 182

#define MSP_MOTOR_CONFIG 131 // out message         Motor configuration (min/max throttle, etc)

#define MSP_SET_MOTOR 214   // in message          PropBalance function
#define MSP_SET_4WAY_IF 245 // in message          Sets 4way interface

#define MSP_HEADER_LEN 5

#define MSP_BUILD_DATE_TIME __DATE__ __TIME__