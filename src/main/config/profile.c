#include "profile.h"

#include <string.h>

#include "drv_usb.h"
#include "usb_configurator.h"
#include "util.h"
#include "util/cbor_helper.h"

// Default values for our profile
// ignore -Wmissing-braces here, gcc bug with nested structs
#pragma GCC diagnostic ignored "-Wmissing-braces"

#define DEFAULT_PID_RATE_PRESET 3

const pid_rate_preset_t pid_rate_presets[] = {
    //Brushless Pids
    {
        .index = 0,
        .name = "TWR <8:1 2in",
        .rate = {
            .kp = {91, 91, 78.5},
            .ki = {70, 70, 70},
            .kd = {54, 54, 6},
        },
    },

    {
        .index = 1,
        .name = "TWR 8:1 3in",
        .rate = {
            .kp = {78.5, 91, 78.5},
            .ki = {70, 70, 70},
            .kd = {27.5, 39.5, 6},
        },
    },

    {
        .index = 2,
        .name = "TWR 12:1 4in",
        .rate = {
            .kp = {59.5, 78.5, 63},
            .ki = {70, 70, 70},
            .kd = {27.5, 39.5, 6},
        },
    },

    {
        .index = 3,
        .name = "TWR 14:1 5in",
        .rate = {
            .kp = {47, 47, 56.5},
            .ki = {70, 70, 70},
            .kd = {27.5, 27.5, 4.5},
        },
    },

    {
        .index = 4,
        .name = "65mm 1s brushless whoop",
        .rate = {
            .kp = {117, 117, 150},
            .ki = {77, 77, 77},
            .kd = {83, 83, 13},
        },
    },

    {
        .index = 5,
        .name = "65mm 1s brushless whoop - Hanfer Edition",
        .rate = {
            .kp = {123, 123, 150},
            .ki = {77, 77, 77},
            .kd = {91, 91, 13},
        },
    },

    {
        .index = 6,
        .name = "75mm brushless whoop",
        .rate = {
            .kp = {127, 127, 148},
            .ki = {77, 77, 77},
            .kd = {101, 101, 13},
        },
    },

    {
        .index = 7,
        .name = "6mm & 7mm Abduction Pids for whoops (Team Alienwhoop)",
        //  - set filtering ALIENWHOOP_ZERO_FILTERING
        .rate = {
            .kp = {135, 135, 330},
            .ki = {70, 75, 75},
            .kd = {89, 89, 66},
        },
    },

    {
        .index = 8,
        .name = "5in Chameleon, T - Motor 2306 2600kV HQ 5x4.3x3 - Bobnova edition",
        .rate = {
            .kp = {41, 47, 47},
            .ki = {60, 60, 60},
            .kd = {20.5, 29, 3.5},
        },
    },

    {
        .index = 9,
        .name = "BOSS 7 with 716 motors and 46mm Props",
        // set filtering to BETA_FILTERING and adjust pass 1 and pass 2 for KALMAN_GYRO both to 70hz, set DTERM_LPF_2ND_HZ to 120hz, disable motor filtering
        // set TORQUE_BOOST to 1.0, and add #define THROTTLE_TRANSIENT_COMPENSATION and #define THROTTLE_TRANSIENT_COMPENSATION_FACTOR 4.0
        .rate = {
            .kp = {122.5, 122.5, 298},
            .ki = {60, 60, 40},
            .kd = {128.5, 128.5, 24},
        },
    },

    //***************  The following tunes beyond this point are all pretty dated.  I have not built/flown/tuned any of these in a long time and there have been alot of changes.
    //***************  If your build best matches some of the specs below ... consider the tune a starting point and give me feedback/adjust as necessary.

    {
        .index = 10,
        .name = "(OLD) 6mm experimental AwesomeSauce 20000kv Pids (Team Alienwhoop)",
        //  - set filtering ALIENWHOOP_ZERO_FILTERING
        .rate = {
            .kp = {160, 160, 361},
            .ki = {102.5, 102.5, 80},
            .kd = {137, 137, 59},
        },
    },

    {
        .index = 11,
        .name = "(OLD) BOSS 6 & 7 - 615 and 716 motors, hm830 46mm props",
        //   - set filtering to VERY_STRONG_FILTERING
        .rate = {
            .kp = {154, 154, 298},
            .ki = {60, 60, 40},
            .kd = {169, 169, 84},
        },
    },

    {
        .index = 12,
        .name = "(OLD) BOSS 8.0 - 816 motors, kingkong 66mm props",
        //   - set filtering to WEAK_FILTERING
        .rate = {
            .kp = {167.5, 167.5, 298},
            .ki = {60, 60, 40},
            .kd = {194.5, 194.5, 84},
        },
    },

    {
        .index = 13,
        .name = "(OLD) BOSS 8.5 - 820 motors, kingkong 66mm props",
        //   - set filtering to STRONG_FILTERING
        .rate = {
            .kp = {185, 185, 361},
            .ki = {60, 60, 60},
            .kd = {210, 210, 84},
        },
    },
};

const uint32_t pid_rate_presets_count = sizeof(pid_rate_presets) / sizeof(pid_rate_preset_t);
const profile_t default_profile = {
    .meta = {
        .name = "default",
        .datetime = 0,
    },

    .motor = {
#ifdef INVERT_YAW_PID
        .invert_yaw = 1,
#else
        .invert_yaw = 0,
#endif
        .digital_idle = DIGITAL_IDLE,
        .dshot_time = DSHOT_TIME_600,

#ifdef TORQUE_BOOST
        .torque_boost = TORQUE_BOOST,
#else
        .torque_boost = 0.0,
#endif
#ifdef THROTTLE_BOOST
        .throttle_boost = THROTTLE_BOOST,
#else
        .throttle_boost = 0.0,
#endif
#ifdef SENSOR_ROTATE_90_CW
        .gyro_orientation = GYRO_ROTATE_90_CW,
#endif
#ifdef SENSOR_ROTATE_45_CCW
        .gyro_orientation = GYRO_ROTATE_45_CCW,
#endif
#ifdef SENSOR_ROTATE_45_CW
        .gyro_orientation = GYRO_ROTATE_45_CW,
#endif
#ifdef SENSOR_ROTATE_90_CCW
        .gyro_orientation = GYRO_ROTATE_90_CCW,
#endif
#ifdef SENSOR_ROTATE_135_CCW
        .gyro_orientation = GYRO_ROTATE_90_CCW | GYRO_ROTATE_45_CCW,
#endif
#ifdef SENSOR_ROTATE_135_CW
        .gyro_orientation = GYRO_ROTATE_90_CW | GYRO_ROTATE_45_CW,
#endif
#ifdef SENSOR_ROTATE_180
        .gyro_orientation = GYRO_ROTATE_180,
#endif
#ifdef SENSOR_FLIP_180
        .gyro_orientation = GYRO_FLIP_180,
#endif
#if !defined(SENSOR_ROTATE_90_CW) && !defined(SENSOR_ROTATE_45_CCW) && !defined(SENSOR_ROTATE_45_CW) && !defined(SENSOR_ROTATE_90_CCW) && !defined(SENSOR_ROTATE_180) && !defined(SENSOR_FLIP_180)
        .gyro_orientation = GYRO_ROTATE_NONE,
#endif
#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel) MOTOR_PIN_IDENT(port, pin),
        .motor_pins = {MOTOR_PINS},
#undef MOTOR_PIN
#ifndef DISABLE_FLIP_SEQUENCER
        .turtle_throttle_percent = 10.0f,
#else
        .turtle_throttle_percent = 0.00f,
#endif
    },

    .serial = {
#ifdef RX_USART
        .rx = RX_USART,
#else
        .rx = USART_PORT_INVALID,
#endif
#ifdef SMART_AUDIO_USART
        .smart_audio = SMART_AUDIO_USART,
#else
        .smart_audio = USART_PORT_INVALID,
#endif
    },

    .filter = {
        .gyro = {
#if defined(GYRO_FILTER_PASS1_PT1) || defined(GYRO_FILTER_PASS1_PT2)
            {
#ifdef GYRO_FILTER_PASS1_PT1
                .type = FILTER_LP_PT1,
                .cutoff_freq = GYRO_FREQ_PASS1,
#else //GYRO_FILTER_PASS1_PT2
                .type = FILTER_LP2_PT1,
                .cutoff_freq = GYRO_FREQ_PASS1,
#endif
            },
#else
            {
                .type = FILTER_NONE,
            },
#endif
#if defined(GYRO_FILTER_PASS2_PT1) || defined(GYRO_FILTER_PASS2_PT2)
            {
#ifdef GYRO_FILTER_PASS2_PT1
                .type = FILTER_LP_PT1,
                .cutoff_freq = GYRO_FREQ_PASS1,
#else //GYRO_FILTER_PASS2_PT2
                .type = FILTER_LP2_PT1,
                .cutoff_freq = GYRO_FREQ_PASS1,
#endif
            }
#else
            {
                .type = FILTER_NONE,
            }
#endif
        },

        .dterm = {
#if defined(DTERM_FILTER_PASS1_PT1) || defined(DTERM_FILTER_PASS1_PT2)
            {
#ifdef DTERM_FILTER_PASS1_PT1
                .type = FILTER_LP_PT1,
                .cutoff_freq = DTERM_FREQ_PASS1,
#else //DTERM_FILTER_PASS1_PT2
                .type = FILTER_LP2_PT1,
                .cutoff_freq = DTERM_FREQ_PASS1,
#endif
            },
#else
            {
                .type = FILTER_NONE,
            },
#endif
#if defined(DTERM_FILTER_PASS2_PT1) || defined(DTERM_FILTER_PASS2_PT2)
            {
#ifdef DTERM_FILTER_PASS2_PT1
                .type = FILTER_LP_PT1,
                .cutoff_freq = DTERM_FREQ_PASS2,
#else //DTERM_FILTER_PASS2_PT2
                .type = FILTER_LP2_PT1,
                .cutoff_freq = DTERM_FREQ_PASS2,
#endif
            }
#else
            {
                .type = FILTER_NONE,
            }
#endif
        },

#ifdef DTERM_DYNAMIC_LPF
        .dterm_dynamic_enable = 1,
#else
        .dterm_dynamic_enable = 0,
#endif
#ifdef DYNAMIC_FREQ_MIN
        .dterm_dynamic_min = DYNAMIC_FREQ_MIN,
#endif
#ifdef DYNAMIC_FREQ_MAX
        .dterm_dynamic_max = DYNAMIC_FREQ_MAX,
#endif
    },

    .rate = {
#ifdef SILVERWARE_RATES
        .mode = RATE_MODE_SILVERWARE,
#endif
#ifdef BETAFLIGHT_RATES
        .mode = RATE_MODE_BETAFLIGHT,
#endif
        .silverware = {
            .max_rate = {
                MAX_RATE,
                MAX_RATE,
                MAX_RATEYAW,
            },
            .acro_expo = {
                ACRO_EXPO_ROLL,
                ACRO_EXPO_PITCH,
                ACRO_EXPO_YAW,
            },
            .angle_expo = {
                ANGLE_EXPO_ROLL,
                ANGLE_EXPO_PITCH,
                ANGLE_EXPO_YAW,
            },
        },
        .betaflight = {
            .rc_rate = {
                BF_RC_RATE_ROLL,
                BF_RC_RATE_PITCH,
                BF_RC_RATE_YAW,
            },
            .super_rate = {
                BF_SUPER_RATE_ROLL,
                BF_SUPER_RATE_PITCH,
                BF_SUPER_RATE_YAW,
            },
            .expo = {
                BF_EXPO_ROLL,
                BF_EXPO_PITCH,
                BF_EXPO_YAW,
            },
        },

        .level_max_angle = LEVEL_MAX_ANGLE,
        .low_rate_mulitplier = LOW_RATES_MULTI,
        .sticks_deadband = STICKS_DEADBAND,
    },

    //************************************PIDS****************************************
    .pid = {
        .pid_profile = PID_PROFILE_1,
        .pid_rates = {},
        .stick_profile = STICK_PROFILE_OFF,
        .stick_rates = {
            //**************************ADVANCED PID CONTROLLER - WITH PROFILE SWITCHING ON AUX SWITCH STICK_BOOST_PROFILE*******************************
            // GENERAL SUMMARY OF THIS FEATURE:
            // stickAccelerator and stickTransition are a more detailed version of the traditional D term setpoint weight and transition variables that you may be familiar with in other firmwares.
            // The difference here is that we name the D term setpoint weight "Stick Accelerator" because it's actual function is to accelerate the response of the pid controller to stick inputs.
            // Another difference is that negative stick transitions are possible meaning that you can have a higher stick acceleration near center stick which fades to a lower stick acceleration at
            // full stick throws should you desire to see what that feels like.  Traditionally we are only used to being able to transition from a low setpoint to a higher one.
            // The final differences are that you can adjust each axis independently and also set up two seperate profiles so that you can switch "feels" in flight with the STICK_BOOST_PROFILE aux
            // channel selection set up in the receiver section of config.h
            //
            //HOW TO USE THIS FEATURE:
            // Safe values for stickAccelerator are from 0 to about 2.5 where 0 represents a "MEASUREMENT" based D term calculation and is the traditional Silverware PID controller, and a
            // a value of 1 represents an "ERROR" based D term calculation.  Values above 1 add even more acceleration but be reasonable and keep this below about 2.5.

            // Range of acceptable values for stickTransition are from -1 to 1.  Do not input a value outside of this range.  When stick transition is 0 - no stick transition will take place
            // and stick acceleration will remain constant regardless of stick position.  Positive values up to 1 will represent a transition where stick acceleration at it's maximum at full
            // stick deflection and is reduced by whatever percentage you enter here at stick center.  For example accelerator at 1 and transition at .3 means that there will be 30% reduction
            // of acceleration at stick center, and acceleration strength of 1 at full stick.
            {
                //pid profile A	Roll  PITCH  YAW
                .accelerator = {0.0, 0.0, 0.0}, //keep values between 0 and 2.5
                .transition = {0.0, 0.0, 0.0},  //keep values between -1 and 1
            },
            {
                //pid profile B	Roll  PITCH  YAW
                .accelerator = {1.5, 1.5, 1.0}, //keep values between 0 and 2.5
                .transition = {0.3, 0.3, 0.0},  //keep values between -1 and 1
            },
        },
        //**************************** ANGLE PIDS - used in level mode to set leveling strength

        // Leveling algorithm coefficients for small errors  (normal flying)
        .small_angle = {
            .kp = 10.00, // P TERM GAIN ROLL + PITCH
            .kd = 3.0,   // D TERM GAIN ROLL + PITCH
        },

        // Leveling algorithm coefficients for large errors  (stick banging or collisions)
        .big_angle = {
            .kp = 5.00, // P TERM GAIN ROLL + PITCH
            .kd = 0.0,  // D TERM GAIN ROLL + PITCH
        },

        .throttle_dterm_attenuation = {
#ifdef THROTTLE_D_ATTENUATION
            .tda_active = THROTTLE_D_ATTENUATION_ACTIVE,
#else
            .tda_active = THROTTLE_D_ATTENUATION_NONE,
#endif
            .tda_breakpoint = TDA_BREAKPOINT,
            .tda_percent = TDA_PERCENT,
        },
    },
    .voltage = {
#ifdef LIPO_CELL_COUNT
        .lipo_cell_count = LIPO_CELL_COUNT,
#else
        .lipo_cell_count = 0,
#endif
#ifdef PID_VOLTAGE_COMPENSATION
        .pid_voltage_compensation = PID_VOLTAGE_COMPENSATION_ACTIVE,
#else
        .pid_voltage_compensation = PID_VOLTAGE_COMPENSATION_NONE,
#endif
        .vbattlow = VBATTLOW,
        .actual_battery_voltage = ACTUAL_BATTERY_VOLTAGE,
        .reported_telemetry_voltage = REPORTED_TELEMETRY_VOLTAGE,
    },
    .receiver = {
        .aux = {
            ARMING,              //AUX_ARMING
            IDLE_UP,             //AUX_IDLE_UP
            LEVELMODE,           //AUX_LEVELMODE
            RACEMODE,            //AUX_RACEMODE
            HORIZON,             //AUX_HORIZON
            STICK_BOOST_PROFILE, //AUX_STICK_BOOST_PROFILE
#ifdef STICK_TRAVEL_CHECK        //AUX_TRAVEL_CHECK
            STICK_TRAVEL_CHECK,
#else
            AUX_CHANNEL_OFF,
#endif
            HIGH_RATES, //AUX_HIGH_RATES
            LEDS_ON,    //AUX_LEDS_ON
#ifdef BUZZER_ENABLE    //AUX_BUZZER_ENABLE
            BUZZER_ENABLE,
#else
            AUX_CHANNEL_OFF,
#endif
            TURTLE, //AUX_TURTLE

#ifdef MOTORS_TO_THROTTLE_MODE //AUX_MOTORS_TO_THROTTLE_MODE
            MOTORS_TO_THROTTLE_MODE,
#else
            AUX_CHANNEL_OFF,
#endif
            RSSI,
#ifdef FPV_ON //AUX_FPV_ON
            FPV_ON,
#else
            AUX_CHANNEL_OFF,
#endif
#ifdef FN_INVERTED //AUX_FN_INVERTED
            FN_INVERTED,
#else
            AUX_CHANNEL_OFF,
#endif
        },
        .lqi_source = RX_LQI_SOURCE_PACKET_RATE,
    },
    .osd = {
        .elements = {
            0xA7,
            0x43495551,
            0x4C49534B,
            0x3F524556,
            0x3F3F3F3F,
            0x3F3F3F3F,
            0x704,
            0x72D,
            0x755,
            0x6A9,
            0xE1,
            0x6E1,
            0x321,
            0x681,
            0x84,
        },
    },
};

#define _MACRO_STR(arg) #arg
#define MACRO_STR(name) _MACRO_STR(name)

target_info_t target_info = {
    .target_name = MACRO_STR(TARGET),
    .git_version = MACRO_STR(GIT_VERSION),

    .features = 0
#ifdef BRUSHLESS_TARGET
                | FEATURE_BRUSHLESS
#endif
#ifdef ENABLE_OSD
                | FEATURE_OSD
#endif
#ifdef USE_M25P16
                | FEATURE_BLACKBOX
#endif
    ,
    .rx_protocol = RX_PROTOCOL,
    .quic_protocol_version = QUIC_PROTOCOL_VERSION,

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel) "P" #port #pin,
    .motor_pins = {MOTOR_PINS},
#undef MOTOR_PIN

#define USART_PORT(channel, rx_pin, tx_pin) "USART_" #channel,
    .usart_ports = {"NONE", USART_PORTS},
#undef USART_PORT

    .gyro_id = 0x0,
};

#pragma GCC diagnostic pop

// the actual profile
profile_t profile;

void profile_set_defaults() {
  memcpy(&profile, &default_profile, sizeof(profile_t));

  for (uint8_t i = 0; i < PID_PROFILE_MAX; i++) {
    profile.pid.pid_rates[i] = pid_rate_presets[DEFAULT_PID_RATE_PRESET].rate;
  }
}

pid_rate_t *profile_current_pid_rates() {
  return &profile.pid.pid_rates[profile.pid.pid_profile];
}

cbor_result_t cbor_encode_profile_metadata_t(cbor_value_t *enc, const profile_metadata_t *meta) {
  cbor_result_t res = CBOR_OK;

  CBOR_CHECK_ERROR(res = cbor_encode_map_indefinite(enc));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "name"));
  CBOR_CHECK_ERROR(res = cbor_encode_tstr(enc, meta->name, 36));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "datetime"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &meta->datetime));

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));
  return res;
}

#define MEMBER CBOR_ENCODE_MEMBER
#define STR_MEMBER CBOR_ENCODE_STR_MEMBER
#define ARRAY_MEMBER CBOR_ENCODE_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_ENCODE_STR_ARRAY_MEMBER

CBOR_START_STRUCT_ENCODER(rate_mode_silverware_t)
SILVERWARE_RATE_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(rate_mode_betaflight_t)
BETAFLIGHT_RATE_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(profile_rate_t)
RATE_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(profile_motor_t)
MOTOR_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(profile_serial_t)
SERIAL_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(profile_filter_parameter_t)
FILTER_PARAMETER_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(profile_filter_t)
FILTER_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(profile_osd_t)
OSD_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(profile_voltage_t)
VOLTAGE_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(pid_rate_t)
PID_RATE_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(angle_pid_rate_t)
ANGLE_PID_RATE_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(pid_rate_preset_t)
PID_RATE_PRESET_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(stick_rate_t)
STICK_RATE_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(throttle_dterm_attenuation_t)
DTERM_ATTENUATION_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(profile_pid_t)
PID_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(profile_receiver_t)
RECEIVER_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(profile_t)
PROFILE_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(target_info_t)
TARGET_INFO_MEMBERS
CBOR_END_STRUCT_ENCODER()

#undef MEMBER
#undef STR_MEMBER
#undef ARRAY_MEMBER
#undef STR_ARRAY_MEMBER

cbor_result_t cbor_decode_profile_metadata_t(cbor_value_t *dec, profile_metadata_t *meta) {
  cbor_result_t res = CBOR_OK;

  cbor_container_t map;
  res = cbor_decode_map(dec, &map);
  if (res < CBOR_OK)
    return res;

  const uint8_t *name;
  uint32_t name_len;
  for (uint32_t i = 0; i < cbor_decode_map_size(dec, &map); i++) {
    res = cbor_decode_tstr(dec, &name, &name_len);
    if (res < CBOR_OK)
      return res;

    if (buf_equal_string(name, name_len, "name")) {
      CBOR_CHECK_ERROR(res = cbor_decode_tstr(dec, &name, &name_len));

      if (name_len > 36) {
        name_len = 36;
      }
      memset(meta->name, 0, 36);
      memcpy(meta->name, name, name_len);
      continue;
    }

    if (buf_equal_string(name, name_len, "datetime")) {
      CBOR_CHECK_ERROR(res = cbor_decode_uint32(dec, &meta->datetime));
      continue;
    }

    res = cbor_decode_skip(dec);
    if (res < CBOR_OK)
      return res;
  }
  return res;
}

#define MEMBER CBOR_DECODE_MEMBER
#define STR_MEMBER CBOR_DECODE_STR_MEMBER
#define ARRAY_MEMBER CBOR_DECODE_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_DECODE_STR_ARRAY_MEMBER

CBOR_START_STRUCT_DECODER(rate_mode_silverware_t)
SILVERWARE_RATE_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(rate_mode_betaflight_t)
BETAFLIGHT_RATE_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(profile_rate_t)
RATE_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(profile_motor_t)
MOTOR_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(profile_serial_t)
SERIAL_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(profile_filter_parameter_t)
FILTER_PARAMETER_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(profile_filter_t)
FILTER_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(profile_osd_t)
OSD_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(profile_voltage_t)
VOLTAGE_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(pid_rate_t)
PID_RATE_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(angle_pid_rate_t)
ANGLE_PID_RATE_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(stick_rate_t)
STICK_RATE_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(throttle_dterm_attenuation_t)
DTERM_ATTENUATION_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(profile_pid_t)
PID_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(profile_receiver_t)
RECEIVER_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(profile_t)
PROFILE_MEMBERS
CBOR_END_STRUCT_DECODER()

#undef MEMBER
#undef STR_MEMBER
#undef ARRAY_MEMBER
#undef STR_ARRAY_MEMBER
