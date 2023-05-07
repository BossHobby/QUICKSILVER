#include "core/profile.h"

#include <string.h>

#include "driver/usb.h"
#include "io/quic.h"
#include "osd/render.h"
#include "rx/rx.h"
#include "util/cbor_helper.h"
#include "util/util.h"

// Default values for our profile
// ignore -Wmissing-braces here, gcc bug with nested structs
#pragma GCC diagnostic ignored "-Wmissing-braces"

#define DEFAULT_BLACKBOX_PRESET 0
const blackbox_preset_t blackbox_presets[] = {
    {
        .field_flags = ((1 << BBOX_FIELD_MAX) - 1), // Set all bits
        .sample_rate_hz = 1000,
        .name = "All fields, Sample Rate=1000",
        .name_osd = "ALL, HZ=1000",
    },
    {
        .field_flags = (1 << BBOX_FIELD_GYRO_FILTER) | (1 << BBOX_FIELD_LOOP) | (1 << BBOX_FIELD_TIME),
        .sample_rate_hz = 200,
        .name = "Gyro Filtered, Sample Rate=200",
        .name_osd = "GYRO FILTERED, HZ=200",
    },
};
const uint32_t blackbox_presets_count = sizeof(blackbox_presets) / sizeof(blackbox_preset_t);

void blackbox_preset_apply(const blackbox_preset_t *preset, profile_blackbox_t *profile) {
  profile->field_flags = preset->field_flags | (1 << BBOX_FIELD_LOOP) | (1 << BBOX_FIELD_TIME);
  profile->sample_rate_hz = preset->sample_rate_hz;
}

uint8_t blackbox_preset_equals(const blackbox_preset_t *preset, profile_blackbox_t *profile) {
  return preset->field_flags == profile->field_flags && preset->sample_rate_hz == profile->sample_rate_hz;
}

#define DEFAULT_PID_RATE_PRESET 0

const pid_rate_preset_t pid_rate_presets[] = {
    // Brushless Pids
    {
        .index = 0,
        .name = "Thrust/Weight Ratio 14:1 5in",
        .rate = {
            .kp = {47, 54, 56.5},
            .ki = {70, 70, 70},
            .kd = {27.5, 33.5, 4.5},
        },
    },

    {
        .index = 1,
        .name = "Thrust/Weight Ratio 12:1 4in",
        .rate = {
            .kp = {65.5, 70.5, 63},
            .ki = {70, 70, 70},
            .kd = {35.5, 39.5, 6},
        },
    },

    {
        .index = 2,
        .name = "Thrust/Weight Ratio 10:1 3in",
        .rate = {
            .kp = {78.5, 80.5, 78.5},
            .ki = {70, 70, 70},
            .kd = {42.5, 44.5, 6},
        },
    },

    {
        .index = 3,
        .name = "Thrust/Weight Ratio <8:1 2in",
        .rate = {
            .kp = {91, 91, 78.5},
            .ki = {70, 70, 70},
            .kd = {54, 54, 6},
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
        .name = "75mm 1s brushless whoop",
        .rate = {
            .kp = {127, 127, 148},
            .ki = {77, 77, 77},
            .kd = {101, 101, 13},
        },
    },

    {
        .index = 6,
        .name = "6mm & 7mm brushed whoop (Alienwhoop ZER0)",
        //  - set filtering ALIENWHOOP_ZERO_FILTERING
        .rate = {
            .kp = {135, 135, 330},
            .ki = {70, 75, 75},
            .kd = {89, 89, 66},
        },
    },

    {
        .index = 7,
        .name = "7mm brushed micro",
        .rate = {
            .kp = {122.5, 122.5, 298},
            .ki = {60, 60, 40},
            .kd = {128.5, 128.5, 24},
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
        .motor_limit = MOTOR_LIMIT,
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
#ifdef GYRO_ORIENTATION
        .gyro_orientation = GYRO_ORIENTATION,
#else
        .gyro_orientation = GYRO_ROTATE_NONE,
#endif
        .motor_pins = {
            MOTOR_PIN0,
            MOTOR_PIN1,
            MOTOR_PIN2,
            MOTOR_PIN3,
        },
        .turtle_throttle_percent = 10.0f,
    },

    .serial = {
#ifdef RX_USART
        .rx = RX_USART,
#else
        .rx = SERIAL_PORT_INVALID,
#endif
#ifdef SMART_AUDIO_USART
        .smart_audio = SMART_AUDIO_USART,
#else
        .smart_audio = SERIAL_PORT_INVALID,
#endif
#ifdef HDZERO_USART
        .hdzero = HDZERO_USART,
#else
        .hdzero = SERIAL_PORT_INVALID,
#endif
    },

    .filter = {
        .gyro = {
#if defined(GYRO_FILTER_PASS1_PT1) || defined(GYRO_FILTER_PASS1_PT2)
            {
#ifdef GYRO_FILTER_PASS1_PT1
                .type = FILTER_LP_PT1,
                .cutoff_freq = GYRO_FREQ_PASS1,
#else // GYRO_FILTER_PASS1_PT2
                .type = FILTER_LP_PT2,
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
#else // GYRO_FILTER_PASS2_PT2
                .type = FILTER_LP_PT2,
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
#else // DTERM_FILTER_PASS1_PT2
                .type = FILTER_LP_PT2,
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
#else // DTERM_FILTER_PASS2_PT2
                .type = FILTER_LP_PT2,
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
        .profile = STICK_RATE_PROFILE_1,
        .rates = {
#ifdef SILVERWARE_RATES
            {
                .mode = RATE_MODE_SILVERWARE,
                .rate = {
                    {
                        MAX_RATE,
                        MAX_RATE,
                        MAX_RATEYAW,
                    },
                    {
                        ACRO_EXPO_ROLL,
                        ACRO_EXPO_PITCH,
                        ACRO_EXPO_YAW,
                    },
                    {
                        ANGLE_EXPO_ROLL,
                        ANGLE_EXPO_PITCH,
                        ANGLE_EXPO_YAW,
                    },
                },
            },
            {
                .mode = RATE_MODE_BETAFLIGHT,
                .rate = {
                    {
                        BF_RC_RATE_ROLL,
                        BF_RC_RATE_PITCH,
                        BF_RC_RATE_YAW,
                    },
                    {
                        BF_SUPER_RATE_ROLL,
                        BF_SUPER_RATE_PITCH,
                        BF_SUPER_RATE_YAW,
                    },
                    {
                        BF_EXPO_ROLL,
                        BF_EXPO_PITCH,
                        BF_EXPO_YAW,
                    },
                },
            },
#endif
#ifdef BETAFLIGHT_RATES
            {
                .mode = RATE_MODE_BETAFLIGHT,
                .rate = {
                    {
                        BF_RC_RATE_ROLL,
                        BF_RC_RATE_PITCH,
                        BF_RC_RATE_YAW,
                    },
                    {
                        BF_SUPER_RATE_ROLL,
                        BF_SUPER_RATE_PITCH,
                        BF_SUPER_RATE_YAW,
                    },
                    {
                        BF_EXPO_ROLL,
                        BF_EXPO_PITCH,
                        BF_EXPO_YAW,
                    },
                },
            },
            {
                .mode = RATE_MODE_SILVERWARE,
                .rate = {
                    {
                        MAX_RATE,
                        MAX_RATE,
                        MAX_RATEYAW,
                    },
                    {
                        ACRO_EXPO_ROLL,
                        ACRO_EXPO_PITCH,
                        ACRO_EXPO_YAW,
                    },
                    {
                        ANGLE_EXPO_ROLL,
                        ANGLE_EXPO_PITCH,
                        ANGLE_EXPO_YAW,
                    },
                },
            },
#endif
        },

        .level_max_angle = LEVEL_MAX_ANGLE,
        .sticks_deadband = STICKS_DEADBAND,
        .throttle_mid = THROTTLE_MID,
        .throttle_expo = THROTTLE_EXPO,
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
            // HOW TO USE THIS FEATURE:
            // Safe values for stickAccelerator are from 0 to about 2.5 where 0 represents a "MEASUREMENT" based D term calculation and is the traditional Silverware PID controller, and a
            // a value of 1 represents an "ERROR" based D term calculation.  Values above 1 add even more acceleration but be reasonable and keep this below about 2.5.

            // Range of acceptable values for stickTransition are from -1 to 1.  Do not input a value outside of this range.  When stick transition is 0 - no stick transition will take place
            // and stick acceleration will remain constant regardless of stick position.  Positive values up to 1 will represent a transition where stick acceleration at it's maximum at full
            // stick deflection and is reduced by whatever percentage you enter here at stick center.  For example accelerator at 1 and transition at .3 means that there will be 30% reduction
            // of acceleration at stick center, and acceleration strength of 1 at full stick.
            {
                // pid profile A	Roll  PITCH  YAW
                .accelerator = {0.0, 0.0, 0.0}, // keep values between 0 and 2.5
                .transition = {0.0, 0.0, 0.0},  // keep values between -1 and 1
            },
            {
                // pid profile B	Roll  PITCH  YAW
                .accelerator = {1.5, 1.5, 1.0}, // keep values between 0 and 2.5
                .transition = {0.3, 0.3, 0.0},  // keep values between -1 and 1
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
        .vbat_scale = 110,
#ifdef IBAT_SCALE
        .ibat_scale = IBAT_SCALE,
#else
        .ibat_scale = 0,
#endif
    },
    .receiver = {
#if defined(RX_EXPRESS_LRS)
        .protocol = RX_PROTOCOL_EXPRESS_LRS,
#elif defined(RX_FRSKY)
        .protocol = RX_PROTOCOL_REDPINE,
#elif defined(RX_FLYSKY)
        .protocol = RX_PROTOCOL_FLYSKY_AFHDS2A,
#elif defined(RX_NRF24_BAYANG_TELEMETRY)
        .protocol = RX_PROTOCOL_NRF24_BAYANG_TELEMETRY,
#elif defined(RX_BAYANG_PROTOCOL_BLE_BEACON)
        .protocol = RX_PROTOCOL_BAYANG_PROTOCOL_BLE_BEACON,
#elif defined(RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND)
        .protocol = RX_PROTOCOL_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND,
#else
        .protocol = RX_PROTOCOL_UNIFIED_SERIAL,
#endif

        .aux = {
            ARMING,              // AUX_ARMING
            IDLE_UP,             // AUX_IDLE_UP
            LEVELMODE,           // AUX_LEVELMODE
            RACEMODE,            // AUX_RACEMODE
            HORIZON,             // AUX_HORIZON
            STICK_BOOST_PROFILE, // AUX_STICK_BOOST_PROFILE
            AUX_CHANNEL_OFF,     // AUX_RATE_PROFILE
#ifdef BUZZER_ENABLE             // AUX_BUZZER_ENABLE
            BUZZER_ENABLE,
#else
            AUX_CHANNEL_OFF,
#endif
            TURTLE, // AUX_TURTLE

#ifdef MOTORS_TO_THROTTLE_MODE // AUX_MOTOR_TEST
            MOTORS_TO_THROTTLE_MODE,
#else
            AUX_CHANNEL_OFF,
#endif
            RSSI,
#ifdef FPV_SWITCH // AUX_FPV_SWITCH
            FPV_SWITCH,
#else
            AUX_CHANNEL_OFF,
#endif
            AUX_CHANNEL_OFF, // AUX_BLACKBOX
            PREARM,          // AUX_PREARM
        },
        .lqi_source = RX_LQI_SOURCE_PACKET_RATE,
        .channel_mapping = RX_MAPPING_AETR,
        .stick_calibration_limits = {
            {.min = -1, .max = 1}, // axis[0]
            {.min = -1, .max = 1}, // axis[1]
            {.min = -1, .max = 1}, // axis[2]
            {.min = 0, .max = 1}   // axis[3]
        },
    },
    .osd = {
        .callsign = "QUICKSILVER",
        .elements = {
            ENCODE_OSD_ELEMENT(1, 1, 9, 1),   // OSD_CALLSIGN
            ENCODE_OSD_ELEMENT(1, 0, 11, 14), // OSD_CELL_COUNT
            ENCODE_OSD_ELEMENT(0, 0, 1, 14),  // OSD_FUELGAUGE_VOLTS
            ENCODE_OSD_ELEMENT(1, 0, 14, 14), // OSD_FILTERED_VOLTS
            ENCODE_OSD_ELEMENT(1, 0, 24, 14), // OSD_GYRO_TEMP
            ENCODE_OSD_ELEMENT(1, 0, 10, 13), // OSD_FLIGHT_MODE
            ENCODE_OSD_ELEMENT(1, 0, 24, 1),  // OSD_RSSI
            ENCODE_OSD_ELEMENT(1, 0, 24, 13), // OSD_STOPWATCH
            ENCODE_OSD_ELEMENT(1, 0, 8, 6),   // OSD_SYSTEM_STATUS
            ENCODE_OSD_ELEMENT(1, 0, 1, 13),  // OSD_THROTTLE
            ENCODE_OSD_ELEMENT(0, 0, 1, 1),   // OSD_VTX_CHANNEL
            ENCODE_OSD_ELEMENT(1, 0, 1, 14),  // OSD_CURRENT_DRAW
        },
        .elements_hd = {
            ENCODE_OSD_ELEMENT(1, 1, 19, 0),  // OSD_CALLSIGN
            ENCODE_OSD_ELEMENT(1, 0, 21, 17), // OSD_CELL_COUNT
            ENCODE_OSD_ELEMENT(0, 0, 0, 17),  // OSD_FUELGAUGE_VOLTS
            ENCODE_OSD_ELEMENT(1, 0, 24, 17), // OSD_FILTERED_VOLTS
            ENCODE_OSD_ELEMENT(1, 0, 44, 17), // OSD_GYRO_TEMP
            ENCODE_OSD_ELEMENT(1, 0, 20, 16), // OSD_FLIGHT_MODE
            ENCODE_OSD_ELEMENT(1, 0, 44, 0),  // OSD_RSSI
            ENCODE_OSD_ELEMENT(1, 0, 44, 16), // OSD_STOPWATCH
            ENCODE_OSD_ELEMENT(1, 0, 18, 8),  // OSD_SYSTEM_STATUS
            ENCODE_OSD_ELEMENT(1, 0, 0, 16),  // OSD_THROTTLE
            ENCODE_OSD_ELEMENT(0, 0, 0, 0),   // OSD_VTX_CHANNEL
            ENCODE_OSD_ELEMENT(1, 0, 0, 17),  // OSD_CURRENT_DRAW
        },
    },
    .blackbox = {
        // Initialized by profile_set_defaults(), so nothing to do here
    },
};

#define _MACRO_STR(arg) #arg
#define MACRO_STR(name) _MACRO_STR(name)

target_info_t target_info = {
    .target_name = MACRO_STR(TARGET),
    .git_version = MACRO_STR(GIT_VERSION),

    .features = FEATURE_OSD
#ifdef ENABLE_BLACKBOX
                | FEATURE_BLACKBOX
#endif
#ifdef DEBUG
                | FEATURE_DEBUG
#endif
    ,
    .rx_protocols = {
        RX_PROTOCOL_UNIFIED_SERIAL,

#if defined(RX_NRF24_BAYANG_TELEMETRY)
        RX_PROTOCOL_NRF24_BAYANG_TELEMETRY,
#endif
#if defined(RX_BAYANG_PROTOCOL_BLE_BEACON)
        RX_PROTOCOL_BAYANG_PROTOCOL_BLE_BEACON,
#endif
#if defined(RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND)
        RX_PROTOCOL_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND,
#endif

#if defined(RX_FRSKY)
        RX_PROTOCOL_FRSKY_D8,
        RX_PROTOCOL_FRSKY_D16_FCC,
        RX_PROTOCOL_FRSKY_D16_LBT,
        RX_PROTOCOL_REDPINE,
#endif

#if defined(RX_EXPRESS_LRS)
        RX_PROTOCOL_EXPRESS_LRS,
#endif

#if defined(RX_FLYSKY)
        RX_PROTOCOL_FLYSKY_AFHDS,
        RX_PROTOCOL_FLYSKY_AFHDS2A,
#endif
    },
    .quic_protocol_version = QUIC_PROTOCOL_VERSION,

    .gyro_id = 0x0,
};

#pragma GCC diagnostic pop

// the actual profile
FAST_RAM profile_t profile;

void profile_set_defaults() {
  memcpy(&profile, &default_profile, sizeof(profile_t));

  for (uint8_t i = 0; i < PID_PROFILE_MAX; i++) {
    profile.pid.pid_rates[i] = pid_rate_presets[DEFAULT_PID_RATE_PRESET].rate;
  }

  blackbox_preset_apply(&blackbox_presets[DEFAULT_BLACKBOX_PRESET], &profile.blackbox);
}

pid_rate_t *profile_current_pid_rates() {
  return &profile.pid.pid_rates[profile.pid.pid_profile];
}

rate_t *profile_current_rates() {
  return &profile.rate.rates[profile.rate.profile];
}

cbor_result_t cbor_encode_profile_metadata_t(cbor_value_t *enc, const profile_metadata_t *meta) {
  cbor_result_t res = CBOR_OK;

  CBOR_CHECK_ERROR(res = cbor_encode_map_indefinite(enc));

  const uint32_t version = PROFILE_VERSION;
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "version"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &version));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "name"));
  CBOR_CHECK_ERROR(res = cbor_encode_tstr(enc, meta->name, 36));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "datetime"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &meta->datetime));

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));
  return res;
}

#define START_STRUCT CBOR_START_STRUCT_ENCODER
#define END_STRUCT CBOR_END_STRUCT_ENCODER
#define MEMBER CBOR_ENCODE_MEMBER
#define STR_MEMBER CBOR_ENCODE_STR_MEMBER
#define TSTR_MEMBER CBOR_ENCODE_TSTR_MEMBER
#define ARRAY_MEMBER CBOR_ENCODE_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_ENCODE_STR_ARRAY_MEMBER

RATE_MEMBERS
PROFILE_RATE_MEMBERS
MOTOR_MEMBERS
SERIAL_MEMBERS
FILTER_PARAMETER_MEMBERS
FILTER_MEMBERS
OSD_MEMBERS
VOLTAGE_MEMBERS
PID_RATE_MEMBERS
ANGLE_PID_RATE_MEMBERS
PID_RATE_PRESET_MEMBERS
STICK_RATE_MEMBERS
DTERM_ATTENUATION_MEMBERS
PID_MEMBERS
CALIBRATION_LIMIT_MEMBERS
RECEIVER_MEMBERS
BLACKBOX_MEMBERS
BLACKBOX_PRESET_MEMBERS
PROFILE_MEMBERS
TARGET_INFO_MEMBERS

#undef START_STRUCT
#undef END_STRUCT
#undef MEMBER
#undef STR_MEMBER
#undef TSTR_MEMBER
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

#define START_STRUCT CBOR_START_STRUCT_DECODER
#define END_STRUCT CBOR_END_STRUCT_DECODER
#define MEMBER CBOR_DECODE_MEMBER
#define STR_MEMBER CBOR_DECODE_STR_MEMBER
#define TSTR_MEMBER CBOR_DECODE_TSTR_MEMBER
#define ARRAY_MEMBER CBOR_DECODE_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_DECODE_STR_ARRAY_MEMBER

RATE_MEMBERS
PROFILE_RATE_MEMBERS
MOTOR_MEMBERS
SERIAL_MEMBERS
FILTER_PARAMETER_MEMBERS
FILTER_MEMBERS
OSD_MEMBERS
VOLTAGE_MEMBERS
PID_RATE_MEMBERS
ANGLE_PID_RATE_MEMBERS
STICK_RATE_MEMBERS
DTERM_ATTENUATION_MEMBERS
PID_MEMBERS
CALIBRATION_LIMIT_MEMBERS
RECEIVER_MEMBERS
BLACKBOX_MEMBERS
PROFILE_MEMBERS

#undef START_STRUCT
#undef END_STRUCT
#undef MEMBER
#undef STR_MEMBER
#undef TSTR_MEMBER
#undef ARRAY_MEMBER
#undef STR_ARRAY_MEMBER
