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
            .kp = {54, 54, 56},
            .ki = {70, 70, 70},
            .kd = {33, 33, 4},
        },
    },

    {
        .index = 1,
        .name = "Thrust/Weight Ratio 12:1 4in",
        .rate = {
            .kp = {70, 70, 63},
            .ki = {70, 70, 70},
            .kd = {39, 39, 6},
        },
    },

    {
        .index = 2,
        .name = "Thrust/Weight Ratio 10:1 3in",
        .rate = {
            .kp = {81, 81, 78},
            .ki = {77, 77, 77},
            .kd = {44, 44, 6},
        },
    },

    {
        .index = 3,
        .name = "Thrust/Weight Ratio <8:1 2in",
        .rate = {
            .kp = {91, 91, 78.5},
            .ki = {77, 77, 77},
            .kd = {54, 54, 6},
        },
    },

    {
        .index = 4,
        .name = "65mm 1s brushless whoop",
        .rate = {
            .kp = {108, 108, 118},
            .ki = {77, 77, 77},
            .kd = {68, 68, 12},
        },
    },

    {
        .index = 5,
        .name = "75mm 1s brushless whoop",
        .rate = {
            .kp = {127, 127, 128},
            .ki = {77, 77, 77},
            .kd = {101, 101, 13},
        },
    },

    {
        .index = 6,
        .name = "6mm & 7mm brushed whoop (Alienwhoop ZER0)",
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
        .dshot_telemetry = false,

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
        .gyro_orientation = GYRO_ROTATE_NONE,
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
#ifdef DISPLAYPORT_USART
        .hdzero = DISPLAYPORT_USART,
#else
        .hdzero = SERIAL_PORT_INVALID,
#endif
    },

    .filter = {
        .gyro = {
            {
                .type = GYRO_PASS1_TYPE,
                .cutoff_freq = GYRO_PASS1_FREQ,
            },
            {
                .type = GYRO_PASS2_TYPE,
                .cutoff_freq = GYRO_PASS2_FREQ,
            },
        },

        .dterm = {
            {
                .type = DTERM_PASS1_TYPE,
                .cutoff_freq = DTERM_PASS1_FREQ,
            },
            {
                .type = DTERM_PASS2_TYPE,
                .cutoff_freq = DTERM_PASS2_FREQ,
            },
        },

#ifdef DTERM_DYNAMIC_LPF
        .dterm_dynamic_enable = 1,
#else
        .dterm_dynamic_enable = 0,
#endif
#ifdef DTERM_DYNAMIC_FREQ_MIN
        .dterm_dynamic_min = DTERM_DYNAMIC_FREQ_MIN,
#endif
#ifdef DTERM_DYNAMIC_FREQ_MAX
        .dterm_dynamic_max = DTERM_DYNAMIC_FREQ_MAX,
#endif
#ifdef GYRO_DYNAMIC_NOTCH
        .gyro_dynamic_notch_enable = 1,
#else
        .gyro_dynamic_notch_enable = 0,
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
        .use_filtered_voltage_for_warnings = USE_FILTERED_VOLTAGE_FOR_WARNINGS,
        .vbat_scale = 110,
#ifdef IBAT_SCALE
        .ibat_scale = IBAT_SCALE,
#else
        .ibat_scale = 0,
#endif
    },
    .receiver = {
        .protocol = RX_PROTOCOL_UNIFIED_SERIAL,

        .aux = {
            ARMING,              // AUX_ARMING
            IDLE_UP,             // AUX_IDLE_UP
            LEVELMODE,           // AUX_LEVELMODE
            RACEMODE,            // AUX_RACEMODE
            HORIZON,             // AUX_HORIZON
            STICK_BOOST_PROFILE, // AUX_STICK_BOOST_PROFILE
            AUX_CHANNEL_OFF,     // UNUSED_AUX_HIGH_RATES
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
            AUX_CHANNEL_OFF, // AUX_OSD_PROFILE
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
        .guac_mode = 0,
        .profiles = {
            [OSD_PROFILE_1] = {
                .callsign = "QUICKSILVER",
                .elements = {
                    ENCODE_OSD_ELEMENT(1, 1, 9, 1, 19, 0),    // OSD_CALLSIGN
                    ENCODE_OSD_ELEMENT(1, 0, 11, 14, 21, 17), // OSD_CELL_COUNT
                    ENCODE_OSD_ELEMENT(0, 0, 1, 14, 0, 17),   // OSD_FUELGAUGE_VOLTS
                    ENCODE_OSD_ELEMENT(1, 0, 14, 14, 24, 17), // OSD_FILTERED_VOLTS
                    ENCODE_OSD_ELEMENT(1, 0, 24, 14, 44, 17), // OSD_GYRO_TEMP
                    ENCODE_OSD_ELEMENT(1, 0, 10, 13, 20, 16), // OSD_FLIGHT_MODE
                    ENCODE_OSD_ELEMENT(1, 0, 24, 1, 44, 0),   // OSD_RSSI
                    ENCODE_OSD_ELEMENT(1, 0, 24, 13, 44, 16), // OSD_STOPWATCH
                    ENCODE_OSD_ELEMENT(1, 0, 4, 6, 14, 8),    // OSD_SYSTEM_STATUS
                    ENCODE_OSD_ELEMENT(1, 0, 1, 1, 0, 0),     // OSD_THROTTLE
                    ENCODE_OSD_ELEMENT(0, 0, 1, 1, 0, 0),     // OSD_VTX_CHANNEL
                    ENCODE_OSD_ELEMENT(1, 0, 1, 14, 0, 17),   // OSD_CURRENT_DRAW
                    ENCODE_OSD_ELEMENT(0, 0, 14, 6, 15, 8),   // OSD_CROSSHAIR
                    ENCODE_OSD_ELEMENT(1, 0, 1, 13, 0, 16),   // OSD_CURRENT_DRAWN
                },
            },
            [OSD_PROFILE_2] = {
                .callsign = "QUICKSILVER",
                .elements = {
                    ENCODE_OSD_ELEMENT(0, 1, 9, 1, 19, 0),    // OSD_CALLSIGN
                    ENCODE_OSD_ELEMENT(0, 0, 11, 14, 21, 17), // OSD_CELL_COUNT
                    ENCODE_OSD_ELEMENT(0, 0, 1, 14, 0, 17),   // OSD_FUELGAUGE_VOLTS
                    ENCODE_OSD_ELEMENT(0, 0, 14, 14, 24, 17), // OSD_FILTERED_VOLTS
                    ENCODE_OSD_ELEMENT(0, 0, 24, 14, 44, 17), // OSD_GYRO_TEMP
                    ENCODE_OSD_ELEMENT(0, 0, 10, 13, 20, 16), // OSD_FLIGHT_MODE
                    ENCODE_OSD_ELEMENT(0, 0, 24, 1, 44, 0),   // OSD_RSSI
                    ENCODE_OSD_ELEMENT(0, 0, 24, 13, 44, 16), // OSD_STOPWATCH
                    ENCODE_OSD_ELEMENT(0, 0, 4, 6, 14, 8),    // OSD_SYSTEM_STATUS
                    ENCODE_OSD_ELEMENT(0, 0, 1, 1, 0, 0),     // OSD_THROTTLE
                    ENCODE_OSD_ELEMENT(0, 0, 1, 1, 0, 0),     // OSD_VTX_CHANNEL
                    ENCODE_OSD_ELEMENT(0, 0, 1, 14, 0, 17),   // OSD_CURRENT_DRAW
                    ENCODE_OSD_ELEMENT(0, 0, 14, 6, 15, 8),   // OSD_CROSSHAIR
                    ENCODE_OSD_ELEMENT(0, 0, 1, 13, 0, 16),   // OSD_CURRENT_DRAWN
                },
            },
        },

    },
    .blackbox = {
#ifdef BLACKBOX_DEBUG_FLAGS
        .debug_flags = BLACKBOX_DEBUG_FLAGS,
#endif
        // rest is initialized by profile_set_defaults()
    },
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
  profile.motor.gyro_orientation = target.gyro_orientation;
  if (target.vbat_scale > 0) {
    profile.voltage.vbat_scale = target.vbat_scale;
  }
  if (target.ibat_scale > 0) {
    profile.voltage.ibat_scale = target.ibat_scale;
  }
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
  CBOR_CHECK_ERROR(res = cbor_encode_uint32_t(enc, &version));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "name"));
  CBOR_CHECK_ERROR(res = cbor_encode_tstr(enc, meta->name, 36));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "datetime"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint32_t(enc, &meta->datetime));

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
OSD_PROFILE_MEMBERS
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
      CBOR_CHECK_ERROR(res = cbor_decode_uint32_t(dec, &meta->datetime));
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
OSD_PROFILE_MEMBERS
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
