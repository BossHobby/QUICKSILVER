#include "profile.h"

#include <string.h>

#include "drv_usb.h"
#include "usb_configurator.h"
#include "util.h"

// Default values for our profile
// ignore -Wmissing-braces here, gcc bug with nested structs
#pragma GCC diagnostic ignored "-Wmissing-braces"

#define DEFAULT_PID_RATE_PRESET 3

const pid_rate_preset_t pid_rate_presets[] = {
    //Brushless Pids
    {
        .index = 0,
        .name = "TWR <8:1 2",
        .rate = {
            .kp = {14.5e-2, 14.5e-2, 2.5e-1},
            .ki = {14.0e-1, 14.0e-1, 14.0e-1},
            .kd = {4.5e-1, 4.5e-1, 0.5e-1},
        },
    },

    {
        .index = 1,
        .name = "TWR 8:1 3",
        .rate = {
            .kp = {12.5e-2, 14.5e-2, 25.0e-2},
            .ki = {14.0e-1, 14.0e-1, 14.0e-1},
            .kd = {2.3e-1, 3.3e-1, 0.5e-1},
        },
    },

    {
        .index = 2,
        .name = "TWR 12:1 4",
        .rate = {
            .kp = {9.5e-2, 12.5e-2, 20.0e-2},
            .ki = {14.0e-1, 14.0e-1, 14.0e-1},
            .kd = {2.3e-1, 3.3e-1, 0.5e-1},
        },
    },

    {
        .index = 3,
        .name = "TWR 14:1 5",
        .rate = {
            .kp = {7.5e-2, 7.5e-2, 18.0e-2},
            .ki = {14.0e-1, 14.0e-1, 14.0e-1},
            .kd = {2.3e-1, 2.3e-1, 0.4e-1},
        },
    },

    {
        .index = 4,
        .name = "65mm 1s brushless whoop",
        .rate = {
            .kp = {14e-2, 14e-2, 2.5e-1},
            .ki = {14.0e-1, 14.0e-1, 14.0e-1},
            .kd = {5.6e-1, 5.6e-1, 0.5e-1},
        },
    },

    {
        .index = 5,
        .name = "65mm 2s brushless whoop",
        .rate = {
            .kp = {10.5e-2, 10.5e-2, 2.15e-1},
            .ki = {14.0e-1, 14.0e-1, 14.0e-1},
            .kd = {4.8e-1, 4.8e-1, 0.5e-1},
        },
    },

    {
        .index = 6,
        .name = "6mm & 7mm Abduction Pids for whoops (Team Alienwhoop)",
        //  - set filtering ALIENWHOOP_ZERO_FILTERING
        .rate = {
            .kp = {21.5e-2, 21.5e-2, 105.0e-2},
            .ki = {14e-1, 15e-1, 15e-1},
            .kd = {7.4e-1, 7.4e-1, 5.5e-1},
        },
    },

    {
        .index = 7,
        .name = "5in Chameleon, T - Motor 2306 2600kV HQ 5x4.3x3 - Bobnova edition",
        .rate = {
            .kp = {6.5e-2, 7.5e-2, 15.0e-2},
            .ki = {12.0e-1, 12.0e-1, 12.0e-1},
            .kd = {1.7e-1, 2.4e-1, 0.3e-1},
        },
    },

    {
        .index = 8,
        .name = "BOSS 7 with 716 motors and 46mm Props",
        // set filtering to BETA_FILTERING and adjust pass 1 and pass 2 for KALMAN_GYRO both to 70hz, set DTERM_LPF_2ND_HZ to 120hz, disable motor filtering
        // set TORQUE_BOOST to 1.0, and add #define THROTTLE_TRANSIENT_COMPENSATION and #define THROTTLE_TRANSIENT_COMPENSATION_FACTOR 4.0
        .rate = {
            .kp = {19.5e-2, 19.5e-2, 95.0e-2},
            .ki = {12e-1, 12e-1, 8e-1},
            .kd = {10.7e-1, 10.7e-1, 2.0e-1},
        },
    },

    //***************  The following tunes beyond this point are all pretty dated.  I have not built/flown/tuned any of these in a long time and there have been alot of changes.
    //***************  If your build best matches some of the specs below ... consider the tune a starting point and give me feedback/adjust as necessary.

    {
        .index = 9,
        .name = "(OLD) 6mm experimental AwesomeSauce 20000kv Pids (Team Alienwhoop)",
        //  - set filtering ALIENWHOOP_ZERO_FILTERING
        .rate = {
            .kp = {25.5e-2, 25.5e-2, 11.5e-1},
            .ki = {20.5e-1, 20.5e-1, 16e-1},
            .kd = {11.4e-1, 11.4e-1, 4.9e-1},
        },
    },

    {
        .index = 10,
        .name = "(OLD) BOSS 6 & 7 - 615 and 716 motors, hm830 46mm props",
        //   - set filtering to VERY_STRONG_FILTERING
        .rate = {
            .kp = {24.5e-2, 24.5e-2, 9.5e-1},
            .ki = {12e-1, 12e-1, 8e-1},
            .kd = {14.1e-1, 14.1e-1, 7e-1},
        },
    },

    {
        .index = 11,
        .name = "(OLD) BOSS 8.0 - 816 motors, kingkong 66mm props",
        //   - set filtering to WEAK_FILTERING
        .rate = {
            .kp = {26.7e-2, 26.7e-2, 9.5e-1},
            .ki = {12e-1, 12e-1, 8e-1},
            .kd = {16.2e-1, 16.2e-1, 7e-1},
        },
    },

    {
        .index = 12,
        .name = "(OLD) BOSS 8.5 - 820 motors, kingkong 66mm props",
        //   - set filtering to STRONG_FILTERING
        .rate = {
            .kp = {29.5e-2, 29.5e-2, 11.5e-1},
            .ki = {12e-1, 12e-1, 12.0e-1},
            .kd = {17.5e-1, 17.5e-1, 7e-1},
        },
    },
};

uint32_t pid_rate_presets_count = sizeof(pid_rate_presets) / sizeof(pid_rate_preset_t);

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
    },
    .voltage = {
#ifdef LIPO_CELL_COUNT
        .lipo_cell_count = LIPO_CELL_COUNT,
#else
        .lipo_cell_count = 0,
#endif
#ifdef PID_VOLTAGE_COMPENSATION
        .pid_voltage_compensation = 1,
#else
        .pid_voltage_compensation = 0,
#endif
        .vbattlow = VBATTLOW,
        .actual_battery_voltage = ACTUAL_BATTERY_VOLTAGE,
        .reported_telemetry_voltage = REPORTED_TELEMETRY_VOLTAGE,
    },
    .channel = {
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
            RATES,   //AUX_RATES
            LEDS_ON, //AUX_LEDS_ON
#ifdef BUZZER_ENABLE //AUX_BUZZER_ENABLE
            BUZZER_ENABLE,
#else
            AUX_CHANNEL_OFF,
#endif
            STARTFLIP, //AUX_STARTFLIP

#ifdef MOTORS_TO_THROTTLE_MODE //AUX_MOTORS_TO_THROTTLE_MODE
            MOTORS_TO_THROTTLE_MODE,
#else
            AUX_CHANNEL_OFF,
#endif
            RSSI,
#ifdef FPV_ON //AUX_FN_INVERTED
            FPV_ON,
#else
            AUX_CHANNEL_OFF,
#endif
        },
    },
};

#define MACRO_STR(arg) #arg

target_info_t target_info = {
    .target_name = MACRO_STR(TARGET),
    .quic_protocol_version = QUIC_PROTOCOL_VERSION,

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel) "P" #port #pin,
    .motor_pins = {MOTOR_PINS},
#undef MOTOR_PIN

#define USART_PORT(channel, port, rx_pin, tx_pin) "USART_" #channel,
    .usart_ports = {"NONE", USART_PORTS},
#undef USART_PORT
};

#pragma GCC diagnostic pop

// the actual profile
profile_t profile;

void profile_set_defaults() {
  memcpy(&profile, &default_profile, sizeof(profile_t));

  profile.pid.pid_rates[0] = pid_rate_presets[DEFAULT_PID_RATE_PRESET].rate;
}

pid_rate_t *profile_current_pid_rates() {
  return &profile.pid.pid_rates[profile.pid.pid_profile];
}

cbor_result_t cbor_encode_vector_t(cbor_value_t *enc, const vector_t *vec) {
  cbor_result_t res = cbor_encode_array(enc, 3);
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_float(enc, &vec->axis[0]);
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_float(enc, &vec->axis[1]);
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_float(enc, &vec->axis[2]);
  if (res < CBOR_OK) {
    return res;
  }
  return res;
}

cbor_result_t cbor_encode_metadata_t(cbor_value_t *enc, const metadata_t *meta) {
  cbor_result_t res = CBOR_OK;

  res = cbor_encode_map_indefinite(enc);
  if (res < CBOR_OK) {
    return res;
  }

  res = cbor_encode_str(enc, "name");
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_tstr(enc, meta->name, 36);
  if (res < CBOR_OK) {
    return res;
  }

  res = cbor_encode_str(enc, "datetime");
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_uint32(enc, &meta->datetime);
  if (res < CBOR_OK) {
    return res;
  }

  res = cbor_encode_end_indefinite(enc);
  if (res < CBOR_OK) {
    return res;
  }
  return res;
}

#define START_STRUCT_ENCODER(type)                                     \
  cbor_result_t cbor_encode_##type(cbor_value_t *enc, const type *o) { \
    cbor_result_t res = CBOR_OK;                                       \
    res = cbor_encode_map_indefinite(enc);                             \
    if (res < CBOR_OK)                                                 \
      return res;

#define END_STRUCT_ENCODER()              \
  return cbor_encode_end_indefinite(enc); \
  }

#define MEMBER(member, type)                 \
  res = cbor_encode_str(enc, #member);       \
  if (res < CBOR_OK)                         \
    return res;                              \
  res = cbor_encode_##type(enc, &o->member); \
  if (res < CBOR_OK)                         \
    return res;

#define STR_MEMBER(member)               \
  res = cbor_encode_str(enc, #member);   \
  if (res < CBOR_OK)                     \
    return res;                          \
  res = cbor_encode_str(enc, o->member); \
  if (res < CBOR_OK)                     \
    return res;

#define ARRAY_MEMBER(member, size, type)          \
  res = cbor_encode_str(enc, #member);            \
  if (res < CBOR_OK)                              \
    return res;                                   \
  res = cbor_encode_array(enc, size);             \
  if (res < CBOR_OK)                              \
    return res;                                   \
  for (uint32_t i = 0; i < size; i++) {           \
    res = cbor_encode_##type(enc, &o->member[i]); \
    if (res < CBOR_OK)                            \
      return res;                                 \
  }

#define STR_ARRAY_MEMBER(member, size)        \
  res = cbor_encode_str(enc, #member);        \
  if (res < CBOR_OK)                          \
    return res;                               \
  res = cbor_encode_array(enc, size);         \
  if (res < CBOR_OK)                          \
    return res;                               \
  for (uint32_t i = 0; i < size; i++) {       \
    res = cbor_encode_str(enc, o->member[i]); \
    if (res < CBOR_OK)                        \
      return res;                             \
  }

START_STRUCT_ENCODER(rate_mode_silverware_t)
SILVERWARE_RATE_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(rate_mode_betaflight_t)
BETAFLIGHT_RATE_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(rate_t)
RATE_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(motor_t)
MOTOR_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(serial_t)
SERIAL_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(voltage_t)
VOLTAGE_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(pid_rate_t)
PID_RATE_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(angle_pid_rate_t)
ANGLE_PID_RATE_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(pid_rate_preset_t)
PID_RATE_PRESET_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(stick_rate_t)
STICK_RATE_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(profile_pid_t)
PID_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(channel_t)
CHANNEL_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(profile_t)
PROFILE_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(target_info_t)
TARGET_INFO_MEMBERS
END_STRUCT_ENCODER()

#undef MEMBER
#undef ARRAY_MEMBER

cbor_result_t cbor_decode_vector_t(cbor_value_t *it, vector_t *vec) {
  cbor_result_t res = CBOR_OK;

  cbor_container_t array;
  res = cbor_decode_array(it, &array);
  if (res < CBOR_OK)
    return res;

  res = cbor_decode_float(it, &vec->axis[0]);
  if (res < CBOR_OK)
    return res;

  res = cbor_decode_float(it, &vec->axis[1]);
  if (res < CBOR_OK)
    return res;

  res = cbor_decode_float(it, &vec->axis[2]);
  if (res < CBOR_OK)
    return res;

  return res;
}

cbor_result_t cbor_decode_metadata_t(cbor_value_t *dec, metadata_t *meta) {
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
      res = cbor_decode_tstr(dec, &name, &name_len);
      if (res < CBOR_OK) {
        return res;
      }

      if (name_len > 36) {
        name_len = 36;
      }
      memset(meta->name, 0, 36);
      memcpy(meta->name, name, name_len);
      continue;
    }

    if (buf_equal_string(name, name_len, "datetime")) {
      res = cbor_decode_uint32(dec, &meta->datetime);
      if (res < CBOR_OK) {
        return res;
      }
      continue;
    }

    res = cbor_decode_skip(dec);
    if (res < CBOR_OK)
      return res;
  }
  return res;
}

#define START_STRUCT_DECODER(type)                                   \
  cbor_result_t cbor_decode_##type(cbor_value_t *dec, type *o) {     \
    cbor_result_t res = CBOR_OK;                                     \
    cbor_container_t map;                                            \
    res = cbor_decode_map(dec, &map);                                \
    if (res < CBOR_OK)                                               \
      return res;                                                    \
    const uint8_t *name;                                             \
    uint32_t name_len;                                               \
    for (uint32_t i = 0; i < cbor_decode_map_size(dec, &map); i++) { \
      res = cbor_decode_tstr(dec, &name, &name_len);                 \
      if (res < CBOR_OK)                                             \
        return res;

#define END_STRUCT_DECODER()   \
  res = cbor_decode_skip(dec); \
  if (res < CBOR_OK)           \
    return res;                \
  }                            \
  return res;                  \
  }

#define MEMBER(member, type)                       \
  if (buf_equal_string(name, name_len, #member)) { \
    res = cbor_decode_##type(dec, &o->member);     \
    if (res < CBOR_OK)                             \
      return res;                                  \
    continue;                                      \
  }

#define ARRAY_MEMBER(member, size, type)            \
  if (buf_equal_string(name, name_len, #member)) {  \
    cbor_container_t array;                         \
    res = cbor_decode_array(dec, &array);           \
    if (res < CBOR_OK)                              \
      return res;                                   \
    for (uint32_t i = 0; i < size; i++) {           \
      res = cbor_decode_##type(dec, &o->member[i]); \
      if (res < CBOR_OK)                            \
        return res;                                 \
    }                                               \
    continue;                                       \
  }

START_STRUCT_DECODER(rate_mode_silverware_t)
SILVERWARE_RATE_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(rate_mode_betaflight_t)
BETAFLIGHT_RATE_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(rate_t)
RATE_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(motor_t)
MOTOR_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(serial_t)
SERIAL_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(voltage_t)
VOLTAGE_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(pid_rate_t)
PID_RATE_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(angle_pid_rate_t)
ANGLE_PID_RATE_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(stick_rate_t)
STICK_RATE_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(profile_pid_t)
PID_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(channel_t)
CHANNEL_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(profile_t)
PROFILE_MEMBERS
END_STRUCT_DECODER()
#undef MEMBER
#undef ARRAY_MEMBER
