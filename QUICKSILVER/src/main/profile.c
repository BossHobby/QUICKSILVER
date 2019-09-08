#include "profile.h"

#include <string.h>

#include "drv_usb.h"

// Default values for our profile
// ignore -Wmissing-braces here, gcc bug with nested structs
#pragma GCC diagnostic ignored "-Wmissing-braces"
const profile_t default_profile = {
    .motor = {
#ifdef INVERT_YAW_PID
        .invert_yaw = 1,
#else
        .invert_yaw = 0,
#endif
        .digital_idle = DIGITAL_IDLE,
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
        //Brushless Pids
        //65mm burshless whoop
        //.kp = {12e-2, 12e-2, 2.5e-1},
        //.ki = {14.0e-1, 14.0e-1, 14.0e-1},
        //.kd = {4.5e-1, 4.5e-1, 0.5e-1},

        //TWR <8:1 2"
        //.kp = {14.5e-2 , 14.5e-2  , 2.5e-1 },
        //.ki = { 14.0e-1  , 14.0e-1 , 14.0e-1 },
        //.kd = { 4.5e-1 , 4.5e-1  , 0.5e-1 },

        //TWR 8:1 3"
        //.kp = {12.5e-2 , 14.5e-2  , 25.0e-2 },
        //.ki = { 14.0e-1  , 14.0e-1 , 14.0e-1 },
        //.kd = { 2.3e-1 , 3.3e-1  , 0.5e-1 },

        //TWR 12:1 4"
        //.kp = {9.5e-2 , 12.5e-2  , 20.0e-2 },
        //.ki = { 14.0e-1  , 14.0e-1 , 14.0e-1 },
        //.kd = { 2.3e-1 , 3.3e-1  , 0.5e-1 },

        //5" Chameleon, T-Motor 2306 2600kV HQ 5x4.3x3 -Bobnova edition
        //                         ROLL       PITCH     YAW
        //.kp = {6.5e-2 , 7.5e-2  , 15.0e-2 },
        //.ki = { 12.0e-1  , 12.0e-1 , 12.0e-1 },
        //.kd = { 1.7e-1 , 2.4e-1  , 0.3e-1 },

        //TWR 14:1 5"
        //.kp = {7.5e-2, 7.5e-2, 18.0e-2},
        //.ki = {14.0e-1, 14.0e-1, 14.0e-1},
        //.kd = {2.3e-1, 2.3e-1, 0.4e-1},

        //6mm & 7mm Abduction Pids for whoops (Team Alienwhoop)- set filtering ALIENWHOOP_ZERO_FILTERING
        //                         ROLL       PITCH     YAW
        .kp = {21.5e-2, 21.5e-2, 105.0e-2},
        .ki = {14e-1, 15e-1, 15e-1},
        .kd = {7.4e-1, 7.4e-1, 5.5e-1},

        //BOSS 7 with 716 motors and 46mm Props - set filtering to BETA_FILTERING and adjust pass 1 and pass 2 for KALMAN_GYRO both to 70hz, set DTERM_LPF_2ND_HZ to 120hz, disable motor filtering
        //                                        set TORQUE_BOOST to 1.0, and add #define THROTTLE_TRANSIENT_COMPENSATION and #define THROTTLE_TRANSIENT_COMPENSATION_FACTOR 4.0
        //                         ROLL       PITCH     YAW
        //.kp = { 19.5e-2 , 19.5e-2  , 95.0e-2 },
        //.ki = { 12e-1  , 12e-1 , 8e-1 },
        //.kd = {10.7e-1 , 10.7e-1  , 2.0e-1 },

        //***************  The following tunes beyond this point are all pretty dated.  I have not built/flown/tuned any of these in a long time and there have been alot of changes.
        //***************  If your build best matches some of the specs below ... consider the tune a starting point and give me feedback/adjust as necessary.

        // (OLD) 6mm experimental AwesomeSauce 20000kv Pids (Team Alienwhoop) - set filtering ALIENWHOOP_ZERO_FILTERING
        //                         ROLL       PITCH     YAW
        //.kp = { 25.5e-2 , 25.5e-2  , 11.5e-1 },
        //.ki = { 20.5e-1  , 20.5e-1 , 16e-1 },
        //.kd = { 11.4e-1 , 11.4e-1  , 4.9e-1 },

        // (OLD) BOSS 6 & 7 - 615 and 716 motors, hm830 46mm props  - set filtering to VERY_STRONG_FILTERING
        //                         ROLL       PITCH     YAW
        //.kp = { 24.5e-2 , 24.5e-2  , 9.5e-1 },
        //.ki = { 12e-1  , 12e-1 , 8e-1 },
        //.kd = {14.1e-1 , 14.1e-1  , 7e-1 },
        // (OLD) BOSS 8.0 - 816 motors, kingkong 66mm props  - set filtering to WEAK_FILTERING
        //                         ROLL       PITCH     YAW
        //.kp = { 26.7e-2 , 26.7e-2  , 9.5e-1 },
        //.ki = { 12e-1  , 12e-1 , 8e-1 },
        //.kd = {16.2e-1 , 16.2e-1  , 7e-1 },

        // (OLD) BOSS 8.5 - 820 motors, kingkong 66mm props  - set filtering to STRONG_FILTERING
        //                         ROLL       PITCH     YAW
        //.kp = { 29.5e-2 , 29.5e-2  , 11.5e-1 },
        //.ki = { 12e-1  , 12e-1 , 12.0e-1 },
        //.kd = {17.5e-1 , 17.5e-1  , 7e-1 },
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
            ARMING,       //AUX_ARMING
            IDLE_UP,      //AUX_IDLE_UP
            LEVELMODE,    //AUX_LEVELMODE
            RACEMODE,     //AUX_RACEMODE
            HORIZON,      //AUX_HORIZON
            PIDPROFILE,   //AUX_PIDPROFILE
            TRAVEL_CHECK, //AUX_TRAVEL_CHECK
            RATES,        //AUX_RATES
            LEDS_ON,      //AUX_LEDS_ON
#ifdef BUZZER_ENABLE      //AUX_BUZZER_ENABLE
            BUZZER_ENABLE,
#else
            AUX_CHANNEL_OFF,
#endif
            STARTFLIP, //AUX_STARTFLIP
#ifdef FN_INVERTED     //AUX_FN_INVERTED
            FN_INVERTED,
#else
            AUX_CHANNEL_OFF,
#endif
#ifdef MOTORS_TO_THROTTLE_MODE //AUX_MOTORS_TO_THROTTLE_MODE
            MOTORS_TO_THROTTLE_MODE,
#else
            AUX_CHANNEL_OFF,
#endif
        },
    },
};
#pragma GCC diagnostic pop

// the actual profile
profile_t profile;

void profile_set_defaults() {
  profile = default_profile;
}

int8_t buf_equal(const uint8_t *str1, size_t len1, const uint8_t *str2, size_t len2) {
  if (len2 != len1) {
    return 0;
  }
  for (size_t i = 0; i < len1; i++) {
    if (str1[i] != str2[i]) {
      return 0;
    }
  }
  return 1;
}

int8_t buf_equal_string(const uint8_t *str1, size_t len1, const char *str2) {
  return buf_equal(str1, len1, (const uint8_t *)str2, strlen(str2));
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

cbor_result_t cbor_encode_channel_t(cbor_value_t *enc, const channel_t *chan) {
  cbor_result_t res = CBOR_OK;

  res = cbor_encode_map_indefinite(enc);
  if (res < CBOR_OK) {
    return res;
  }

  res = cbor_encode_str(enc, "aux");
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_array(enc, AUX_FUNCTION_MAX);
  if (res < CBOR_OK) {
    return res;
  }

  for (uint32_t i = 0; i < AUX_FUNCTION_MAX; i++) {
    res = cbor_encode_uint8(enc, &chan->aux[i]);
    if (res < CBOR_OK) {
      return res;
    }
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

START_STRUCT_ENCODER(voltage_t)
VOLTAGE_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(pid_rate_t)
PID_RATE_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(profile_t)
PROFILE_MEMBERS
END_STRUCT_ENCODER()
#undef MEMBER

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

cbor_result_t cbor_decode_channel_t(cbor_value_t *dec, channel_t *chan) {
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

    if (buf_equal_string(name, name_len, "aux")) {
      cbor_container_t array;
      res = cbor_decode_array(dec, &array);
      if (res < CBOR_OK)
        return res;

      for (uint32_t i = 0; i < AUX_FUNCTION_MAX; i++) {
        res = cbor_decode_uint8(dec, &chan->aux[i]);
        if (res < CBOR_OK) {
          return res;
        }
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

START_STRUCT_DECODER(voltage_t)
VOLTAGE_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(pid_rate_t)
PID_RATE_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(profile_t)
PROFILE_MEMBERS
END_STRUCT_DECODER()
#undef MEMBER