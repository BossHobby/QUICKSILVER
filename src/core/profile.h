#pragma once

#include <cbor.h>

#include "core/project.h"
#include "rx/rx.h"
#include "util/filter.h"
#include "util/vector.h"

#define OSD_NUMBER_ELEMENTS 32
#define BIND_RAW_STORAGE_SIZE 60
#define VTX_POWER_LABEL_LEN 5

#define PROFILE_VERSION MAKE_SEMVER(0, 3, 0)

typedef enum {
  VTX_BAND_A,
  VTX_BAND_B,
  VTX_BAND_E,
  VTX_BAND_F,
  VTX_BAND_R,
  VTX_BAND_L,

  VTX_BAND_MAX
} vtx_band_t;

typedef enum {
  VTX_CHANNEL_1,
  VTX_CHANNEL_2,
  VTX_CHANNEL_3,
  VTX_CHANNEL_4,
  VTX_CHANNEL_5,
  VTX_CHANNEL_6,
  VTX_CHANNEL_7,
  VTX_CHANNEL_8,

  VTX_CHANNEL_MAX,
} vtx_channel_t;

typedef enum {
  VTX_POWER_LEVEL_1,
  VTX_POWER_LEVEL_2,
  VTX_POWER_LEVEL_3,
  VTX_POWER_LEVEL_4,
  VTX_POWER_LEVEL_5,
  VTX_POWER_LEVEL_6,
  VTX_POWER_LEVEL_7,
  VTX_POWER_LEVEL_8,

  VTX_POWER_LEVEL_MAX,
} vtx_power_level_t;

typedef enum {
  VTX_PIT_MODE_OFF,
  VTX_PIT_MODE_ON,
  VTX_PIT_MODE_NO_SUPPORT,

  VTX_PIT_MODE_MAX,
} vtx_pit_mode_t;

typedef enum {
  VTX_PROTOCOL_INVALID,
  VTX_PROTOCOL_TRAMP,
  VTX_PROTOCOL_SMART_AUDIO,
  VTX_PROTOCOL_MSP_VTX,

  VTX_PROTOCOL_MAX,
} vtx_protocol_t;

// Rates
typedef enum {
  RATE_MODE_SILVERWARE,
  RATE_MODE_BETAFLIGHT,
  RATE_MODE_ACTUAL,
} __attribute__((__packed__)) rate_modes_t;

typedef enum {
  SILVERWARE_MAX_RATE,
  SILVERWARE_ACRO_EXPO,
  SILVERWARE_ANGLE_EXPO,
} __attribute__((__packed__)) silverware_rates_t;

typedef enum {
  BETAFLIGHT_RC_RATE,
  BETAFLIGHT_SUPER_RATE,
  BETAFLIGHT_EXPO,
} __attribute__((__packed__)) betaflight_rates_t;

typedef enum {
  ACTUAL_CENTER_SENSITIVITY,
  ACTUAL_MAX_RATE,
  ACTUAL_EXPO,
} __attribute__((__packed__)) actual_rates_t;

typedef enum {
  STICK_RATE_PROFILE_1,
  STICK_RATE_PROFILE_2,
  STICK_RATE_PROFILE_MAX
} __attribute__((__packed__)) rate_profiles_t;

typedef struct {
  rate_modes_t mode;
  vec3_t rate[3];
} rate_t;

#define RATE_MEMBERS            \
  START_STRUCT(rate_t)          \
  MEMBER(mode, uint8_t)         \
  ARRAY_MEMBER(rate, 3, vec3_t) \
  END_STRUCT()

typedef struct {
  rate_profiles_t profile;
  rate_t rates[STICK_RATE_PROFILE_MAX];
  float level_max_angle;
  float sticks_deadband;
  float throttle_mid;
  float throttle_expo;
} profile_rate_t;

#define PROFILE_RATE_MEMBERS                          \
  START_STRUCT(profile_rate_t)                        \
  MEMBER(profile, uint8_t)                            \
  ARRAY_MEMBER(rates, STICK_RATE_PROFILE_MAX, rate_t) \
  MEMBER(level_max_angle, float)                      \
  MEMBER(sticks_deadband, float)                      \
  MEMBER(throttle_mid, float)                         \
  MEMBER(throttle_expo, float)                        \
  END_STRUCT()

typedef struct {
  vec3_t kp;
  vec3_t ki;
  vec3_t kd;
} pid_rate_t;

#define PID_RATE_MEMBERS   \
  START_STRUCT(pid_rate_t) \
  MEMBER(kp, vec3_t)       \
  MEMBER(ki, vec3_t)       \
  MEMBER(kd, vec3_t)       \
  END_STRUCT()

typedef struct {
  float kp;
  float kd;
} angle_pid_rate_t;

#define ANGLE_PID_RATE_MEMBERS   \
  START_STRUCT(angle_pid_rate_t) \
  MEMBER(kp, float)              \
  MEMBER(kd, float)              \
  END_STRUCT()

typedef struct {
  uint32_t index;
  const char *name;
  pid_rate_t rate;
} pid_rate_preset_t;

#define PID_RATE_PRESET_MEMBERS   \
  START_STRUCT(pid_rate_preset_t) \
  MEMBER(index, uint32_t)         \
  STR_MEMBER(name)                \
  MEMBER(rate, pid_rate_t)        \
  END_STRUCT()

typedef enum {
  PID_PROFILE_1,
  PID_PROFILE_2,
  PID_PROFILE_MAX
} __attribute__((__packed__)) pid_profile_t;

typedef struct {
  vec3_t accelerator;
  vec3_t transition;
} stick_rate_t;

#define STICK_RATE_MEMBERS    \
  START_STRUCT(stick_rate_t)  \
  MEMBER(accelerator, vec3_t) \
  MEMBER(transition, vec3_t)  \
  END_STRUCT()

typedef enum {
  STICK_PROFILE_OFF,
  STICK_PROFILE_ON,
  STICK_PROFILE_MAX
} __attribute__((__packed__)) stick_profile_t;

typedef enum {
  THROTTLE_D_ATTENTUATION_NONE,
  THROTTLE_D_ATTENUATION_ACTIVE,
  THROTTLE_D_ATTENUATION_MAX,
} __attribute__((__packed__)) tda_active_t;

typedef struct {
  tda_active_t tda_active;
  float tda_breakpoint;
  float tda_percent;
} throttle_dterm_attenuation_t;

#define DTERM_ATTENUATION_MEMBERS            \
  START_STRUCT(throttle_dterm_attenuation_t) \
  MEMBER(tda_active, uint8_t)                \
  MEMBER(tda_breakpoint, float)              \
  MEMBER(tda_percent, float)                 \
  END_STRUCT()

typedef struct {
  pid_profile_t pid_profile;
  pid_rate_t pid_rates[PID_PROFILE_MAX];
  stick_profile_t stick_profile;
  stick_rate_t stick_rates[STICK_PROFILE_MAX];
  angle_pid_rate_t big_angle;
  angle_pid_rate_t small_angle;
  throttle_dterm_attenuation_t throttle_dterm_attenuation;
} profile_pid_t;

#define PID_MEMBERS                                                \
  START_STRUCT(profile_pid_t)                                      \
  MEMBER(pid_profile, uint8_t)                                     \
  ARRAY_MEMBER(pid_rates, PID_PROFILE_MAX, pid_rate_t)             \
  MEMBER(stick_profile, uint8_t)                                   \
  ARRAY_MEMBER(stick_rates, STICK_PROFILE_MAX, stick_rate_t)       \
  MEMBER(big_angle, angle_pid_rate_t)                              \
  MEMBER(small_angle, angle_pid_rate_t)                            \
  MEMBER(throttle_dterm_attenuation, throttle_dterm_attenuation_t) \
  END_STRUCT()

typedef enum {
  GYRO_ROTATE_NONE = 0x0,
  GYRO_ROTATE_45_CCW = 0x1,
  GYRO_ROTATE_45_CW = 0x2,
  GYRO_ROTATE_90_CW = 0x4,
  GYRO_ROTATE_90_CCW = 0x8,
  GYRO_ROTATE_180 = 0x10,
  GYRO_FLIP_180 = 0x20,
} __attribute__((__packed__)) gyro_rotation_t;

typedef enum {
  DSHOT_TIME_150 = 150,
  DSHOT_TIME_300 = 300,
  DSHOT_TIME_600 = 600,
  DSHOT_TIME_MAX = 600,
} __attribute__((__packed__)) dshot_time_t;

typedef struct {
  float digital_idle;
  float motor_limit;
  dshot_time_t dshot_time;
  bool dshot_telemetry;
  uint8_t gyro_orientation;
  float torque_boost;
  float throttle_boost;
  float turtle_throttle_percent;
} profile_motor_t;

#define MOTOR_MEMBERS                              \
  START_STRUCT(profile_motor_t)                    \
  MEMBER(digital_idle, float)                      \
  MEMBER(motor_limit, float)                       \
  MEMBER(dshot_time, uint16_t)                     \
  MEMBER(dshot_telemetry, bool)                    \
  MEMBER(gyro_orientation, uint8_t)                \
  MEMBER(torque_boost, float)                      \
  MEMBER(throttle_boost, float)                    \
  MEMBER(turtle_throttle_percent, float)           \
  END_STRUCT()

typedef enum {
  PID_VOLTAGE_COMPENSATION_NONE,
  PID_VOLTAGE_COMPENSATION_ACTIVE,
} __attribute__((__packed__)) pid_voltage_compensation_t;

typedef struct {
  uint8_t lipo_cell_count;
  pid_voltage_compensation_t pid_voltage_compensation;
  float vbattlow;
  float actual_battery_voltage;
  float reported_telemetry_voltage;
  uint8_t use_filtered_voltage_for_warnings;
  float vbat_scale;
  float ibat_scale;
} profile_voltage_t;

#define VOLTAGE_MEMBERS                              \
  START_STRUCT(profile_voltage_t)                    \
  MEMBER(lipo_cell_count, uint8_t)                   \
  MEMBER(pid_voltage_compensation, uint8_t)          \
  MEMBER(vbattlow, float)                            \
  MEMBER(actual_battery_voltage, float)              \
  MEMBER(reported_telemetry_voltage, float)          \
  MEMBER(use_filtered_voltage_for_warnings, uint8_t) \
  MEMBER(vbat_scale, float)                          \
  MEMBER(ibat_scale, float)                          \
  END_STRUCT()

typedef struct {
  int8_t offset;
  uint8_t idx;
  uint8_t tx_id[2];
  uint8_t hop_data[50];
  uint8_t rx_num;
  uint8_t _pad;
} profile_receiver_bind_frsky_t;

typedef struct {
  uint8_t protocol;
} profile_receiver_bind_unified_t;

typedef struct {
  uint8_t is_set;
  uint8_t uid[6];
  uint8_t magic;
  uint8_t switch_mode;
  uint8_t model_id;
} profile_receiver_bind_elrs_t;

typedef struct {
  uint8_t rx_channel_map[16];
  uint32_t tx_id;
} profile_receiver_bind_flysky_t;

typedef struct {
  uint8_t bind_saved;
  union {
    profile_receiver_bind_frsky_t frsky;
    profile_receiver_bind_unified_t unified;
    profile_receiver_bind_elrs_t elrs;
    profile_receiver_bind_flysky_t flysky;
    uint8_t raw[BIND_RAW_STORAGE_SIZE];
  };
} profile_receiver_bind_t;

#define PROFILE_RECEIVER_BIND_MEMBERS           \
  START_STRUCT(profile_receiver_bind_t)         \
  MEMBER(bind_saved, uint8_t)                   \
  BSTR_MEMBER(raw, BIND_RAW_STORAGE_SIZE)       \
  END_STRUCT()

typedef struct {
  rx_protocol_t protocol;
  profile_receiver_bind_t bind;
  aux_function_map_t aux[AUX_FUNCTION_MAX];
  rx_role_map_t role_map[RX_ROLE_MAX];
  rx_lqi_source_t lqi_source;
} profile_receiver_t;

#define RX_ROLE_MAP_MEMBERS   \
  START_STRUCT(rx_role_map_t) \
  MEMBER(channel, uint8_t)    \
  MEMBER(min, float)          \
  MEMBER(center, float)       \
  MEMBER(max, float)          \
  END_STRUCT()

#define AUX_FUNCTION_MAP_MEMBERS   \
  START_STRUCT(aux_function_map_t) \
  MEMBER(channel, uint8_t)         \
  MEMBER(range_min, uint16_t)      \
  MEMBER(range_max, uint16_t)      \
  END_STRUCT()

#define RECEIVER_MEMBERS                                  \
  START_STRUCT(profile_receiver_t)                        \
  MEMBER(protocol, uint8_t)                               \
  MEMBER(bind, profile_receiver_bind_t)                   \
  ARRAY_MEMBER(aux, AUX_FUNCTION_MAX, aux_function_map_t) \
  MEMBER(lqi_source, uint8_t)                             \
  ARRAY_MEMBER(role_map, RX_ROLE_MAX, rx_role_map_t)      \
  END_STRUCT()

typedef enum {
  OUTPUT_PROTOCOL_NONE,
  OUTPUT_PROTOCOL_DSHOT,
  OUTPUT_PROTOCOL_BRUSHED,
  OUTPUT_PROTOCOL_PWM,
} __attribute__((__packed__)) output_protocol_t;

typedef enum {
  OUTPUT_SOURCE_NONE,
  OUTPUT_SOURCE_THROTTLE,
  OUTPUT_SOURCE_ROLL,
  OUTPUT_SOURCE_PITCH,
  OUTPUT_SOURCE_YAW,
  OUTPUT_SOURCE_RX_CHANNEL,
  OUTPUT_SOURCE_MAX,
} __attribute__((__packed__)) output_source_t;

typedef struct {
  uint8_t target_output;
  output_protocol_t protocol;
  uint8_t invert;
  int16_t trim;
  int16_t min;
  int16_t max;
  uint16_t rate_hz;
} profile_output_t;

#define PROFILE_OUTPUT_MEMBERS   \
  START_STRUCT(profile_output_t) \
  MEMBER(target_output, uint8_t) \
  MEMBER(protocol, uint8_t)      \
  MEMBER(invert, uint8_t)        \
  MEMBER(trim, int16_t)          \
  MEMBER(min, int16_t)           \
  MEMBER(max, int16_t)           \
  MEMBER(rate_hz, uint16_t)      \
  END_STRUCT()

#define MIXER_RULE_MAX (MOTOR_PIN_MAX * 4)

typedef struct {
  uint8_t output_index;
  output_source_t source;
  uint8_t source_index;
  int8_t weight;
} profile_mixer_rule_t;

#define PROFILE_MIXER_RULE_MEMBERS \
  START_STRUCT(profile_mixer_rule_t) \
  MEMBER(output_index, uint8_t)      \
  MEMBER(source, uint8_t)            \
  MEMBER(source_index, uint8_t)      \
  MEMBER(weight, int8_t)             \
  END_STRUCT()

typedef struct {
  serial_ports_t rx;
  serial_ports_t smart_audio;
  serial_ports_t hdzero;
  serial_ports_t gps;
} profile_serial_t;

#define SERIAL_MEMBERS           \
  START_STRUCT(profile_serial_t) \
  MEMBER(rx, uint8_t)            \
  MEMBER(smart_audio, uint8_t)   \
  MEMBER(hdzero, uint8_t)        \
  MEMBER(gps, uint8_t)           \
  END_STRUCT()

typedef enum {
  OSD_PROFILE_1,
  OSD_PROFILE_2,
  OSD_PROFILE_MAX,
} __attribute__((__packed__)) osd_profiles_t;

typedef struct {
  uint8_t callsign[36];
  uint32_t elements[OSD_NUMBER_ELEMENTS];
} profile_osd_profile_t;

#define OSD_PROFILE_MEMBERS                             \
  START_STRUCT(profile_osd_profile_t)                   \
  TSTR_MEMBER(callsign, 36)                             \
  ARRAY_MEMBER(elements, OSD_NUMBER_ELEMENTS, uint32_t) \
  END_STRUCT()

typedef struct {
  uint8_t guac_mode;
  profile_osd_profile_t profiles[OSD_PROFILE_MAX];
} profile_osd_t;

#define OSD_MEMBERS                                              \
  START_STRUCT(profile_osd_t)                                    \
  MEMBER(guac_mode, uint8_t)                                     \
  ARRAY_MEMBER(profiles, OSD_PROFILE_MAX, profile_osd_profile_t) \
  END_STRUCT()

typedef struct {
  filter_type_t type;
  float cutoff_freq;
} profile_filter_parameter_t;

#define FILTER_PARAMETER_MEMBERS           \
  START_STRUCT(profile_filter_parameter_t) \
  MEMBER(type, uint8_t)                    \
  MEMBER(cutoff_freq, float)               \
  END_STRUCT()

typedef struct {
  profile_filter_parameter_t gyro[FILTER_MAX_SLOTS];
  profile_filter_parameter_t dterm[FILTER_MAX_SLOTS];
  filter_type_t dterm_dynamic_type;
  float dterm_dynamic_min;
  float dterm_dynamic_max;
  uint8_t gyro_dynamic_notch_enable;
} profile_filter_t;

#ifdef VEHICLE_ROVER
#define FILTER_MEMBERS                                              \
  START_STRUCT(profile_filter_t)                                    \
  ARRAY_MEMBER(gyro, FILTER_MAX_SLOTS, profile_filter_parameter_t)  \
  ARRAY_MEMBER(dterm, FILTER_MAX_SLOTS, profile_filter_parameter_t) \
  MEMBER(gyro_dynamic_notch_enable, uint8_t)                        \
  END_STRUCT()
#else
#define FILTER_MEMBERS                                              \
  START_STRUCT(profile_filter_t)                                    \
  ARRAY_MEMBER(gyro, FILTER_MAX_SLOTS, profile_filter_parameter_t)  \
  ARRAY_MEMBER(dterm, FILTER_MAX_SLOTS, profile_filter_parameter_t) \
  MEMBER(dterm_dynamic_type, uint8_t)                               \
  MEMBER(dterm_dynamic_min, float)                                  \
  MEMBER(dterm_dynamic_max, float)                                  \
  MEMBER(gyro_dynamic_notch_enable, uint8_t)                        \
  END_STRUCT()
#endif

typedef struct {
  uint8_t name[36];
  uint32_t datetime;
} profile_metadata_t;

typedef struct {
  uint32_t field_flags;
  uint32_t debug_flags;
  uint32_t sample_rate_hz;
} profile_blackbox_t;

typedef enum {
  ROVER_STEER_MODE_MANUAL,
  ROVER_STEER_MODE_RATE_ASSIST,
  ROVER_STEER_MODE_RATE_THROTTLE,
} __attribute__((__packed__)) rover_steer_mode_t;

typedef struct {
  float kp;
  float ki;
  float kd;
} rover_pid_rate_t;

#define ROVER_PID_RATE_MEMBERS    \
  START_STRUCT(rover_pid_rate_t)  \
  MEMBER(kp, float)               \
  MEMBER(ki, float)               \
  MEMBER(kd, float)               \
  END_STRUCT()

typedef struct {
  rover_pid_rate_t pid;
  float center_deadband;
  float yaw_rate;
  float throttle_scale_breakpoint;
  float throttle_scale_factor;
  uint8_t reversible;
} profile_rover_t;

#define ROVER_MEMBERS                      \
  START_STRUCT(profile_rover_t)            \
  MEMBER(pid, rover_pid_rate_t)            \
  MEMBER(center_deadband, float)           \
  MEMBER(yaw_rate, float)                  \
  MEMBER(throttle_scale_breakpoint, float) \
  MEMBER(throttle_scale_factor, float)     \
  MEMBER(reversible, uint8_t)              \
  END_STRUCT()

#define BLACKBOX_MEMBERS           \
  START_STRUCT(profile_blackbox_t) \
  MEMBER(field_flags, uint32_t)    \
  MEMBER(debug_flags, uint32_t)    \
  MEMBER(sample_rate_hz, uint32_t) \
  END_STRUCT()

typedef struct vtx_power_table_s {
  uint8_t levels;
  char labels[VTX_POWER_LEVEL_MAX][VTX_POWER_LABEL_LEN];
  uint16_t values[VTX_POWER_LEVEL_MAX];
} vtx_power_table_t;

#define VTX_POWER_TABLE_MEMBERS                                       \
  MEMBER(levels, uint8_t)                                             \
  TSTR_ARRAY_MEMBER(labels, VTX_POWER_LEVEL_MAX, VTX_POWER_LABEL_LEN) \
  ARRAY_MEMBER(values, VTX_POWER_LEVEL_MAX, uint16_t)

typedef struct profile_vtx_s {
  vtx_protocol_t protocol;
  vtx_band_t band;
  vtx_channel_t channel;
  vtx_pit_mode_t pit_mode;
  vtx_power_level_t power_level;
  vtx_power_table_t power_table;
} profile_vtx_t;

#define PROFILE_VTX_MEMBERS             \
  MEMBER(protocol, uint8_t)             \
  MEMBER(band, uint8_t)                 \
  MEMBER(channel, uint8_t)              \
  MEMBER(pit_mode, uint8_t)             \
  MEMBER(power_level, uint8_t)          \
  MEMBER(power_table, vtx_power_table_t)

typedef struct {
  uint32_t field_flags;
  uint32_t sample_rate_hz;
  const char *name;
  const char *name_osd;
} blackbox_preset_t;

#define BLACKBOX_PRESET_MEMBERS    \
  START_STRUCT(blackbox_preset_t)  \
  MEMBER(field_flags, uint32_t)    \
  MEMBER(sample_rate_hz, uint32_t) \
  STR_MEMBER(name)                 \
  STR_MEMBER(name_osd)             \
  END_STRUCT()

// Full Profile
typedef struct {
  profile_metadata_t meta;
  profile_output_t outputs[MOTOR_PIN_MAX];
  profile_mixer_rule_t mixer[MIXER_RULE_MAX];
  profile_motor_t motor;
  profile_serial_t serial;
  profile_filter_t filter;
  profile_osd_t osd;
#ifndef VEHICLE_ROVER
  profile_rate_t rate;
#endif
  profile_receiver_t receiver;
  profile_pid_t pid;
  profile_voltage_t voltage;
  profile_blackbox_t blackbox;
  profile_vtx_t vtx;
  profile_rover_t rover;
} profile_t;

#ifdef VEHICLE_ROVER
#define PROFILE_MEMBERS                                                                     \
  START_STRUCT(profile_t)                                                                   \
  MEMBER(meta, profile_metadata_t)                                                          \
  COUNT_ARRAY_MEMBER(outputs, MOTOR_PIN_MAX, profile_output_t, profile_output_count)        \
  COUNT_ARRAY_MEMBER(mixer, MIXER_RULE_MAX, profile_mixer_rule_t, profile_mixer_rule_count) \
  MEMBER(motor, profile_motor_t)                                                            \
  MEMBER(serial, profile_serial_t)                                                          \
  MEMBER(filter, profile_filter_t)                                                          \
  MEMBER(osd, profile_osd_t)                                                                \
  MEMBER(receiver, profile_receiver_t)                                                      \
  MEMBER(pid, profile_pid_t)                                                                \
  MEMBER(voltage, profile_voltage_t)                                                        \
  MEMBER(blackbox, profile_blackbox_t)                                                      \
  MEMBER(vtx, profile_vtx_t)                                                                \
  MEMBER(rover, profile_rover_t)                                                            \
  END_STRUCT()
#else
#define PROFILE_MEMBERS                                                                     \
  START_STRUCT(profile_t)                                                                   \
  MEMBER(meta, profile_metadata_t)                                                          \
  COUNT_ARRAY_MEMBER(outputs, MOTOR_PIN_MAX, profile_output_t, profile_output_count)        \
  COUNT_ARRAY_MEMBER(mixer, MIXER_RULE_MAX, profile_mixer_rule_t, profile_mixer_rule_count) \
  MEMBER(motor, profile_motor_t)                                                            \
  MEMBER(serial, profile_serial_t)                                                          \
  MEMBER(filter, profile_filter_t)                                                          \
  MEMBER(osd, profile_osd_t)                                                                \
  MEMBER(rate, profile_rate_t)                                                              \
  MEMBER(receiver, profile_receiver_t)                                                      \
  MEMBER(pid, profile_pid_t)                                                                \
  MEMBER(voltage, profile_voltage_t)                                                        \
  MEMBER(blackbox, profile_blackbox_t)                                                      \
  MEMBER(vtx, profile_vtx_t)                                                                \
  END_STRUCT()
#endif

extern profile_t profile;
extern const profile_t default_profile;

extern const pid_rate_preset_t pid_rate_presets[];
extern const uint32_t pid_rate_presets_count;

extern const blackbox_preset_t blackbox_presets[];
extern const uint32_t blackbox_presets_count;

void blackbox_preset_apply(const blackbox_preset_t *preset, profile_blackbox_t *profile);
uint8_t blackbox_preset_equals(const blackbox_preset_t *preset, profile_blackbox_t *profile);

void profile_set_defaults();
pid_rate_t *profile_current_pid_rates();
#ifndef VEHICLE_ROVER
rate_t *profile_current_rates();
#endif
bool profile_output_slot_uses_protocol(uint8_t target_output, output_protocol_t protocol);
bool profile_outputs_use_protocol(output_protocol_t protocol);
bool profile_output_slot_uses_servo(uint8_t target_output);
uint32_t profile_output_count(const profile_output_t *outputs, uint32_t size);
uint32_t profile_mixer_rule_count(const profile_mixer_rule_t *mixer, uint32_t size);

cbor_result_t cbor_encode_profile_t(cbor_value_t *enc, const profile_t *p);
cbor_result_t cbor_decode_profile_t(cbor_value_t *dec, profile_t *p);

cbor_result_t cbor_encode_profile_receiver_bind_t(cbor_value_t *enc, const profile_receiver_bind_t *s);
cbor_result_t cbor_decode_profile_receiver_bind_t(cbor_value_t *enc, profile_receiver_bind_t *s);

cbor_result_t cbor_encode_profile_vtx_t(cbor_value_t *enc, const profile_vtx_t *vtx);
cbor_result_t cbor_decode_profile_vtx_t(cbor_value_t *dec, profile_vtx_t *vtx);

cbor_result_t cbor_encode_blackbox_preset_t(cbor_value_t *enc, const blackbox_preset_t *p);
cbor_result_t cbor_decode_blackbox_preset_t(cbor_value_t *dec, blackbox_preset_t *p);

cbor_result_t cbor_encode_pid_rate_preset_t(cbor_value_t *enc, const pid_rate_preset_t *p);
