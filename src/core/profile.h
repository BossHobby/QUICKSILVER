#pragma once

#include <cbor.h>

#include "core/project.h"
#include "flight/filter.h"
#include "rx/rx.h"
#include "util/vector.h"

#define OSD_NUMBER_ELEMENTS 32

#define PROFILE_VERSION MAKE_SEMVER(0, 2, 1)

// Rates
typedef enum {
  RATE_MODE_SILVERWARE,
  RATE_MODE_BETAFLIGHT,
  RATE_MODE_ACTUAL,
} rate_modes_t;

typedef enum {
  SILVERWARE_MAX_RATE,
  SILVERWARE_ACRO_EXPO,
  SILVERWARE_ANGLE_EXPO,
} silverware_rates_t;

typedef enum {
  BETAFLIGHT_RC_RATE,
  BETAFLIGHT_SUPER_RATE,
  BETAFLIGHT_EXPO,
} betaflight_rates_t;

typedef enum {
  ACTUAL_CENTER_SENSITIVITY,
  ACTUAL_MAX_RATE,
  ACTUAL_EXPO,
} actual_rates_t;

typedef enum {
  STICK_RATE_PROFILE_1,
  STICK_RATE_PROFILE_2,
  STICK_RATE_PROFILE_MAX
} rate_profiles_t;

typedef struct {
  rate_modes_t mode;
  vec3_t rate[3];
} rate_t;

#define RATE_MEMBERS            \
  START_STRUCT(rate_t)          \
  MEMBER(mode, uint8)           \
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
  MEMBER(profile, uint8)                              \
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
  MEMBER(index, uint32)           \
  STR_MEMBER(name)                \
  MEMBER(rate, pid_rate_t)        \
  END_STRUCT()

typedef enum {
  PID_PROFILE_1,
  PID_PROFILE_2,
  PID_PROFILE_MAX
} pid_profile_t;

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
} stick_profile_t;

typedef enum {
  THROTTLE_D_ATTENTUATION_NONE,
  THROTTLE_D_ATTENUATION_ACTIVE,
  THROTTLE_D_ATTENUATION_MAX,
} tda_active_t;

typedef struct {
  tda_active_t tda_active;
  float tda_breakpoint;
  float tda_percent;
} throttle_dterm_attenuation_t;

#define DTERM_ATTENUATION_MEMBERS            \
  START_STRUCT(throttle_dterm_attenuation_t) \
  MEMBER(tda_active, uint8)                  \
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
  MEMBER(pid_profile, uint8)                                       \
  ARRAY_MEMBER(pid_rates, PID_PROFILE_MAX, pid_rate_t)             \
  MEMBER(stick_profile, uint8)                                     \
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
} gyro_rotation_t;

typedef enum {
  DSHOT_TIME_150 = 150,
  DSHOT_TIME_300 = 300,
  DSHOT_TIME_600 = 600,
} dshot_time_t;

typedef struct {
  float digital_idle;
  float motor_limit;
  dshot_time_t dshot_time;
  uint8_t invert_yaw;
  uint8_t gyro_orientation;
  float torque_boost;
  float throttle_boost;
  motor_pin_ident_t motor_pins[4];
  float turtle_throttle_percent;
} profile_motor_t;

#define MOTOR_MEMBERS                    \
  START_STRUCT(profile_motor_t)          \
  MEMBER(digital_idle, float)            \
  MEMBER(motor_limit, float)             \
  MEMBER(dshot_time, uint16)             \
  MEMBER(invert_yaw, uint8)              \
  MEMBER(gyro_orientation, uint8)        \
  MEMBER(torque_boost, float)            \
  MEMBER(throttle_boost, float)          \
  ARRAY_MEMBER(motor_pins, 4, uint8)     \
  MEMBER(turtle_throttle_percent, float) \
  END_STRUCT()

typedef enum {
  PID_VOLTAGE_COMPENSATION_NONE,
  PID_VOLTAGE_COMPENSATION_ACTIVE,
} pid_voltage_compensation_t;

typedef struct {
  uint8_t lipo_cell_count;
  pid_voltage_compensation_t pid_voltage_compensation;
  float vbattlow;
  float actual_battery_voltage;
  float reported_telemetry_voltage;
  float ibat_scale;
} profile_voltage_t;

#define VOLTAGE_MEMBERS                     \
  START_STRUCT(profile_voltage_t)           \
  MEMBER(lipo_cell_count, uint8)            \
  MEMBER(pid_voltage_compensation, uint8)   \
  MEMBER(vbattlow, float)                   \
  MEMBER(actual_battery_voltage, float)     \
  MEMBER(reported_telemetry_voltage, float) \
  MEMBER(ibat_scale, float)                 \
  END_STRUCT()

typedef struct {
  float min;
  float max;
} profile_stick_calibration_limits_t;

#define CALIBRATION_LIMIT_MEMBERS                  \
  START_STRUCT(profile_stick_calibration_limits_t) \
  MEMBER(min, float)                               \
  MEMBER(max, float)                               \
  END_STRUCT()

typedef struct {
  rx_protocol_t protocol;
  aux_channel_t aux[AUX_FUNCTION_MAX];
  rx_channel_mapping_t channel_mapping;
  rx_lqi_source_t lqi_source;
  profile_stick_calibration_limits_t stick_calibration_limits[4];
} profile_receiver_t;

#define RECEIVER_MEMBERS                                                        \
  START_STRUCT(profile_receiver_t)                                              \
  MEMBER(protocol, uint8)                                                       \
  ARRAY_MEMBER(aux, AUX_FUNCTION_MAX, uint8)                                    \
  MEMBER(lqi_source, uint8)                                                     \
  MEMBER(channel_mapping, uint8)                                                \
  ARRAY_MEMBER(stick_calibration_limits, 4, profile_stick_calibration_limits_t) \
  END_STRUCT()

typedef struct {
  usart_ports_t rx;
  usart_ports_t smart_audio;
  usart_ports_t hdzero;
} profile_serial_t;

#define SERIAL_MEMBERS           \
  START_STRUCT(profile_serial_t) \
  MEMBER(rx, uint8)              \
  MEMBER(smart_audio, uint8)     \
  MEMBER(hdzero, uint8)          \
  END_STRUCT()

typedef struct {
  uint8_t guac_mode;
  uint8_t callsign[36];
  uint32_t elements[OSD_NUMBER_ELEMENTS];
  uint32_t elements_hd[OSD_NUMBER_ELEMENTS];
} profile_osd_t;

#define OSD_MEMBERS                                      \
  START_STRUCT(profile_osd_t)                            \
  MEMBER(guac_mode, uint8)                               \
  TSTR_MEMBER(callsign, 36)                              \
  ARRAY_MEMBER(elements, OSD_NUMBER_ELEMENTS, uint32)    \
  ARRAY_MEMBER(elements_hd, OSD_NUMBER_ELEMENTS, uint32) \
  END_STRUCT()

typedef struct {
  filter_type_t type;
  float cutoff_freq;
} profile_filter_parameter_t;

#define FILTER_PARAMETER_MEMBERS           \
  START_STRUCT(profile_filter_parameter_t) \
  MEMBER(type, uint8)                      \
  MEMBER(cutoff_freq, float)               \
  END_STRUCT()

typedef struct {
  profile_filter_parameter_t gyro[FILTER_MAX_SLOTS];
  profile_filter_parameter_t dterm[FILTER_MAX_SLOTS];
  uint8_t dterm_dynamic_enable;
  float dterm_dynamic_min;
  float dterm_dynamic_max;
} profile_filter_t;

#define FILTER_MEMBERS                                              \
  START_STRUCT(profile_filter_t)                                    \
  ARRAY_MEMBER(gyro, FILTER_MAX_SLOTS, profile_filter_parameter_t)  \
  ARRAY_MEMBER(dterm, FILTER_MAX_SLOTS, profile_filter_parameter_t) \
  MEMBER(dterm_dynamic_enable, uint8)                               \
  MEMBER(dterm_dynamic_min, float)                                  \
  MEMBER(dterm_dynamic_max, float)                                  \
  END_STRUCT()

typedef struct {
  uint8_t name[36];
  uint32_t datetime;
} profile_metadata_t;

typedef struct {
  uint32_t field_flags;
  uint32_t sample_rate_hz;
} profile_blackbox_t;

#define BLACKBOX_MEMBERS           \
  START_STRUCT(profile_blackbox_t) \
  MEMBER(field_flags, uint32)      \
  MEMBER(sample_rate_hz, uint32)   \
  END_STRUCT()

typedef struct {
  uint32_t field_flags;
  uint32_t sample_rate_hz;
  const char *name;
  const char *name_osd;
} blackbox_preset_t;

#define BLACKBOX_PRESET_MEMBERS   \
  START_STRUCT(blackbox_preset_t) \
  MEMBER(field_flags, uint32)     \
  MEMBER(sample_rate_hz, uint32)  \
  STR_MEMBER(name)                \
  STR_MEMBER(name_osd)            \
  END_STRUCT()

// Full Profile
typedef struct {
  profile_metadata_t meta;
  profile_motor_t motor;
  profile_serial_t serial;
  profile_filter_t filter;
  profile_osd_t osd;
  profile_rate_t rate;
  profile_receiver_t receiver;
  profile_pid_t pid;
  profile_voltage_t voltage;
  profile_blackbox_t blackbox;
} profile_t;

#define PROFILE_MEMBERS                \
  START_STRUCT(profile_t)              \
  MEMBER(meta, profile_metadata_t)     \
  MEMBER(motor, profile_motor_t)       \
  MEMBER(serial, profile_serial_t)     \
  MEMBER(filter, profile_filter_t)     \
  MEMBER(osd, profile_osd_t)           \
  MEMBER(rate, profile_rate_t)         \
  MEMBER(receiver, profile_receiver_t) \
  MEMBER(pid, profile_pid_t)           \
  MEMBER(voltage, profile_voltage_t)   \
  MEMBER(blackbox, profile_blackbox_t) \
  END_STRUCT()

typedef enum {
  FEATURE_BRUSHLESS = (1 << 1),
  FEATURE_OSD = (1 << 2),
  FEATURE_BLACKBOX = (1 << 3),
  FEATURE_DEBUG = (1 << 4),
} target_feature_t;

typedef struct {
  const char *target_name;
  const char *git_version;

  uint32_t features;
  rx_protocol_t rx_protocols[RX_PROTOCOL_MAX];
  uint32_t quic_protocol_version;

  const char *motor_pins[MOTOR_PIN_MAX];
  const char *usart_ports[SOFT_SERIAL_PORTS_MAX];

  uint8_t gyro_id;
} target_info_t;

#define TARGET_INFO_MEMBERS                            \
  START_STRUCT(target_info_t)                          \
  STR_MEMBER(target_name)                              \
  STR_MEMBER(git_version)                              \
  MEMBER(features, uint32)                             \
  ARRAY_MEMBER(rx_protocols, RX_PROTOCOL_MAX, uint8)   \
  MEMBER(quic_protocol_version, uint32)                \
  STR_ARRAY_MEMBER(motor_pins, MOTOR_PIN_MAX)          \
  STR_ARRAY_MEMBER(usart_ports, SOFT_SERIAL_PORTS_MAX) \
  MEMBER(gyro_id, uint8)                               \
  END_STRUCT()

extern profile_t profile;
extern const profile_t default_profile;

extern target_info_t target_info;

extern const pid_rate_preset_t pid_rate_presets[];
extern const uint32_t pid_rate_presets_count;

extern const blackbox_preset_t blackbox_presets[];
extern const uint32_t blackbox_presets_count;

void blackbox_preset_apply(const blackbox_preset_t *preset, profile_blackbox_t *profile);
uint8_t blackbox_preset_equals(const blackbox_preset_t *preset, profile_blackbox_t *profile);

void profile_set_defaults();
pid_rate_t *profile_current_pid_rates();
rate_t *profile_current_rates();

cbor_result_t cbor_encode_profile_t(cbor_value_t *enc, const profile_t *p);
cbor_result_t cbor_decode_profile_t(cbor_value_t *dec, profile_t *p);

cbor_result_t cbor_encode_blackbox_preset_t(cbor_value_t *enc, const blackbox_preset_t *p);
cbor_result_t cbor_decode_blackbox_preset_t(cbor_value_t *dec, blackbox_preset_t *p);

cbor_result_t cbor_encode_pid_rate_preset_t(cbor_value_t *enc, const pid_rate_preset_t *p);
cbor_result_t cbor_encode_target_info_t(cbor_value_t *enc, const target_info_t *i);
