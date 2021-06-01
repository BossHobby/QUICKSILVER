#pragma once

#include <cbor.h>

#include "filter.h"
#include "project.h"
#include "util/vector.h"

// Rates
typedef enum {
  RATE_MODE_SILVERWARE,
  RATE_MODE_BETAFLIGHT,
} rate_modes_t;

typedef struct {
  vec3_t max_rate;
  vec3_t acro_expo;
  vec3_t angle_expo;
} rate_mode_silverware_t;

#define SILVERWARE_RATE_MEMBERS \
  MEMBER(max_rate, vec3_t)      \
  MEMBER(acro_expo, vec3_t)     \
  MEMBER(angle_expo, vec3_t)

typedef struct {
  vec3_t rc_rate;
  vec3_t super_rate;
  vec3_t expo;
} rate_mode_betaflight_t;

#define BETAFLIGHT_RATE_MEMBERS \
  MEMBER(rc_rate, vec3_t)       \
  MEMBER(super_rate, vec3_t)    \
  MEMBER(expo, vec3_t)

typedef struct {
  rate_modes_t mode;
  rate_mode_silverware_t silverware;
  rate_mode_betaflight_t betaflight;
  float level_max_angle;
  float low_rate_mulitplier;
  float sticks_deadband;
} profile_rate_t;

#define RATE_MEMBERS                         \
  MEMBER(mode, uint8)                        \
  MEMBER(silverware, rate_mode_silverware_t) \
  MEMBER(betaflight, rate_mode_betaflight_t) \
  MEMBER(level_max_angle, float)             \
  MEMBER(low_rate_mulitplier, float)         \
  MEMBER(sticks_deadband, float)

typedef struct {
  vec3_t kp;
  vec3_t ki;
  vec3_t kd;
} pid_rate_t;

#define PID_RATE_MEMBERS \
  MEMBER(kp, vec3_t)     \
  MEMBER(ki, vec3_t)     \
  MEMBER(kd, vec3_t)

typedef struct {
  float kp;
  float kd;
} angle_pid_rate_t;

#define ANGLE_PID_RATE_MEMBERS \
  MEMBER(kp, float)            \
  MEMBER(kd, float)

typedef struct {
  uint32_t index;
  const char *name;
  pid_rate_t rate;
} pid_rate_preset_t;

#define PID_RATE_PRESET_MEMBERS \
  MEMBER(index, uint32)         \
  STR_MEMBER(name)              \
  MEMBER(rate, pid_rate_t)

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
  MEMBER(accelerator, vec3_t) \
  MEMBER(transition, vec3_t)

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

#define DTERM_ATTENUATION_MEMBERS \
  MEMBER(tda_active, uint8)       \
  MEMBER(tda_breakpoint, float)   \
  MEMBER(tda_percent, float)

typedef struct {
  pid_profile_t pid_profile;
  pid_rate_t pid_rates[PID_PROFILE_MAX];
  stick_profile_t stick_profile;
  stick_rate_t stick_rates[STICK_PROFILE_MAX];
  angle_pid_rate_t big_angle;
  angle_pid_rate_t small_angle;
  throttle_dterm_attenuation_t throttle_dterm_attenuation;
} profile_pid_t;

#define PID_MEMBERS                                          \
  MEMBER(pid_profile, uint8)                                 \
  ARRAY_MEMBER(pid_rates, PID_PROFILE_MAX, pid_rate_t)       \
  MEMBER(stick_profile, uint8)                               \
  ARRAY_MEMBER(stick_rates, STICK_PROFILE_MAX, stick_rate_t) \
  MEMBER(big_angle, angle_pid_rate_t)                        \
  MEMBER(small_angle, angle_pid_rate_t)                      \
  MEMBER(throttle_dterm_attenuation, throttle_dterm_attenuation_t)

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
  dshot_time_t dshot_time;
  uint8_t invert_yaw;
  uint8_t gyro_orientation;
  float torque_boost;
  float throttle_boost;
  motor_pin_ident_t motor_pins[4];
  float turtle_throttle_percent;
} profile_motor_t;

#define MOTOR_MEMBERS                \
  MEMBER(digital_idle, float)        \
  MEMBER(dshot_time, uint16)         \
  MEMBER(invert_yaw, uint8)          \
  MEMBER(gyro_orientation, uint8)    \
  MEMBER(torque_boost, float)        \
  MEMBER(throttle_boost, float)      \
  ARRAY_MEMBER(motor_pins, 4, uint8) \
  MEMBER(turtle_throttle_percent, float)

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
} profile_voltage_t;

#define VOLTAGE_MEMBERS                   \
  MEMBER(lipo_cell_count, uint8)          \
  MEMBER(pid_voltage_compensation, uint8) \
  MEMBER(vbattlow, float)                 \
  MEMBER(actual_battery_voltage, float)   \
  MEMBER(reported_telemetry_voltage, float)

typedef struct {
  aux_channel_t aux[AUX_FUNCTION_MAX];
  rx_lqi_source_t lqi_source;
  rx_stick_calibration_limits_t stick_calibration_limits[4];
} profile_receiver_t;

#define RECEIVER_MEMBERS                     \
  ARRAY_MEMBER(aux, AUX_FUNCTION_MAX, uint8) \
  MEMBER(lqi_source, uint8)

typedef struct {
  usart_ports_t rx;
  usart_ports_t smart_audio;
} profile_serial_t;

#define SERIAL_MEMBERS \
  MEMBER(rx, uint8)    \
  MEMBER(smart_audio, uint8)

typedef struct {
  uint32_t elements[OSD_NUMBER_ELEMENTS];
} profile_osd_t;

#define OSD_MEMBERS \
  ARRAY_MEMBER(elements, OSD_NUMBER_ELEMENTS, uint32)

typedef struct {
  filter_type_t type;
  float cutoff_freq;
} profile_filter_parameter_t;

#define FILTER_PARAMETER_MEMBERS \
  MEMBER(type, uint8)            \
  MEMBER(cutoff_freq, float)

typedef struct {
  profile_filter_parameter_t gyro[FILTER_MAX_SLOTS];
  profile_filter_parameter_t dterm[FILTER_MAX_SLOTS];
  uint8_t dterm_dynamic_enable;
  float dterm_dynamic_min;
  float dterm_dynamic_max;
} profile_filter_t;

#define FILTER_MEMBERS                                              \
  ARRAY_MEMBER(gyro, FILTER_MAX_SLOTS, profile_filter_parameter_t)  \
  ARRAY_MEMBER(dterm, FILTER_MAX_SLOTS, profile_filter_parameter_t) \
  MEMBER(dterm_dynamic_enable, uint8)                               \
  MEMBER(dterm_dynamic_min, float)                                  \
  MEMBER(dterm_dynamic_max, float)

typedef struct {
  uint8_t name[36];
  uint32_t datetime;
} profile_metadata_t;

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
} profile_t;

#define PROFILE_MEMBERS                \
  MEMBER(meta, profile_metadata_t)     \
  MEMBER(motor, profile_motor_t)       \
  MEMBER(serial, profile_serial_t)     \
  MEMBER(filter, profile_filter_t)     \
  MEMBER(osd, profile_osd_t)           \
  MEMBER(rate, profile_rate_t)         \
  MEMBER(receiver, profile_receiver_t) \
  MEMBER(pid, profile_pid_t)           \
  MEMBER(voltage, profile_voltage_t)

typedef enum {
  FEATURE_BRUSHLESS = (1 << 1),
  FEATURE_OSD = (1 << 2),
  FEATURE_BLACKBOX = (1 << 3),
} target_feature_t;

typedef struct {
  const char *target_name;
  const char *git_version;

  uint32_t features;
  uint32_t rx_protocol;
  uint32_t quic_protocol_version;

  const char *motor_pins[MOTOR_PIN_IDENT_MAX];
  const char *usart_ports[USART_PORTS_MAX];

  uint8_t gyro_id;
} target_info_t;

#define TARGET_INFO_MEMBERS                         \
  STR_MEMBER(target_name)                           \
  STR_MEMBER(git_version)                           \
  MEMBER(features, uint32)                          \
  MEMBER(rx_protocol, uint32)                       \
  MEMBER(quic_protocol_version, uint32)             \
  STR_ARRAY_MEMBER(motor_pins, MOTOR_PIN_IDENT_MAX) \
  STR_ARRAY_MEMBER(usart_ports, USART_PORTS_MAX)    \
  MEMBER(gyro_id, uint8)

extern profile_t profile;
extern target_info_t target_info;

void profile_set_defaults();
pid_rate_t *profile_current_pid_rates();

cbor_result_t cbor_encode_profile_t(cbor_value_t *enc, const profile_t *p);
cbor_result_t cbor_decode_profile_t(cbor_value_t *dec, profile_t *p);

cbor_result_t cbor_encode_pid_rate_preset_t(cbor_value_t *enc, const pid_rate_preset_t *p);
cbor_result_t cbor_encode_target_info_t(cbor_value_t *enc, const target_info_t *i);
