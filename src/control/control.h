#pragma once

#include <stdint.h>

#include "core/failloop.h"
#include "core/profile.h"
#include "core/project.h"
#include "core/scheduler.h"
#include "io/gps.h"
#include "rx/rx.h"
#include "util/vector.h"

// Throttle must drop below this value if arming feature is enabled for arming to take place.
// Brushed mix increase will also not activate on the ground until this threshold is passed during takeoff.
#define THROTTLE_SAFETY .10f

#define RXMODE_BIND 0
#define RXMODE_NORMAL 1

typedef struct {
  int32_t lon;
  int32_t lat;
} gps_coord_t;

#define GPS_COORD_MEMBERS   \
  START_STRUCT(gps_coord_t) \
  MEMBER(lon, int32_t)      \
  MEMBER(lat, int32_t)      \
  END_STRUCT()

typedef enum {
  ARMING_DISABLED_NONE = 0,
  ARMING_DISABLED_ARM_SWITCH = 1U << 0,
  ARMING_DISABLED_THROTTLE = 1U << 1,
  ARMING_DISABLED_FAILSAFE = 1U << 2,
  ARMING_DISABLED_USB = 1U << 3,
} arming_disabled_flags_t;

typedef enum {
  FAILSAFE_PHASE_IDLE,
  FAILSAFE_PHASE_HOLD_LAST,
  FAILSAFE_PHASE_STAGE1_GUARD,
  FAILSAFE_PHASE_STAGE2_DROP,
  FAILSAFE_PHASE_RECOVERY,
} failsafe_phase_t;

typedef enum {
  RTH_STATE_INACTIVE,
  RTH_STATE_CLIMB,
  RTH_STATE_ACQUIRE_HEADING,
  RTH_STATE_TURN,
  RTH_STATE_NAVIGATE,
  RTH_STATE_HOVER_HOME,
  RTH_STATE_HEADING_FAILED,
  RTH_STATE_ABORTED,
} rth_state_t;

constexpr float RTH_MIN_HEADING_CONFIDENCE = 0.2f;

// Reasons GPS course correction is blocked; also navigation debug channel 7.
enum {
  HEADING_NO_FIX = 1 << 0,
  HEADING_STALE = 1 << 1,
  HEADING_LOW_SPEED = 1 << 2,
  HEADING_POOR_ACCURACY = 1 << 3,
  HEADING_ROLL = 1 << 4,
  HEADING_YAW = 1 << 5,
  HEADING_NOT_FORWARD = 1 << 6,
  HEADING_RTH_PHASE = 1 << 7,
};

typedef enum {
  WING_LAUNCH_IDLE,
  WING_LAUNCH_IDLE_DELAY,
  WING_LAUNCH_WAIT,
  WING_LAUNCH_DETECTED,
  WING_LAUNCH_MOTOR_DELAY,
  WING_LAUNCH_SPINUP,
  WING_LAUNCH_ACTIVE,
  WING_LAUNCH_FINISH,
  WING_LAUNCH_DONE,
  WING_LAUNCH_ABORTED,
} wing_launch_state_t;

typedef enum {
  WING_AUTOTRIM_IDLE,
  WING_AUTOTRIM_ACTIVE, // Collecting commanded surface positions for two seconds.
  WING_AUTOTRIM_SAVE_PENDING,
  WING_AUTOTRIM_SAVED,
} wing_autotrim_state_t;

// THE UN OF STRUCTS
typedef struct {
  uint8_t arm_request : 1; // arming AUX is currently requested
  uint8_t arm_state : 1;   // armed after all safety checks have passed

  uint8_t in_air : 1;    // throttle was raised above THROTTLE_SAFETY (10%), only resets on disarm
  uint8_t on_ground : 1; // vehicle-owned ground state; wings stay airborne through glides until disarm

  uint8_t failsafe : 1;                 // failsafe warning / recovery is active
  uint8_t failsafe_outputs_blocked : 1; // failsafe has blocked output writes
  uint8_t failsafe_signal_lost : 1;     // receiver has reported signal loss

  uint8_t lowbatt : 1; // signal for lowbattery

  uint8_t rx_mode : 1; // bind / normal rx mode
  uint8_t rx_ready : 1;

  uint8_t controls_override : 1;  // will activate rx_override below & will write directly to the motors (motor_test)
  uint8_t motortest_override : 1; // tuns off digital idle in the dshot driver & will write either sticks or usb_motortest values directly to motors
  uint8_t turtle : 1;
  uint8_t turtle_ready : 1;
  uint8_t gestures_disabled : 1;

  volatile uint8_t usb_active;
  uint32_t arming_disabled_flags;
} control_flags_t;

extern control_flags_t flags;

const char *control_flight_mode_name(void);

typedef struct {
  failloop_t failloop;

  float looptime;            // looptime in seconds
  float looptime_us;         // looptime in us
  float looptime_autodetect; // desired looptime in us
  float looptime_inverse;    // 1/looptime for derivative calculations
  uint32_t loop_counter;     // number of loops ran
  uint8_t looptime_warning;  // Scheduler rate reductions since init; nonzero means fallback occurred.

  float uptime;      // running sum of looptimes
  float armtime;     // running sum of looptimes (while armed)
  uint32_t cpu_load; // Execution time in microseconds before busy-wait padding.

  uint32_t failsafe_time_ms; // time the current failsafe started in ms
  uint8_t failsafe_phase;

  uint8_t lipo_cell_count;

  float cpu_temp;

  float vbat;                      // battery in volts
  float vbat_filtered;             // filtered battery in volts (slow for display)
  float vbat_sag_filtered;         // filtered battery in volts (fast for warnings)
  float vbat_cell_avg;             // filtered battery divided by cell count
  float vbat_compensated;          // battery compensated for sag
  float vbat_compensated_cell_avg; // battery compensated for sag divided by cell count

  float ibat;              // battery current in amps
  float ibat_filtered;     // filtered current in amps (slow for display)
  float ibat_sag_filtered; // filtered current in amps (fast for mAh tracking)
  float ibat_drawn;        // total mAh consumed

  vec4_t rx;                            // holds raw or calibrated role channels; rover steering uses yaw
  vec4_t rx_filtered;                   // same as above, but with constraints (just in case), smoothing and deadband applied
  vec4_t rx_override;                   // override values, activated by controls_override
  uint16_t rx_channels[RX_CHANNEL_MAX]; // full resolution 16-bit receiver channel values
  float rx_filter_hz;

  stick_wizard_state_t stick_calibration_wizard; // current phase of the calibration wizard

  float rx_rssi;
  uint32_t rx_status;
  uint32_t last_frame_time_us;

  float throttle; // input throttle with idle etc applied
  float thrsum;   // average of all 4 motor thrusts

  uint32_t aux_active; // cached active aux_function_t bitmask

  vec3_t accel_raw; // raw accel reading with rotation and scaling applied
  vec3_t accel;     // filtered accel readings

  float gyro_temp;         // gyro temperature reading
  vec3_t gyro_raw;         // raw gyro reading with rotation and scaling applied
  vec3_t gyro;             // filtered gyro reading
  vec3_t gyro_delta_angle; // angle covered in  last time interval

  // IMU: attitude and GPS-aided heading.
  vec3_t GEstG;                     // gravity vector
  vec3_t attitude;                  // roll/pitch/yaw radians, positive nose-down pitch; yaw is zero without GPS configured.
  float heading;                    // Heading in degrees, 0..360; zero without GPS configured.
  float heading_confidence;         // 0..1; reset at init/arming, retained during fresh-GPS hover
  uint8_t heading_correction_flags; // latest blocking reasons, 0 permits correction on a new GPS sample

  // GPS: latest receiver solution.
  bool gps_lock;    // valid 3D fix with enough satellites; cleared after 500ms without NAV-PVT.
  uint8_t gps_sats; // Latest NAV-PVT satellites used, including while armed.
  float gps_speed;
  float gps_vel_north;
  float gps_vel_east;
  float gps_heading;
  float gps_heading_accuracy;
  float gps_horizontal_accuracy;
  uint32_t gps_last_update_ms;
  gps_coord_t gps_coord;
  float gps_altitude;

  // Barometer: detection, sample health and filtered outputs.
  bool baro_detected;           // Set by baro_init after probing; controls task registration.
  bool baro_valid;              // last filtered sample was finite; check timestamp for freshness.
  uint32_t baro_last_update_ms; // time of last finite sample; zero at init, valid distinguishes a sample at time zero.
  float baro_vertical_speed;    // filtered vertical velocity, m/s up; zero at init/reacquisition.
  float altitude;               // filtered meters above launch; zero at init/disarmed, held without a new valid sample.

  // Navigation: home reference and return-to-home commands.
  gps_coord_t gps_home;
  float home_bearing;
  float home_distance;
  uint8_t rth_state; // rth_state_t
  bool rth_active;
  bool rth_failsafe_active;
  float rth_yaw_rate; // radians/s, independent of pilot rates

  vec3_t setpoint; // Requested body rates (rad/s), from sticks or attitude control.
  vec3_t error;    // setpoint - gyro = error in angular velocity

  vec3_t pid_p_term;
  vec3_t pid_i_term;
  vec3_t pid_d_term;
  vec3_t pidoutput; // Limited normalized P + I + D output, including rate FF on wings.

  float mixer_source[OUTPUT_SOURCE_MAX];
  float output[MOTOR_PIN_MAX];
  uint8_t output_active[MOTOR_PIN_MAX];

  vec3_t angle_error;
  vec3_t stick_vector;
  uint8_t wing_launch_state;
  bool wing_launch_available;
  uint8_t wing_autotrim_state;

  uint32_t dshot_rpm[4];
} control_state_t;

#define STATE_MEMBERS                                  \
  START_STRUCT(control_state_t)                        \
  MEMBER(failloop, uint8_t)                            \
  MEMBER(looptime, float)                              \
  MEMBER(looptime_us, float)                           \
  MEMBER(looptime_autodetect, float)                   \
  MEMBER(looptime_inverse, float)                      \
  MEMBER(loop_counter, uint32_t)                       \
  MEMBER(looptime_warning, uint8_t)                    \
  MEMBER(uptime, float)                                \
  MEMBER(armtime, float)                               \
  MEMBER(cpu_load, uint32_t)                           \
  MEMBER(failsafe_time_ms, uint32_t)                   \
  MEMBER(failsafe_phase, uint8_t)                      \
  MEMBER(lipo_cell_count, uint8_t)                     \
  MEMBER(cpu_temp, float)                              \
  MEMBER(vbat, float)                                  \
  MEMBER(vbat_filtered, float)                         \
  MEMBER(vbat_sag_filtered, float)                     \
  MEMBER(vbat_cell_avg, float)                         \
  MEMBER(vbat_compensated, float)                      \
  MEMBER(vbat_compensated_cell_avg, float)             \
  MEMBER(ibat, float)                                  \
  MEMBER(ibat_filtered, float)                         \
  MEMBER(ibat_sag_filtered, float)                     \
  MEMBER(ibat_drawn, float)                            \
  MEMBER(rx, vec4_t)                                   \
  MEMBER(rx_filtered, vec4_t)                          \
  MEMBER(rx_override, vec4_t)                          \
  ARRAY_MEMBER(rx_channels, RX_CHANNEL_MAX, uint16_t)  \
  MEMBER(stick_calibration_wizard, uint8_t)            \
  MEMBER(rx_rssi, float)                               \
  MEMBER(rx_status, uint32_t)                          \
  MEMBER(last_frame_time_us, uint32_t)                 \
  MEMBER(throttle, float)                              \
  MEMBER(thrsum, float)                                \
  MEMBER(aux_active, uint32_t)                         \
  MEMBER(accel_raw, vec3_t)                            \
  MEMBER(accel, vec3_t)                                \
  MEMBER(gyro_temp, float)                             \
  MEMBER(gyro_raw, vec3_t)                             \
  MEMBER(gyro, vec3_t)                                 \
  MEMBER(gyro_delta_angle, vec3_t)                     \
  MEMBER(GEstG, vec3_t)                                \
  MEMBER(attitude, vec3_t)                             \
  MEMBER(heading, float)                               \
  MEMBER(heading_confidence, float)                    \
  MEMBER(heading_correction_flags, uint8_t)            \
  MEMBER(gps_lock, bool)                               \
  MEMBER(gps_sats, uint8_t)                            \
  MEMBER(gps_speed, float)                             \
  MEMBER(gps_vel_north, float)                         \
  MEMBER(gps_vel_east, float)                          \
  MEMBER(gps_heading, float)                           \
  MEMBER(gps_heading_accuracy, float)                  \
  MEMBER(gps_horizontal_accuracy, float)               \
  MEMBER(gps_last_update_ms, uint32_t)                 \
  MEMBER(gps_coord, gps_coord_t)                       \
  MEMBER(gps_altitude, float)                          \
  MEMBER(baro_detected, bool)                          \
  MEMBER(baro_valid, bool)                             \
  MEMBER(baro_last_update_ms, uint32_t)                \
  MEMBER(baro_vertical_speed, float)                   \
  MEMBER(altitude, float)                              \
  MEMBER(gps_home, gps_coord_t)                        \
  MEMBER(home_bearing, float)                          \
  MEMBER(home_distance, float)                         \
  MEMBER(rth_state, uint8_t)                           \
  MEMBER(rth_active, bool)                             \
  MEMBER(rth_failsafe_active, bool)                    \
  MEMBER(rth_yaw_rate, float)                          \
  MEMBER(setpoint, vec3_t)                             \
  MEMBER(error, vec3_t)                                \
  MEMBER(pid_p_term, vec3_t)                           \
  MEMBER(pid_i_term, vec3_t)                           \
  MEMBER(pid_d_term, vec3_t)                           \
  MEMBER(pidoutput, vec3_t)                            \
  ARRAY_MEMBER(mixer_source, OUTPUT_SOURCE_MAX, float) \
  ARRAY_MEMBER(output, MOTOR_PIN_MAX, float)           \
  ARRAY_MEMBER(output_active, MOTOR_PIN_MAX, uint8_t)  \
  MEMBER(angle_error, vec3_t)                          \
  MEMBER(stick_vector, vec3_t)                         \
  MEMBER(wing_launch_state, uint8_t)                   \
  MEMBER(wing_launch_available, bool)                  \
  MEMBER(wing_autotrim_state, uint8_t)                 \
  ARRAY_MEMBER(dshot_rpm, 4, uint32_t)                 \
  END_STRUCT()

typedef struct {
  uint8_t active;
  float value[MOTOR_PIN_MAX];
} motor_test_t;

extern control_state_t state;
extern motor_test_t motor_test;

cbor_result_t cbor_encode_control_state_t(cbor_value_t *enc, const control_state_t *s);

void control_update_arming();
void control_failsafe_update();
bool control_failsafe_active();
void control();

// Apply profile edits with reset=true; refresh loop timing with reset=false
// to retain filter history. Call between control iterations.
void control_filter_update(bool reset);
