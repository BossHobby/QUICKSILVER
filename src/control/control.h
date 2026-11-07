#pragma once

#include <stdint.h>

#include "core/failloop.h"
#include "core/profile.h"
#include "core/project.h"
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
  ARMING_DISABLED_CONFIGURATOR = 1U << 3, // USB configurator session
} arming_disabled_flags_t;

typedef enum : uint8_t {
  FAILSAFE_PHASE_IDLE,
  FAILSAFE_PHASE_HOLD_LAST,
  FAILSAFE_PHASE_STAGE1_GUARD,
  FAILSAFE_PHASE_STAGE2_DROP,
  FAILSAFE_PHASE_RECOVERY,
} failsafe_phase_t;

// Navigation flying through a receiver loss instead of the failsafe
// procedure, until the link is stable and the pilot moves a stick. The
// switches stay locked meanwhile.
typedef enum : uint8_t {
  FAILSAFE_OWNER_NONE,
  FAILSAFE_OWNER_MISSION, // the mission continues
  FAILSAFE_OWNER_RTH,     // multirotor RTH returns
} failsafe_owner_t;

// Multirotor navigation phases. From NAV_STATE_CLIMB on, navigation owns the
// controls.
typedef enum : uint8_t {
  NAV_STATE_INACTIVE,
  NAV_STATE_ABORTED, // gave up; not retried until the pilot ends the request or disarms
  NAV_STATE_CLIMB,
  NAV_STATE_ACQUIRE_HEADING,
  NAV_STATE_TURN,
  NAV_STATE_NAVIGATE,
  NAV_STATE_HOLD,  // hovering on the target: home, a reached waypoint or the guided target
  NAV_STATE_BRAKE, // a guided HOLD command stops over the ground, then holds where it came to rest
  NAV_STATE_HEADING_FAILED,
  NAV_STATE_NAV_FAILED,
  NAV_STATE_TAKEOFF_READY, // mission TAKEOFF armed on the ground, motors stopped until the throttle stick is raised
  NAV_STATE_TAKEOFF_SPINUP, // spinning up on the ground until lift-off
  NAV_STATE_TAKEOFF,        // lifted off, climbing in place
  NAV_STATE_LAND,          // descending on a mission LAND waypoint until touchdown disarms
} nav_state_t;

// Where rotor wash may make the barometer read low, and the ground it reads
// low against.
typedef enum : uint8_t {
  GROUND_EFFECT_NONE,
  GROUND_EFFECT_LAUNCH,  // on or just above the launch ground, at zero altitude
  GROUND_EFFECT_LANDING, // landing on ground learned at touchdown
} ground_effect_t;

// What navigation flies to.
typedef enum : uint8_t {
  NAV_TARGET_HOME,
  NAV_TARGET_MISSION, // the active waypoint of the stored mission
  NAV_TARGET_GUIDED,  // the ground station target in nav_route.guided
} nav_target_t;

constexpr float NAV_MIN_HEADING_CONFIDENCE = 0.2f;

// Reasons GPS course correction is blocked; also navigation debug channel 7.
enum {
  HEADING_NO_FIX = 1 << 0,
  HEADING_STALE = 1 << 1,
  HEADING_LOW_SPEED = 1 << 2,
  HEADING_POOR_ACCURACY = 1 << 3,
  HEADING_ROLL = 1 << 4,
  HEADING_YAW = 1 << 5,
  HEADING_NOT_FORWARD = 1 << 6,
  HEADING_NAV_PHASE = 1 << 7,
};

typedef enum : uint8_t {
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

typedef enum : uint8_t {
  WING_AUTOTRIM_IDLE,
  WING_AUTOTRIM_NO_SURFACE,    // No PWM output carries a roll, pitch or yaw mix.
  WING_AUTOTRIM_ACTIVE,        // Collecting commanded surface positions for two seconds.
  WING_AUTOTRIM_CENTER_STICKS, // Captured in Manual, applied once the sticks center.
  WING_AUTOTRIM_SAVE_PENDING,
  WING_AUTOTRIM_SAVED,
} wing_autotrim_state_t;

typedef enum : uint8_t {
  WING_NAV_INACTIVE,
  WING_NAV_LOITER,        // GPS circle around a center placed on entry or GPS recovery.
  WING_NAV_LOITER_BANK,   // No valid GPS: constant-bank turn holding altitude.
  WING_NAV_RTH_RETURN,    // Flying the GPS ground track toward home.
  WING_NAV_RTH_HOME,      // Circling home at the return altitude.
  WING_NAV_RTH_HEADING,   // GPS lost while returning: home bearing on IMU heading for a limited time.
  WING_NAV_DESCEND,       // Unpowered failsafe glide, held until receiver recovery.
  WING_NAV_CRUISE_STEER,  // Cruise with the pilot banking; the course locks after centered roll and yaw.
  WING_NAV_CRUISE,        // Cruise holding the GPS ground track latched at lock.
  WING_NAV_CRUISE_NO_GPS, // Cruise without valid GPS: the pilot banks and the course cannot lock.
  WING_NAV_MISSION,       // Flying the GPS ground track toward the active waypoint.
  WING_NAV_MISSION_HOLD,  // Circling the last waypoint after the mission, or a waypoint until its altitude is reached.
} wing_nav_state_t;

// Shared control and status bits. Bitfields are written by Flight only; a
// flag written by another thread gets its own byte below.
typedef struct {
  // Arming.
  uint8_t arm_request : 1; // arming AUX is currently requested
  uint8_t arm_state : 1;   // armed after all safety checks have passed

  // Ground state.
  uint8_t in_air : 1;    // throttle was raised above THROTTLE_SAFETY (10%), only resets on disarm
  uint8_t on_ground : 1; // vehicle-owned ground state; wings stay airborne through glides until disarm

  // Failsafe.
  uint8_t failsafe : 1;                 // failsafe warning / recovery is active
  uint8_t failsafe_outputs_blocked : 1; // failsafe has blocked output writes
  uint8_t failsafe_hold : 1;            // airborne stage 1 failsafe levels and holds altitude instead of idling; owned by control
  uint8_t wing_nav_failsafe : 1;        // wing navigation owns a receiver failsafe; failsafe holds stage 1 with live outputs

  // Output overrides and pilot modes.
  uint8_t controls_override : 1;  // will activate rx_override below & will write directly to the motors (motor_test)
  uint8_t motortest_override : 1; // tuns off digital idle in the dshot driver & will write either sticks or usb_motortest values directly to motors
  uint8_t turtle : 1;
  uint8_t turtle_ready : 1;
  uint8_t gestures_disabled : 1;
  uint8_t wing_launch_available : 1;

  // Sensors.
  uint8_t baro_detected : 1; // set once by baro_init() during Flight's peripheral startup; enables IO barometer updates

  // Flight navigation status. Shared navigation owns the sensor checks and
  // home; gps_valid and home_valid are clear without a GPS port.
  uint8_t altitude_valid : 1; // altitude and climb rate are aided: a finite barometer sample at most BARO_STALE_MS old, checked at the 10 ms navigation cadence
  uint8_t gps_valid : 1;      // lock, accuracy, freshness and state.gps_trust are sufficient, and no position jump is settling
  uint8_t gps_glitch : 1;     // GPS position disagrees with the estimate beyond its gate and is not fused
  uint8_t home_valid : 1;     // home latched from a valid GPS position on arming, or at the first one after it before leaving the ground; clear while disarmed
  uint8_t guided_ready : 1;    // the pilot grants guided flight: ground station commands are accepted
  uint8_t mission_engaged : 1; // navigation flies the mission, its end action or a return the mission switch ends

  // Separate bytes prevent IO flag writes from overwriting Flight's bitfields.
  uint8_t failsafe_signal_lost; // RX reports loss; Flight also enforces timeout.
  uint8_t lowbatt;
  uint8_t rx_mode;
  uint8_t rx_ready;

  volatile uint8_t usb_active;
  uint32_t arming_disabled_flags;
} control_flags_t;

extern control_flags_t flags;

const char *control_flight_mode_name(void);

typedef struct {
  // System and loop timing, owned by Flight (main.cpp).
  failloop_t failloop;
  float looptime;            // looptime in seconds
  float looptime_us;         // looptime in us
  float looptime_autodetect; // desired looptime in us
  float looptime_inverse;    // 1/looptime for derivative calculations
  uint32_t loop_counter;     // number of loops ran
  uint8_t looptime_warning;  // Scheduler rate reductions since init; nonzero means fallback occurred.
  uint32_t uptime_ms;        // ms of Flight loop periods since timing init
  uint32_t armtime_ms;       // ms of those periods spent armed; never reset
  uint32_t cpu_load;         // System load percentage 0-100 from the idle task's cycle share; ISR time is charged to the interrupted task. Published by Flight on a fixed window.
  float cpu_temp;            // degrees C

  // Battery, owned by IO (vbat.cpp).
  uint8_t lipo_cell_count;
  float vbat;                      // battery in volts
  float vbat_filtered;             // filtered battery in volts (slow for display)
  float vbat_sag_filtered;         // filtered battery in volts (fast for warnings)
  float vbat_cell_avg;             // filtered battery divided by cell count
  float vbat_compensated;          // battery compensated for sag
  float vbat_compensated_cell_avg; // battery compensated for sag divided by cell count
  float ibat;                      // battery current in milliamps
  float ibat_filtered;             // filtered current in milliamps (slow for display)
  float ibat_sag_filtered;         // filtered current in milliamps (fast for mAh tracking)
  float ibat_drawn;                // total mAh consumed

  // Receiver. Channels, roles and aux are Flight-owned; link status is
  // written by the IO receiver drivers.
  uint16_t rx_channels[RX_CHANNEL_MAX]; // Flight-owned latest complete RC frame, 0..65535; initialized to zero.
  vec4_t rx;                            // Flight-owned calibrated, deadbanded roles; rover steering uses yaw
  vec4_t rx_filtered;                   // Flight-owned constrained, smoothed roles; held input is smoothed every loop
  vec4_t rx_override;                   // override values, activated by controls_override
  float rx_filter_hz;
  uint32_t aux_active;                           // cached active aux_function_t bitmask
  stick_wizard_state_t stick_calibration_wizard; // current phase of the calibration wizard
  float rx_rssi;
  uint32_t rx_status;          // SPI receiver bind state (rx_spi_status_t)
  rx_protocol_t rx_protocol;   // IO-owned active receiver protocol; SERIAL_AUTO until detection locks
  uint32_t rx_frame_time_us;   // time_micros() of the latest received frame

  // Gyro and accelerometer, owned by sixaxis.
  float gyro_temp;         // gyro temperature reading
  vec3_t gyro_raw;         // raw gyro reading with rotation and scaling applied
  vec3_t gyro;             // filtered gyro reading
  vec3_t gyro_delta_angle; // angle covered in  last time interval
  vec3_t accel_raw;        // raw accel reading with rotation and scaling applied
  vec3_t accel;            // filtered specific force, g in the accel_raw frame

  // IMU: attitude and GPS-aided heading.
  vec3_t GEstG;                     // gravity vector
  vec3_t attitude;                  // roll/pitch/yaw radians, positive nose-down pitch; yaw is zero without GPS configured.
  float heading;                    // Heading in degrees, 0..360; zero without GPS configured.
  float heading_confidence;         // 0..1; reset at init/arming, retained during fresh-GPS hover
  uint8_t heading_correction_flags; // latest blocking reasons, 0 permits correction on a new GPS sample
  // Radians every correction turned the heading by, summed and wrapped to
  // -pi..pi. Readers difference it to follow the corrections.
  float heading_correction_total;

  // GPS: latest receiver solution, owned by IO (gps.cpp).
  gps_health_t gps_health;       // lock, jamming and spoofing
  uint8_t gps_sats;              // Latest NAV-PVT satellites used, including while armed.
  gps_coord_t gps_coord;         // degrees * 1e7
  float gps_altitude;            // m MSL
  vec3_t gps_velocity;           // m/s north, east, up (NAV-PVT velN, velE, -velD)
  float gps_speed;               // m/s ground speed
  float gps_heading;             // degrees, course over ground
  float gps_horizontal_accuracy; // m
  float gps_vertical_accuracy;   // m, receiver 1-sigma altitude estimate (NAV-PVT vAcc)
  float gps_speed_accuracy;      // m/s, receiver 1-sigma velocity estimate (NAV-PVT sAcc)
  float gps_heading_accuracy;    // degrees
  // time_millis() of the latest NAV-PVT epoch: receiver iTOW mapped onto the
  // local clock at the fastest delivery, so spacing follows the measurement
  // interval rather than delivery jitter. Never ahead of its delivery.
  uint32_t gps_last_update_ms;

  // Navigation estimate, published by shared navigation at its 10 ms cadence:
  // the accelerometer in the earth frame, corrected by the barometer and by
  // GPS position and velocity at the time the receiver measured them.
  // Meters above launch, or above the first barometer sample before arming;
  // arming re-zeroes it. Zero at init and held while flags.altitude_valid is
  // clear. Armed, GPS altitude slowly corrects barometer drift.
  float altitude;
  // Horizontal position, valid with flags.gps_valid: GPS position carried to
  // the present by the velocity below, which also rides out GPS position
  // glitches; it restarts at gps_coord at the first valid fix, after GPS
  // lapsed and after a lasting jump.
  gps_coord_t position;
  // m/s in the earth frame: north, east, up. Up decays toward zero while
  // altitude is unaided. North and east, valid with flags.gps_valid, follow
  // the accelerometer with a trusted heading and are GPS velocity otherwise.
  // They turn with the heading, so heading checks (IMU fusion, multirotor
  // divergence), horizon correction and wing speed learning read the
  // gps_velocity measurement.
  vec3_t velocity;
  // m/s^2 north and east, how gps_velocity changes between consecutive GPS
  // samples, low-passed over 0.2 s; zero without them. Derived by shared
  // navigation; independent of the accelerometer, so the IMU removes it from
  // the gravity reference while navigation steers on a trusted heading.
  vec2_t gps_accel;
  // 0..1, how consistently GPS agrees with itself and the accelerometer,
  // graded by shared navigation; it scales every GPS correction.
  float gps_trust;

  // Home, latched by shared navigation; valid with flags.home_valid. Bearing
  // and distance update only while GPS and home are valid, and otherwise
  // hold their last values.
  gps_coord_t home;
  float home_altitude; // GPS altitude at the home capture, meters MSL.
  float home_bearing;  // degrees true from the vehicle, 0..360
  float home_distance; // meters

  // Navigation, published at its 10 ms cadence by vehicle navigation
  // (multirotor or wing) in Flight, through nav_update_target() for the target.
  nav_state_t nav_state;     // owned by multirotor navigation
  nav_target_t nav_target;
  // The guided target, the active mission waypoint or home. For home, bearing
  // and distance are home_bearing and home_distance, valid with flags.home_valid.
  gps_coord_t nav_target_position;
  float nav_target_bearing;  // degrees true from the vehicle, 0..360
  float nav_target_distance; // meters
  uint8_t mission_index;     // active waypoint while flying the mission, zero otherwise
  float nav_yaw_rate;        // radians/s commanded by multirotor navigation, independent of pilot rates; zero otherwise
  float hover_throttle; // learned level-hover throttle; zero until learned in flight.
  ground_effect_t ground_effect; // owned by multirotor navigation; from arming until clear of the launch ground or 5 s of raised throttle, and while landing

  // Failsafe, owned by control.
  failsafe_phase_t failsafe_phase;
  uint32_t failsafe_time_ms; // time the current failsafe started in ms
  failsafe_owner_t failsafe_owner;

  // Control: commands and PID, owned by the vehicle controller.
  float throttle;      // input throttle with idle etc applied
  vec3_t stick_vector;
  vec3_t angle_error;
  vec3_t setpoint;     // Requested body rates (rad/s), from sticks or attitude control.
  vec3_t error;        // setpoint - gyro = error in angular velocity
  vec3_t pid_p_term;
  vec3_t pid_i_term;
  vec3_t pid_d_term;
  vec3_t pidoutput;    // Limited normalized P + I + D output, including rate FF on wings.

  // Outputs.
  float mixer_source[OUTPUT_SOURCE_MAX];
  float output[MOTOR_PIN_MAX];
  uint8_t output_active[MOTOR_PIN_MAX];
  float thrsum;                      // average of all 4 motor thrusts
  uint32_t dshot_rpm[MOTOR_PIN_MAX]; // eRPM / 100 per target output slot, written by the DShot driver

  // Wing.
  wing_launch_state_t wing_launch_state;
  wing_autotrim_state_t wing_autotrim_state;
  wing_nav_state_t wing_nav_state; // owned by wing navigation
  float wing_speed_scale;      // cruise / estimated airspeed, 0.6..1.25, from throttle and climb; 1 on the ground. Owned by wing control.
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
  MEMBER(uptime_ms, uint32_t)                          \
  MEMBER(armtime_ms, uint32_t)                         \
  MEMBER(cpu_load, uint32_t)                           \
  MEMBER(cpu_temp, float)                              \
  MEMBER(lipo_cell_count, uint8_t)                     \
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
  ARRAY_MEMBER(rx_channels, RX_CHANNEL_MAX, uint16_t)  \
  MEMBER(rx, vec4_t)                                   \
  MEMBER(rx_filtered, vec4_t)                          \
  MEMBER(rx_override, vec4_t)                          \
  MEMBER(rx_filter_hz, float)                          \
  MEMBER(aux_active, uint32_t)                         \
  MEMBER(stick_calibration_wizard, uint8_t)            \
  MEMBER(rx_rssi, float)                               \
  MEMBER(rx_status, uint32_t)                          \
  MEMBER(rx_protocol, uint8_t)                         \
  MEMBER(rx_frame_time_us, uint32_t)                   \
  MEMBER(gyro_temp, float)                             \
  MEMBER(gyro_raw, vec3_t)                             \
  MEMBER(gyro, vec3_t)                                 \
  MEMBER(gyro_delta_angle, vec3_t)                     \
  MEMBER(accel_raw, vec3_t)                            \
  MEMBER(accel, vec3_t)                                \
  MEMBER(GEstG, vec3_t)                                \
  MEMBER(attitude, vec3_t)                             \
  MEMBER(heading, float)                               \
  MEMBER(heading_confidence, float)                    \
  MEMBER(heading_correction_flags, uint8_t)            \
  MEMBER(gps_health, gps_health_t)                     \
  MEMBER(gps_sats, uint8_t)                            \
  MEMBER(gps_coord, gps_coord_t)                       \
  MEMBER(gps_altitude, float)                          \
  MEMBER(gps_velocity, vec3_t)                         \
  MEMBER(gps_speed, float)                             \
  MEMBER(gps_heading, float)                           \
  MEMBER(gps_horizontal_accuracy, float)               \
  MEMBER(gps_vertical_accuracy, float)                 \
  MEMBER(gps_speed_accuracy, float)                    \
  MEMBER(gps_heading_accuracy, float)                  \
  MEMBER(gps_last_update_ms, uint32_t)                 \
  MEMBER(altitude, float)                              \
  MEMBER(position, gps_coord_t)                        \
  MEMBER(velocity, vec3_t)                             \
  MEMBER(gps_accel, vec2_t)                            \
  MEMBER(gps_trust, float)                             \
  MEMBER(home, gps_coord_t)                            \
  MEMBER(home_altitude, float)                         \
  MEMBER(home_bearing, float)                          \
  MEMBER(home_distance, float)                         \
  MEMBER(nav_state, uint8_t)                           \
  MEMBER(nav_target, uint8_t)                          \
  MEMBER(nav_target_position, gps_coord_t)             \
  MEMBER(nav_target_bearing, float)                    \
  MEMBER(nav_target_distance, float)                   \
  MEMBER(mission_index, uint8_t)                       \
  MEMBER(nav_yaw_rate, float)                          \
  MEMBER(hover_throttle, float)                        \
  MEMBER(ground_effect, uint8_t)                       \
  MEMBER(failsafe_phase, uint8_t)                      \
  MEMBER(failsafe_time_ms, uint32_t)                   \
  MEMBER(failsafe_owner, uint8_t)                      \
  MEMBER(throttle, float)                              \
  MEMBER(stick_vector, vec3_t)                         \
  MEMBER(angle_error, vec3_t)                          \
  MEMBER(setpoint, vec3_t)                             \
  MEMBER(error, vec3_t)                                \
  MEMBER(pid_p_term, vec3_t)                           \
  MEMBER(pid_i_term, vec3_t)                           \
  MEMBER(pid_d_term, vec3_t)                           \
  MEMBER(pidoutput, vec3_t)                            \
  ARRAY_MEMBER(mixer_source, OUTPUT_SOURCE_MAX, float) \
  ARRAY_MEMBER(output, MOTOR_PIN_MAX, float)           \
  ARRAY_MEMBER(output_active, MOTOR_PIN_MAX, uint8_t)  \
  MEMBER(thrsum, float)                                \
  ARRAY_MEMBER(dshot_rpm, MOTOR_PIN_MAX, uint32_t)     \
  MEMBER(wing_launch_state, uint8_t)                   \
  MEMBER(wing_autotrim_state, uint8_t)                 \
  MEMBER(wing_nav_state, uint8_t)                      \
  MEMBER(wing_speed_scale, float)                      \
  END_STRUCT()

typedef struct {
  uint8_t active;
  float value[MOTOR_PIN_MAX];
} motor_test_t;

extern control_state_t state;
extern motor_test_t motor_test;

cbor_result_t cbor_encode_control_state_t(cbor_value_t *enc, const control_state_t *s);

void control_update_arming();
// Flight: disarm at the next arming update, as if the arm switch had been
// lowered; arming again needs the switch cycled. For a landing or an aborted
// takeoff, so the motors stop in the same pass that disarms.
void control_request_disarm();
void control_failsafe_disarm();
void control_failsafe_update();
bool control_failsafe_active();
void control();

// Apply profile edits with reset=true; refresh loop timing with reset=false
// to retain filter history. Call between control iterations.
void control_filter_update(bool reset);
