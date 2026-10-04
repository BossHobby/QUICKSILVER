#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "control/control.h"
#include "control/imu.h"
#include "control/multi/navigation.h"
#include "control/navigation.h"
#include "control/pid.h"
#include "control/sixaxis.h"
#include "core/profile.h"
#include "driver/baro/baro.h"
#include "driver/motor.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "io/blackbox.h"
#include "io/gps.h"
#include "mock_helpers.h"
#include "rx/rx.h"
#include "util/filter.h"

// Deterministic control integration model (SI units): a 0.6 kg X quad with
// 0.10 m arm projections and diagonal inertia {0.0025, 0.0025, 0.0045} kg m^2.
// Motor command follows a 25 ms first-order speed response; thrust is 6*speed^2
// N per rotor, reaction torque is +/-0.015*thrust Nm. Euler's rigid-body
// equation includes omega x I*omega and 0.002 Nm/(rad/s) rotational damping.
// Translation uses gravity, rotated thrust and 0.3/s linear airspeed drag.
// Integrate at 1 ms with semi-implicit Euler and a normalized quaternion.
// The physical heading starts at 35 degrees; the estimator must discover it.
//
// Flight runs sixaxis_read/imu_calc/rx_process/control/blackbox_capture/nav_update
// in production order. Native sixaxis_read is a no-op: feed gyro (rad/s), its
// measured delta, and specific force R^T*(acceleration-gravity)/g, NOT a gravity
// estimate. Motor feedback is simulator_motor_values after output routing,
// limiting and driver update. GPS uses real NAV-PVT parsing at 10 Hz (optional
// 80/120 ms jitter, delayed truth, 0.06 m/s deterministic velocity noise).
// Barometer altitude enters the IO mailbox at 50 Hz with 3 cm sinusoidal noise.
// RC uses complete AETR frames and the RX mailbox at 100 Hz, including LQI loss
// timing. The pilot height helper only supplies RC before RTH/after handback.
//
// Limits: one idealized rigid airframe, constant battery/thrust coefficient,
// ideal calibrated inertial sensors, flat earth near the equator, no propwash,
// ground effect, vibration, contact/landing dynamics or actuator damage. Ground
// support only permits takeoff; impacts fail. Native sensor/ESC electrical
// drivers, GPS boot negotiation, Blackbox storage and FreeRTOS scheduling are
// not exercised. This is control regression coverage, not flight-safety proof.
// Run: SKIP_TARGETS_CHECKOUT=1 pio test -e multi-test --filter test_navigation

extern float simulator_motor_values[MOTOR_PIN_MAX];
extern serial_port_t serial_gps;
extern void simulator_rx_test_frame(const uint16_t *channels);
extern void baro_test_sample(float altitude, uint32_t timestamp_ms);

namespace {
static constexpr double DT = 0.001;
static constexpr double GRAVITY = 9.80665;
static constexpr double MASS = 0.6;
static constexpr double MAX_THRUST = 6.0; // N per motor at unit speed
static constexpr double ARM = 0.10;      // X and Y arm projections, meters
static constexpr double INERTIA[3] = {0.0025, 0.0025, 0.0045};
static constexpr unsigned HISTORY_SIZE = 512;

// Independent physical truth: body forward/right/down, earth north/east/down.
// Do not use firmware quaternion/vector helpers to generate the sensor oracle.
struct plant_t {
  double position[3] = {};
  double velocity[3] = {};
  double acceleration[3] = {};
  double omega[3] = {};
  double speed[4] = {};
  double q[4] = {cos(35.0 * M_PI / 360), 0, 0, sin(35.0 * M_PI / 360)};
  double rotation[3][3] = {};
  double wind[3] = {};

  void rotate() {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    rotation[0][0] = 1 - 2 * (y * y + z * z);
    rotation[0][1] = 2 * (x * y - w * z);
    rotation[0][2] = 2 * (x * z + w * y);
    rotation[1][0] = 2 * (x * y + w * z);
    rotation[1][1] = 1 - 2 * (x * x + z * z);
    rotation[1][2] = 2 * (y * z - w * x);
    rotation[2][0] = 2 * (x * z - w * y);
    rotation[2][1] = 2 * (y * z + w * x);
    rotation[2][2] = 1 - 2 * (x * x + y * y);
  }

  void step(const float motors[4]) {
    double thrust[4];
    for (unsigned i = 0; i < 4; i++) {
      const double command = motors[i] == MOTOR_OFF ? 0 : double(motors[i]);
      speed[i] += (1 - exp(-DT / 0.025)) * (command - speed[i]);
      thrust[i] = MAX_THRUST * speed[i] * speed[i];
    }
    // Physical positions BL, FL, BR, FR; alternating reaction torque signs.
    const double torque[3] = {
        ARM * (thrust[0] + thrust[1] - thrust[2] - thrust[3]),
        ARM * (-thrust[0] + thrust[1] - thrust[2] + thrust[3]),
        0.015 * (thrust[0] - thrust[1] - thrust[2] + thrust[3]),
    };
    double derivative[3];
    for (unsigned i = 0; i < 3; i++) {
      const unsigned j = (i + 1) % 3, k = (i + 2) % 3;
      derivative[i] = (torque[i] - (INERTIA[k] - INERTIA[j]) * omega[j] * omega[k] - 0.002 * omega[i]) / INERTIA[i];
    }
    for (unsigned i = 0; i < 3; i++) omega[i] += derivative[i] * DT;
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    q[0] += 0.5 * DT * (-x * omega[0] - y * omega[1] - z * omega[2]);
    q[1] += 0.5 * DT * (w * omega[0] + y * omega[2] - z * omega[1]);
    q[2] += 0.5 * DT * (w * omega[1] + z * omega[0] - x * omega[2]);
    q[3] += 0.5 * DT * (w * omega[2] + x * omega[1] - y * omega[0]);
    const double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (double &component : q) component /= norm;
    rotate();
    const double total = thrust[0] + thrust[1] + thrust[2] + thrust[3];
    for (unsigned i = 0; i < 3; i++) {
      acceleration[i] = -rotation[i][2] * total / MASS - 0.3 * (velocity[i] - wind[i]);
      if (i == 2) acceleration[i] += GRAVITY;
    }
    // Flat ground supports a stationary vehicle until thrust lifts it.
    if (position[2] >= 0 && acceleration[2] >= 0 && velocity[2] >= 0) {
      for (unsigned i = 0; i < 3; i++) acceleration[i] = velocity[i] = omega[i] = 0;
    }
    for (unsigned i = 0; i < 3; i++) {
      velocity[i] += acceleration[i] * DT;
      position[i] += velocity[i] * DT;
    }
  }

  void imu_sample() const {
    double force[3] = {};
    for (unsigned body = 0; body < 3; body++) {
      for (unsigned earth = 0; earth < 3; earth++) {
        force[body] += rotation[earth][body] * (acceleration[earth] - (earth == 2 ? GRAVITY : 0));
      }
    }
    // Native simulator boundary: rad/s, positive nose-down pitch; legacy
    // accelerometer axes left/back/up in g, including drag and ground support.
    state.gyro = state.gyro_raw = {{float(omega[0]), float(-omega[1]), float(omega[2])}};
    state.gyro_delta_angle = {{float(omega[0] * DT), float(-omega[1] * DT), float(omega[2] * DT)}};
    state.accel_raw = {{float(-force[1] / GRAVITY), float(-force[0] / GRAVITY), float(-force[2] / GRAVITY)}};
  }
};

struct fix_t {
  double north, east, height, vn, ve, vd;
};

static void put_u32(uint8_t *bytes, int32_t value) {
  const uint32_t bits = static_cast<uint32_t>(value);
  for (unsigned i = 0; i < 4; i++) bytes[i] = bits >> (8 * i);
}

static void gps_sample(const fix_t &fix, unsigned sample) {
  // UBX NAV-PVT, same native serial/parser boundary as test_common/test_gps.
  uint8_t packet[100] = {0xb5, 0x62, 1, 7, 92, 0};
  uint8_t *p = packet + 6;
  put_u32(p, time_millis());
  p[20] = GPS_FIX_3D;
  p[21] = 1;
  p[23] = 12;
  const double noise = 0.06 * sin(sample * 1.7);
  const double vn = fix.vn + noise, ve = fix.ve - noise;
  put_u32(p + 24, lround(fix.east * 1e7 / 111194.93));
  put_u32(p + 28, lround(fix.north * 1e7 / 111194.93));
  put_u32(p + 36, lround(fix.height * 1000));
  put_u32(p + 40, 500);
  put_u32(p + 44, 800);
  put_u32(p + 48, lround(vn * 1000));
  put_u32(p + 52, lround(ve * 1000));
  put_u32(p + 56, lround(fix.vd * 1000));
  put_u32(p + 60, lround(hypot(vn, ve) * 1000));
  double course = atan2(ve, vn) * 180 / M_PI;
  if (course < 0) course += 360;
  put_u32(p + 64, lround(course * 100000));
  put_u32(p + 68, 100);
  put_u32(p + 72, 200000);
  for (unsigned i = 2; i < 98; i++) {
    packet[98] += packet[i];
    packet[99] += packet[98];
  }
  TEST_ASSERT_EQUAL_UINT32(sizeof(packet), ring_buffer_write_multi(serial_gps.rx_buffer, packet, sizeof(packet)));
}

struct scenario_t {
  const char *name;
  unsigned gps_delay_ms = 0;
  bool gps_jitter = false;
  double wind_north = 0, wind_east = 0;
  float configured_hover = 0.5f;
  bool climbing_departure = false;
};

struct flight_t {
  scenario_t scenario;
  plant_t truth;
  fix_t history[HISTORY_SIZE] = {};
  unsigned tick = 0, next_gps = 0, gps_count = 0;
  bool arm = false, rth = false, rc_available = true, gps_available = true, baro_available = true;
  float roll = 0, pitch = 0, yaw = 0, throttle = 0;
  double return_height = 0, takeover_height = 0;
  char context[512];

  explicit flight_t(scenario_t config) : scenario(config) {
    mock_hardware_reset_all();
    time_test_reset();
    state = {};
    flags = {};
    flags.on_ground = 1;
    target = {};
    profile_set_defaults(&profile);
    for (unsigned i = 0; i < 4; i++) {
      // Deliberately permute physical pins; plant reads actual mapped outputs.
      const unsigned pin = (i + 2) % 4;
      profile.outputs[i].target_output = pin;
      target.outputs[pin].pin = static_cast<gpio_pins_t>(PIN_A0 + pin);
      target.outputs[pin].caps = OUTPUT_CAP_DSHOT;
    }
    for (auto &aux : profile.receiver.aux) aux = {RX_CHANNEL_OFF, 0, 0};
    profile.receiver.aux[AUX_ARMING] = {static_cast<rx_channel_t>(4), AUX_VALUE_MID, AUX_VALUE_MAX};
    profile.receiver.aux[AUX_RETURN_TO_HOME] = {static_cast<rx_channel_t>(5), AUX_VALUE_MID, AUX_VALUE_MAX};
    profile.receiver.aux[AUX_PREARM] = {RX_CHANNEL_ON, 0, 0};
    profile.receiver.aux[AUX_LEVELMODE] = {RX_CHANNEL_ON, 0, 0};
    profile.receiver.aux[AUX_IDLE_UP] = {RX_CHANNEL_ON, 0, 0};
    profile.serial.gps = SERIAL_PORT1;
    profile.rate.level_max_angle = 35;
    profile.navigation.rth_altitude = 12;
    profile.navigation.rth_cruise_speed = 6;
    profile.navigation.rth_throttle_min = 0.1f;
    profile.navigation.rth_throttle_max = 0.85f;
    profile.navigation.rth_throttle_hover = config.configured_hover;
    profile.navigation.rth_on_failsafe = true;
    state.looptime = DT;
    state.looptime_inverse = 1 / DT;
    state.looptime_autodetect = DT * 1e6;
    state.rx_filter_hz = 50;
    state.vbat_cell_avg = 4.0f;
    filter_global_init();
    motor_init();
    motor_set_all(MOTOR_OFF);
    motor_update();
    baro_init();
    nav_init();
    sixaxis_init();
    pid_init();
    rx_init();
    gps_init();
    // Receiver boot/config exchange is outside this control integration test.
    gps_status.state = GPS_RUNNING;
    gps_status.version = 0x000A0000U;
    ring_buffer_clear(serial_gps.rx_buffer);
    ring_buffer_clear(serial_gps.tx_buffer);
    truth.rotate();
    truth.imu_sample();
    imu_init();
  }

  void check(bool condition, const char *message) {
    if (condition) return;
    snprintf(context, sizeof(context), "%s: %s t=%.3f rth=%u fs=%u arm=%u disabled=%u truth=(%.2f,%.2f,%.2f) v=(%.2f,%.2f,%.2f) est_alt=%.2f tilt=%.1f/%.1f heading=%.1f confidence=%.2f throttle=%.3f motors=(%.3f,%.3f,%.3f,%.3f)",
        scenario.name, message, tick * DT, unsigned(state.rth_state), unsigned(state.failsafe_phase), unsigned(flags.arm_state), unsigned(flags.arming_disabled_flags),
        truth.position[0], truth.position[1], -truth.position[2], truth.velocity[0], truth.velocity[1], -truth.velocity[2],
        double(state.altitude), double(state.attitude.roll) * 180 / M_PI, double(state.attitude.pitch) * 180 / M_PI, double(state.heading), double(state.heading_confidence), double(state.throttle),
        double(simulator_motor_values[2]), double(simulator_motor_values[3]), double(simulator_motor_values[0]), double(simulator_motor_values[1]));
    TEST_FAIL_MESSAGE(context);
  }

  void step() {
    time_test_advance_us(1000);
    history[tick % HISTORY_SIZE] = {truth.position[0], truth.position[1], -truth.position[2], truth.velocity[0], truth.velocity[1], truth.velocity[2]};
    if (tick >= next_gps) {
      if (gps_available) gps_sample(history[(tick + HISTORY_SIZE - scenario.gps_delay_ms) % HISTORY_SIZE], gps_count);
      next_gps += scenario.gps_jitter ? (gps_count % 2 ? 80 : 120) : 100;
      gps_count++;
    }
    if (tick % 10 == 0) {
      // Drain virtual TX; let GPS's normal model-change timeout run.
      ring_buffer_clear(serial_gps.tx_buffer);
      serial_gps.tx_done = true;
      gps_task();
    }
    if (baro_available && tick % 20 == 0)
      baro_test_sample(-truth.position[2] + 0.03 * sin(tick * DT * 2.3), time_millis());
    if (rc_available && tick % 10 == 0) {
      uint16_t channels[RX_CHANNEL_MAX] = {};
      channels[0] = lround((double(roll) + 1) * 0.5 * AUX_VALUE_MAX);
      channels[1] = lround((double(pitch) + 1) * 0.5 * AUX_VALUE_MAX);
      channels[2] = lroundf(throttle * AUX_VALUE_MAX);
      channels[3] = lround((double(yaw) + 1) * 0.5 * AUX_VALUE_MAX);
      channels[4] = arm ? AUX_VALUE_MAX : 0;
      channels[5] = rth ? AUX_VALUE_MAX : 0;
      flags.rx_ready = 1; // Native transport has no RF detection handshake.
      rx_lqi_got_packet();
      simulator_rx_test_frame(channels);
    }
    // SIMULATOR rx_process omits link timeout; use the normal LQI timeout.
    rx_lqi_update(100);
    rx_update();
    truth.imu_sample();
    sixaxis_read();
    imu_calc();
    rx_process();
    control();
    blackbox_capture(); // Logging disabled by AUX; same Flight call order.
    nav_update();
    state.loop_counter++;
    float motors[4];
    for (unsigned i = 0; i < 4; i++) {
      motors[i] = simulator_motor_values[(i + 2) % 4];
      check(motors[i] == MOTOR_OFF || (isfinite(motors[i]) && motors[i] >= 0 && motors[i] <= 1), "invalid motor output");
      if (!flags.arm_state) check(motors[i] == MOTOR_OFF, "disarmed motor running");
      if (flags.in_air && flags.arm_state) check(motors[i] != MOTOR_OFF, "airborne motor stopped");
    }
    truth.step(motors);
    tick++;
    check(isfinite(truth.position[2]) && truth.position[2] <= 0.02 && truth.position[2] > -40, "ground impact or altitude envelope");
    check(truth.rotation[2][2] > 0.5, "tilt exceeds 60 degrees");
    check(hypot(truth.position[0], truth.position[1]) < 250, "position envelope");
  }

  void run(unsigned milliseconds) {
    for (unsigned i = 0; i < milliseconds; i++) step();
  }

  // Scripted pilot uses truth only to hold height through RC during departure.
  // Never runs during automatic flight; it cannot steer or trim RTH outputs.
  void pilot_height(double height) {
    const double acceleration = fmax(-3, fmin(3, 1.5 * (height + truth.position[2]) + 2.0 * truth.velocity[2]));
    const double motor = sqrt(MASS * (GRAVITY + acceleration) / (4 * MAX_THRUST * truth.rotation[2][2]));
    const double idle = 0.0001 + double(profile.motor.digital_idle) * 0.01;
    throttle = (motor - idle) / (1 - idle);
  }

  void depart() {
    run(3000);
    check(!flags.arm_state && state.gps_lock && nav_altitude_ready(), "ground initialization");
    arm = true;
    run(300);
    check(flags.arm_state && nav_rth_home_valid(), "arming and home capture");
    for (unsigned i = 0; i < 4000; i++) {
      pilot_height(scenario.climbing_departure ? 2 + i * DT : 8);
      step();
    }
    check(-truth.position[2] > (scenario.climbing_departure ? 3 : 5), "takeoff");
    pitch = 0.4f;
    for (unsigned i = 0; i < 12000; i++) {
      pilot_height(scenario.climbing_departure ? 6 + i * DT : 8);
      step();
    }
    check(hypot(truth.position[0], truth.position[1]) > 40, "departure distance");
    check(state.heading_confidence > 0.6f, "heading learned from motion");
    const double heading_error = remainder(double(state.heading) - atan2(truth.rotation[1][0], truth.rotation[0][0]) * 180 / M_PI, 360);
    check(fabs(heading_error) < 8, "heading agrees with independent truth");
    check(fabs(double(state.altitude) + truth.position[2]) < 1, "altitude estimate");
    pitch = 0;
    truth.wind[0] = scenario.wind_north;
    truth.wind[1] = scenario.wind_east;
  }

  void start_return() {
    takeover_height = -truth.position[2];
    return_height = takeover_height + double(profile.navigation.rth_altitude);
    rth = true;
    run(100);
    check(state.rth_active, "RTH accepted");
  }

  void finish_return() {
    for (unsigned i = 0; i < 90000; i++) {
      step();
      check(state.rth_active && flags.arm_state, "RTH remains armed and active");
      check(-truth.position[2] > takeover_height - 2 && -truth.position[2] < return_height + 3, "RTH altitude transient bounded");
      if (i >= 80000) {
        check(state.rth_state == RTH_STATE_HOVER_HOME, "sustained home hover");
        check(hypot(truth.position[0], truth.position[1]) < 3, "sustained home position error below 3m");
        check(fabs(-truth.position[2] - return_height) < 1, "sustained home altitude error below 1m");
        check(hypot(truth.velocity[0], truth.velocity[1]) < 1, "settled horizontal speed");
      }
    }
    printf("%s: home error=%.2fm altitude=%.2fm heading confidence=%.2f\n", scenario.name,
        hypot(truth.position[0], truth.position[1]), -truth.position[2], double(state.heading_confidence));
  }
};
} // namespace

void test_navigation_plant_specific_force_and_motor_signs() {
  // A ballistic body has zero specific force; feeding a gravity vector here
  // would silently give the estimator perfect attitude during acceleration.
  plant_t falling;
  falling.position[2] = -10;
  const float stopped[4] = {MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF};
  falling.step(stopped);
  falling.imu_sample();
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, state.accel_raw.roll);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, state.accel_raw.pitch);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, state.accel_raw.yaw);
  TEST_ASSERT_TRUE(falling.velocity[2] > 0);

  plant_t hovering;
  hovering.position[2] = -10;
  float motors[4];
  for (unsigned i = 0; i < 4; i++) {
    motors[i] = sqrt(MASS * GRAVITY / (4 * MAX_THRUST));
    hovering.speed[i] = double(motors[i]);
  }
  for (unsigned i = 0; i < 1000; i++) hovering.step(motors);
  hovering.imu_sample();
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -10, hovering.position[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1, state.accel_raw.yaw);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0, state.gyro.roll);

  // Left-rear thrust produces right roll, nose-down pitch and positive yaw.
  motors[0] += 0.1f;
  hovering.step(motors);
  hovering.imu_sample();
  TEST_ASSERT_TRUE(state.gyro.roll > 0 && state.gyro.pitch > 0 && state.gyro.yaw > 0);
  TEST_ASSERT_TRUE(hovering.speed[0] < double(motors[0])); // Motor response is not instantaneous.
}

void test_navigation_motor_loop_nominal() {
  flight_t flight({"nominal"});
  flight.depart();
  flight.start_return();
  flight.finish_return();
}

void test_navigation_motor_loop_delayed_gps_and_wind() {
  flight_t flight({"delay/wind", 250, true, 2.0, -1.5});
  flight.depart();
  flight.start_return();
  flight.finish_return();
}

void test_navigation_motor_loop_wrong_hover() {
  const float settings[] = {0.2f, 0.75f};
  for (float hover : settings) {
    flight_t flight({hover < 0.5f ? "low hover" : "high hover", 100, true, 0, 0, hover, true});
    flight.depart();
    flight.check(state.hover_throttle == 0, "climbing departure has no learned hover trim");
    flight.start_return();
    flight.finish_return();
  }
}

void test_navigation_motor_loop_sensor_outages() {
  flight_t flight({"sensor outages", 150, true});
  flight.depart();
  flight.start_return();
  for (unsigned i = 0; i < 20000 && state.rth_state != RTH_STATE_NAVIGATE; i++) flight.step();
  flight.check(state.rth_state == RTH_STATE_NAVIGATE, "navigating before outage");
  flight.run(1000);
  for (unsigned sensor = 0; sensor < 2; sensor++) {
    const double height = -flight.truth.position[2];
    flight.gps_available = sensor != 0;
    flight.baro_available = sensor != 1;
    for (unsigned i = 0; i < 2000; i++) {
      flight.step();
      flight.check(state.rth_active && flags.arm_state, "temporary outage preserves RTH");
      flight.check(fabs(-flight.truth.position[2] - height) < 2, "outage altitude hold");
      if (i > 1000) {
        flight.check(state.rx_override.roll == 0 && state.rx_override.pitch == 0 && state.rth_yaw_rate == 0, "stale sensor levels craft");
        flight.check(sensor == 0 ? !nav_gps_ready() : !nav_altitude_ready(), "missing sensor becomes stale");
      }
    }
    flight.gps_available = flight.baro_available = true;
    flight.run(1500);
    flight.check(nav_gps_ready() && nav_altitude_ready() && state.rth_active, "sensor reacquisition");
  }
  flight.finish_return();
}

void test_navigation_motor_loop_rc_loss_and_handback() {
  flight_t flight({"RC recovery", 100, true});
  flight.depart();
  flight.run(100); // Publish centered attitude sticks before disconnecting.
  flight.takeover_height = -flight.truth.position[2];
  flight.return_height = flight.takeover_height + double(profile.navigation.rth_altitude);
  flight.rc_available = false;
  flight.run(2000);
  flight.check(flags.failsafe_signal_lost && state.rth_failsafe_active && flags.controls_override, "RC timeout starts failsafe RTH");
  // Remain disconnected beyond MOTOR_BEEPS_TIMEOUT: airborne failsafe must
  // continue writing motor commands instead of entering the ground beeper.
  flight.finish_return();

  // Deflected sticks on brief link recoveries must not interrupt the return.
  flight.yaw = 0.4f;
  for (unsigned i = 0; i < 3; i++) {
    flight.rc_available = true;
    flight.run(100);
    flight.check(state.rth_failsafe_active, "brief recovered link cannot hand back");
    flight.rc_available = false;
    flight.run(400);
  }
  flight.yaw = 0;
  flight.rc_available = true;
  flight.run(1500);
  flight.check(!flags.failsafe_signal_lost && state.rth_failsafe_active, "stable centered link retains automatic control");

  // A deliberate yaw input acknowledges recovery without commanding lateral
  // tilt. Low throttle exposes an abrupt handback as a motor/height transient.
  const double height = -flight.truth.position[2];
  const float automatic_throttle = state.throttle;
  flight.throttle = 0.12f;
  flight.yaw = 0.4f;
  for (unsigned i = 0; i < 200 && state.rth_active; i++) flight.step();
  flight.check(!state.rth_active && !flags.controls_override && !flags.failsafe && flags.arm_state, "pilot acknowledgement releases override");
  flight.check(fabsf(state.throttle - automatic_throttle) < 0.02f, "handback starts from automatic throttle");
  flight.yaw = 0;
  float previous = state.throttle;
  for (unsigned i = 0; i < 500; i++) {
    flight.step();
    flight.check(fabsf(state.throttle - previous) < 0.01f, "handback throttle slew");
    previous = state.throttle;
    flight.check(fabs(-flight.truth.position[2] - height) < 1, "handback altitude transient");
  }
  flight.check(state.throttle < 0.15f, "pilot throttle ultimately owns output");
  for (unsigned i = 0; i < 5000; i++) { flight.pilot_height(height); flight.step(); }
  flight.check(fabs(-flight.truth.position[2] - height) < 1 && flags.arm_state && !state.rth_active, "pilot recovers height after handback");
  flight.arm = false;
  flight.run(20);
  flight.check(!flags.arm_state, "pilot disarm");
}
