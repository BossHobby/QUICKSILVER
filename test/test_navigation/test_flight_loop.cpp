#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "control/control.h"
#include "control/imu.h"
#include "control/mission.h"
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
// ground effect, vibration, bounce, tip-over or actuator damage. Ground contact
// stops a descent and supports the vehicle; impacts deeper than 2 cm fail. Native sensor/ESC electrical
// drivers, GPS boot negotiation, Blackbox storage and FreeRTOS scheduling are
// not exercised. This is control regression coverage, not flight-safety proof.
// Run: SKIP_TARGETS_CHECKOUT=1 pio test -e multi-test --filter test_navigation

extern float simulator_motor_values[MOTOR_PIN_MAX];
extern serial_port_t serial_gps;
extern void simulator_rx_test_frame(const uint16_t *channels);
extern void baro_test_sample(float altitude, uint32_t timestamp_ms);

namespace {
static constexpr double DT = 0.001;
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
  double ground_tilt = 0; // radians the ground rolls a resting vehicle
  double accel_scale = 1; // accelerometer reading per true specific force

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

  // A slope rolls a resting vehicle onto it, keeping its heading.
  void rest_on_slope() {
    const double yaw = atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
    const double cy = cos(yaw / 2), sy = sin(yaw / 2), cr = cos(ground_tilt / 2), sr = sin(ground_tilt / 2);
    q[0] = cy * cr;
    q[1] = cy * sr;
    q[2] = sy * sr;
    q[3] = sy * cr;
    rotate();
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
    // Flat ground stops a descent at up to 3 g, as landing gear does, without
    // bounce, and supports a stationary vehicle until thrust lifts it.
    if (position[2] >= 0 && velocity[2] > 0) {
      acceleration[2] = -fmin(velocity[2] / DT, 3 * GRAVITY);
    } else if (position[2] >= 0 && velocity[2] >= -1e-9 && acceleration[2] >= 0) {
      for (unsigned i = 0; i < 3; i++) acceleration[i] = velocity[i] = omega[i] = 0;
      if (ground_tilt != 0) rest_on_slope();
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
    const double scale = accel_scale / GRAVITY;
    state.accel_raw = {{float(-force[1] * scale), float(-force[0] * scale), float(-force[2] * scale)}};
  }
};

struct fix_t {
  double north, east, height, vn, ve, vd;
};

static void put_u32(uint8_t *bytes, int32_t value) {
  const uint32_t bits = static_cast<uint32_t>(value);
  for (unsigned i = 0; i < 4; i++) bytes[i] = bits >> (8 * i);
}

// Deterministic pseudo-random sequence, uniform in [-1, 1].
static double noise_sample(unsigned sample, unsigned axis) {
  uint32_t x = sample * 2654435761U + axis * 40503U;
  x ^= x >> 15;
  x *= 2246822519U;
  x ^= x >> 13;
  return x / 2147483647.5 - 1;
}

static void gps_sample(const fix_t &fix, unsigned sample, double velocity_noise) {
  // UBX NAV-PVT, same native serial/parser boundary as test_common/test_gps.
  uint8_t packet[100] = {0xb5, 0x62, 1, 7, 92, 0};
  uint8_t *p = packet + 6;
  put_u32(p, time_millis());
  p[20] = GPS_FIX_3D;
  p[21] = 1;
  p[23] = 12;
  // Uniform noise of this amplitude has a standard deviation of amplitude / sqrt(3).
  const double noise = 0.06 * sin(sample * 1.7);
  const double vn = fix.vn + noise + velocity_noise * noise_sample(sample, 0);
  const double ve = fix.ve - noise + velocity_noise * noise_sample(sample, 1);
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
  double velocity_noise = 0; // m/s, amplitude of uniform GPS velocity noise
  double baro_noise = 0;     // meters, standard deviation of turbulent barometer noise
  double ground_dip = 0;     // meters the barometer reads low with rotors spinning on the ground
  double baro_lag = 0;       // seconds, first-order response of the barometer and its filtering
  double wash_height = 0;    // meters above the ground where the wash begins to build, full on it
};

struct flight_t {
  scenario_t scenario;
  plant_t truth;
  fix_t history[HISTORY_SIZE] = {};
  unsigned tick = 0, next_gps = 0, gps_count = 0;
  bool arm = false, rth = false, mission_switch = false, rc_available = true, gps_available = true, baro_available = true;
  float roll = 0, pitch = 0, yaw = 0, throttle = 0;
  double gps_velocity_bias[2] = {}; // north, east m/s; reported velocity disagrees with position
  double gps_offset[2] = {};        // north, east meters; reported position glitches by this much
  double gps_height_error = 0;      // meters the reported altitude is off
  bool gps_frozen = false;          // the receiver keeps reporting the fix it froze at
  fix_t frozen_fix = {};
  double baro_drift = 0;            // meters the barometer reads high, from temperature or weather
  double return_height = 0, takeover_height = 0;
  double baro_error = 0;
  double baro_wash = 0; // meters the barometer reads low from rotor wash
  double baro_reading = 0;
  char context[512];

  explicit flight_t(scenario_t config) : scenario(config) {
    mock_hardware_reset_all();
    time_test_reset();
    state = {};
    flags = {};
    flags.on_ground = 1;
    target = {};
    mission = {};
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
    profile.receiver.aux[AUX_MISSION] = {static_cast<rx_channel_t>(6), AUX_VALUE_MID, AUX_VALUE_MAX};
    profile.receiver.aux[AUX_PREARM] = {RX_CHANNEL_ON, 0, 0};
    profile.receiver.aux[AUX_LEVELMODE] = {RX_CHANNEL_ON, 0, 0};
    profile.receiver.aux[AUX_IDLE_UP] = {RX_CHANNEL_ON, 0, 0};
    profile.serial.gps = SERIAL_PORT1;
    profile.rate.level_max_angle = 35;
    profile.navigation.rth_altitude = 12;
    profile.navigation.travel_speed = 6;
    profile.navigation.throttle_min = 0.1f;
    profile.navigation.throttle_max = 0.85f;
    profile.navigation.throttle_hover = config.configured_hover;
    profile.navigation.rth_on_failsafe = true;
    state.looptime = DT;
    state.looptime_inverse = 1 / DT;
    state.looptime_autodetect = DT * 1e6;
    state.rx_filter_hz = 50;
    state.lipo_cell_count = 1;
    state.vbat_sag_filtered = 4.0f;
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
        scenario.name, message, tick * DT, unsigned(state.nav_state), unsigned(state.failsafe_phase), unsigned(flags.arm_state), unsigned(flags.arming_disabled_flags),
        truth.position[0], truth.position[1], -truth.position[2], truth.velocity[0], truth.velocity[1], -truth.velocity[2],
        double(state.altitude), double(state.attitude.roll) * 180 / M_PI, double(state.attitude.pitch) * 180 / M_PI, double(state.heading), double(state.heading_confidence), double(state.throttle),
        double(simulator_motor_values[2]), double(simulator_motor_values[3]), double(simulator_motor_values[0]), double(simulator_motor_values[1]));
    TEST_FAIL_MESSAGE(context);
  }

  void step() {
    time_test_advance_us(1000);
    history[tick % HISTORY_SIZE] = {truth.position[0], truth.position[1], -truth.position[2], truth.velocity[0], truth.velocity[1], truth.velocity[2]};
    if (tick >= next_gps) {
      fix_t fix = history[(tick + HISTORY_SIZE - scenario.gps_delay_ms) % HISTORY_SIZE];
      fix.vn += gps_velocity_bias[0];
      fix.ve += gps_velocity_bias[1];
      fix.north += gps_offset[0];
      fix.east += gps_offset[1];
      fix.height += gps_height_error;
      if (!gps_frozen) frozen_fix = fix;
      else fix = frozen_fix;
      if (gps_available) gps_sample(fix, gps_count, scenario.velocity_noise);
      next_gps += scenario.gps_jitter ? (gps_count % 2 ? 80 : 120) : 100;
      gps_count++;
    }
    if (tick % 10 == 0) {
      // Drain virtual TX; let GPS's normal model-change timeout run.
      ring_buffer_clear(serial_gps.tx_buffer);
      serial_gps.tx_done = true;
      gps_task();
    }
    if (baro_available && tick % 20 == 0) {
      // Turbulence wanders the static pressure over about 0.4 s, with a
      // standard deviation of baro_noise.
      baro_error += 0.05 * (scenario.baro_noise * noise_sample(tick / 20, 2) * 11 - baro_error);
      // Rotor wash on the ground raises the static pressure, as logged: it
      // builds within 0.4 s of spin-up or touchdown, none shows during a
      // descent, and it clears within a second of lift-off.
      const double spin = (truth.speed[0] + truth.speed[1] + truth.speed[2] + truth.speed[3]) / 4;
      const double wash_depth = truth.position[2] + 0.05 + scenario.wash_height;
      const double wash_share = scenario.wash_height > 0 ? fmin(1, wash_depth / scenario.wash_height) : 1;
      const double wash = wash_depth > 0 ? scenario.ground_dip * fmin(1, spin / 0.3) * wash_share : 0;
      baro_wash += (1 - exp(-0.02 / 0.3)) * (wash - baro_wash);
      const double pressure_height = -truth.position[2] + 0.03 * sin(tick * DT * 2.3) + baro_error - baro_wash + baro_drift;
      baro_reading += (scenario.baro_lag > 0 ? 1 - exp(-0.02 / scenario.baro_lag) : 1) * (pressure_height - baro_reading);
      baro_test_sample(baro_reading, time_millis());
    }
    if (rc_available && tick % 10 == 0) {
      uint16_t channels[RX_CHANNEL_MAX] = {};
      channels[0] = lround((double(roll) + 1) * 0.5 * AUX_VALUE_MAX);
      channels[1] = lround((double(pitch) + 1) * 0.5 * AUX_VALUE_MAX);
      channels[2] = lroundf(throttle * AUX_VALUE_MAX);
      channels[3] = lround((double(yaw) + 1) * 0.5 * AUX_VALUE_MAX);
      channels[4] = arm ? AUX_VALUE_MAX : 0;
      channels[5] = rth ? AUX_VALUE_MAX : 0;
      channels[6] = mission_switch ? AUX_VALUE_MAX : 0;
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
    check(!flags.arm_state && state.gps_health.lock && flags.altitude_valid, "ground initialization");
    arm = true;
    run(300);
    check(flags.arm_state && flags.home_valid, "arming and home capture");
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
    check(state.nav_state >= NAV_STATE_CLIMB, "RTH accepted");
  }

  void finish_return() {
    for (unsigned i = 0; i < 90000; i++) {
      step();
      check(state.nav_state >= NAV_STATE_CLIMB && flags.arm_state, "RTH remains armed and active");
      check(-truth.position[2] > takeover_height - 2 && -truth.position[2] < return_height + 3, "RTH altitude transient bounded");
      if (i >= 80000) {
        check(state.nav_state == NAV_STATE_HOLD, "sustained home hover");
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

void test_navigation_motor_loop_brakes_in_wind() {
  // Headwind on the way home: cruise drag needs more than half the tilt limit.
  flight_t flight({"brake/wind", 100, true, 5.0, 2.0});
  profile.rate.level_max_angle = 30;
  flight.depart();
  flight.start_return();
  const double start[2] = {flight.truth.position[0], flight.truth.position[1]};
  // Braking from the departure speed at the 2 m/s^2 navigation limit; leveling
  // instead drifts downwind until navigation fails.
  const double takeover_speed = hypot(flight.truth.velocity[0], flight.truth.velocity[1]);
  const double stopping_distance = takeover_speed * takeover_speed / (2 * 2.0);
  double drift = 0, speed_error = 0;
  unsigned cruise_samples = 0;
  for (unsigned i = 0; i < 60000 && state.nav_state != NAV_STATE_HOLD; i++) {
    const double speed = hypot(flight.truth.velocity[0], flight.truth.velocity[1]);
    flight.step();
    if (state.nav_state == NAV_STATE_CLIMB || state.nav_state == NAV_STATE_TURN)
      drift = fmax(drift, hypot(flight.truth.position[0] - start[0], flight.truth.position[1] - start[1]));
    const double distance = hypot(flight.truth.position[0], flight.truth.position[1]);
    if (state.nav_state == NAV_STATE_NAVIGATE && distance > 30 && distance < 50) {
      speed_error += double(profile.navigation.travel_speed) - speed;
      cruise_samples++;
    }
  }
  flight.check(cruise_samples > 1000, "cruise segment flown");
  speed_error /= cruise_samples;
  printf("%s: climb/turn drift=%.2fm stopping=%.2fm cruise speed error=%.2fm/s\n", flight.scenario.name,
      drift, stopping_distance, speed_error);
  // Beyond the stopping distance: the jerk-limited onset of the brake and the
  // velocity loop's response, then the hold settles where the brake ended.
  flight.check(drift < stopping_distance + 6, "climb and turn hold position in wind");
  flight.check(fabs(speed_error) < 0.25, "cruise trim reaches the commanded speed");
  flight.finish_return();
}

void test_navigation_motor_loop_guided_hold_stops_smoothly() {
  flight_t flight({"guided hold", 100, true, 1.0, -1.0});
  flight.depart();
  // The mission switch grants guided flight; without a mission it flies nothing yet.
  flight.mission_switch = true;
  flight.run(100);
  flight.check(flags.guided_ready && state.nav_state < NAV_STATE_CLIMB, "guided flight granted");
  nav_command_t command = {.type = NAV_COMMAND_GOTO, .id = 1};
  // Home is the 0/0 origin, which reads as an unset waypoint: aim 1 m north of it.
  command.waypoint = {.coord = {.lon = 0, .lat = 90}, .altitude = int16_t(lroundf(state.altitude)), .speed = 60}; // 6 m/s
  flight.check(nav_command_apply(&command) == NAV_RESULT_OK, "GOTO accepted");
  for (unsigned i = 0; i < 30000 && !(state.nav_state == NAV_STATE_NAVIGATE && hypot(flight.truth.velocity[0], flight.truth.velocity[1]) > 5.5); i++)
    flight.step();
  flight.check(state.nav_state == NAV_STATE_NAVIGATE, "cruising toward the GOTO target");

  const double stop_from[2] = {flight.truth.position[0], flight.truth.position[1]};
  const double speed = hypot(flight.truth.velocity[0], flight.truth.velocity[1]);
  command = {.type = NAV_COMMAND_HOLD, .id = 2};
  flight.check(nav_command_apply(&command) == NAV_RESULT_OK, "HOLD accepted");
  double travel = 0;
  for (unsigned i = 0; i < 20000; i++) {
    flight.step();
    flight.check(state.nav_target == NAV_TARGET_GUIDED &&
                     (state.nav_state == NAV_STATE_BRAKE || state.nav_state == NAV_STATE_HOLD),
                 "HOLD hovers in place");
    flight.check(flight.truth.rotation[2][2] > cos(25 * M_PI / 180), "HOLD brakes without a hard reversal");
    travel = fmax(travel, hypot(flight.truth.position[0] - stop_from[0], flight.truth.position[1] - stop_from[1]));
  }
  const double rest = hypot(flight.truth.position[0] - stop_from[0], flight.truth.position[1] - stop_from[1]);
  printf("%s: speed=%.2fm/s travel=%.2fm rest=%.2fm\n", flight.scenario.name, speed, travel, rest);
  // It holds where the brake ends instead of flying back to where HOLD began.
  // The 2 m/s^2 brake profile plus the velocity loop's response time.
  flight.check(travel < speed * speed / (2 * 2.0) + speed * 1.5, "HOLD stopping distance");
  flight.check(travel - rest < 3, "HOLD keeps the point where it stopped");
  flight.check(hypot(flight.truth.velocity[0], flight.truth.velocity[1]) < 0.5, "HOLD settles");
}

void test_navigation_motor_loop_goto_flies_smoothly() {
  flight_t flight({"goto smooth", 100, true, 1.0, -0.5});
  flight.depart();
  flight.mission_switch = true;
  flight.run(100);
  nav_command_t command = {.type = NAV_COMMAND_HOLD, .id = 1};
  flight.check(nav_command_apply(&command) == NAV_RESULT_OK, "HOLD accepted");
  flight.run(15000);
  flight.check(state.nav_state == NAV_STATE_HOLD, "hovering before the GOTO");

  // 80 m straight ahead of where it hovers, from a standstill to a stop.
  const double from[2] = {flight.truth.position[0], flight.truth.position[1]};
  const double to[2] = {from[0] + 80, from[1]};
  command = {.type = NAV_COMMAND_GOTO, .id = 2};
  command.waypoint = {.coord = {.lon = int32_t(lround(to[1] * 1e7 / 111194.93)), .lat = int32_t(lround(to[0] * 1e7 / 111194.93))},
                      .altitude = int16_t(lroundf(state.altitude)), .speed = 80}; // 8 m/s
  flight.check(nav_command_apply(&command) == NAV_RESULT_OK, "GOTO accepted");

  double overshoot = 0, top_speed = 0, tilt_rate = 0, previous_tilt = NAN;
  unsigned arrived = 0;
  for (unsigned i = 0; i < 60000; i++) {
    flight.step();
    const double tilt = acos(fmin(1.0, flight.truth.rotation[2][2]));
    if (i % 20 == 0) {
      // Over 20 ms windows; the attitude loop itself is much faster.
      if (!isnan(previous_tilt)) tilt_rate = fmax(tilt_rate, fabs(tilt - previous_tilt) / 0.02);
      previous_tilt = tilt;
    }
    overshoot = fmax(overshoot, flight.truth.position[0] - to[0]);
    top_speed = fmax(top_speed, hypot(flight.truth.velocity[0], flight.truth.velocity[1]));
    if (!arrived && hypot(flight.truth.position[0] - to[0], flight.truth.position[1] - to[1]) < 1 &&
        hypot(flight.truth.velocity[0], flight.truth.velocity[1]) < 0.3)
      arrived = i;
  }
  printf("%s: arrived after %.1fs top speed=%.2fm/s overshoot=%.2fm peak tilt rate=%.0fdeg/s\n", flight.scenario.name,
      arrived * 1e-3, top_speed, overshoot, tilt_rate * 180 / M_PI);
  // Shaped acceleration flown as tilt: 21.5 s and no overshoot before, when
  // the velocity loop's lag set a long, creeping approach.
  flight.check(arrived && arrived < 20000, "settles on the GOTO target");
  flight.check(overshoot < 0.75, "stops on the target without overshooting");
}

void test_navigation_motor_loop_climb_holds_position() {
  flight_t flight({"climb hold", 100, true, 1.5, -1.0});
  flight.depart();
  // GPS velocity reads 0.6 m/s east of the position track; a zero-velocity
  // loop alone would drift 0.6 m/s west for the whole climb.
  flight.gps_velocity_bias[1] = 0.6;
  const double origin[2] = {flight.truth.position[0], flight.truth.position[1]};
  const double climb_to = -flight.truth.position[2] + 25;
  mission = {.count = 1, .end_action = MISSION_END_HOLD};
  mission.waypoints[0] = {
      .coord = {.lon = int32_t(lround(origin[1] * 1e7 / 111194.93)),
                .lat = int32_t(lround((origin[0] + 60) * 1e7 / 111194.93))},
      .altitude = int16_t(lround(climb_to)),
  };
  flight.mission_switch = true;
  flight.run(100);
  flight.check(state.nav_target == NAV_TARGET_MISSION && state.nav_state == NAV_STATE_CLIMB, "mission climbs in place");

  // Measure from where the brake has stopped the vehicle.
  for (unsigned i = 0; i < 10000 && hypot(flight.truth.velocity[0], flight.truth.velocity[1]) > 0.3; i++)
    flight.step();
  const double rest[2] = {flight.truth.position[0], flight.truth.position[1]};
  double wander = 0;
  unsigned climb_ms = 0;
  while (state.nav_state == NAV_STATE_CLIMB || state.nav_state == NAV_STATE_TURN) {
    flight.step();
    flight.check(++climb_ms < 30000, "climb and turn finish");
    wander = fmax(wander, hypot(flight.truth.position[0] - rest[0], flight.truth.position[1] - rest[1]));
  }
  printf("%s: climb and turn %.1fs, wander=%.2fm\n", flight.scenario.name, climb_ms * 1e-3, wander);
  flight.check(state.nav_state == NAV_STATE_NAVIGATE, "navigates after the climb");
  flight.check(wander < 3, "climb and turn hold the point where the brake stopped");
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
  for (unsigned i = 0; i < 20000 && state.nav_state != NAV_STATE_NAVIGATE; i++) flight.step();
  flight.check(state.nav_state == NAV_STATE_NAVIGATE, "navigating before outage");
  flight.run(1000);
  for (unsigned sensor = 0; sensor < 2; sensor++) {
    const double height = -flight.truth.position[2];
    flight.gps_available = sensor != 0;
    flight.baro_available = sensor != 1;
    for (unsigned i = 0; i < 2000; i++) {
      flight.step();
      flight.check(state.nav_state >= NAV_STATE_CLIMB && flags.arm_state, "temporary outage preserves RTH");
      flight.check(fabs(-flight.truth.position[2] - height) < 2, "outage altitude hold");
      if (i > 1000) {
        flight.check(state.rx_override.roll == 0 && state.rx_override.pitch == 0 && state.nav_yaw_rate == 0, "stale sensor levels craft");
        flight.check(sensor == 0 ? !flags.gps_valid : !flags.altitude_valid, "missing sensor becomes stale");
      }
    }
    flight.gps_available = flight.baro_available = true;
    flight.run(1500);
    flight.check(flags.gps_valid && flags.altitude_valid && state.nav_state >= NAV_STATE_CLIMB, "sensor reacquisition");
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
  flight.check(flags.failsafe_signal_lost && state.failsafe_owner == FAILSAFE_OWNER_RTH && flags.controls_override, "RC timeout starts failsafe RTH");
  // Remain disconnected beyond MOTOR_BEEPS_TIMEOUT: airborne failsafe must
  // continue writing motor commands instead of entering the ground beeper.
  flight.finish_return();

  // Deflected sticks on brief link recoveries must not interrupt the return.
  flight.yaw = 0.4f;
  for (unsigned i = 0; i < 3; i++) {
    flight.rc_available = true;
    flight.run(100);
    flight.check(state.failsafe_owner == FAILSAFE_OWNER_RTH, "brief recovered link cannot hand back");
    flight.rc_available = false;
    flight.run(400);
  }
  flight.yaw = 0;
  flight.rc_available = true;
  flight.run(1500);
  flight.check(!flags.failsafe_signal_lost && state.failsafe_owner == FAILSAFE_OWNER_RTH, "stable centered link retains automatic control");

  // A deliberate yaw input acknowledges recovery without commanding lateral
  // tilt. Low throttle exposes an abrupt handback as a motor/height transient.
  const double height = -flight.truth.position[2];
  const float automatic_throttle = state.throttle;
  flight.throttle = 0.12f;
  flight.yaw = 0.4f;
  for (unsigned i = 0; i < 200 && state.nav_state >= NAV_STATE_CLIMB; i++) flight.step();
  flight.check(state.nav_state < NAV_STATE_CLIMB && !flags.controls_override && !flags.failsafe && flags.arm_state, "pilot acknowledgement releases override");
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
  flight.check(fabs(-flight.truth.position[2] - height) < 1 && flags.arm_state && state.nav_state < NAV_STATE_CLIMB, "pilot recovers height after handback");
  flight.arm = false;
  flight.run(20);
  flight.check(!flags.arm_state, "pilot disarm");
}

void test_navigation_motor_loop_mission() {
  flight_t flight({"mission", 100, true, 1.0, 0.5});
  flight.depart();
  // A square leg sequence from the departure point, changing altitude on the way.
  const double origin[2] = {flight.truth.position[0], flight.truth.position[1]};
  const double route[3][4] = {{30, 0, 11, 0}, {30, 30, 14, 0}, {0, 30, 10, 3}}; // north, east, altitude, speed m/s
  mission = {.count = 3, .end_action = MISSION_END_HOLD};
  for (unsigned i = 0; i < 3; i++) {
    mission.waypoints[i] = {
        .coord = {.lon = int32_t(lround((origin[1] + route[i][1]) * 1e7 / 111194.93)),
                  .lat = int32_t(lround((origin[0] + route[i][0]) * 1e7 / 111194.93))},
        .altitude = int16_t(route[i][2]),
        .speed = uint8_t(route[i][3] * 10),
    };
  }
  const double takeover_height = -flight.truth.position[2];
  flight.mission_switch = true;
  flight.run(100);
  flight.check(state.nav_target == NAV_TARGET_MISSION && state.nav_state >= NAV_STATE_CLIMB, "mission accepted");

  // Each waypoint is reached before the next becomes active.
  double closest[3] = {1e9, 1e9, 1e9};
  // Altitude halfway along the legs between waypoints, while still descending or climbing.
  double halfway[3] = {NAN, NAN, NAN};
  for (unsigned i = 0; i < 90000; i++) {
    flight.step();
    flight.check(state.nav_target == NAV_TARGET_MISSION && flags.arm_state, "mission remains armed and active");
    const unsigned index = state.mission_index;
    flight.check(index < 3, "waypoint index stays within the route");
    const double north = flight.truth.position[0] - origin[0] - route[index][0];
    const double east = flight.truth.position[1] - origin[1] - route[index][1];
    closest[index] = fmin(closest[index], hypot(north, east));
    if (index > 0 && isnan(halfway[index]) && hypot(north, east) < 15) halfway[index] = -flight.truth.position[2];
    if (index > 0) flight.check(closest[index - 1] < 4, "previous waypoint reached before advancing");
    if (index == 1 && state.nav_state == NAV_STATE_NAVIGATE && fabs(east + route[1][1] / 2) < 0.5)
      flight.check(fabs(-flight.truth.position[2] - (route[0][2] + route[1][2]) / 2) < 1, "altitude changes evenly along the leg");
    flight.check(-flight.truth.position[2] > fmin(takeover_height, 10) - 2 && -flight.truth.position[2] < 16, "mission altitude bounded");
    if (i >= 80000) {
      flight.check(index == 2 && state.nav_state == NAV_STATE_HOLD, "holds the last waypoint");
      flight.check(hypot(north, east) < 3, "sustained last waypoint position error below 3m");
      flight.check(fabs(-flight.truth.position[2] - route[2][2]) < 1, "sustained last waypoint altitude error below 1m");
    }
  }
  printf("%s: last waypoint error=%.2fm altitude=%.2fm closest=(%.2f,%.2f,%.2f) halfway=(%.2f,%.2f)\n", flight.scenario.name,
      closest[2], -flight.truth.position[2], closest[0], closest[1], closest[2], halfway[1], halfway[2]);
  for (unsigned i = 1; i < 3; i++)
    flight.check(fabs(halfway[i] - 0.5 * (route[i - 1][2] + route[i][2])) < 1, "altitude changes over the whole leg");

  // Switching the mission off hands control back to the pilot.
  flight.mission_switch = false;
  flight.run(100);
  flight.check(state.nav_state < NAV_STATE_CLIMB && !flags.controls_override && flags.arm_state, "mission switch hands back");
}

void test_navigation_motor_loop_wrong_heading() {
  // Acro flying can leave the integrated heading far off while its confidence
  // still reads as usable. Turn the airframe without a gyro reading to model it.
  const struct { scenario_t scenario; double error_deg; } cases[] = {
      {{"wrong heading", 100, true}, 100},
      {{"reversed heading/wind", 250, true, 2.0, -1.5}, 170},
  };
  for (const auto &test : cases) {
    flight_t flight(test.scenario);
    flight.depart();
    const double half = test.error_deg * M_PI / 360;
    const double c = cos(half), s = sin(half);
    const double q[4] = {flight.truth.q[0], flight.truth.q[1], flight.truth.q[2], flight.truth.q[3]};
    flight.truth.q[0] = c * q[0] - s * q[3];
    flight.truth.q[1] = c * q[1] - s * q[2];
    flight.truth.q[2] = c * q[2] + s * q[1];
    flight.truth.q[3] = c * q[3] + s * q[0];
    flight.truth.rotate();
    flight.start_return();
    const double start[2] = {flight.truth.position[0], flight.truth.position[1]};
    bool acquired = false;
    double stray = 0;
    for (unsigned i = 0; i < 40000 && !(acquired && state.nav_state == NAV_STATE_NAVIGATE); i++) {
      flight.step();
      acquired |= state.nav_state == NAV_STATE_ACQUIRE_HEADING;
      stray = fmax(stray, hypot(flight.truth.position[0] - start[0], flight.truth.position[1] - start[1]));
    }
    printf("%s: stray before relearning=%.2fm\n", flight.scenario.name, stray);
    flight.check(acquired && state.nav_state == NAV_STATE_NAVIGATE, "diverging return relearns the heading");
    flight.check(stray < 130, "divergence caught before flying far off");
    flight.finish_return();
  }
}

void test_navigation_motor_loop_gusty_cruise() {
  // Navigation optimizes for the distance closing at a steady rate: gusts may
  // move the speed, but should not pitch the vehicle back and forth.
  // Real receivers report velocity with about 0.2 m/s of noise, and the
  // barometer reads turbulence in forward flight.
  flight_t flight({"gusty cruise", 100, true, 0, 0, 0.5f, false, 0.35, 0.4});
  flight.depart();
  flight.mission_switch = true;
  flight.run(100);
  nav_command_t command = {.type = NAV_COMMAND_GOTO, .id = 1};
  // Waypoints store whole meters.
  const int16_t altitude = int16_t(lroundf(state.altitude));
  command.waypoint = {.coord = {.lon = int32_t(lround(-70 * 1e7 / 111194.93)), .lat = int32_t(lround(-70 * 1e7 / 111194.93))},
                      .altitude = altitude};
  flight.check(nav_command_apply(&command) == NAV_RESULT_OK, "GOTO accepted");

  double previous_distance = -1, closing_sum = 0, closing_square = 0, pitch_sum = 0, pitch_square = 0, top_speed = 0;
  double throttle_sum = 0, throttle_square = 0, altitude_square = 0, altitude_worst = 0;
  unsigned samples = 0, cruising = 0;
  for (unsigned i = 0; i < 90000 && state.nav_state != NAV_STATE_HOLD; i++) {
    const double t = flight.tick * DT;
    // Two incommensurate gust periods around a steady crosswind.
    flight.truth.wind[0] = 1.5 * sin(0.7 * t) + 0.8 * sin(1.9 * t + 1);
    flight.truth.wind[1] = 1.0 + 1.2 * sin(0.5 * t + 2);
    flight.step();
    const double distance = hypot(flight.truth.position[0] + 70, flight.truth.position[1] + 70);
    // Cruise begins once the leg has had time to reach its speed.
    if (state.nav_state == NAV_STATE_NAVIGATE && !cruising) cruising = i + 6000;
    if (state.nav_state != NAV_STATE_NAVIGATE || i < cruising || distance < 40 || flight.tick % 100) continue;
    if (previous_distance > 0 && distance < 220) {
      const double closing = (previous_distance - distance) / 0.1;
      const double pitch = double(state.rx_override.pitch) * profile.rate.level_max_angle;
      closing_sum += closing;
      closing_square += closing * closing;
      pitch_sum += pitch;
      pitch_square += pitch * pitch;
      top_speed = fmax(top_speed, hypot(flight.truth.velocity[0], flight.truth.velocity[1]));
      throttle_sum += double(state.throttle);
      throttle_square += double(state.throttle) * double(state.throttle);
      const double altitude_error = -flight.truth.position[2] - altitude;
      altitude_square += altitude_error * altitude_error;
      altitude_worst = fmax(altitude_worst, fabs(altitude_error));
      samples++;
    }
    previous_distance = distance;
  }
  flight.check(samples > 200, "cruise segment flown");
  const auto spread = [samples](double sum, double square) {
    const double mean = sum / samples;
    return sqrt(fmax(square / samples - mean * mean, 0));
  };
  const double pitch_spread = spread(pitch_sum, pitch_square);
  printf("%s: closing=%.2f+-%.2fm/s pitch=%.1f+-%.2fdeg top speed=%.2fm/s throttle=+-%.3f altitude rms=%.2fm worst=%.2fm\n",
      flight.scenario.name, closing_sum / samples, spread(closing_sum, closing_square), pitch_sum / samples, pitch_spread,
      top_speed, spread(throttle_sum, throttle_square), sqrt(altitude_square / samples), altitude_worst);
  flight.check(top_speed < double(profile.navigation.travel_speed) + 2, "gusts do not run away with the speed");
  // Trimming the cruise speed at the full integral rate reaches 1.5 degrees here.
  flight.check(pitch_spread < 1.45, "cruise pitch rides out gusts");
  flight.check(altitude_worst < 1, "altitude holds within a meter");
}

static mission_waypoint_t flight_waypoint(double north, double east, int16_t altitude, uint8_t type) {
  return {
      .coord = {.lon = int32_t(lround(east * 1e7 / 111194.93)), .lat = int32_t(lround(north * 1e7 / 111194.93))},
      .altitude = altitude,
      .type = type,
  };
}

// Distance from the line through a and b, meters.
static double flight_cross_track(const double *a, const double *b, double north, double east) {
  const double leg[2] = {b[0] - a[0], b[1] - a[1]};
  return fabs(leg[0] * (east - a[1]) - leg[1] * (north - a[0])) / hypot(leg[0], leg[1]);
}

void test_navigation_motor_loop_pass_mission() {
  flight_t flight({"pass", 100, true, 1.0, 0.5});
  flight.depart();
  // Two 90 degree corners flown through, then a stop on the last waypoint.
  const double origin[2] = {flight.truth.position[0], flight.truth.position[1]};
  const double route[3][2] = {{origin[0] + 50, origin[1]}, {origin[0] + 50, origin[1] + 50}, {origin[0], origin[1] + 50}};
  const uint8_t types[3] = {MISSION_WAYPOINT_PASS, MISSION_WAYPOINT_PASS, MISSION_WAYPOINT_STOP};
  mission = {.count = 3, .end_action = MISSION_END_HOLD};
  for (unsigned i = 0; i < 3; i++)
    mission.waypoints[i] = flight_waypoint(route[i][0], route[i][1], 10, types[i]);
  flight.mission_switch = true;
  flight.run(100);
  flight.check(state.nav_target == NAV_TARGET_MISSION && state.nav_state >= NAV_STATE_CLIMB, "mission accepted");

  double corner_speed[2] = {1e9, 1e9}; // slowest within 10 m of each corner
  double cross_track = 0;              // worst after settling onto each later leg
  double corner_miss[2] = {1e9, 1e9};  // closest approach to each corner
  for (unsigned i = 0; i < 80000; i++) {
    flight.step();
    flight.check(state.nav_target == NAV_TARGET_MISSION && flags.arm_state, "mission remains armed and active");
    const unsigned index = state.mission_index;
    const double north = flight.truth.position[0], east = flight.truth.position[1];
    const double speed = hypot(flight.truth.velocity[0], flight.truth.velocity[1]);
    for (unsigned c = 0; c < 2; c++)
      corner_miss[c] = fmin(corner_miss[c], hypot(north - route[c][0], east - route[c][1]));
    if (index > 0) {
      flight.check(state.nav_state != NAV_STATE_TURN, "corners are flown through without stopping to turn");
      const double from_corner = hypot(north - route[index - 1][0], east - route[index - 1][1]);
      if (index < 3 && from_corner < 10)
        corner_speed[index - 1] = fmin(corner_speed[index - 1], speed);
      // Measured once the corner is behind and before the final stop.
      if (from_corner > 25 && hypot(north - route[index][0], east - route[index][1]) > 10)
        cross_track = fmax(cross_track, flight_cross_track(route[index - 1], route[index], north, east));
    }
    if (i >= 70000) {
      flight.check(index == 2 && state.nav_state == NAV_STATE_HOLD, "holds the last waypoint");
      flight.check(hypot(north - route[2][0], east - route[2][1]) < 3, "stops on the last waypoint");
    }
  }
  printf("%s: corner speed=(%.2f,%.2f)m/s corner miss=(%.2f,%.2f)m cross-track=%.2fm\n", flight.scenario.name,
      corner_speed[0], corner_speed[1], corner_miss[0], corner_miss[1], cross_track);
  for (double speed : corner_speed)
    flight.check(speed > 1.5 && speed < 1e9, "corner speed carried through the waypoint");
  flight.check(cross_track < 5, "later legs follow the line between waypoints");
}

// Ground start through a mission TAKEOFF to the first waypoint at cruise
// meters, ending in a LAND.
static void flight_takeoff_mission(flight_t &flight, int16_t cruise = 10) {
  flight.run(3000);
  flight.check(!flags.arm_state && state.gps_health.lock && flags.altitude_valid, "ground initialization");
  mission = {.count = 3, .end_action = MISSION_END_HOLD};
  mission.waypoints[0] = flight_waypoint(1, 1, 8, MISSION_WAYPOINT_TAKEOFF);
  mission.waypoints[1] = flight_waypoint(60, 20, cruise, MISSION_WAYPOINT_STOP);
  mission.waypoints[2] = flight_waypoint(40, 50, 0, MISSION_WAYPOINT_LAND);
  flight.arm = true;
  flight.run(300);
  flight.check(flags.arm_state && flags.home_valid, "arming and home capture");

  // The takeoff climbs where the craft stands: only at its waypoint.
  const mission_waypoint_t takeoff = mission.waypoints[0];
  mission.waypoints[0] = flight_waypoint(20, 0, 8, MISSION_WAYPOINT_TAKEOFF);
  flight.mission_switch = true;
  flight.run(500);
  flight.check(state.nav_state < NAV_STATE_CLIMB && !flags.controls_override, "no takeoff away from its waypoint");
  flight.mission_switch = false;
  flight.run(100);
  mission.waypoints[0] = takeoff;

  flight.mission_switch = true;
  flight.run(500);
  flight.check(state.nav_state == NAV_STATE_TAKEOFF_READY && state.nav_target == NAV_TARGET_MISSION, "takeoff waits for the throttle stick");
  flight.check(state.throttle == 0 && flight.truth.position[2] >= 0, "motors stay stopped on the ground");
  flight.throttle = 0.5f;
}

void test_navigation_motor_loop_takeoff_and_land() {
  const scenario_t scenarios[] = {
      {"takeoff/land", 100, true, 1.0, -0.5},
      {"takeoff/land in ground effect", 100, true, 1.0, -0.5, 0.5f, false, 0, 0, 4},
  };
  for (const auto &scenario : scenarios) {
  flight_t flight(scenario);
  flight_takeoff_mission(flight);

  bool lifted = false, landing = false;
  double climb_top = 0, touchdown_speed = -1, flight_error = 0, landing_error = 0;
  unsigned i = 0;
  for (; i < 150000 && flags.arm_state; i++) {
    flight.step();
    const double height = -flight.truth.position[2];
    double &altitude_error = landing ? landing_error : flight_error;
    if (flags.altitude_valid)
      altitude_error = fmax(altitude_error, fabs(double(state.altitude) - height));
    lifted = lifted || height > 0.5;
    if (state.mission_index == 0 && lifted)
      climb_top = fmax(climb_top, height);
    landing = landing || state.nav_state == NAV_STATE_LAND;
    if (landing && touchdown_speed < 0 && height < 0.01)
      touchdown_speed = flight.truth.velocity[2];
    if (flags.arm_state && !landing)
      flight.check(state.nav_target == NAV_TARGET_MISSION, "mission flies until the landing");
  }
  const double north = flight.truth.position[0], east = flight.truth.position[1];
  printf("%s: climb top=%.2fm touchdown=%.2fm/s disarmed after %.1fs at (%.2f,%.2f) altitude error=%.2fm, landing %.2fm\n",
      flight.scenario.name, climb_top, touchdown_speed, i * 1e-3, north, east, flight_error, landing_error);
  flight.check(flight_error < 1, "altitude estimate within a meter from takeoff until the landing");
  flight.check(lifted && fabs(climb_top - 8) < 1.5, "takeoff climbs in place to its altitude");
  flight.check(landing && !flags.arm_state, "touchdown disarms");
  flight.check(touchdown_speed >= 0 && touchdown_speed < 0.8, "slow final descent");
  flight.check(hypot(north - 40, east - 50) < 3, "lands on the LAND waypoint");

  // The raised arm switch does not arm again.
  flight.throttle = 0;
  flight.run(500);
  flight.check(!flags.arm_state, "re-arming needs the arm switch cycled");
  flight.check(state.nav_state < NAV_STATE_CLIMB && !flags.controls_override, "navigation released after disarm");
  }
}

// A barometer in the craft's own wash wanders and lags through the descent.
// That is neither the ground nor a stop: the landing keeps descending, slows
// for the ground and touches down softly.
void test_navigation_motor_loop_land_with_disturbed_barometer() {
  const scenario_t scenarios[] = {
      {.name = "land, barometer wanders 0.5m", .gps_delay_ms = 100, .gps_jitter = true, .baro_noise = 0.5, .baro_lag = 0.2},
      {.name = "land, barometer wanders 1m", .gps_delay_ms = 100, .gps_jitter = true, .baro_noise = 1.0, .baro_lag = 0.2},
      {.name = "land, barometer wanders 1m over wash", .gps_delay_ms = 100, .gps_jitter = true, .baro_noise = 1.0, .ground_dip = 4, .baro_lag = 0.3},
  };
  for (const auto &scenario : scenarios) {
    flight_t flight(scenario);
    flight_takeoff_mission(flight, 20);

    bool landing = false;
    double touchdown_speed = -1, landing_error = 0, slowest = 10, stalled = 0, longest_stall = 0;
    unsigned i = 0, landing_start = 0;
    for (; i < 200000 && flags.arm_state; i++) {
      flight.step();
      const double height = -flight.truth.position[2];
      if (!landing && state.nav_state == NAV_STATE_LAND) {
        landing = true;
        landing_start = i;
      }
      if (!landing)
        continue;
      if (touchdown_speed < 0 && height < 0.01)
        touchdown_speed = flight.truth.velocity[2];
      if (touchdown_speed >= 0 || i - landing_start < 3000)
        continue;
      // Descending, clear of the ground.
      landing_error = fmax(landing_error, fabs(double(state.altitude) - height));
      slowest = fmin(slowest, flight.truth.velocity[2]);
      stalled = flight.truth.velocity[2] < 0.15 ? stalled + DT : 0;
      longest_stall = fmax(longest_stall, stalled);
    }
    printf("%s: touchdown=%.2fm/s slowest descent=%.2fm/s longest stall=%.2fs altitude error=%.2fm disarmed after %.1fs\n",
        flight.scenario.name, touchdown_speed, slowest, longest_stall, landing_error, i * 1e-3);
    flight.check(landing && !flags.arm_state, "touchdown disarms");
    flight.check(touchdown_speed >= 0 && touchdown_speed < 1.0, "soft touchdown");
    flight.check(longest_stall < 0.5, "the descent does not stop in the air");
    // No worse than the barometer itself: three standard deviations and its lag.
    flight.check(landing_error < 3 * scenario.baro_noise + 0.5, "altitude estimate follows the descent");
  }
}

void test_navigation_motor_loop_takeoff_stick_abort() {
  flight_t flight({"takeoff abort", 100, true});
  flight_takeoff_mission(flight);
  for (unsigned i = 0; i < 20000 && -flight.truth.position[2] < 4; i++) flight.step();
  flight.check(state.nav_state == NAV_STATE_TAKEOFF, "climbing out of the takeoff");

  // Roll hands control back from the climb, at the raised throttle stick.
  const double height = -flight.truth.position[2];
  flight.roll = 0.3f;
  flight.run(100);
  flight.check(state.nav_state < NAV_STATE_CLIMB && !flags.controls_override && flags.arm_state, "stick takes control back");
  flight.roll = 0;
  double lowest = height;
  for (unsigned i = 0; i < 3000; i++) {
    flight.pilot_height(height);
    flight.step();
    lowest = fmin(lowest, -flight.truth.position[2]);
  }
  printf("%s: handback at %.2fm, lowest %.2fm\n", flight.scenario.name, height, lowest);
  flight.check(lowest > height - 1.5, "handback does not drop the craft");

  // A takeoff runs once per arm: the mission switch cycled in the air climbs
  // in place to the takeoff altitude and continues the route.
  flight.mission_switch = false;
  flight.run(100);
  flight.mission_switch = true;
  flight.run(100);
  flight.check(state.nav_target == NAV_TARGET_MISSION && state.nav_state != NAV_STATE_TAKEOFF_READY &&
                   state.nav_state != NAV_STATE_TAKEOFF_SPINUP && state.nav_state != NAV_STATE_TAKEOFF,
               "the restarted mission climbs in place instead of taking off again");
  for (unsigned i = 0; i < 20000 && state.mission_index == 0; i++) flight.step();
  flight.check(state.mission_index == 1, "continues to the first waypoint");
}

void test_navigation_motor_loop_takeoff_rth_switch_cancels() {
  flight_t flight({"takeoff cancel", 100, true});
  flight_takeoff_mission(flight);
  flight.throttle = 0;
  // The held RTH switch cancels the waiting takeoff and does not lift off itself.
  flight.rth = true;
  flight.run(1000);
  flight.check(state.nav_state < NAV_STATE_CLIMB && !flags.controls_override, "the RTH switch cancels the takeoff");
  // Handed back to the pilot's lowered throttle stick.
  flight.check(state.throttle < 0.05f && flight.truth.position[2] >= 0, "stays on the ground");
  flight.rth = false;
  flight.mission_switch = false;
  flight.run(100);
  flight.check(state.nav_state == NAV_STATE_INACTIVE, "releasing the switches clears the cancel");
}

void test_navigation_motor_loop_takeoff_from_a_slope() {
  // Resting tilted, angle mode reads the slope as an attitude error. Waiting
  // armed and spinning up on the ground must not integrate it.
  flight_t flight({"sloped takeoff", 100, true});
  flight.truth.ground_tilt = 8 * M_PI / 180;
  flight_takeoff_mission(flight);
  flight.throttle = 0;
  flight.run(2000);
  flight.check(state.nav_state == NAV_STATE_TAKEOFF_READY, "waiting for the throttle stick");
  flight.throttle = 0.5f;
  double worst_iterm = 0;
  for (unsigned i = 0; i < 20000 && state.nav_state != NAV_STATE_TAKEOFF; i++) {
    flight.step();
    const vec3_t *ierror = pid_get_ierror();
    worst_iterm = fmax(worst_iterm, fmax(fabs(ierror->roll), fabs(ierror->pitch)));
  }
  printf("%s: worst I-term before lift-off %.3f\n", flight.scenario.name, worst_iterm);
  flight.check(state.nav_state == NAV_STATE_TAKEOFF, "lifts off");
  flight.check(worst_iterm < 0.01, "no I-term before lift-off");
  for (unsigned i = 0; i < 20000 && state.mission_index == 0; i++) flight.step();
  flight.check(state.mission_index == 1, "climbs out to the first waypoint");
}

void test_navigation_motor_loop_takeoff_spinup_cancels() {
  // Spinning up on the ground, an RTH request, a lost sensor, the mission
  // switch turned off or a lost link stops the motors instead of climbing
  // home, jumping to the raised throttle stick or lifting off without a pilot.
  static const char *const names[] = {"spin-up rth", "spin-up gps loss", "spin-up switch off", "spin-up link loss"};
  for (unsigned cause = 0; cause < 4; cause++) {
    flight_t flight({names[cause], 100, true});
    flight_takeoff_mission(flight);
    flight.run(300);
    flight.check(state.nav_state == NAV_STATE_TAKEOFF_SPINUP && state.throttle > 0 && flight.truth.position[2] >= 0, "spinning up on the ground");
    if (cause == 0)
      flight.rth = true;
    else if (cause == 1)
      flight.gps_available = false;
    else if (cause == 2)
      flight.mission_switch = false;
    else
      flight.rc_available = false;
    for (unsigned i = 0; i < 3000 && flags.arm_state; i++) flight.step();
    flight.check(!flags.arm_state && state.nav_state < NAV_STATE_CLIMB, "the takeoff ends disarmed");
    flight.check(flight.truth.position[2] > -0.1, "never left the ground");
  }
}

void test_navigation_motor_loop_land_through_gps_loss() {
  flight_t flight({"land gps loss", 100, true});
  flight_takeoff_mission(flight);
  for (unsigned i = 0; i < 150000 && state.nav_state != NAV_STATE_LAND; i++) flight.step();
  flight.check(state.nav_state == NAV_STATE_LAND && -flight.truth.position[2] > 5, "landing from height");

  // The descent needs only the barometer: it lands level where it is.
  flight.gps_available = false;
  unsigned i = 0;
  for (; i < 60000 && flags.arm_state; i++) {
    flight.step();
    if (flags.arm_state)
      flight.check(state.nav_state == NAV_STATE_LAND, "the landing continues without GPS");
  }
  printf("%s: disarmed %.1fs after the loss, %.2fm off the waypoint\n", flight.scenario.name, i * 1e-3,
      hypot(flight.truth.position[0] - 40, flight.truth.position[1] - 50));
  flight.check(!flags.arm_state && flight.truth.position[2] > -0.05, "touchdown disarms");
}

// As flown: the LAND ground entered tens of meters too high starts the final
// descent far up, where the barometer rising for a few seconds holds the
// altitude still. Winding the trim down for that, the accelerometer shows
// the craft falling with its thrust, and the trim comes straight back.
void test_navigation_motor_loop_land_stall_aloft() {
  flight_t flight({"land, descent stalls aloft", 100, true});
  flight_takeoff_mission(flight, 30);
  mission.waypoints[2].altitude = 20;
  for (unsigned i = 0; i < 150000 && state.nav_state != NAV_STATE_LAND; i++) flight.step();
  for (unsigned i = 0; i < 60000 && -flight.truth.position[2] > 23; i++) flight.step();
  flight.check(state.nav_state == NAV_STATE_LAND && -flight.truth.position[2] > 20, "final descent from height");

  double fastest = 0;
  for (unsigned i = 0; i < 12000 && flags.arm_state; i++) {
    if (i < 4000)
      flight.baro_drift += 0.75 * DT;
    flight.step();
    fastest = fmax(fastest, flight.truth.velocity[2]);
  }
  printf("%s: fastest descent=%.2fm/s at %.1fm\n", flight.scenario.name, fastest, -flight.truth.position[2]);
  flight.check(flags.arm_state && state.nav_state == NAV_STATE_LAND, "the landing continues");
  flight.check(fastest < 1.5, "no plunge while the altitude holds still in the air");
}

// As flown: GPS altitude 10 m off drags the estimate low, and a wash that
// builds from a meter up keeps the barometer sinking after touchdown, so the
// altitude shows a descent going on. The accelerometer shows the ground
// carrying the craft once the trim winds down: no hop on hover throttle, and
// a prompt disarm.
void test_navigation_motor_loop_land_in_strong_wash() {
  struct {
    const char *name;
    double wash_height;
    int16_t ground;
  } const cases[] = {
      {"land, strong wash from 0.5m", 0.5, 0},
      {"land, strong wash from 1m", 1.0, 0},
      {"land, strong wash from 1m, ground entered 20m high", 1.0, 20},
  };
  for (const auto &test : cases) {
    flight_t flight({.name = test.name, .gps_delay_ms = 100, .gps_jitter = true, .configured_hover = 0.9f, .baro_noise = 0.3, .ground_dip = 6, .baro_lag = 0.2, .wash_height = test.wash_height});
    flight_takeoff_mission(flight, 30);
    mission.waypoints[2].altitude = test.ground;
    bool landing = false, down = false;
    double top = 0;
    unsigned touchdown = 0, hops = 0, i = 0;
    for (; i < 200000 && flags.arm_state; i++) {
      if (!landing && -flight.truth.position[2] > 5)
        flight.gps_height_error = fmax(flight.gps_height_error - 0.5 * DT, -12);
      flight.step();
      landing = landing || state.nav_state == NAV_STATE_LAND;
      const double height = -flight.truth.position[2];
      if (landing && !down && height < 0.01) {
        down = true;
        if (!touchdown)
          touchdown = i;
      } else if (down && height > 0.05) {
        down = false;
        hops++;
      }
      if (touchdown)
        top = fmax(top, height);
    }
    printf("%s: disarmed %.1fs after touchdown, %u hops to %.2fm\n", test.name, (i - touchdown) * 1e-3, hops, top);
    flight.check(landing && touchdown && !flags.arm_state, "touchdown disarms");
    flight.check(hops == 0, "stays on the ground");
    flight.check(i - touchdown < 5000, "disarms soon after touchdown");
  }
}

void test_navigation_motor_loop_land_reposition() {
  flight_t flight({"land reposition", 100, true});
  flight_takeoff_mission(flight);
  for (unsigned i = 0; i < 150000 && state.nav_state != NAV_STATE_LAND; i++) flight.step();
  for (unsigned i = 0; i < 3000; i++) flight.step();
  flight.check(state.nav_state == NAV_STATE_LAND && -flight.truth.position[2] > 4, "landing from height");

  // Forward stick, 1.2 m/s for 3 s, moves the landing point along the nose.
  const double heading = atan2(flight.truth.rotation[1][0], flight.truth.rotation[0][0]);
  const double start_north = flight.truth.position[0], start_east = flight.truth.position[1];
  flight.pitch = 0.6f;
  flight.run(3000);
  flight.pitch = 0;
  flight.check(state.nav_state == NAV_STATE_LAND, "the sticks do not end the landing");
  unsigned i = 0;
  for (; i < 60000 && flags.arm_state; i++) flight.step();
  const double north = flight.truth.position[0] - start_north, east = flight.truth.position[1] - start_east;
  const double along = north * cos(heading) + east * sin(heading);
  printf("%s: landed %.2fm along the nose, %.2fm across\n", flight.scenario.name, along,
      fabs(east * cos(heading) - north * sin(heading)));
  flight.check(!flags.arm_state && flight.truth.position[2] > -0.05, "touchdown disarms");
  flight.check(along > 2 && fabs(east * cos(heading) - north * sin(heading)) < 1, "lands where the sticks moved it");
}

void test_navigation_motor_loop_land_on_slope() {
  // An accelerometer reading low must not take the slope for a fall once the
  // trim winds down.
  struct {
    const char *name;
    double accel_scale;
  } const cases[] = {
      {"slope landing", 1.0},
      {"slope landing, accelerometer 8% low", 0.92},
  };
  for (const auto &test : cases) {
    flight_t flight({test.name, 100, true});
    flight_takeoff_mission(flight);
    for (unsigned i = 0; i < 20000 && -flight.truth.position[2] < 2; i++) flight.step();
    flight.check(state.nav_state == NAV_STATE_TAKEOFF, "climbing out of the takeoff");
    // The landing ground is steeper than the land detector's level check.
    flight.truth.ground_tilt = 35 * M_PI / 180;

    bool landing = false;
    unsigned i = 0, touchdown = 0;
    for (; i < 150000 && flags.arm_state; i++) {
      flight.step();
      if (!landing && state.nav_state == NAV_STATE_LAND) {
        landing = true;
        flight.truth.accel_scale = test.accel_scale;
      }
      if (landing && !touchdown && -flight.truth.position[2] < 0.01)
        touchdown = i;
    }
    printf("%s: disarmed %.1fs after touchdown, tilt %.1f deg\n", test.name, (i - touchdown) * 1e-3,
        double(state.attitude.roll) * 180 / M_PI);
    flight.check(landing && touchdown && !flags.arm_state, "touchdown on a slope disarms");
    // The estimated vertical speed and roll take seconds to settle on the slope.
    flight.check(i - touchdown < 15000, "disarms soon after touchdown");
  }
}

void test_navigation_motor_loop_gps_glitch() {
  // Multipath moves the reported position 25 m for 4 s while the craft holds
  // home; the reported velocity stays right. Navigation must not fly to it.
  flight_t flight({"gps glitch", 100, true, 1.0, -0.5});
  flight.depart();
  flight.start_return();
  flight.finish_return();
  double excursion = 0;
  for (unsigned i = 0; i < 40000; i++) {
    flight.gps_offset[0] = i < 4000 ? 25 : 0;
    flight.step();
    flight.check(state.nav_state >= NAV_STATE_CLIMB && flags.arm_state, "RTH survives the glitch");
    excursion = fmax(excursion, hypot(flight.truth.position[0], flight.truth.position[1]));
  }
  printf("%s: excursion=%.2fm home error=%.2fm\n", flight.scenario.name, excursion,
      hypot(flight.truth.position[0], flight.truth.position[1]));
  flight.check(excursion < 5, "a GPS glitch does not move the craft");
  flight.check(state.nav_state == NAV_STATE_HOLD && hypot(flight.truth.position[0], flight.truth.position[1]) < 3, "holds home again");
}

void test_navigation_motor_loop_gps_stall() {
  // As logged on 2026-11-02: accelerating toward a target, the receiver keeps
  // reporting the fix it stalled at for 4 s, then catches up at once.
  flight_t flight({"gps stall", 100, true});
  flight.depart();
  flight.mission_switch = true;
  flight.run(100);
  nav_command_t command = {.type = NAV_COMMAND_HOLD, .id = 1};
  flight.check(nav_command_apply(&command) == NAV_RESULT_OK, "HOLD accepted");
  flight.run(15000);
  const double to[2] = {flight.truth.position[0] + 120, flight.truth.position[1]};
  command = {.type = NAV_COMMAND_GOTO, .id = 2};
  command.waypoint = {.coord = {.lon = int32_t(lround(to[1] * 1e7 / 111194.93)), .lat = int32_t(lround(to[0] * 1e7 / 111194.93))},
                      .altitude = int16_t(lroundf(state.altitude)), .speed = 80}; // 8 m/s
  flight.check(nav_command_apply(&command) == NAV_RESULT_OK, "GOTO accepted");
  for (unsigned i = 0; i < 20000 && hypot(flight.truth.velocity[0], flight.truth.velocity[1]) < 2; i++)
    flight.step();

  double top_speed = 0, overshoot = 0;
  unsigned arrived = 0;
  for (unsigned i = 0; i < 60000; i++) {
    flight.gps_frozen = i < 4000;
    flight.step();
    flight.check(state.nav_state >= NAV_STATE_CLIMB && flags.arm_state, "navigation survives the stall");
    top_speed = fmax(top_speed, hypot(flight.truth.velocity[0], flight.truth.velocity[1]));
    overshoot = fmax(overshoot, flight.truth.position[0] - to[0]);
    if (!arrived && hypot(flight.truth.position[0] - to[0], flight.truth.position[1] - to[1]) < 1 &&
        hypot(flight.truth.velocity[0], flight.truth.velocity[1]) < 0.3)
      arrived = i;
  }
  printf("%s: top speed=%.2fm/s overshoot=%.2fm arrived after %.1fs\n", flight.scenario.name, top_speed, overshoot, arrived * 1e-3);
  flight.check(top_speed < 11, "a stalled GPS does not run away with the speed");
  flight.check(overshoot < 5, "nor overshoot the target far");
  flight.check(arrived, "reaches the target once GPS catches up");
}

void test_navigation_motor_loop_baro_drift() {
  // Hovering at home, warming electronics make the barometer read 6 m high
  // over ten minutes, while GPS altitude wanders 1.5 m.
  flight_t flight({"baro drift", 100, true});
  flight.depart();
  flight.start_return();
  flight.finish_return();
  double worst = 0;
  for (unsigned i = 0; i < 600000; i++) {
    const double t = i * DT;
    flight.baro_drift = 6 * (1 - exp(-t / 200));
    flight.gps_height_error = 1.5 * sin(2 * M_PI * t / 100);
    flight.step();
    if (i >= 540000)
      worst = fmax(worst, fabs(-flight.truth.position[2] - flight.return_height));
  }
  printf("%s: worst altitude error over the last minute=%.2fm\n", flight.scenario.name, worst);
  flight.check(worst < 2.5, "GPS altitude corrects barometer drift");
}

void test_navigation_motor_loop_gps_altitude_off() {
  // As flown: within a minute of takeoff GPS altitude reads 12 m below its
  // home reference, while the barometer is right. That much disagreement is
  // not barometer drift, and the altitude stays on the barometer.
  flight_t flight({"GPS altitude off", 100, true});
  flight.depart();
  flight.start_return();
  flight.finish_return();
  double worst = 0;
  for (unsigned i = 0; i < 180000; i++) {
    flight.gps_height_error = fmax(flight.gps_height_error - 0.3 * DT, -12);
    flight.step();
    worst = fmax(worst, fabs(double(state.altitude) + flight.truth.position[2]));
  }
  printf("%s: worst altitude error=%.2fm\n", flight.scenario.name, worst);
  flight.check(worst < 1.5, "a GPS altitude far off does not move the altitude");
}

void test_navigation_motor_loop_home_altitude_averaged() {
  // GPS altitude reads 4 m high for the few seconds around arming after half
  // a minute on the bench. Home altitude, which the barometer drift
  // correction measures from, must not take that error.
  flight_t flight({"home altitude", 100, true});
  flight.run(30000);
  flight.gps_height_error = 4;
  flight.run(3000);
  flight.arm = true;
  flight.run(300);
  flight.check(flags.arm_state && flags.home_valid, "arming and home capture");
  flight.gps_height_error = 0;
  double worst = 0;
  for (unsigned i = 0; i < 240000; i++) {
    flight.pilot_height(8);
    flight.step();
    if (i >= 210000)
      worst = fmax(worst, fabs(double(state.altitude) + flight.truth.position[2]));
  }
  printf("%s: worst altitude error over the last 30 s=%.2fm\n", flight.scenario.name, worst);
  flight.check(worst < 1.5, "a GPS altitude error at arming does not become barometer drift");
}

void test_navigation_motor_loop_home_after_arming() {
  // Armed before GPS has a fix: home latches at the first valid fix while
  // still on the ground, but never once airborne, away from launch.
  flight_t flight({"home after arming", 100, true});
  flight.gps_available = false;
  flight.run(3000);
  flight.arm = true;
  flight.run(300);
  flight.check(flags.arm_state && !flags.home_valid, "no home without GPS");
  flight.gps_available = true;
  flight.run(3000);
  flight.check(flags.gps_valid && flags.home_valid, "home at the first valid fix on the ground");
  flight.check(hypot(double(state.home_distance), 0) < 2, "home is the launch point");

  flight_t airborne({"home not latched airborne", 100, true});
  airborne.gps_available = false;
  airborne.run(3000);
  airborne.arm = true;
  airborne.run(300);
  for (unsigned i = 0; i < 3000; i++) {
    airborne.pilot_height(5);
    airborne.step();
  }
  airborne.check(flags.in_air, "airborne");
  airborne.gps_available = true;
  for (unsigned i = 0; i < 3000; i++) {
    airborne.pilot_height(5);
    airborne.step();
  }
  airborne.check(flags.gps_valid && !flags.home_valid, "no home latched in the air");
}
