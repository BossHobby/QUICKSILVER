#include <string.h>
#include <unity.h>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "control/control.h"
#include "core/profile.h"
#include "core/tasks.h"
#include "driver/time.h"
#include "driver/osd/osd.h"
#include "io/msp.h"
#include "io/quic.h"
#include "osd/render.h"
#include "osd/status.h"

extern void simulator_osd_test_reset(bool reject);
extern uint32_t simulator_osd_test_push_count();
extern uint8_t simulator_osd_test_char(uint8_t x, uint8_t y);

static uint8_t msp_reply_direction;
static uint16_t msp_reply_size;

void test_osd_transfer_is_bounded_and_retries() {
  osd_device_init();
  osd_clear();
  osd_start(OSD_ATTR_TEXT, 0, 0);
  osd_write_str("AB");
  osd_start(OSD_ATTR_INVERT, 2, 0);
  osd_write_str("CD");
  osd_start(OSD_ATTR_TEXT, 0, 1);
  osd_write_str("EF");

  simulator_osd_test_reset(true);
  TEST_ASSERT_FALSE(osd_update());
  TEST_ASSERT_EQUAL_UINT32(1, simulator_osd_test_push_count());
  TEST_ASSERT_EQUAL_UINT8(' ', simulator_osd_test_char(0, 0));
  simulator_osd_test_reset(false);
  if (xSemaphoreTake(profile_mutex, 0) != pdTRUE) _exit(9);
  for (uint32_t i = 1; i <= 3; i++) {
    TEST_ASSERT_FALSE(osd_update());
    TEST_ASSERT_EQUAL_UINT32(i, simulator_osd_test_push_count());
  }
  TEST_ASSERT_EQUAL_UINT8('A', simulator_osd_test_char(0, 0));
  TEST_ASSERT_EQUAL_UINT8('D', simulator_osd_test_char(3, 0));
  TEST_ASSERT_EQUAL_UINT8('F', simulator_osd_test_char(1, 1));
  TEST_ASSERT_FALSE(osd_update()); // Flush after the final string.
  TEST_ASSERT_TRUE(osd_update());

  // Clearing a partial transfer must restart at the first row.
  osd_start(OSD_ATTR_TEXT, 0, 5);
  osd_write_str("G");
  TEST_ASSERT_FALSE(osd_update());
  osd_clear();
  osd_start(OSD_ATTR_TEXT, 0, 0);
  osd_write_str("H");
  TEST_ASSERT_FALSE(osd_update());
  TEST_ASSERT_EQUAL_UINT8('H', simulator_osd_test_char(0, 0));
  osd_clear();
}

void test_osd_render_completes_before_transfer() {
  const auto saved_osd = profile.osd;
  for (auto &p : profile.osd.profiles) {
    memset(p.elements, 0, sizeof(p.elements));
    p.elements[OSD_CALLSIGN] = ENCODE_OSD_ELEMENT(1, 0, 0, 0, 0, 0);
    p.elements[OSD_CELL_COUNT] = ENCODE_OSD_ELEMENT(1, 0, 0, 1, 0, 1);
    strcpy((char *)p.callsign, "TEST");
  }
  osd_device_init();
  osd_clear();
  osd_display_reset();
  simulator_osd_test_reset(false);
  // Allow system/profile detection and screen clearing to finish.
  for (uint32_t i = 0; i < 40; i++) {
    time_test_advance_us(1000);
    osd_display();
  }
  osd_clear();
  osd_display_reset();
  simulator_osd_test_reset(false);
  const uint8_t saved_cell_count = state.lipo_cell_count;
  state.lipo_cell_count = 4;
  osd_display(); // Render both elements in one pass before any transfer.
  TEST_ASSERT_EQUAL_UINT32(0, simulator_osd_test_push_count());
  state.lipo_cell_count = 9;
  for (unsigned i = 0; i < 8; i++) osd_display();
  TEST_ASSERT_EQUAL_UINT8('T', simulator_osd_test_char(0, 0));
  TEST_ASSERT_EQUAL_UINT8('4', simulator_osd_test_char(0, 1));

  time_test_advance_us(33332);
  for (unsigned i = 0; i < 8; i++) osd_display();
  TEST_ASSERT_EQUAL_UINT8('4', simulator_osd_test_char(0, 1));
  time_test_advance_us(1);
  for (unsigned i = 0; i < 8; i++) osd_display();
  TEST_ASSERT_EQUAL_UINT8('9', simulator_osd_test_char(0, 1));
  state.lipo_cell_count = saved_cell_count;
  osd_clear();
  osd_display_reset();
  profile.osd = saved_osd;
}

static void assert_looptime_label(bool visible) {
  for (uint32_t i = 0; i < 64; i++) {
    if (osd_update())
      break;
  }
  char row[51] = {};
  for (uint8_t x = 0; x < 50; x++)
    row[x] = simulator_osd_test_char(x, 0);
  TEST_ASSERT_EQUAL(visible, strstr(row, "LOOPTIME") != nullptr);
}

void test_osd_looptime_warning_is_temporary_until_2khz() {
  const auto saved_state = state;
  const auto saved_flags = flags;
  const auto saved_guac = profile.osd.guac_mode;
  flags = {};
  flags.rx_ready = 1;
  state = {};
  state.looptime_autodetect = 125;
  profile.osd.guac_mode = false;
  osd_element_t el = {};
  osd_device_init();
  osd_clear();
  osd_status_reset();
  simulator_osd_test_reset(false);
  osd_status_update(&el);
  osd_status_update(&el);

  state.looptime_warning = 1;
  state.looptime_autodetect = 250;
  osd_status_update(&el);
  assert_looptime_label(true);
  time_test_advance_us(1001000);
  osd_status_update(&el);
  osd_status_update(&el);
  assert_looptime_label(false);
  for (unsigned i = 0; i < 10; i++)
    osd_status_update(&el);
  assert_looptime_label(false);

  // A new scheduler session can show the same temporary warning again.
  state.looptime_warning = 0;
  state.looptime_autodetect = 125;
  osd_status_update(&el);
  state.looptime_warning = 1;
  state.looptime_autodetect = 250;
  osd_status_update(&el);
  assert_looptime_label(true);

  // Promote an active temporary warning to a persistent one at 2 kHz.
  state.looptime_warning = 2;
  state.looptime_autodetect = 500;
  osd_status_update(&el);
  time_test_advance_us(2000000);
  osd_status_update(&el);
  osd_status_update(&el);
  assert_looptime_label(true);

  state = saved_state;
  flags = saved_flags;
  profile.osd.guac_mode = saved_guac;
  osd_clear();
  osd_status_reset();
}

void test_osd_renders_latest_telemetry() {
  const auto saved_osd = profile.osd;
  const auto saved_state = state;
  const auto saved_flags = flags;
  flags = {};
  state = {};
  for (auto &p : profile.osd.profiles) {
    memset(p.elements, 0, sizeof(p.elements));
    p.elements[OSD_CELL_COUNT] = ENCODE_OSD_ELEMENT(1, 0, 0, 0, 0, 0);
  }
  osd_device_init();
  osd_clear();
  osd_display_reset();
  simulator_osd_test_reset(false);
  state.lipo_cell_count = 4;
  for (unsigned i = 0; i < 40; i++) {
    time_test_advance_us(1000);
    osd_display();
  }
  TEST_ASSERT_EQUAL_UINT8('4', simulator_osd_test_char(0, 0));
  state.lipo_cell_count = 9;
  for (unsigned i = 0; i < 40; i++) {
    time_test_advance_us(1000);
    osd_display();
  }
  TEST_ASSERT_EQUAL_UINT8('9', simulator_osd_test_char(0, 0));
  state = saved_state;
  flags = saved_flags;
  profile.osd = saved_osd;
  osd_clear();
  osd_display_reset();
}

static void osd_test_flight(void *) {
  flags = {};
  state = {};
  for (auto &p : profile.osd.profiles) {
    memset(p.elements, 0, sizeof(p.elements));
    p.elements[OSD_CELL_COUNT] = ENCODE_OSD_ELEMENT(1, 0, 0, 0, 0, 0);
  }
  state.lipo_cell_count = 4;
  profile_mutex_init();
  osd_init();

  // OSD can wait for configuration ownership across the arming transition.
  if (xSemaphoreTake(profile_mutex, 0) != pdTRUE) _exit(1);
  osd_push_screen(OSD_SCREEN_MAIN_MENU);
  simulator_osd_test_reset(true);
  vTaskDelay(2);
  if (osd_state.screen != OSD_SCREEN_MAIN_MENU) _exit(2);
  flags.arm_state = 1;
  threads_update();
  xSemaphoreGive(profile_mutex);
  vTaskDelay(4);
  if (osd_state.screen != OSD_SCREEN_CLEAR && osd_state.screen != OSD_SCREEN_REGULAR) _exit(4);

  // Rendering still progresses while armed; a stalled transfer didn't strand
  // the menu or suspend the worker.
  simulator_osd_test_reset(false);
  for (unsigned i = 0; i < 80; i++) {
    vTaskDelay(1);
  }
  if (simulator_osd_test_push_count() == 0) _exit(5);
  if (osd_state.screen != OSD_SCREEN_CLEAR && osd_state.screen != OSD_SCREEN_REGULAR) _exit(3);
  if (eTaskGetState(threads[THREAD_OSD].handle) == eSuspended) _exit(6);
  xSemaphoreGive(profile_mutex);

  {
    mutex_guard_t configuration(profile_mutex);
    flags.arm_state = 0;
    threads_update();
    osd_push_screen(OSD_SCREEN_MAIN_MENU);
  }
  vTaskDelay(5);
  if (osd_state.screen != OSD_SCREEN_MAIN_MENU) _exit(7);
  _exit(0);
}

void test_osd_worker_serializes_configuration_and_keeps_rendering_armed() {
  const pid_t child = fork();
  TEST_ASSERT_TRUE(child >= 0);
  if (child == 0) {
    threads[THREAD_FLIGHT].entry = osd_test_flight;
    thread_start(THREAD_FLIGHT);
    vTaskStartScheduler();
    _exit(8);
  }
  int status;
  for (unsigned i = 0; i < 2000; i++) {
    if (waitpid(child, &status, WNOHANG) == child) {
      TEST_ASSERT_TRUE(WIFEXITED(status));
      TEST_ASSERT_EQUAL_INT(0, WEXITSTATUS(status));
      return;
    }
    usleep(1000);
  }
  kill(child, SIGKILL);
  waitpid(child, &status, 0);
  TEST_FAIL_MESSAGE("OSD worker or configuration transition stalled");
}

static void capture_msp_reply(msp_magic_t, uint8_t direction, uint16_t, const uint8_t *, uint16_t size) {
  msp_reply_direction = direction;
  msp_reply_size = size;
}

static void osd_msp_request(uint8_t command, const uint8_t *payload, uint8_t size, bool fault_mode = false) {
  uint8_t buffer[64];
  msp_t msp = {
      .buffer = buffer,
      .buffer_size = sizeof(buffer),
      .send = capture_msp_reply,
      .device = MSP_DEVICE_VTX,
  };
  msp_reply_direction = 0;
  const uint8_t header[] = {'$', 'M', '<', size, command};
  for (uint8_t byte : header) msp_process_serial(&msp, byte, fault_mode);
  uint8_t checksum = size ^ command;
  for (uint8_t i = 0; i < size; i++) {
    msp_process_serial(&msp, payload[i], fault_mode);
    checksum ^= payload[i];
  }
  TEST_ASSERT_EQUAL(MSP_SUCCESS, msp_process_serial(&msp, checksum, fault_mode));
}

static volatile uint8_t maintenance_stage;
static uint32_t quic_reply_count;

static void capture_quic_reply(uint8_t *, uint32_t, void *) {
  quic_reply_count++;
}

static void maintenance_test_worker(void *) {
  osd_msp_request(MSP_ANALOG, nullptr, 0);
  if (msp_reply_direction != '>') _exit(1);
  maintenance_stage = 1;
  uint16_t motors[MOTOR_PIN_MAX] = {};
  for (auto &motor : motors) motor = 1500;
  osd_msp_request(MSP_SET_MOTOR, (const uint8_t *)motors, sizeof(motors));
  if (msp_reply_direction != '!' || motor_test.active) _exit(2);
  maintenance_stage = 2;
  for (;;) vTaskDelay(1);
}

static void maintenance_test_flight(void *) {
  flags = {};
  motor_test.active = 0;
  profile_mutex_init();
  if (xSemaphoreTake(profile_mutex, 0) != pdTRUE) _exit(3);

  // Receiving an incomplete USB command must not wait on configuration.
  quic_t quic = {.send = capture_quic_reply};
  uint8_t request[] = {QUIC_MAGIC, QUIC_CMD_GET, 0, 1, QUIC_VAL_INFO};
  if (quic_process(&quic, request, sizeof(request) - 1)) _exit(4);
  if (quic_reply_count != 0) _exit(5);

  // Fault USB explicitly bypasses locks, even if the stopped task owned one.
  if (!quic_process(&quic, request, sizeof(request), true) || quic_reply_count != 1) _exit(6);
  uint16_t motors[MOTOR_PIN_MAX] = {};
  for (auto &motor : motors) motor = 1500;
  osd_msp_request(MSP_SET_MOTOR, (const uint8_t *)motors, sizeof(motors), true);
  if (msp_reply_direction != '>' || !motor_test.active) _exit(7);
  motor_test.active = 0;

  thread_start(THREAD_IO);
  vTaskDelay(3);
  if (maintenance_stage != 1) _exit(8);
  flags.arm_state = 1;
  xSemaphoreGive(profile_mutex);
  vTaskDelay(3);
  if (maintenance_stage != 2 || motor_test.active) _exit(9);
  _exit(0);
}

void test_msp_maintenance_rechecks_arming_after_configuration_wait() {
  const pid_t child = fork();
  TEST_ASSERT_TRUE(child >= 0);
  if (child == 0) {
    threads[THREAD_FLIGHT].entry = maintenance_test_flight;
    threads[THREAD_IO].entry = maintenance_test_worker;
    thread_start(THREAD_FLIGHT);
    vTaskStartScheduler();
    _exit(10);
  }
  int status;
  for (unsigned i = 0; i < 2000; i++) {
    if (waitpid(child, &status, WNOHANG) == child) {
      TEST_ASSERT_TRUE(WIFEXITED(status));
      TEST_ASSERT_EQUAL_INT(0, WEXITSTATUS(status));
      return;
    }
    usleep(1000);
  }
  kill(child, SIGKILL);
  waitpid(child, &status, 0);
  TEST_FAIL_MESSAGE("MSP maintenance or fault USB blocked on configuration");
}

void test_osd_msp_keeps_telemetry_but_rejects_airborne_maintenance() {
  const auto saved_flags = flags;
  const auto saved_motor_test = motor_test;
  uint16_t motors[MOTOR_PIN_MAX];
  for (auto &motor : motors) motor = 1500;
  const uint8_t calibration[] = {QUIC_MAGIC, QUIC_CMD_CAL_IMU, 0, 0};
  for (unsigned armed = 0; armed < 2; armed++) {
    flags = {};
    flags.arm_state = armed;
    flags.in_air = !armed;
    motor_test.active = 0;
    osd_msp_request(MSP_SET_MOTOR, (const uint8_t *)motors, sizeof(motors));
    TEST_ASSERT_EQUAL_UINT8('!', msp_reply_direction);
    TEST_ASSERT_FALSE(motor_test.active);
    const uint32_t before = time_micros();
    osd_msp_request(251, calibration, sizeof(calibration)); // Removed QUIC tunnel.
    TEST_ASSERT_EQUAL_UINT8('!', msp_reply_direction);
    TEST_ASSERT_EQUAL_UINT32(before, time_micros());
    osd_msp_request(MSP_ANALOG, nullptr, 0);
    TEST_ASSERT_EQUAL_UINT8('>', msp_reply_direction);
    TEST_ASSERT_EQUAL_UINT16(9, msp_reply_size);
  }
  flags = {};
  const uint32_t before = time_micros();
  osd_msp_request(251, calibration, sizeof(calibration));
  TEST_ASSERT_EQUAL_UINT8('!', msp_reply_direction);
  TEST_ASSERT_EQUAL_UINT32(before, time_micros());
  osd_msp_request(MSP_SET_MOTOR, (const uint8_t *)motors, sizeof(motors));
  TEST_ASSERT_EQUAL_UINT8('>', msp_reply_direction);
  TEST_ASSERT_TRUE(motor_test.active);
  flags = saved_flags;
  motor_test = saved_motor_test;
}
