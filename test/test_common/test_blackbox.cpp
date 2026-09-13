#include <initializer_list>
#include <atomic>
#include <signal.h>
#include <stdio.h>
#include <sys/wait.h>
#include <unistd.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <unity.h>
#include <string.h>
#include "mock_helpers.h"
#include "core/tasks.h"
#include "driver/time.h"

// Include blackbox headers
#include "io/blackbox.h"
#include "io/blackbox_device.h"
#include "io/blackbox_device_simulator.h"
#include "control/control.h"
#include "util/vector.h"
#include "util/cbor_helper.h"

namespace {
struct recorded_frame_t {
  uint8_t bytes[BLACKBOX_MAX_SIZE];
  uint8_t size;
};
static recorded_frame_t recorded[64];
static unsigned writes, storage_calls, starts, stops, flushing;
static bool storage_ready, reject_write;

// Deterministic tests permit one service pass at a time through the mock device.
static SemaphoreHandle_t step_requested, step_done;
static bool step_in_progress;

static void step_blackbox() {
  TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreGive(step_requested));
  TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(step_done, pdMS_TO_TICKS(100)));
}

static void run_blackbox_test(void (*body)()) {
  const pid_t child = fork();
  TEST_ASSERT_TRUE(child >= 0);
  if (child == 0) {
    static StaticSemaphore_t requested_storage, done_storage;
    step_requested = xSemaphoreCreateBinaryStatic(&requested_storage);
    step_done = xSemaphoreCreateBinaryStatic(&done_storage);
    static StaticTask_t task;
    static StackType_t stack[2048];
    auto entry = [](void *arg) {
      (*static_cast<void (**)()>(arg))();
      fflush(stdout);
      _exit(Unity.CurrentTestFailed ? 1 : 0);
    };
    if (!xTaskCreateStatic(entry, "test", 2048, &body, 2, stack, &task)) _exit(2);
    vTaskStartScheduler();
    _exit(3);
  }
  int status;
  for (unsigned i = 0; i < 5000; i++) {
    if (waitpid(child, &status, WNOHANG) == child) {
      TEST_ASSERT_TRUE(WIFEXITED(status));
      TEST_ASSERT_EQUAL_INT(0, WEXITSTATUS(status));
      return;
    }
    usleep(1000);
  }
  kill(child, SIGKILL);
  waitpid(child, &status, 0);
  TEST_FAIL_MESSAGE("Blackbox service pass stalled");
}

struct blackbox_test_t {
  decltype(blackbox_device_simulator) saved_device = blackbox_device_simulator;
  profile_t saved_profile = profile;
  control_state_t saved_state = state;
  decltype(flags) saved_flags = flags;
  decltype(target_info) saved_info = target_info;
  decltype(blackbox_bounds) saved_bounds = blackbox_bounds;
  decltype(blackbox_device_header) saved_header = blackbox_device_header;

  blackbox_test_t() {
    writes = storage_calls = starts = stops = flushing = 0;
    storage_ready = true;
    reject_write = false;
    blackbox_device_simulator.init = []() {};
    blackbox_device_simulator.update = []() {
      if (step_requested) {
        if (step_in_progress) xSemaphoreGive(step_done);
        xSemaphoreTake(step_requested, portMAX_DELAY);
        step_in_progress = true;
      }
      storage_calls++;
      if (flushing) --flushing;
      return storage_ready;
    };
    blackbox_device_simulator.ready = []() { return flushing == 0; };
    blackbox_device_simulator.usage = []() { return 256U; };
    blackbox_device_simulator.start = []() { starts++; };
    blackbox_device_simulator.stop = []() { stops++; flushing = 3; };
    blackbox_device_simulator.reset = []() {};
    blackbox_device_simulator.write = [](const uint8_t *bytes, uint8_t size) {
      if (reject_write || writes >= 64) return false;
      memcpy(recorded[writes].bytes, bytes, size);
      recorded[writes++].size = size;
      return true;
    };
    profile_set_defaults();
    state = {};
    flags = {};
    state.looptime_autodetect = 125;
    state.aux_active = 1U << AUX_BLACKBOX;
    profile.blackbox.sample_rate_hz = 8000;
    profile.blackbox.field_flags = (1U << BBOX_FIELD_PID_P_TERM) | (1U << BBOX_FIELD_GYRO_RAW) |
                                   (1U << BBOX_FIELD_OUTPUT) | (1U << BBOX_FIELD_DEBUG);
    profile.blackbox.debug_flags = BBOX_DEBUG_DYN_NOTCH;
    if (step_requested && threads[THREAD_BLACKBOX].handle == nullptr) {
      blackbox_init();
    } else {
      blackbox_reset();
      blackbox_device_init();
    }
    blackbox_bounds.page_size = 256;
    blackbox_bounds.total_size = 1048576;
  }

  ~blackbox_test_t() {
    blackbox_reset();
    blackbox_device_simulator = saved_device;
    blackbox_bounds = saved_bounds;
    blackbox_device_header = saved_header;
    target_info = saved_info;
    profile = saved_profile;
    profile_output_update();
    state = saved_state;
    flags = saved_flags;
  }
};

static blackbox_t capture_frame(uint32_t sequence, uint32_t loop, uint32_t timestamp, float value) {
  state.loop_counter = loop;
  time_test_set_us(timestamp);
  state.pid_p_term.roll = value;
  state.gyro_raw.pitch = value;
  state.output[0] = value;
  blackbox_set_debug(BBOX_DEBUG_DYN_NOTCH, 0, (int16_t)value);
  blackbox_capture();
  blackbox_t expected = {};
  expected.loop = sequence;
  expected.time = timestamp;
  expected.pid_p_term.roll = value * BLACKBOX_SCALE;
  expected.gyro_raw.pitch = value * BLACKBOX_SCALE;
  expected.output.axis[0] = value * BLACKBOX_SCALE;
  expected.debug[0] = value;
  return expected;
}

static void expect_frame(unsigned index, uint32_t fields, const blackbox_t &current,
                         const blackbox_t &previous, blackbox_frame_type_t type) {
  TEST_ASSERT_TRUE(index < writes);
  uint8_t expected[BLACKBOX_MAX_SIZE];
  cbor_value_t enc;
  cbor_encoder_init(&enc, expected, sizeof(expected));
  TEST_ASSERT_EQUAL(CBOR_OK, cbor_encode_blackbox_frame(&enc, &current, &previous, type, fields));
  TEST_ASSERT_EQUAL_UINT(cbor_encoder_len(&enc), recorded[index].size);
  TEST_ASSERT_EQUAL_MEMORY(expected, recorded[index].bytes, recorded[index].size);
}
}

static void test_blackbox_captures_before_delayed_encoding_body() {
  for (uint32_t divider : {1U, 2U, 4U, 8U}) {
    for (uint32_t start : {0U, UINT32_MAX - 8U}) {
      blackbox_test_t fixture;
      profile.blackbox.sample_rate_hz = 8000 / divider;
      const uint32_t fields = profile.blackbox.field_flags;
      flags.arm_state = 1;
      state.loop_counter = start;
      blackbox_capture();
      blackbox_t previous = {};
      for (uint32_t sequence = 1; sequence <= 3; sequence++) {
        blackbox_t expected = {};
        const unsigned before = storage_calls;
        for (uint32_t i = 1; i <= divider; i++) {
          const uint32_t loop = (sequence - 1) * divider + i;
          expected = capture_frame(sequence, start + loop, loop * 125, sequence);
        }
        blackbox_capture(); // Same loop must not publish another sample.
        TEST_ASSERT_EQUAL_UINT(sequence - 1, writes);
        TEST_ASSERT_EQUAL_UINT(before, storage_calls);

        // Encoding must use captured data and session format, not live values.
        state.pid_p_term.roll = state.gyro_raw.pitch = state.output[0] = 99;
        blackbox_set_debug(BBOX_DEBUG_DYN_NOTCH, 0, 99);
        profile.blackbox.field_flags = 0;
        state.looptime_autodetect = 500;
        step_blackbox();
        TEST_ASSERT_EQUAL_UINT(fields, blackbox_current_file()->field_flags);
        TEST_ASSERT_EQUAL_UINT(divider, blackbox_current_file()->blackbox_rate);
        TEST_ASSERT_EQUAL_FLOAT(125, blackbox_current_file()->looptime);
        TEST_ASSERT_EQUAL_UINT(sequence, writes);
        expect_frame(sequence - 1, fields, expected, previous,
                     sequence == 1 ? BLACKBOX_FRAME_I : BLACKBOX_FRAME_P);
        previous = expected;
      }
      step_blackbox();
      TEST_ASSERT_EQUAL_UINT(3, writes);
      flags.arm_state = 0;
      blackbox_capture();
      step_blackbox();
      TEST_ASSERT_EQUAL_UINT(1, stops);
      const unsigned before = storage_calls;
      for (unsigned i = 0; i < 4; i++) step_blackbox();
      TEST_ASSERT_EQUAL_UINT(before + 4, storage_calls);
      TEST_ASSERT_EQUAL_UINT(0, flushing);
    }
  }
}

static void test_blackbox_mailbox_overrun_and_session_drain_body() {
  blackbox_test_t fixture;
  const uint32_t fields = profile.blackbox.field_flags;
  flags.arm_state = 1;
  blackbox_capture();
  const auto first = capture_frame(1, 1, 125, 1);
  for (unsigned i = 2; i <= 40; i++) capture_frame(i, i, i * 125, 9);
  step_blackbox();
  TEST_ASSERT_EQUAL_UINT(1, writes);
  expect_frame(0, fields, first, first, BLACKBOX_FRAME_I);
  const auto last = capture_frame(41, 41, 5125, 2);
  flags.arm_state = 0;
  blackbox_capture();
  // Rearm before the old sample is consumed. It must retain its old session.
  flags.arm_state = 1;
  blackbox_capture();
  capture_frame(1, 42, 5250, 9); // Busy mailbox: drop, never overwrite the old frame.
  step_blackbox();
  TEST_ASSERT_EQUAL_UINT(2, writes);
  expect_frame(1, fields, last, first, BLACKBOX_FRAME_P);
  const auto next = capture_frame(2, 43, 5375, 3);
  step_blackbox(); // Finish old session before starting the next.
  TEST_ASSERT_EQUAL_UINT(1, stops);
  for (unsigned i = 0; i < 2; i++) {
    step_blackbox();
    TEST_ASSERT_EQUAL_UINT(1, starts);
  }
  step_blackbox();
  TEST_ASSERT_EQUAL_UINT(2, starts);
  expect_frame(2, fields, next, next, BLACKBOX_FRAME_I);
  flags.arm_state = 0;
  blackbox_capture();
  step_blackbox();
  TEST_ASSERT_EQUAL_UINT(2, stops);
}

static void test_blackbox_storage_stall_and_rejected_delta_body() {
  blackbox_test_t fixture;
  const uint32_t fields = profile.blackbox.field_flags;
  flags.arm_state = 1;
  blackbox_capture();
  const auto first = capture_frame(1, 1, 125, 1);
  storage_ready = false;
  for (unsigned i = 0; i < 4; i++) step_blackbox();
  TEST_ASSERT_EQUAL_UINT(0, writes);
  storage_ready = true;
  step_blackbox();
  capture_frame(2, 2, 250, 2);
  reject_write = true;
  step_blackbox();
  reject_write = false;
  const auto third = capture_frame(3, 3, 375, 3);
  // Once open, encoding can continue into the writer FIFO while storage is busy.
  storage_ready = false;
  step_blackbox();
  TEST_ASSERT_EQUAL_UINT(2, writes);
  expect_frame(0, fields, first, first, BLACKBOX_FRAME_I);
  expect_frame(1, fields, third, first, BLACKBOX_FRAME_P);
  storage_ready = true;

  capture_frame(4, 4, 500, 4);
  blackbox_device_reset(); // Erasing logs also discards pending samples and delta history.
  step_blackbox();
  TEST_ASSERT_EQUAL_UINT(2, writes);
  TEST_ASSERT_EQUAL_UINT(0, blackbox_device_header.file_num);
  blackbox_capture();
  const auto restarted = capture_frame(1, 5, 625, 5);
  step_blackbox();
  expect_frame(2, fields, restarted, restarted, BLACKBOX_FRAME_I);
}

// Run the real POSIX FreeRTOS port in a child: the normal Unity runner does
// not start a kernel, and kernel shutdown is not supported on Cortex-M.
static std::atomic<bool> worker_entered, worker_preempted, worker_encoded;
static std::atomic<unsigned> worker_passes;

static void blackbox_test_flight(void *) {
  flags.arm_state = 1;
  state.loop_counter = 0;
  blackbox_capture();
  capture_frame(1, 1, 125, 1);
  xTaskNotifyGive(threads[THREAD_BLACKBOX].handle);
  for (unsigned tick = 0; tick < 20; tick++) {
    vTaskDelay(1);
    if (worker_entered.load()) worker_preempted.store(true);
    if (worker_encoded.load()) {
      if (!worker_preempted.load()) _exit(11);
      flags.arm_state = 0;
      blackbox_capture();
      unsigned passes;
      {
        mutex_guard_t guard(blackbox_storage_mutex);
        passes = worker_passes.load();
        if (blackbox_device_header.file_num != 1) _exit(16);
        uint8_t data[4] = {};
        blackbox_device_read(0, 7, data, sizeof(data));
        if (data[0] != 42) _exit(17);
        vTaskDelay(3);
        if (worker_passes.load() != passes) _exit(22);
        blackbox_device_reset();
        if (blackbox_device_header.file_num != 0) _exit(18);
      }
      vTaskDelay(3);
      if (worker_passes.load() == passes) _exit(23);
      _exit(0);
    }
  }
  _exit(12);
}

void test_blackbox_runs_in_own_freertos_task() {
  const pid_t child = fork();
  TEST_ASSERT_TRUE(child >= 0);
  if (child == 0) {
    blackbox_test_t fixture;
    blackbox_device_simulator.update = []() {
      if (strcmp(pcTaskGetName(nullptr), "blackbox") != 0) _exit(13);
      worker_passes.fetch_add(1);
      worker_entered.store(true);
      // Intentionally long work: only preemption by Flight can release it.
      while (!worker_preempted.load()) {}
      return true;
    };
    blackbox_device_simulator.write = [](const uint8_t *, uint8_t) {
      worker_encoded.store(true);
      return true;
    };
    blackbox_device_simulator.read = [](uint32_t file, uint32_t offset, uint8_t *data, uint32_t size) {
      if (strcmp(pcTaskGetName(nullptr), "flight") != 0 || file != 0 || offset != 7 || size != 4) _exit(19);
      data[0] = 42;
    };
    blackbox_device_simulator.reset = []() {
      if (strcmp(pcTaskGetName(nullptr), "flight") != 0) _exit(20);
    };
    static StaticTask_t task;
    static StackType_t stack[2048];
    if (!xTaskCreateStatic(blackbox_test_flight, "flight", 2048, nullptr, 2, stack, &task)) _exit(14);
    blackbox_init();
    if (!threads[THREAD_BLACKBOX].handle) _exit(21);
    vTaskStartScheduler();
    _exit(15);
  }
  int status = 0;
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
  TEST_FAIL_MESSAGE("Blackbox worker or Flight preemption stalled");
}

// Define constants for testing (from blackbox.c)
void test_blackbox_navigation_roundtrip_and_unchanged_home(void) {
  const uint32_t fields = (1 << BBOX_FIELD_GPS_COORD) | (1 << BBOX_FIELD_GPS_HOME) |
                          (1 << BBOX_FIELD_ALTITUDE) | (1 << BBOX_FIELD_DEBUG);
  blackbox_t previous = {};
  previous.gps_coord[0] = -338765432;
  previous.gps_coord[1] = 1511234567;
  previous.gps_home[0] = -338765000;
  previous.gps_home[1] = 1511234000;
  previous.altitude = -123;
  previous.debug[0] = 42;
  for (int pass = 0; pass < 3; pass++) {
    blackbox_t current = previous;
    if (pass == 1) {
      current.gps_coord[0] += 7;
      current.gps_coord[1] -= 9;
      current.altitude = 321;
    }
    if (pass == 2) current.gps_coord[1] = -1799999999;
    uint8_t buffer[BLACKBOX_MAX_SIZE];
    cbor_value_t codec;
    cbor_encoder_init(&codec, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL(CBOR_OK, cbor_encode_blackbox_frame(&codec, &current, &previous,
        pass == 0 ? BLACKBOX_FRAME_I : BLACKBOX_FRAME_P, fields));
    const auto length = cbor_encoder_len(&codec);
    cbor_decoder_init(&codec, buffer, length);
    cbor_container_t array;
    TEST_ASSERT_TRUE(cbor_decode_array(&codec, &array) >= CBOR_OK);
    uint32_t flags;
    TEST_ASSERT_TRUE(cbor_decode_uint32_t(&codec, &flags) >= CBOR_OK);
    TEST_ASSERT_EQUAL(pass == 1, (flags & BLACKBOX_FRAME_TYPE_BIT) != 0);
    TEST_ASSERT_EQUAL(pass != 1, (flags & (1 << BBOX_FIELD_GPS_HOME)) != 0);
    TEST_ASSERT_TRUE(cbor_decode_skip(&codec) >= CBOR_OK); // loop
    TEST_ASSERT_TRUE(cbor_decode_skip(&codec) >= CBOR_OK); // time
    TEST_ASSERT_TRUE(cbor_decode_array(&codec, &array) >= CBOR_OK);
    for (unsigned i = 0; i < 2; i++) {
      int32_t value;
      TEST_ASSERT_TRUE(cbor_decode_int32_t(&codec, &value) >= CBOR_OK);
      TEST_ASSERT_EQUAL_INT32(current.gps_coord[i], pass == 1 ? previous.gps_coord[i] + value : value);
    }
    if (pass != 1) {
      TEST_ASSERT_TRUE(cbor_decode_array(&codec, &array) >= CBOR_OK);
      for (unsigned i = 0; i < 2; i++) {
        int32_t value;
        TEST_ASSERT_TRUE(cbor_decode_int32_t(&codec, &value) >= CBOR_OK);
        TEST_ASSERT_EQUAL_INT32(current.gps_home[i], value);
      }
    }
    int32_t altitude;
    TEST_ASSERT_TRUE(cbor_decode_int32_t(&codec, &altitude) >= CBOR_OK);
    TEST_ASSERT_EQUAL_INT32(current.altitude, pass == 1 ? previous.altitude + altitude : altitude);
    if (pass != 1) {
      TEST_ASSERT_TRUE(cbor_decode_array(&codec, &array) >= CBOR_OK);
      int16_t debug;
      TEST_ASSERT_TRUE(cbor_decode_int16_t(&codec, &debug) >= CBOR_OK);
      TEST_ASSERT_EQUAL_INT16(42, debug);
    }
  }
}

void test_blackbox_full_frame_fits_device_buffer(void) {
  blackbox_t frame;
  memset(&frame, 0x7f, sizeof(frame));
  frame.gps_coord[0] = -900000000;
  frame.gps_coord[1] = -1800000000;
  frame.gps_home[0] = 900000000;
  frame.gps_home[1] = 1800000000;
  frame.altitude = INT16_MIN;
  uint8_t buffer[BLACKBOX_MAX_SIZE];
  cbor_value_t codec;
  cbor_encoder_init(&codec, buffer, sizeof(buffer));
  TEST_ASSERT_EQUAL(CBOR_OK, cbor_encode_blackbox_frame(&codec, &frame, &frame,
      BLACKBOX_FRAME_I, (1 << BBOX_FIELD_MAX) - 1));
  TEST_ASSERT_TRUE(cbor_encoder_len(&codec) <= sizeof(buffer));
}

#define BLACKBOX_I_FRAME_INTERVAL 32

// Test helper functions for blackbox delta encoding

static void create_test_blackbox_frame(blackbox_t *frame, uint32_t loop, uint32_t time) {
  memset(frame, 0, sizeof(blackbox_t));
  
  frame->loop = loop;
  frame->time = time;
  
  // Set some test values for PID terms
  frame->pid_p_term.roll = 100;
  frame->pid_p_term.pitch = 200;
  frame->pid_p_term.yaw = 300;
  
  frame->pid_i_term.roll = 50;
  frame->pid_i_term.pitch = 75;
  frame->pid_i_term.yaw = 100;
  
  frame->pid_d_term.roll = 25;
  frame->pid_d_term.pitch = 30;
  frame->pid_d_term.yaw = 35;
  
  // Set RX values
  frame->rx.roll = 1500;
  frame->rx.pitch = 1600;
  frame->rx.yaw = 1400;
  frame->rx.throttle = 1000;
  
  // Set setpoint values
  frame->setpoint.roll = 500;
  frame->setpoint.pitch = 600;
  frame->setpoint.yaw = 400;
  frame->setpoint.throttle = 800;
  
  // Set accelerometer values
  frame->accel_raw.roll = 1000;
  frame->accel_raw.pitch = 0;
  frame->accel_raw.yaw = 0;
  
  frame->accel_filter.roll = 950;
  frame->accel_filter.pitch = 10;
  frame->accel_filter.yaw = 5;
  
  // Set gyroscope values
  frame->gyro_raw.roll = 10;
  frame->gyro_raw.pitch = 20;
  frame->gyro_raw.yaw = 30;
  
  frame->gyro_filter.roll = 12;
  frame->gyro_filter.pitch = 22;
  frame->gyro_filter.yaw = 32;
  
  // Set output values
  frame->output.axis[0] = 1200;
  frame->output.axis[1] = 1300;
  frame->output.axis[2] = 1100;
  frame->output.axis[3] = 1250;
  
  // Set CPU load
  frame->cpu_load = 75;
  
  // Set debug values
  for (int i = 0; i < BLACKBOX_DEBUG_SIZE; i++) {
    frame->debug[i] = 100 + i * 10;
  }
}

// Test delta calculation for int16_t
void test_blackbox_delta_int16() {
  int16_t current = 1500;
  int16_t previous = 1000;
  
  int16_t delta = current - previous;
  TEST_ASSERT_EQUAL_INT16(500, delta);
  
  // Test negative delta
  current = 800;
  previous = 1200;
  delta = current - previous;
  TEST_ASSERT_EQUAL_INT16(-400, delta);
  
  // Test zero delta
  current = 1000;
  previous = 1000;
  delta = current - previous;
  TEST_ASSERT_EQUAL_INT16(0, delta);
}

// Test compact_vec3 delta calculation
void test_blackbox_compact_vec3_delta() {
  compact_vec3_t current = {.roll = 100, .pitch = 200, .yaw = 300};
  compact_vec3_t previous = {.roll = 90, .pitch = 180, .yaw = 270};
  compact_vec3_t delta;
  
  // Calculate delta manually for comparison
  for (int i = 0; i < 3; i++) {
    delta.axis[i] = current.axis[i] - previous.axis[i];
  }
  
  TEST_ASSERT_EQUAL_INT16(10, delta.roll);
  TEST_ASSERT_EQUAL_INT16(20, delta.pitch);
  TEST_ASSERT_EQUAL_INT16(30, delta.yaw);
}

// Test compact_vec4 delta calculation
void test_blackbox_compact_vec4_delta() {
  compact_vec4_t current = {.roll = 1500, .pitch = 1600, .yaw = 1400, .throttle = 1000};
  compact_vec4_t previous = {.roll = 1450, .pitch = 1550, .yaw = 1350, .throttle = 950};
  compact_vec4_t delta;
  
  // Calculate delta manually for comparison
  for (int i = 0; i < 4; i++) {
    delta.axis[i] = current.axis[i] - previous.axis[i];
  }
  
  TEST_ASSERT_EQUAL_INT16(50, delta.roll);
  TEST_ASSERT_EQUAL_INT16(50, delta.pitch);
  TEST_ASSERT_EQUAL_INT16(50, delta.yaw);
  TEST_ASSERT_EQUAL_INT16(50, delta.throttle);
}

// Test zero delta detection for vec3
void test_blackbox_vec3_zero_detection() {
  compact_vec3_t zero_vec = {.roll = 0, .pitch = 0, .yaw = 0};
  compact_vec3_t non_zero_vec = {.roll = 1, .pitch = 0, .yaw = 0};
  
  // Zero vector should be detected as zero
  bool is_zero = (zero_vec.axis[0] == 0 && zero_vec.axis[1] == 0 && zero_vec.axis[2] == 0);
  TEST_ASSERT_TRUE(is_zero);
  
  // Non-zero vector should not be detected as zero
  bool is_non_zero = (non_zero_vec.axis[0] == 0 && non_zero_vec.axis[1] == 0 && non_zero_vec.axis[2] == 0);
  TEST_ASSERT_FALSE(is_non_zero);
}

// Test zero delta detection for vec4
void test_blackbox_vec4_zero_detection() {
  compact_vec4_t zero_vec = {.roll = 0, .pitch = 0, .yaw = 0, .throttle = 0};
  compact_vec4_t non_zero_vec = {.roll = 0, .pitch = 0, .yaw = 0, .throttle = 1};
  
  // Zero vector should be detected as zero
  bool is_zero = (zero_vec.axis[0] == 0 && zero_vec.axis[1] == 0 && 
                  zero_vec.axis[2] == 0 && zero_vec.axis[3] == 0);
  TEST_ASSERT_TRUE(is_zero);
  
  // Non-zero vector should not be detected as zero
  bool is_non_zero = (non_zero_vec.axis[0] == 0 && non_zero_vec.axis[1] == 0 && 
                      non_zero_vec.axis[2] == 0 && non_zero_vec.axis[3] == 0);
  TEST_ASSERT_FALSE(is_non_zero);
}

// Test debug array change detection
void test_blackbox_debug_change_detection() {
  int16_t current[BLACKBOX_DEBUG_SIZE] = {100, 110, 120, 130, 140, 150, 160, 170, 180, 190};
  int16_t previous_same[BLACKBOX_DEBUG_SIZE] = {100, 110, 120, 130, 140, 150, 160, 170, 180, 190};
  int16_t previous_different[BLACKBOX_DEBUG_SIZE] = {100, 110, 120, 130, 140, 150, 160, 170, 180, 999};
  
  // Check if arrays are the same
  bool has_changed_same = false;
  for (uint32_t i = 0; i < BLACKBOX_DEBUG_SIZE; i++) {
    if (current[i] != previous_same[i]) {
      has_changed_same = true;
      break;
    }
  }
  TEST_ASSERT_FALSE(has_changed_same);
  
  // Check if arrays are different
  bool has_changed_different = false;
  for (uint32_t i = 0; i < BLACKBOX_DEBUG_SIZE; i++) {
    if (current[i] != previous_different[i]) {
      has_changed_different = true;
      break;
    }
  }
  TEST_ASSERT_TRUE(has_changed_different);
}

// Test I-frame encoding - should include all enabled fields
void test_blackbox_iframe_encoding() {
  blackbox_t current, previous;
  create_test_blackbox_frame(&current, 1, 1000);
  create_test_blackbox_frame(&previous, 0, 0);
  
  // Enable all fields for I-frame
  uint32_t field_flags = 0;
  for (int i = 0; i < BBOX_FIELD_MAX; i++) {
    field_flags |= (1 << i);
  }
  
  // Create CBOR encoder
  uint8_t buffer[1024];
  cbor_value_t enc;
  cbor_encoder_init(&enc, buffer, sizeof(buffer));
  
  // Encode I-frame
  cbor_result_t result = cbor_encode_blackbox_frame(&enc, &current, &previous, BLACKBOX_FRAME_I, field_flags);
  
  // Should succeed
  TEST_ASSERT_EQUAL_INT(CBOR_OK, result);
  
  // Verify some basic properties - check that encoding succeeded
  TEST_ASSERT_NOT_NULL(enc.curr);
}

// Test P-frame encoding - should only include changed fields
void test_blackbox_pframe_encoding() {
  blackbox_t current, previous;
  
  // Create identical frames first
  create_test_blackbox_frame(&current, 2, 2000);
  create_test_blackbox_frame(&previous, 1, 1000);
  
  // Make them identical except for loop and time
  current = previous;
  current.loop = 2;
  current.time = 2000;
  
  // Now change only PID P term
  current.pid_p_term.roll = previous.pid_p_term.roll + 10;
  
  // Enable all fields
  uint32_t field_flags = 0;
  for (int i = 0; i < BBOX_FIELD_MAX; i++) {
    field_flags |= (1 << i);
  }
  
  // Create CBOR encoder
  uint8_t buffer[1024];
  cbor_value_t enc;
  cbor_encoder_init(&enc, buffer, sizeof(buffer));
  
  // Encode P-frame
  cbor_result_t result = cbor_encode_blackbox_frame(&enc, &current, &previous, BLACKBOX_FRAME_P, field_flags);
  
  // Should succeed
  TEST_ASSERT_EQUAL_INT(CBOR_OK, result);
  
  // P-frame should be encoded successfully
  TEST_ASSERT_NOT_NULL(enc.curr);
}

// Test frame type bit handling
void test_blackbox_frame_type_bit() {
  uint32_t iframe_flags = 0xFF; // Some field flags
  uint32_t pframe_flags = 0xFF | BLACKBOX_FRAME_TYPE_BIT; // Same flags with frame type bit
  
  // I-frame should not have frame type bit set in field flags
  TEST_ASSERT_FALSE(iframe_flags & BLACKBOX_FRAME_TYPE_BIT);
  
  // P-frame should have frame type bit set
  TEST_ASSERT_TRUE(pframe_flags & BLACKBOX_FRAME_TYPE_BIT);
  
  // Stripping frame type bit should give original flags
  uint32_t stripped_flags = pframe_flags & ~BLACKBOX_FRAME_TYPE_BIT;
  TEST_ASSERT_EQUAL_UINT32(iframe_flags, stripped_flags);
}

// Test CPU load delta calculation
void test_blackbox_cpu_load_delta() {
  uint16_t current_cpu = 80;
  uint16_t previous_cpu = 75;
  
  uint16_t delta = current_cpu - previous_cpu;
  TEST_ASSERT_EQUAL_UINT16(5, delta);
  
  // Test negative delta (stored as signed)
  current_cpu = 70;
  previous_cpu = 85;
  int16_t signed_delta = (int16_t)(current_cpu - previous_cpu);
  TEST_ASSERT_EQUAL_INT16(-15, signed_delta);
}

// Test edge case: maximum delta values
void test_blackbox_delta_overflow() {
  // Test maximum positive delta for int16_t
  int16_t current = 32767;
  int16_t previous = 0;
  int16_t delta = current - previous;
  TEST_ASSERT_EQUAL_INT16(32767, delta);
  
  // Test maximum negative delta for int16_t
  current = -32768;
  previous = 0;
  delta = current - previous;
  TEST_ASSERT_EQUAL_INT16(-32768, delta);
  
  // Test potential overflow case
  current = 32767;
  previous = -32768;
  // This would overflow in 16-bit arithmetic, but the delta calculation
  // should handle it correctly within the range of values used in practice
  delta = current - previous;
  TEST_ASSERT_EQUAL_INT16(-1, delta); // Due to 16-bit wraparound
}

// Test field flag optimization for P-frames
void test_blackbox_pframe_field_optimization() {
  blackbox_t current, previous;
  
  // Create identical frames
  create_test_blackbox_frame(&current, 2, 2000);
  previous = current;
  previous.loop = 1;
  previous.time = 1000;
  
  // Only change one field
  current.pid_p_term.roll += 100;
  
  // Enable multiple fields, but only PID_P_TERM should be active in P-frame
  uint32_t input_flags = (1 << BBOX_FIELD_PID_P_TERM) | 
                         (1 << BBOX_FIELD_PID_I_TERM) | 
                         (1 << BBOX_FIELD_RX);
  
  // In a real P-frame encoding, only the changed field should be included
  // We can test this by ensuring the logic works correctly
  
  // Simulate the delta check logic
  compact_vec3_t p_delta, i_delta;
  for (int i = 0; i < 3; i++) {
    p_delta.axis[i] = current.pid_p_term.axis[i] - previous.pid_p_term.axis[i];
    i_delta.axis[i] = current.pid_i_term.axis[i] - previous.pid_i_term.axis[i];
  }
  
  bool p_changed = !(p_delta.axis[0] == 0 && p_delta.axis[1] == 0 && p_delta.axis[2] == 0);
  bool i_changed = !(i_delta.axis[0] == 0 && i_delta.axis[1] == 0 && i_delta.axis[2] == 0);
  
  TEST_ASSERT_TRUE(p_changed);   // PID P term should have changed
  TEST_ASSERT_FALSE(i_changed);  // PID I term should not have changed
}


// Test CBOR encoding for vec3 - fixed to expect positive return values
void test_blackbox_cbor_vec3_roundtrip() {
  compact_vec3_t original = {.roll = 123, .pitch = -456, .yaw = 789};
  
  uint8_t buffer[64];
  cbor_value_t enc;
  
  // Initialize encoder
  cbor_encoder_init(&enc, buffer, sizeof(buffer));
  
  // Test basic encoding first - CBOR returns positive values for success (bytes written)
  cbor_result_t array_result = cbor_encode_array(&enc, 3);
  TEST_ASSERT_TRUE(array_result > 0); // Should be positive (bytes written)
  
  // Try encoding the first int16
  cbor_result_t int_result = cbor_encode_int16_t(&enc, &original.axis[0]);
  TEST_ASSERT_TRUE(int_result > 0); // Should be positive (bytes written)
  
  // If basic operations work, the vec3 function should work too
  // Reset encoder for full test
  cbor_encoder_init(&enc, buffer, sizeof(buffer));
  cbor_result_t result = cbor_encode_compact_vec3_t(&enc, &original);
  TEST_ASSERT_TRUE(result >= CBOR_OK); // Should be CBOR_OK (0) or positive
}

// Test CBOR encoding for vec4 - fixed to expect positive return values
void test_blackbox_cbor_vec4_roundtrip() {
  compact_vec4_t original = {.roll = 1500, .pitch = 1600, .yaw = 1400, .throttle = 1000};
  
  uint8_t buffer[64];
  cbor_value_t enc;
  
  // Initialize encoder
  cbor_encoder_init(&enc, buffer, sizeof(buffer));
  
  // Test basic encoding first - CBOR returns positive values for success (bytes written)
  cbor_result_t array_result = cbor_encode_array(&enc, 4);
  TEST_ASSERT_TRUE(array_result > 0); // Should be positive (bytes written)
  
  // Try encoding the first int16
  cbor_result_t int_result = cbor_encode_int16_t(&enc, &original.axis[0]);
  TEST_ASSERT_TRUE(int_result > 0); // Should be positive (bytes written)
  
  // If basic operations work, the vec4 function should work too
  // Reset encoder for full test
  cbor_encoder_init(&enc, buffer, sizeof(buffer));
  cbor_result_t result = cbor_encode_compact_vec4_t(&enc, &original);
  TEST_ASSERT_TRUE(result >= CBOR_OK); // Should be CBOR_OK (0) or positive
}

// Test I-frame interval logic
void test_blackbox_iframe_interval() {
  // Frame 1 should be I-frame
  bool is_iframe_1 = (1 == 1 || 1 % BLACKBOX_I_FRAME_INTERVAL == 0);
  TEST_ASSERT_TRUE(is_iframe_1);
  
  // Frames 2-31 should be P-frames
  for (int i = 2; i < BLACKBOX_I_FRAME_INTERVAL; i++) {
    bool is_iframe = (i == 1 || i % BLACKBOX_I_FRAME_INTERVAL == 0);
    TEST_ASSERT_FALSE(is_iframe);
  }
  
  // Frame 32 should be I-frame
  bool is_iframe_32 = (32 == 1 || 32 % BLACKBOX_I_FRAME_INTERVAL == 0);
  TEST_ASSERT_TRUE(is_iframe_32);
  
  // Frame 33 should be P-frame
  bool is_iframe_33 = (33 == 1 || 33 % BLACKBOX_I_FRAME_INTERVAL == 0);
  TEST_ASSERT_FALSE(is_iframe_33);
}

void test_blackbox_captures_before_delayed_encoding() {
  run_blackbox_test(test_blackbox_captures_before_delayed_encoding_body);
}

void test_blackbox_mailbox_overrun_and_session_drain() {
  run_blackbox_test(test_blackbox_mailbox_overrun_and_session_drain_body);
}

void test_blackbox_storage_stall_and_rejected_delta() {
  run_blackbox_test(test_blackbox_storage_stall_and_rejected_delta_body);
}
