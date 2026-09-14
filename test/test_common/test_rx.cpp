#include <unity.h>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "control/control.h"
#include "core/profile.h"
#include "core/tasks.h"
#include "driver/adc.h"
#include "driver/time.h"
#include "io/vbat.h"
#include "rx/rx.h"

extern uint8_t adc_active_channels;
extern void adc_set_raw_value(adc_chan_t chan, uint16_t value);

extern void simulator_rx_test_frame(const uint16_t *channels);

void test_rx_transport_leaves_conditioning_to_flight() {
  const auto saved_state = state;
  const auto saved_protocol = profile.receiver.protocol;
  profile.receiver.protocol = RX_PROTOCOL_INVALID;
  state.rx_filter_hz = 20.0f;
  state.looptime_autodetect = 125.0f;
  rx_init();

  state.rx.roll = 1.0f;
  state.rx_filtered.roll = 0.0f;
  rx_update();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.rx_filtered.roll);

  rx_process();
  const float first = state.rx_filtered.roll;
  TEST_ASSERT_TRUE(first > 0.0f && first < 1.0f);
  // Smoothing must keep advancing even when transport has not run again.
  rx_process();
  TEST_ASSERT_TRUE(state.rx_filtered.roll > first);
  TEST_ASSERT_TRUE(state.rx_filtered.roll < 1.0f);

  state.rx_filter_hz = 0.0f;
  state.rx.roll = 2.0f;
  state.rx.throttle = -1.0f;
  rx_update();
  TEST_ASSERT_EQUAL_FLOAT(2.0f, state.rx.roll);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, state.rx.throttle);
  rx_process();
  TEST_ASSERT_EQUAL_FLOAT(1.0f, state.rx_filtered.roll);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.rx_filtered.throttle);

  state = saved_state;
  profile.receiver.protocol = saved_protocol;
}

void test_rx_mailbox_coalesces_complete_frames() {
  const auto saved_state = state;
  const auto saved_flags = flags;
  const auto saved_profile = profile;
  profile.receiver.protocol = RX_PROTOCOL_INVALID;
  flags = {};
  rx_init();
  uint16_t channels[RX_CHANNEL_MAX];
  for (auto &channel : channels) channel = 1000;
  simulator_rx_test_frame(channels);
  rx_update();
  for (auto &channel : channels) channel = 2000;
  simulator_rx_test_frame(channels);
  rx_update();
  for (auto channel : state.rx_channels) TEST_ASSERT_EQUAL_UINT16(0, channel);

  // A decoder may be building another frame when Flight preempts it.
  rx_channels[0] = 3000;
  rx_process();
  for (auto channel : state.rx_channels) TEST_ASSERT_EQUAL_UINT16(2000, channel);
  rx_process();
  TEST_ASSERT_EQUAL_UINT16(2000, state.rx_channels[0]);

  state = saved_state;
  flags = saved_flags;
  profile = saved_profile;
}

static void io_test_flight(void *) {
  flags = {};
  state = {};
  profile_set_defaults(&profile);
  profile_output_update();
  profile.receiver.protocol = RX_PROTOCOL_INVALID;
  profile.serial.gps = SERIAL_PORT_INVALID;
  profile.voltage.lipo_cell_count = 1;
  state.looptime_autodetect = 1000;
  rx_init();
  adc_init();
  vbat_init();
  profile_mutex_init();
  if (xSemaphoreTake(profile_mutex, 0) != pdTRUE) _exit(1);
  thread_start(THREAD_IO);

  uint16_t channels[RX_CHANNEL_MAX];
  for (auto &channel : channels) channel = 1000;
  simulator_rx_test_frame(channels);
  // A different source arriving before IO runs must preserve the RX bit.
  xTaskNotify(threads[THREAD_IO].handle, IO_WORK_BARO, eSetBits);
  vTaskDelay(3);
  // Receiving does not wait for configuration ownership; Flight still owns
  // channel publication and conditioning.
  if (state.rx_channels[0] != 0) _exit(2);
  rx_process();
  for (auto channel : state.rx_channels) if (channel != 1000) _exit(4);

  flags.arm_state = 1;
  threads_update();
  for (auto &channel : channels) channel = 2000;
  simulator_rx_test_frame(channels);
  vTaskDelay(3);
  rx_process();
  for (auto channel : state.rx_channels) if (channel != 2000) _exit(5);
  if (eTaskGetState(threads[THREAD_IO].handle) == eSuspended) _exit(6);
  xSemaphoreGive(profile_mutex);
  _exit(0);
}

void test_io_worker_publishes_rx_without_configuration_wait() {
  const pid_t child = fork();
  TEST_ASSERT_TRUE(child >= 0);
  if (child == 0) {
    threads[THREAD_FLIGHT].entry = io_test_flight;
    thread_start(THREAD_FLIGHT);
    vTaskStartScheduler();
    _exit(7);
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
  TEST_FAIL_MESSAGE("IO worker stalled");
}

static void io_cadence_test_flight(void *) {
  flags = {};
  state = {};
  profile_set_defaults(&profile);
  profile_output_update();
  profile.receiver.protocol = RX_PROTOCOL_INVALID;
  profile.serial.gps = SERIAL_PORT_INVALID;
  profile.voltage.lipo_cell_count = 1;
  profile.voltage.ibat_scale = 1000;
  target.ibat = PIN_A2;
  state.looptime_autodetect = 1000;
  rx_init();
  adc_init();
  adc_active_channels = 4;
  adc_set_raw_value(ADC_CHAN_IBAT, 1000);
  vbat_init();
  profile_mutex_init();
  xSemaphoreTake(profile_mutex, 0);
  thread_start(THREAD_IO);
  vTaskDelay(2);
  xSemaphoreGive(profile_mutex);

  // No notification of any kind: the cadence deadlines alone must wake the
  // worker and run the measurement pass.
  for (unsigned i = 0; i < 50; i++) {
    time_test_advance_us(1000);
    vTaskDelay(1);
  }
  if (!(state.ibat_drawn > 0)) _exit(2);
  const float before = state.ibat_drawn;
  // Repeated RX wakes must not postpone the battery's own deadline.
  for (unsigned i = 0; i < 50; i++) {
    time_test_advance_us(1000);
    xTaskNotify(threads[THREAD_IO].handle, IO_WORK_RX, eSetBits);
    vTaskDelay(1);
  }
  if (!(state.ibat_drawn > before)) _exit(5);
  if (eTaskGetState(threads[THREAD_IO].handle) == eSuspended) _exit(3);
  _exit(0);
}

void test_io_worker_services_cadence_without_notification() {
  const pid_t child = fork();
  TEST_ASSERT_TRUE(child >= 0);
  if (child == 0) {
    threads[THREAD_FLIGHT].entry = io_cadence_test_flight;
    thread_start(THREAD_FLIGHT);
    vTaskStartScheduler();
    _exit(4);
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
  TEST_FAIL_MESSAGE("IO worker stalled");
}
