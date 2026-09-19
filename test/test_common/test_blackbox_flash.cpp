#include <string.h>
#include <unity.h>

#include "io/blackbox_device_flash.h"
#include "mock_outputs.h"

static TickType_t wait;

static void service_tick() {
  mock_m25p16_tick();
  // Exhaust immediate software transitions, then model the next timed wake.
  for (unsigned i = 0; i < 8; i++) {
    wait = 1;
    blackbox_device_flash.update(wait);
    if (wait != 0) return;
  }
  TEST_FAIL_MESSAGE("Flash service did not yield");
}

static void finish_recording() {
  blackbox_device_flash.stop();
  for (unsigned i = 0; i < 500 && !blackbox_device_flash.ready(); i++) service_tick();
  TEST_ASSERT_TRUE(blackbox_device_flash.ready());
  TEST_ASSERT_EQUAL_UINT32(0, mock_m25p16_busy());
}

static void flash_test_init() {
  mock_m25p16_reset();
  blackbox_device_header = {};
  blackbox_device_header.magic = BLACKBOX_HEADER_MAGIC;
  memcpy(mock_m25p16_data, &blackbox_device_header, sizeof(blackbox_device_header));
  ring_buffer_clear(&blackbox_encode_buffer);
  blackbox_device_flash.init();
  service_tick();
  TEST_ASSERT_TRUE(blackbox_device_flash.ready());
}

static void start_recording(uint32_t offset) {
  auto &file = blackbox_device_header.files[blackbox_device_header.file_num++];
  file = {};
  file.start = offset;
  blackbox_device_flash.start();
}

void test_flash_streams_without_startup_erase_and_commits_at_stop() {
  flash_test_init();
  start_recording(65536);
  uint8_t sample[60];
  // More than a FIFO's worth of data during the former 240 ms startup erase.
  for (unsigned i = 0; i < 220; i++) {
    memset(sample, i, sizeof(sample));
    TEST_ASSERT_TRUE(blackbox_device_flash.write(sample, sizeof(sample)));
    service_tick();
  }
  TEST_ASSERT_EQUAL_UINT32(0, mock_m25p16_erases());
  finish_recording();
  TEST_ASSERT_EQUAL_UINT32(1, mock_m25p16_erases());
  TEST_ASSERT_EQUAL_UINT32(53, mock_m25p16_programs()); // 52 data pages, one directory page.
  TEST_ASSERT_EQUAL_UINT32(13200, blackbox_current_file()->size);
  for (unsigned i = 0; i < 220; i++) {
    memset(sample, i, sizeof(sample));
    TEST_ASSERT_EQUAL_MEMORY(sample, mock_m25p16_data + 65536 + i * sizeof(sample), sizeof(sample));
  }
  TEST_ASSERT_EQUAL_MEMORY(&blackbox_device_header, mock_m25p16_data, sizeof(blackbox_device_header));

  // A partial final page must not misalign the next recording's page writes.
  start_recording(65536 + 52 * 256);
  memset(sample, 0x5a, sizeof(sample));
  TEST_ASSERT_TRUE(blackbox_device_flash.write(sample, sizeof(sample)));
  finish_recording();
  TEST_ASSERT_EQUAL_UINT32(2, mock_m25p16_erases());
  TEST_ASSERT_EQUAL_UINT32(55, mock_m25p16_programs());
  TEST_ASSERT_EQUAL_MEMORY(sample, mock_m25p16_data + blackbox_current_file()->start, sizeof(sample));

  blackbox_device_header = {};
  blackbox_device_flash.init();
  service_tick();
  TEST_ASSERT_EQUAL_UINT32(2, blackbox_device_header.file_num);
  TEST_ASSERT_EQUAL_UINT32(13200, blackbox_device_header.files[0].size);
  TEST_ASSERT_EQUAL_UINT32(60, blackbox_device_header.files[1].size);
}

void test_flash_flushes_last_page_at_capacity() {
  flash_test_init();
  start_recording(sizeof(mock_m25p16_data) - 256);
  uint8_t sample[200];
  memset(sample, 0x5a, sizeof(sample));
  TEST_ASSERT_TRUE(blackbox_device_flash.write(sample, sizeof(sample)));
  TEST_ASSERT_TRUE(blackbox_device_flash.write(sample, sizeof(sample)));
  wait = 1;
  blackbox_device_flash.update(wait);
  TEST_ASSERT_EQUAL_UINT32(1, mock_m25p16_programs());
  TEST_ASSERT_EQUAL_UINT32(256, blackbox_current_file()->size);
  finish_recording();
  TEST_ASSERT_EQUAL_UINT32(256, blackbox_current_file()->size);
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&blackbox_encode_buffer));
  TEST_ASSERT_EQUAL_UINT32(2, mock_m25p16_programs());
  TEST_ASSERT_EQUAL_MEMORY(sample, mock_m25p16_data + sizeof(mock_m25p16_data) - 256, sizeof(sample));
}
