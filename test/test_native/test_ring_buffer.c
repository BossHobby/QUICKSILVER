#include <string.h>
#include <unity.h>

// Include the ring buffer utilities
#include "util/ring_buffer.h"

// Test buffer size
#define TEST_BUFFER_SIZE 16

// Test fixtures
static uint8_t test_buffer[TEST_BUFFER_SIZE];
static ring_buffer_t ring_buffer = {
    .buffer = test_buffer,
    .head = 0,
    .tail = 0,
    .size = TEST_BUFFER_SIZE
};

static void ring_buffer_setUp(void) {
  // Reset the ring buffer before each test
  ring_buffer.head = 0;
  ring_buffer.tail = 0;
  memset(test_buffer, 0, TEST_BUFFER_SIZE);
}

// Test initial state
void test_ring_buffer_init(void) {
  ring_buffer_setUp();
  
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&ring_buffer));
  TEST_ASSERT_EQUAL_UINT32(TEST_BUFFER_SIZE - 1, ring_buffer_free(&ring_buffer));
}

// Test single write and read
void test_ring_buffer_single_write_read(void) {
  ring_buffer_setUp();
  
  uint8_t write_data = 0x42;
  uint8_t read_data = 0;
  
  // Write should succeed
  TEST_ASSERT_EQUAL_UINT8(1, ring_buffer_write(&ring_buffer, write_data));
  TEST_ASSERT_EQUAL_UINT32(1, ring_buffer_available(&ring_buffer));
  TEST_ASSERT_EQUAL_UINT32(TEST_BUFFER_SIZE - 2, ring_buffer_free(&ring_buffer));
  
  // Read should succeed and return same data
  TEST_ASSERT_EQUAL_UINT8(1, ring_buffer_read(&ring_buffer, &read_data));
  TEST_ASSERT_EQUAL_UINT8(write_data, read_data);
  
  // Buffer should be empty again
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&ring_buffer));
  TEST_ASSERT_EQUAL_UINT32(TEST_BUFFER_SIZE - 1, ring_buffer_free(&ring_buffer));
}

// Test multiple writes and reads
void test_ring_buffer_multiple_write_read(void) {
  ring_buffer_setUp();
  
  uint8_t write_data[] = {1, 2, 3, 4, 5};
  uint8_t read_data[5] = {0};
  
  // Write multiple bytes
  for (int i = 0; i < 5; i++) {
    TEST_ASSERT_EQUAL_UINT8(1, ring_buffer_write(&ring_buffer, write_data[i]));
  }
  
  TEST_ASSERT_EQUAL_UINT32(5, ring_buffer_available(&ring_buffer));
  TEST_ASSERT_EQUAL_UINT32(TEST_BUFFER_SIZE - 1 - 5, ring_buffer_free(&ring_buffer));
  
  // Read multiple bytes
  for (int i = 0; i < 5; i++) {
    TEST_ASSERT_EQUAL_UINT8(1, ring_buffer_read(&ring_buffer, &read_data[i]));
    TEST_ASSERT_EQUAL_UINT8(write_data[i], read_data[i]);
  }
  
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&ring_buffer));
}

// Test multi write/read functions
void test_ring_buffer_multi_operations(void) {
  ring_buffer_setUp();
  
  uint8_t write_data[] = {10, 20, 30, 40, 50};
  uint8_t read_data[5] = {0};
  
  // Write multiple bytes at once
  uint32_t written = ring_buffer_write_multi(&ring_buffer, write_data, 5);
  TEST_ASSERT_EQUAL_UINT32(5, written);
  TEST_ASSERT_EQUAL_UINT32(5, ring_buffer_available(&ring_buffer));
  
  // Read multiple bytes at once
  uint32_t read = ring_buffer_read_multi(&ring_buffer, read_data, 5);
  TEST_ASSERT_EQUAL_UINT32(5, read);
  
  // Verify data
  for (int i = 0; i < 5; i++) {
    TEST_ASSERT_EQUAL_UINT8(write_data[i], read_data[i]);
  }
}

// Test buffer full condition
void test_ring_buffer_full(void) {
  ring_buffer_setUp();
  
  // Fill buffer to capacity - 1 (ring buffer reserves one slot)
  for (int i = 0; i < TEST_BUFFER_SIZE - 1; i++) {
    TEST_ASSERT_EQUAL_UINT8(1, ring_buffer_write(&ring_buffer, i));
  }
  
  // Buffer should be full
  TEST_ASSERT_EQUAL_UINT32(TEST_BUFFER_SIZE - 1, ring_buffer_available(&ring_buffer));
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_free(&ring_buffer));
  
  // Next write should fail
  TEST_ASSERT_EQUAL_UINT8(0, ring_buffer_write(&ring_buffer, 0xFF));
}

// Test read from empty buffer
void test_ring_buffer_empty_read(void) {
  ring_buffer_setUp();
  
  uint8_t data;
  
  // Read from empty buffer should fail
  TEST_ASSERT_EQUAL_UINT8(0, ring_buffer_read(&ring_buffer, &data));
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&ring_buffer));
}

// Test wrap around
void test_ring_buffer_wrap_around(void) {
  ring_buffer_setUp();
  
  // Fill half the buffer
  for (int i = 0; i < TEST_BUFFER_SIZE / 2; i++) {
    ring_buffer_write(&ring_buffer, i);
  }
  
  // Read half the buffer
  uint8_t dummy;
  for (int i = 0; i < TEST_BUFFER_SIZE / 2; i++) {
    ring_buffer_read(&ring_buffer, &dummy);
  }
  
  // Fill buffer again (this will wrap around)
  for (int i = 0; i < TEST_BUFFER_SIZE / 2; i++) {
    TEST_ASSERT_EQUAL_UINT8(1, ring_buffer_write(&ring_buffer, i + 100));
  }
  
  // Read and verify wrapped data
  for (int i = 0; i < TEST_BUFFER_SIZE / 2; i++) {
    uint8_t data;
    TEST_ASSERT_EQUAL_UINT8(1, ring_buffer_read(&ring_buffer, &data));
    TEST_ASSERT_EQUAL_UINT8(i + 100, data);
  }
}

// Test clear function
void test_ring_buffer_clear(void) {
  ring_buffer_setUp();
  
  // Write some data
  for (int i = 0; i < 5; i++) {
    ring_buffer_write(&ring_buffer, i);
  }
  
  TEST_ASSERT_EQUAL_UINT32(5, ring_buffer_available(&ring_buffer));
  
  // Clear the buffer
  ring_buffer_clear(&ring_buffer);
  
  // Buffer should be empty
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&ring_buffer));
  TEST_ASSERT_EQUAL_UINT32(TEST_BUFFER_SIZE - 1, ring_buffer_free(&ring_buffer));
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer.head);
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer.tail);
}

// Test partial multi-write when buffer is nearly full
void test_ring_buffer_partial_multi_write(void) {
  ring_buffer_setUp();
  
  // Fill buffer leaving only 3 free slots
  for (int i = 0; i < TEST_BUFFER_SIZE - 4; i++) {
    ring_buffer_write(&ring_buffer, i);
  }
  
  TEST_ASSERT_EQUAL_UINT32(3, ring_buffer_free(&ring_buffer));
  
  // Try to write 5 bytes when only 3 slots are free
  uint8_t data[] = {100, 101, 102, 103, 104};
  uint32_t written = ring_buffer_write_multi(&ring_buffer, data, 5);
  
  // Should only write 3 bytes
  TEST_ASSERT_EQUAL_UINT32(3, written);
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_free(&ring_buffer));
}

// Test partial multi-read when buffer has less data than requested
void test_ring_buffer_partial_multi_read(void) {
  ring_buffer_setUp();
  
  // Write only 3 bytes
  uint8_t write_data[] = {10, 20, 30};
  ring_buffer_write_multi(&ring_buffer, write_data, 3);
  
  // Try to read 5 bytes
  uint8_t read_data[5] = {0};
  uint32_t read = ring_buffer_read_multi(&ring_buffer, read_data, 5);
  
  // Should only read 3 bytes
  TEST_ASSERT_EQUAL_UINT32(3, read);
  
  // Verify correct data was read
  for (int i = 0; i < 3; i++) {
    TEST_ASSERT_EQUAL_UINT8(write_data[i], read_data[i]);
  }
}