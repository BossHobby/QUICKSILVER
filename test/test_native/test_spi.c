#include <unity.h>

#include "driver/spi.h"
#include "driver/gpio.h"
#include "mock_helpers.h"

// Override target configuration for tests
extern target_t target;

// Test setup - configure target with valid SPI ports
static void test_spi_setup(void) {
  // Configure SPI ports with valid pins for testing
  target.spi_ports[SPI_PORT1] = (target_spi_port_t){
    .index = 1,
    .miso = PIN_B4,
    .mosi = PIN_B5,
    .sck = PIN_B3,
  };
  target.spi_ports[SPI_PORT2] = (target_spi_port_t){
    .index = 2,
    .miso = PIN_B14,
    .mosi = PIN_B15,
    .sck = PIN_B13,
  };
  target.spi_ports[SPI_PORT3] = (target_spi_port_t){
    .index = 3,
    .miso = PIN_C11,
    .mosi = PIN_C12,
    .sck = PIN_C10,
  };
}

// Test that SPI device initialization works
void test_spi_init(void) {
  test_spi_setup();
  
  const spi_bus_device_t bus = {
    .port = SPI_PORT1,
    .nss = PIN_A4,
    .mode = SPI_MODE_LEADING_EDGE,
    .hz = 1000000,
  };
  
  spi_bus_device_init(&bus);
  
  TEST_ASSERT_TRUE(spi_dev[SPI_PORT1].is_init);
  TEST_ASSERT_EQUAL(SPI_MODE_LEADING_EDGE, spi_dev[SPI_PORT1].mode);
  TEST_ASSERT_EQUAL(1000000, spi_dev[SPI_PORT1].hz);
}

// Test that SPI transactions can be queued
void test_spi_txn_queue(void) {
  test_spi_setup();
  
  spi_bus_device_t bus = {
    .port = SPI_PORT1,
    .nss = PIN_A4,
    .mode = SPI_MODE_LEADING_EDGE,
    .hz = 1000000,
  };
  
  spi_bus_device_init(&bus);
  
  // Create a simple SPI transaction
  const spi_txn_segment_t segs[] = {
    spi_make_seg_const(0x01, 0x02, 0x03),
  };
  
  TEST_ASSERT_TRUE(spi_txn_ready(&bus));
  
  spi_seg_submit_wait_ex(&bus, segs, 1);
  
  // Since this is a simulator, transaction completes immediately
  TEST_ASSERT_TRUE(spi_txn_ready(&bus));
}

// Test that SPI DMA is ready
void test_spi_dma_ready(void) {
  TEST_ASSERT_TRUE(spi_dma_is_ready(SPI_PORT1));
  TEST_ASSERT_TRUE(spi_dma_is_ready(SPI_PORT2));
  TEST_ASSERT_TRUE(spi_dma_is_ready(SPI_PORT3));
}

// Test reconfiguring SPI bus
void test_spi_reconfigure(void) {
  test_spi_setup();
  
  spi_bus_device_t bus = {
    .port = SPI_PORT1,
    .nss = PIN_A4,
    .mode = SPI_MODE_LEADING_EDGE,
    .hz = 1000000,
  };
  
  spi_bus_device_init(&bus);
  
  spi_bus_device_reconfigure(&bus, SPI_MODE_TRAILING_EDGE, 2000000);
  
  TEST_ASSERT_EQUAL(SPI_MODE_TRAILING_EDGE, bus.mode);
  TEST_ASSERT_EQUAL(2000000, bus.hz);
}