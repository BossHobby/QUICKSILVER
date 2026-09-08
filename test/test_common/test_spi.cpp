#include <unity.h>

#include "driver/spi.h"
#include "driver/gpio.h"
#include "mock_helpers.h"

template <typename Segments>
concept spi_submittable = requires(spi_bus_device_t *bus, Segments &segs) {
  spi_seg_submit_wait(bus, segs);
  spi_seg_submit(bus, segs);
};

static_assert(spi_submittable<spi_txn_segment_t[1]>);
static_assert(spi_submittable<const spi_txn_segment_t[SPI_TXN_SEG_MAX - 1]>);
static_assert(!spi_submittable<spi_txn_segment_t *>);
static_assert(!spi_submittable<const spi_txn_segment_t *>);
static_assert(!spi_submittable<spi_txn_segment_t[SPI_TXN_SEG_MAX]>);
static_assert(!spi_submittable<uint8_t[4]>);

extern void spi_dma_complete_port(spi_ports_t port);

void test_spi_initial_state(void) {
  for (const auto &dev : spi_dev) {
    TEST_ASSERT_FALSE(dev.is_init);
    TEST_ASSERT_TRUE(dev.dma_done);
    TEST_ASSERT_TRUE(dev.dma_rx_done);
    TEST_ASSERT_TRUE(dev.dma_tx_done);
    TEST_ASSERT_EQUAL_UINT8(0, dev.txn_head);
    TEST_ASSERT_EQUAL_UINT8(0, dev.txn_tail);
  }
}

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

  const uint8_t tx[] = {0x12, 0x34};
  uint8_t rx[sizeof(tx)] = {};
  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(0x01),
      spi_make_seg_buffer(rx, tx, sizeof(tx)),
  };
  bool done = false;
  spi_seg_submit(&bus, segs, .done_fn = spi_txn_set_done, .done_fn_arg = &done);
  TEST_ASSERT_FALSE(spi_txn_ready(&bus));
  TEST_ASSERT_FALSE(done);

  TEST_ASSERT_TRUE(spi_txn_continue(&bus));
  spi_dma_complete_port(bus.port);

  const uint8_t expected[] = {0xed, 0xcb}; // Native SPI inverts transmitted bytes.
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, rx, sizeof(rx));
  TEST_ASSERT_TRUE(done);
  TEST_ASSERT_TRUE(spi_txn_ready(&bus));
}

void test_spi_txn_full_queue(void) {
  test_spi_setup();
  spi_bus_device_t bus = {
      .port = SPI_PORT1,
      .nss = PIN_A4,
      .mode = SPI_MODE_LEADING_EDGE,
      .hz = 1000000,
  };
  spi_bus_device_init(&bus);

  // Repeated fills also exercise wraparound at different ring positions.
  for (uint32_t round = 0; round < 3; round++) {
    uint8_t rx[SPI_TXN_MAX] = {};
    bool done[SPI_TXN_MAX] = {};
    for (uint32_t i = 0; i < SPI_TXN_MAX; i++) {
      const uint8_t tx = i + 1;
      const spi_txn_segment_t segs[] = {spi_make_seg_buffer(&rx[i], &tx, 1)};
      spi_seg_submit(&bus, segs, .done_fn = spi_txn_set_done, .done_fn_arg = &done[i]);
    }
    TEST_ASSERT_FALSE(spi_txn_has_free());
    TEST_ASSERT_EQUAL_UINT8(0, spi_txn_free_count());
    TEST_ASSERT_FALSE(spi_txn_ready(&bus));
    TEST_ASSERT_TRUE(spi_txn_continue(&bus));
    for (uint32_t i = 0; i < SPI_TXN_MAX; i++) {
      TEST_ASSERT_FALSE(done[i]);
      spi_dma_complete_port(bus.port);
      TEST_ASSERT_TRUE(done[i]);
      TEST_ASSERT_EQUAL_UINT8(uint8_t(~(i + 1)), rx[i]);
      TEST_ASSERT_EQUAL_UINT8(i + 1, spi_txn_free_count());
      TEST_ASSERT_EQUAL(i + 1 == SPI_TXN_MAX, spi_txn_ready(&bus));
    }
    TEST_ASSERT_TRUE(spi_txn_has_free());
  }
}

void test_spi_sdcard_block_transfers(void) {
  test_spi_setup();
  spi_bus_device_t bus = {
      .port = SPI_PORT1,
      .nss = PIN_A4,
      .mode = SPI_MODE_LEADING_EDGE,
      .hz = 25000000,
  };
  spi_bus_device_init(&bus);
  uint8_t block[512];
  for (uint32_t i = 0; i < sizeof(block); i++)
    block[i] = 0xa5;
  const spi_txn_segment_t read_segs[] = {
      spi_make_seg_buffer(block, nullptr, sizeof(block)),
      spi_make_seg_const(0xff, 0xff),
  };
  bool done = false;
  spi_seg_submit_continue(&bus, read_segs, .done_fn = spi_txn_set_done, .done_fn_arg = &done);
  spi_dma_complete_port(bus.port);
  TEST_ASSERT_TRUE(done);
  for (const auto byte : block)
    TEST_ASSERT_EQUAL_UINT8(0, byte); // Native SPI inverts the 0xff read clocks.

  uint8_t response = 0xff;
  const spi_txn_segment_t write_segs[] = {
      spi_make_seg_const(0xfc),
      spi_make_seg_buffer(nullptr, block, sizeof(block)),
      spi_make_seg_const(0xff, 0xff),
      spi_make_seg_buffer(&response, nullptr, 1),
  };
  done = false;
  spi_seg_submit_continue(&bus, write_segs, .done_fn = spi_txn_set_done, .done_fn_arg = &done);
  spi_dma_complete_port(bus.port);
  TEST_ASSERT_TRUE(done);
  TEST_ASSERT_EQUAL_UINT8(0, response);
  TEST_ASSERT_TRUE(spi_txn_ready(&bus));
  TEST_ASSERT_EQUAL_UINT8(SPI_TXN_MAX, spi_txn_free_count());
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
