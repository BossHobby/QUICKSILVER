#include <initializer_list>
#include <string.h>
#include <unity.h>

#include "driver/blackbox/sdcard.h"
#include "driver/spi.h"
#include "driver/time.h"

extern void spi_dma_complete_port(spi_ports_t port);

static spi_txn_t *card_transaction() {
  auto &port = spi_dev[SPI_PORT4];
  TEST_ASSERT_NOT_EQUAL(port.txn_head, port.txn_tail);
  return port.txns[(port.txn_tail + 1) % SPI_TXN_QUEUE_SIZE];
}

static void card_complete(std::initializer_list<uint8_t> bytes = {}) {
  auto *txn = card_transaction();
  TEST_ASSERT_EQUAL(TXN_IN_PROGRESS, txn->status);
  TEST_ASSERT_TRUE(bytes.size() <= txn->size);
  memcpy(txn->buffer, bytes.begin(), bytes.size());
  // These transport tests drive DMA directly without a running Blackbox task.
  txn->done_fn = nullptr;
  spi_dma_complete_port(SPI_PORT4);
}

static void card_init_spi() {
  target.sdcard.port = SPI_PORT4;
  target.sdcard.nss = PIN_A4;
  target.spi_ports[SPI_PORT4] = {.index = 4, .miso = PIN_B4, .mosi = PIN_B5, .sck = PIN_B3};
  TEST_ASSERT_TRUE(sdcard_spi.init());
  TEST_ASSERT_EQUAL_UINT(20, card_transaction()->size);
  card_complete();
}

void test_sdcard_spi_async_busy_deadline() {
  const auto saved_target = target;
  card_init_spi();
  // Completion and capture wakes before the deadline must not launch more DMA,
  // including when the microsecond clock wraps.
  time_test_set_us(UINT32_MAX - 500);
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, sdcard_spi.busy());
  const auto head = spi_dev[SPI_PORT4].txn_head;
  for (unsigned i = 0; i < 5; i++)
    TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, sdcard_spi.busy());
  TEST_ASSERT_EQUAL_UINT(head, spi_dev[SPI_PORT4].txn_head);
  card_complete({0});
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, sdcard_spi.busy());
  for (unsigned i = 0; i < 9; i++) {
    time_test_advance_us(100);
    TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, sdcard_spi.busy());
    TEST_ASSERT_EQUAL_UINT(head, spi_dev[SPI_PORT4].txn_head);
  }
  time_test_advance_us(100);
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, sdcard_spi.busy());
  TEST_ASSERT_NOT_EQUAL(head, spi_dev[SPI_PORT4].txn_head);
  card_complete({0xff});
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_DONE, sdcard_spi.busy());
  TEST_ASSERT_EQUAL_UINT(SPI_TXN_MAX, spi_txn_free_count());
  target = saved_target;
}

void test_sdcard_spi_async_command_and_data() {
  const auto saved_target = target;
  card_init_spi();
  sdcard_response_data_t response = {};
  auto command = [&]() { return sdcard_spi.command(8, 0x1aa, SDCARD_RESPONSE_R7, &response); };
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, command()); // Pre-command busy check.
  card_complete({0xff});
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, command()); // Six-byte command.
  TEST_ASSERT_EQUAL_UINT(6, card_transaction()->size);
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, command()); // DMA still pending.
  card_complete();
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, command());
  card_complete({0xff}); // Response not available yet.
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, command());
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, command());
  card_complete({1});
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, command()); // Extended response DMA.
  TEST_ASSERT_EQUAL_UINT(4, card_transaction()->size);
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, command());
  card_complete({0, 0, 1, 0xaa});
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_DONE, command());
  TEST_ASSERT_EQUAL_UINT(1, response.status);
  TEST_ASSERT_EQUAL_HEX32(0x1aa, response.words[0]);

  uint8_t data[16] = {};
  sdcard_spi.data_prepare(data, sizeof(data), true);
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, sdcard_spi.data_poll());
  card_complete({0xfe});
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, sdcard_spi.data_poll());
  TEST_ASSERT_EQUAL_UINT(sizeof(data) + 2, card_transaction()->size);
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, sdcard_spi.data_poll());
  card_complete({0x12, 0x34});
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_DONE, sdcard_spi.data_poll());
  TEST_ASSERT_EQUAL_HEX8(0x12, data[0]);
  TEST_ASSERT_EQUAL_HEX8(0x34, data[1]);

  sdcard_spi.data_prepare(data, sizeof(data), false);
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, sdcard_spi.data_poll());
  auto *txn = card_transaction();
  txn->buffer[txn->size - 1] = 0xff;
  card_complete();
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_WAIT, sdcard_spi.data_poll());
  card_complete({0x05});
  TEST_ASSERT_EQUAL(SDCARD_TRANSFER_DONE, sdcard_spi.data_poll());
  TEST_ASSERT_EQUAL_UINT(SPI_TXN_MAX, spi_txn_free_count());
  target = saved_target;
}
