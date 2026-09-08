#include <initializer_list>
#include <string.h>
#include <unity.h>

#include "core/target.h"
#include "driver/blackbox/sdcard.h"
#include "driver/time.h"
#include "io/blackbox_device_sdcard.h"

static struct {
  bool spi;
  bool legacy;
  bool configured;
  bool aborted;
  bool command_pending;
  bool stall_command;
  bool stall_data;
  bool fail_data;
  bool stall_busy;
  uint32_t busy_polls;
  uint32_t op_cond_polls;
  uint32_t stop_tokens;
  uint32_t command_count;
  uint8_t commands[512];
  uint32_t arguments[512];
  uint8_t *buffer;
  uint32_t size;
  bool read;
  bool data_pending;
  uint32_t blocks_read;
  uint32_t blocks_written;
} card;

static const uint8_t card_csd_v1[16] = {0, 0, 0, 0, 0, 9, 3, 0xff, 0xc0, 3, 0x80};
static const uint8_t card_csd_v2[16] = {0x40, 0, 0, 0, 0, 0, 0, 0, 3, 0xff};

static bool card_init() { return true; }
static void card_abort() { card.aborted = true; }
static void card_configure() { card.configured = true; }
static void card_stop_write() { card.stop_tokens++; }

static sdcard_transfer_status_t card_command(uint8_t index, uint32_t argument, sdcard_response_t response, sdcard_response_data_t *result) {
  if (!card.command_pending) {
    TEST_ASSERT_LESS_THAN_UINT32(sizeof(card.commands), card.command_count);
    card.commands[card.command_count] = index;
    card.arguments[card.command_count++] = argument;
    card.command_pending = true;
    return SDCARD_TRANSFER_WAIT;
  }
  TEST_ASSERT_EQUAL_UINT8(card.commands[card.command_count - 1], index);
  TEST_ASSERT_EQUAL_UINT32(card.arguments[card.command_count - 1], argument);
  if (card.stall_command)
    return SDCARD_TRANSFER_WAIT;
  card.command_pending = false;
  *result = {};
  switch (index) {
  case 0:
    result->status = card.spi ? 1 : 0;
    break;
  case 8:
    TEST_ASSERT_EQUAL(SDCARD_RESPONSE_R7, response);
    if (card.legacy)
      return card.spi ? SDCARD_TRANSFER_UNSUPPORTED : SDCARD_TRANSFER_TIMEOUT;
    result->words[0] = 0x1aa;
    break;
  case 55:
    result->status = card.spi ? 0 : 0x20;
    break;
  case 41:
    TEST_ASSERT_EQUAL_UINT32(card.legacy ? 0 : 1u << 30, argument & (1u << 30));
    card.op_cond_polls++;
    result->status = card.spi && card.op_cond_polls == 1 ? 1 : 0;
    result->words[0] = card.op_cond_polls == 1 ? 0 : card.legacy ? 0x80000000
                                                                 : 0xc0000000;
    break;
  case 58:
    result->words[0] = card.legacy ? 0x80000000 : 0xc0000000;
    break;
  case 3:
    result->words[0] = 0x12340000;
    break;
  case 9:
    if (!card.spi) {
      TEST_ASSERT_EQUAL_UINT32(0x12340000, argument);
      const auto csd = card.legacy ? card_csd_v1 : card_csd_v2;
      for (uint32_t i = 0; i < 16; i++)
        result->words[i / 4] |= uint32_t(csd[i]) << (24 - (i % 4) * 8);
    }
    break;
  case 13:
    card.busy_polls++;
    result->status = card.stall_busy ? 0xe00 : 0x900;
    break;
  case 17:
  case 24:
    TEST_ASSERT_NOT_NULL(card.buffer); // Native DMA must be armed before the command.
    TEST_ASSERT_EQUAL_UINT32(SDCARD_PAGE_SIZE, card.size);
    break;
  default:
    break;
  }
  return SDCARD_TRANSFER_DONE;
}

static void card_data_prepare(uint8_t *buffer, uint32_t size, bool read) {
  card.buffer = buffer;
  card.size = size;
  card.read = read;
  card.data_pending = false;
}

static sdcard_transfer_status_t card_data_poll() {
  if (card.fail_data)
    return SDCARD_TRANSFER_ERROR;
  if (card.stall_data || !card.data_pending) {
    card.data_pending = true;
    return SDCARD_TRANSFER_WAIT;
  }
  if (card.read) {
    if (card.size == 16)
      memcpy(card.buffer, card.legacy ? card_csd_v1 : card_csd_v2, 16);
    else
      memset(card.buffer, ++card.blocks_read, card.size);
  } else {
    card.blocks_written++;
  }
  card.buffer = nullptr;
  return SDCARD_TRANSFER_DONE;
}

static sdcard_transfer_status_t card_busy() {
  card.busy_polls++;
  return card.stall_busy ? SDCARD_TRANSFER_WAIT : SDCARD_TRANSFER_DONE;
}

static sdcard_transport_t card_transport = {
    .init = card_init,
    .abort = card_abort,
    .configure = card_configure,
    .command = card_command,
    .data_prepare = card_data_prepare,
    .data_poll = card_data_poll,
    .busy = card_busy,
    .stop_write = card_stop_write,
};

static void card_wait() {
  sdcard_status_t status = SDCARD_WAIT;
  for (uint32_t i = 0; i < 200 && status == SDCARD_WAIT; i++) {
    time_test_advance_us(1000);
    status = sdcard_update();
  }
  TEST_ASSERT_EQUAL(SDCARD_IDLE, status);
}

static void card_start(bool spi, bool legacy = false) {
  card = {.spi = spi, .legacy = legacy};
  card_transport.spi = spi;
  target.sdcard_detect = {};
  sdcard_init(&card_transport);
  card_wait();
  TEST_ASSERT_TRUE(card.configured);
  TEST_ASSERT_FALSE(card.aborted);
}

void test_sdcard_transport_initialization() {
  for (const bool spi : {false, true}) {
    for (const bool legacy : {false, true}) {
      card_start(spi, legacy);
      blackbox_device_bounds_t bounds;
      sdcard_get_bounds(&bounds);
      TEST_ASSERT_EQUAL_UINT32(legacy ? 2097152 : 1048576, bounds.sectors);
      TEST_ASSERT_EQUAL_UINT32(SDCARD_PAGE_SIZE, bounds.page_size);
      const uint8_t native_commands[] = {0, 8, 55, 41, 55, 41, 2, 3, 9, 7, 13, 55, 6};
      const uint8_t spi_commands[] = {0, 8, 55, 41, 55, 41, 58, 9};
      const auto expected = spi ? spi_commands : native_commands;
      const uint32_t count = spi ? sizeof(spi_commands) : sizeof(native_commands);
      uint32_t found = 0;
      for (uint32_t i = 0; i < card.command_count; i++) {
        if (card.commands[i] == 16) {
          TEST_ASSERT_TRUE(legacy);
          TEST_ASSERT_EQUAL_UINT32(512, card.arguments[i]);
        } else {
          TEST_ASSERT_LESS_THAN_UINT32(count, found);
          TEST_ASSERT_EQUAL_UINT8(expected[found++], card.commands[i]);
        }
      }
      TEST_ASSERT_EQUAL_UINT32(count, found);
      TEST_ASSERT_EQUAL_UINT32(count + legacy, card.command_count);
    }
  }
}

void test_sdcard_transport_late_insertion() {
  for (const bool spi : {false, true}) {
    card = {.spi = spi};
    card_transport.spi = spi;
    // Native GPIO reads high; invert selects absent/present for this test.
    target.sdcard_detect = {.pin = PIN_A4, .invert = true};
    sdcard_init(&card_transport);
    for (uint32_t i = 0; i < 3; i++) {
      time_test_advance_us(3000000);
      TEST_ASSERT_EQUAL(SDCARD_WAIT, sdcard_update());
    }
    TEST_ASSERT_FALSE(card.aborted);
    TEST_ASSERT_EQUAL_UINT32(0, card.command_count);
    time_test_advance_us(3000000);
    target.sdcard_detect.invert = false;
    card_wait();
    TEST_ASSERT_TRUE(card.configured);
    TEST_ASSERT_FALSE(card.aborted);

    // Removal after initialization still fails the active session.
    target.sdcard_detect.invert = true;
    TEST_ASSERT_EQUAL(SDCARD_ERROR, sdcard_update());
    TEST_ASSERT_TRUE(card.aborted);
    target.sdcard_detect = {};
  }
}

void test_sdcard_transport_reads() {
  for (const bool spi : {false, true}) {
    for (const bool legacy : {false, true}) {
      card_start(spi, legacy);
      uint8_t buffer[1024] = {};
      const uint32_t first = card.command_count;
      TEST_ASSERT_FALSE(sdcard_read_pages(buffer, 7, 2));
      card_wait();
      TEST_ASSERT_EQUAL_UINT8(1, buffer[0]);
      TEST_ASSERT_EQUAL_UINT8(2, buffer[512]);
      TEST_ASSERT_TRUE(sdcard_read_pages(buffer, 7, 2));
      TEST_ASSERT_EQUAL_UINT32(legacy ? 7 * 512 : 7, card.arguments[first]);
      if (spi) {
        TEST_ASSERT_EQUAL_UINT8(18, card.commands[first]);
        TEST_ASSERT_EQUAL_UINT8(12, card.commands[first + 1]);
        TEST_ASSERT_EQUAL_UINT32(first + 2, card.command_count);
      } else {
        TEST_ASSERT_EQUAL_UINT8(17, card.commands[first]);
        TEST_ASSERT_EQUAL_UINT8(13, card.commands[first + 1]);
        TEST_ASSERT_EQUAL_UINT8(17, card.commands[first + 2]);
        TEST_ASSERT_EQUAL_UINT32(legacy ? 8 * 512 : 8, card.arguments[first + 2]);
      }
    }
  }
}

void test_sdcard_transport_writes() {
  for (const bool spi : {false, true}) {
    card_start(spi);
    uint8_t buffer[512] = {};
    TEST_ASSERT_FALSE(sdcard_write_pages_start(7, 1));
    card_wait();
    TEST_ASSERT_TRUE(sdcard_write_pages_start(7, 1));
    for (uint32_t i = 0; i < 2; i++) {
      TEST_ASSERT_FALSE(sdcard_write_pages_continue(buffer));
      card_wait();
      TEST_ASSERT_TRUE(sdcard_write_pages_continue(buffer));
      TEST_ASSERT_EQUAL_UINT32(i + 1, card.blocks_written);
    }
    TEST_ASSERT_FALSE(sdcard_write_pages_finish());
    card_wait();
    TEST_ASSERT_TRUE(sdcard_write_pages_finish());
    TEST_ASSERT_EQUAL_UINT32(spi ? 1 : 0, card.stop_tokens);
    uint32_t writes = 0;
    for (uint32_t i = 0; i < card.command_count; i++) {
      if (card.commands[i] == 24 || card.commands[i] == 25)
        TEST_ASSERT_EQUAL_UINT32(7 + writes++, card.arguments[i]);
    }
    TEST_ASSERT_EQUAL_UINT32(spi ? 1 : 2, writes);

    bool done = false;
    for (uint32_t i = 0; i < 100 && !done; i++) {
      sdcard_update();
      done = sdcard_write_page(buffer, 0);
    }
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_EQUAL_UINT32(3, card.blocks_written);
  }
}

void test_sdcard_transport_errors() {
  for (const bool spi : {false, true}) {
    uint8_t buffer[512];
    card_start(spi);
    card.fail_data = true;
    sdcard_read_pages(buffer, 0, 1);
    for (uint32_t i = 0; i < 10; i++)
      sdcard_update();
    TEST_ASSERT_EQUAL(SDCARD_ERROR, sdcard_update());
    TEST_ASSERT_TRUE(card.aborted);
    TEST_ASSERT_FALSE(sdcard_read_pages(buffer, 0, 1));

    card_start(spi);
    card.stall_data = true;
    sdcard_read_pages(buffer, 0, 1);
    for (uint32_t i = 0; i < 10; i++)
      sdcard_update();
    time_test_advance_us(1000001);
    TEST_ASSERT_EQUAL(SDCARD_ERROR, sdcard_update());
    TEST_ASSERT_TRUE(card.aborted);

    card_start(spi);
    card.stall_command = true;
    sdcard_read_pages(buffer, 0, 1);
    sdcard_update();
    time_test_advance_us(100001);
    TEST_ASSERT_EQUAL(SDCARD_ERROR, sdcard_update());
    TEST_ASSERT_TRUE(card.aborted);

    card_start(spi);
    sdcard_read_pages(buffer, 1048575, 2);
    TEST_ASSERT_EQUAL(SDCARD_ERROR, sdcard_update());
    TEST_ASSERT_TRUE(card.aborted);
  }
}

void test_sdcard_transport_busy_and_timer_wrap() {
  for (const bool spi : {false, true}) {
    time_test_set_us(UINT32_MAX - 1000);
    card_start(spi);
    uint8_t buffer[512] = {};
    sdcard_write_pages_start(0, 1);
    card_wait();
    TEST_ASSERT_TRUE(sdcard_write_pages_start(0, 1));
    card.stall_busy = true;
    sdcard_write_pages_continue(buffer);
    for (uint32_t i = 0; i < 30; i++)
      TEST_ASSERT_EQUAL(SDCARD_WAIT, sdcard_update());
    TEST_ASSERT_FALSE(sdcard_write_pages_continue(buffer));
    TEST_ASSERT_GREATER_THAN_UINT32(0, card.busy_polls);
    card.stall_busy = false;
    card_wait();
    TEST_ASSERT_TRUE(sdcard_write_pages_continue(buffer));
    sdcard_write_pages_finish();
    card_wait();
    TEST_ASSERT_TRUE(sdcard_write_pages_finish());
  }
}

static void card_device_wait_ready() {
  for (uint32_t i = 0; i < 200 && !blackbox_device_sdcard.ready(); i++) {
    time_test_advance_us(1000);
    blackbox_device_sdcard.update();
  }
  TEST_ASSERT_TRUE(blackbox_device_sdcard.ready());
}

void test_sdcard_device_samples_during_write() {
  for (const bool spi : {false, true}) {
    ring_buffer_clear(&blackbox_encode_buffer);
    blackbox_device_sdcard.init();
    TEST_ASSERT_FALSE(blackbox_device_sdcard.update());
    card_start(spi);
    card_device_wait_ready();

    blackbox_device_header.file_num = 1;
    blackbox_device_header.files[0] = {.start = 512};
    blackbox_device_sdcard.start();
    TEST_ASSERT_FALSE(blackbox_device_sdcard.update());
    card_device_wait_ready();

    uint8_t sample[32] = {};
    for (uint32_t i = 0; i < 16; i++)
      TEST_ASSERT_TRUE(blackbox_device_sdcard.write(sample, sizeof(sample)));
    const uint32_t writes = card.blocks_written;
    card.stall_busy = true;
    for (uint32_t i = 0; i < 40; i++) {
      time_test_advance_us(125);
      TEST_ASSERT_TRUE(blackbox_device_sdcard.update());
    }
    TEST_ASSERT_EQUAL_UINT32(writes + 1, card.blocks_written);
    TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&blackbox_encode_buffer));

    // Reproduce the longest observed card pause while continuing 1 kHz sampling.
    for (uint32_t i = 0; i < 183; i++) {
      time_test_advance_us(1000);
      TEST_ASSERT_TRUE(blackbox_device_sdcard.update());
      TEST_ASSERT_TRUE(blackbox_device_sdcard.write(sample, sizeof(sample)));
    }
    TEST_ASSERT_EQUAL_UINT32(183 * sizeof(sample), ring_buffer_available(&blackbox_encode_buffer));
    TEST_ASSERT_EQUAL_UINT32(writes + 1, card.blocks_written);

    blackbox_device_sdcard.stop();
    TEST_ASSERT_FALSE(blackbox_device_sdcard.ready());
    card.stall_busy = false;
    card_device_wait_ready();
    TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&blackbox_encode_buffer));
    TEST_ASSERT_EQUAL_UINT32((16 + 183) * sizeof(sample), blackbox_current_file()->size);

    // Finishing the flush must leave the device ready for another file.
    blackbox_device_sdcard.start();
    TEST_ASSERT_FALSE(blackbox_device_sdcard.update());
    card_device_wait_ready();
    TEST_ASSERT_TRUE(blackbox_device_sdcard.update());

    for (uint32_t i = 0; i < 16; i++)
      TEST_ASSERT_TRUE(blackbox_device_sdcard.write(sample, sizeof(sample)));
    card.fail_data = true;
    for (uint32_t i = 0; i < 40 && !card.aborted; i++)
      blackbox_device_sdcard.update();
    TEST_ASSERT_TRUE(card.aborted);
    TEST_ASSERT_FALSE(blackbox_device_sdcard.update());
    ring_buffer_clear(&blackbox_encode_buffer);
  }
}

void test_sdcard_csd_capacity() {
  // CSD v1: READ_BL_LEN=9, C_SIZE=4095, C_SIZE_MULT=7: 1 GiB.
  uint8_t sdsc[16] = {0, 0, 0, 0, 0, 9, 3, 0xff, 0xc0, 3, 0x80};
  TEST_ASSERT_EQUAL_UINT32(2097152, sdcard_parse_csd(sdsc));
  sdsc[5] = 8; // Cannot supply 512-byte blocks.
  TEST_ASSERT_EQUAL_UINT32(0, sdcard_parse_csd(sdsc));

  // CSD v2 C_SIZE=1023: 512 MiB.
  uint8_t sdhc[16] = {0x40, 0, 0, 0, 0, 0, 0, 0, 3, 0xff};
  TEST_ASSERT_EQUAL_UINT32(1048576, sdcard_parse_csd(sdhc));
  sdhc[8] = 0x7f; // 16 GiB, capped by blackbox's 32-bit byte offsets.
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX / SDCARD_PAGE_SIZE, sdcard_parse_csd(sdhc));
  sdhc[7] = 0x3f;
  sdhc[8] = 0xff; // Maximum SDXC CSD must not wrap to zero.
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX / SDCARD_PAGE_SIZE, sdcard_parse_csd(sdhc));
  sdhc[0] = 0x80; // Unsupported CSD version.
  TEST_ASSERT_EQUAL_UINT32(0, sdcard_parse_csd(sdhc));
}

void test_sdcard_target_cbor() {
  // YAML supplies a port selector and a separate indexed pin mapping.
  uint8_t sdio[] = {0xa2, 0x66, 's', 'd', 'c', 'a', 'r', 'd', 0xa1, 0x64, 's', 'd', 'i', 'o', 1,
                    0x6a, 's', 'd', 'i', 'o', '_', 'p', 'o', 'r', 't', 's', 0x81, 0xa7,
                    0x65, 'i', 'n', 'd', 'e', 'x', 1,
                    0x63, 'c', 'l', 'k', 0x64, 'P', 'C', '1', '2',
                    0x63, 'c', 'm', 'd', 0x63, 'P', 'D', '2',
                    0x62, 'd', '0', 0x63, 'P', 'C', '8',
                    0x62, 'd', '1', 0x63, 'P', 'C', '9',
                    0x62, 'd', '2', 0x64, 'P', 'C', '1', '0',
                    0x62, 'd', '3', 0x64, 'P', 'C', '1', '1'};
  target_t decoded = {};
  cbor_value_t codec;
  cbor_decoder_init(&codec, sdio, sizeof(sdio));
  TEST_ASSERT_TRUE(cbor_decode_target_t(&codec, &decoded) >= CBOR_OK);
  TEST_ASSERT_EQUAL_UINT8(1, decoded.sdcard.sdio);
  TEST_ASSERT_EQUAL(SPI_PORT_INVALID, decoded.sdcard.port);
  TEST_ASSERT_EQUAL(PIN_NONE, decoded.sdcard.nss);
  TEST_ASSERT_EQUAL_UINT8(0, decoded.sdio_ports[0].index);
  const auto port = decoded.sdio_ports[1];
  TEST_ASSERT_EQUAL_UINT8(1, port.index);
  TEST_ASSERT_EQUAL(PIN_C12, port.clk);
  TEST_ASSERT_EQUAL(PIN_D2, port.cmd);
  TEST_ASSERT_EQUAL(PIN_C8, port.d0);
  TEST_ASSERT_EQUAL(PIN_C9, port.d1);
  TEST_ASSERT_EQUAL(PIN_C10, port.d2);
  TEST_ASSERT_EQUAL(PIN_C11, port.d3);
  TEST_ASSERT_TRUE(target_sdio_port_valid(&port));

  // Legacy SPI targets remain readable without the new field.
  uint8_t spi[] = {0xa1, 0x66, 's', 'd', 'c', 'a', 'r', 'd', 0xa2,
                   0x64, 'p', 'o', 'r', 't', 1,
                   0x63, 'n', 's', 's', 0x63, 'P', 'A', '4'};
  decoded = {};
  cbor_decoder_init(&codec, spi, sizeof(spi));
  TEST_ASSERT_TRUE(cbor_decode_target_t(&codec, &decoded) >= CBOR_OK);
  TEST_ASSERT_EQUAL_UINT8(0, decoded.sdcard.sdio);
  TEST_ASSERT_EQUAL(SPI_PORT1, decoded.sdcard.port);
  TEST_ASSERT_EQUAL(PIN_A4, decoded.sdcard.nss);

  decoded.sdcard.sdio = 2;
  decoded.sdcard.port = SPI_PORT_INVALID;
  decoded.sdcard.nss = PIN_NONE;
  decoded.sdio_ports[1] = port;
  decoded.sdio_ports[2] = {.index = 2, .clk = PIN_C1, .cmd = PIN_A0, .d0 = PIN_B14, .d1 = PIN_B15, .d2 = PIN_B3, .d3 = PIN_B4};
  TEST_ASSERT_TRUE(target_sdio_port_valid(&decoded.sdio_ports[2]));
  uint8_t buffer[4096];
  cbor_encoder_init(&codec, buffer, sizeof(buffer));
  TEST_ASSERT_TRUE(cbor_encode_target_t(&codec, &decoded) >= CBOR_OK);
  const auto size = cbor_encoder_len(&codec);
  target_t roundtrip = {};
  cbor_decoder_init(&codec, buffer, size);
  TEST_ASSERT_TRUE(cbor_decode_target_t(&codec, &roundtrip) >= CBOR_OK);
  TEST_ASSERT_EQUAL_UINT8(2, roundtrip.sdcard.sdio);
  TEST_ASSERT_EQUAL_UINT8(2, roundtrip.sdio_ports[2].index);
  TEST_ASSERT_EQUAL(PIN_C1, roundtrip.sdio_ports[2].clk);
  TEST_ASSERT_EQUAL(PIN_A0, roundtrip.sdio_ports[2].cmd);
  TEST_ASSERT_EQUAL(PIN_B14, roundtrip.sdio_ports[2].d0);
  TEST_ASSERT_EQUAL(PIN_B15, roundtrip.sdio_ports[2].d1);
  TEST_ASSERT_EQUAL(PIN_B3, roundtrip.sdio_ports[2].d2);
  TEST_ASSERT_EQUAL(PIN_B4, roundtrip.sdio_ports[2].d3);
  TEST_ASSERT_EQUAL(PIN_C12, roundtrip.sdio_ports[1].clk);
  TEST_ASSERT_EQUAL(PIN_D2, roundtrip.sdio_ports[1].cmd);
  TEST_ASSERT_EQUAL(PIN_C8, roundtrip.sdio_ports[1].d0);
  TEST_ASSERT_EQUAL(PIN_C9, roundtrip.sdio_ports[1].d1);
  TEST_ASSERT_EQUAL(PIN_C10, roundtrip.sdio_ports[1].d2);
  TEST_ASSERT_EQUAL(PIN_C11, roundtrip.sdio_ports[1].d3);
  TEST_ASSERT_TRUE(target_sdio_port_valid(&roundtrip.sdio_ports[1]));

  // Omitting unused ports must retain explicit indices, including a gap before SDMMC2.
  decoded.sdio_ports[1] = {};
  cbor_encoder_init(&codec, buffer, sizeof(buffer));
  TEST_ASSERT_TRUE(cbor_encode_target_t(&codec, &decoded) >= CBOR_OK);
  const auto sparse_size = cbor_encoder_len(&codec);
  TEST_ASSERT_LESS_THAN_UINT32(size, sparse_size);
  roundtrip = {};
  cbor_decoder_init(&codec, buffer, sparse_size);
  TEST_ASSERT_TRUE(cbor_decode_target_t(&codec, &roundtrip) >= CBOR_OK);
  TEST_ASSERT_EQUAL_UINT8(0, roundtrip.sdio_ports[1].index);
  TEST_ASSERT_EQUAL_UINT8(2, roundtrip.sdio_ports[2].index);
  TEST_ASSERT_EQUAL(PIN_C1, roundtrip.sdio_ports[2].clk);
  TEST_ASSERT_EQUAL_UINT8(2, roundtrip.sdcard.sdio);

  auto invalid = port;
  invalid.index = SDIO_PORT_MAX;
  TEST_ASSERT_FALSE(target_sdio_port_valid(&invalid));
  invalid = port;
  invalid.d3 = PIN_NONE;
  TEST_ASSERT_FALSE(target_sdio_port_valid(&invalid));
  invalid = port;
  invalid.d3 = port.d0;
  TEST_ASSERT_FALSE(target_sdio_port_valid(&invalid));
}
