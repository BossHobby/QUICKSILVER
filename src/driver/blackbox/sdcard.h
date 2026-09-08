#pragma once

#include <stdint.h>

#include "io/blackbox_device.h"

#define SDCARD_PAGE_SIZE 512

typedef enum {
  SDCARD_WAIT,
  SDCARD_ERROR,
  SDCARD_IDLE,
} sdcard_status_t;

void sdcard_init();

sdcard_status_t sdcard_update();

void sdcard_get_bounds(blackbox_device_bounds_t *blackbox_bounds);

uint8_t sdcard_read_pages(uint8_t *buf, uint32_t page, uint32_t count);
uint8_t sdcard_write_page(uint8_t *buf, uint32_t page);

uint8_t sdcard_write_pages_start(uint32_t page, uint32_t count);
uint8_t sdcard_write_pages_continue(uint8_t *buf);
uint8_t sdcard_write_pages_finish();

// Decode the 16-byte, MSB-first CSD into addressable 512-byte blocks.
uint32_t sdcard_parse_csd(const uint8_t *csd);

typedef enum {
  SDCARD_RESPONSE_NONE,
  SDCARD_RESPONSE_R1,
  SDCARD_RESPONSE_R2,
  SDCARD_RESPONSE_R3,
  SDCARD_RESPONSE_R6,
  SDCARD_RESPONSE_R7,
} sdcard_response_t;

typedef enum {
  SDCARD_TRANSFER_WAIT,
  SDCARD_TRANSFER_DONE,
  SDCARD_TRANSFER_TIMEOUT,
  SDCARD_TRANSFER_UNSUPPORTED,
  SDCARD_TRANSFER_ERROR,
} sdcard_transfer_status_t;

typedef struct {
  uint32_t status;   // SPI R1 byte or native card status.
  uint32_t words[4]; // OCR/R7 or MSB-first native register response.
} sdcard_response_data_t;

// Transport calls are polled until completion; repeated calls must not resend.
// Native data is prepared before its command. SPI also permits preparation
// between blocks of a stream, or after a register-read command.
// Neither transport owns card metadata, block addresses, or operation sequencing.
typedef struct {
  bool spi;
  bool (*init)();
  void (*abort)();
  void (*configure)(); // Transfer clock, and four data lines for native SD.
  sdcard_transfer_status_t (*command)(uint8_t index, uint32_t argument, sdcard_response_t response, sdcard_response_data_t *result);
  void (*data_prepare)(uint8_t *buffer, uint32_t size, bool read);
  sdcard_transfer_status_t (*data_poll)();
  sdcard_transfer_status_t (*busy)(); // SPI busy bytes; native uses CMD13.
  void (*stop_write)();               // SPI multi-block stop token; unused in native SD.
} sdcard_transport_t;

void sdcard_init(const sdcard_transport_t *transport);
extern const sdcard_transport_t sdcard_spi;
extern const sdcard_transport_t sdcard_sdio;
