#include "mock_outputs.h"

#include <string.h>
#include <unity.h>

#include "driver/blackbox/m25p16.h"

float pwm_values[MOTOR_PIN_MAX];
uint16_t pwm_rate;
failloop_t fault;

void failloop(failloop_t value) { fault = value; }
void servo_pwm_write(const float *values) {
  for (unsigned i = 0; i < MOTOR_PIN_MAX; i++)
    pwm_values[i] = values[i];
}
void servo_pwm_init(const gpio_pins_t *, uint16_t rate, const float *values) {
  pwm_rate = rate;
  servo_pwm_write(values);
}
void servo_pwm_stop() {}

uint8_t mock_m25p16_data[2 * 65536];
static unsigned busy_ticks, erases, programs;

void mock_m25p16_reset() {
  memset(mock_m25p16_data, 0xff, sizeof(mock_m25p16_data));
  busy_ticks = erases = programs = 0;
}
unsigned mock_m25p16_busy() { return busy_ticks; }
void mock_m25p16_tick() {
  if (busy_ticks) busy_ticks--;
}
unsigned mock_m25p16_erases() { return erases; }
unsigned mock_m25p16_programs() { return programs; }

void m25p16_init() {}
void m25p16_wait_for_ready() { busy_ticks = 0; }
bool m25p16_is_ready() { return busy_ticks == 0; }

void m25p16_get_bounds(blackbox_device_bounds_t *bounds) {
  *bounds = {.page_size = 256, .pages_per_sector = 256, .sectors = 2,
             .sector_size = 65536, .total_size = sizeof(mock_m25p16_data)};
}

uint8_t m25p16_read_addr(uint8_t cmd, uint32_t addr, uint8_t *data, uint32_t len) {
  TEST_ASSERT_EQUAL(M25P16_READ_DATA_BYTES, cmd);
  TEST_ASSERT_TRUE(addr + len <= sizeof(mock_m25p16_data));
  memcpy(data, mock_m25p16_data + addr, len);
  return 0;
}

bool m25p16_page_program(uint32_t addr, const uint8_t *data, uint32_t size) {
  if (busy_ticks) return false;
  TEST_ASSERT_GREATER_THAN_UINT32(0, size);
  TEST_ASSERT_TRUE(addr + size <= sizeof(mock_m25p16_data));
  TEST_ASSERT_TRUE(addr / M25P16_PAGE_SIZE == (addr + size - 1) / M25P16_PAGE_SIZE);
  for (uint32_t i = 0; i < size; i++) mock_m25p16_data[addr + i] &= data[i];
  programs++;
  busy_ticks = 1;
  return true;
}

bool m25p16_write_addr(uint8_t cmd, uint32_t addr, uint8_t *, uint32_t size) {
  if (busy_ticks) return false;
  TEST_ASSERT_EQUAL(M25P16_SECTOR_ERASE, cmd);
  TEST_ASSERT_EQUAL_UINT32(0, addr);
  TEST_ASSERT_EQUAL_UINT32(0, size);
  memset(mock_m25p16_data, 0xff, 65536);
  erases++;
  busy_ticks = 240;
  return true;
}

bool m25p16_chip_erase() {
  if (busy_ticks) return false;
  memset(mock_m25p16_data, 0xff, sizeof(mock_m25p16_data));
  busy_ticks = 240;
  return true;
}
