#include <string.h>
#include <unity.h>

#include "control/control.h"
#include "core/profile.h"
#include "driver/time.h"
#include "driver/osd/osd.h"
#include "osd/render.h"
#include "osd/status.h"

extern void simulator_osd_test_reset(bool reject);
extern uint32_t simulator_osd_test_push_count();
extern uint8_t simulator_osd_test_char(uint8_t x, uint8_t y);

void test_osd_transfer_is_bounded_and_retries() {
  osd_device_init();
  osd_clear();
  osd_start(OSD_ATTR_TEXT, 0, 0);
  osd_write_str("AB");
  osd_start(OSD_ATTR_INVERT, 2, 0);
  osd_write_str("CD");
  osd_start(OSD_ATTR_TEXT, 0, 1);
  osd_write_str("EF");

  simulator_osd_test_reset(true);
  TEST_ASSERT_FALSE(osd_update());
  TEST_ASSERT_EQUAL_UINT32(1, simulator_osd_test_push_count());
  TEST_ASSERT_EQUAL_UINT8(' ', simulator_osd_test_char(0, 0));
  simulator_osd_test_reset(false);
  for (uint32_t i = 1; i <= 3; i++) {
    TEST_ASSERT_FALSE(osd_update());
    TEST_ASSERT_EQUAL_UINT32(i, simulator_osd_test_push_count());
  }
  TEST_ASSERT_EQUAL_UINT8('A', simulator_osd_test_char(0, 0));
  TEST_ASSERT_EQUAL_UINT8('D', simulator_osd_test_char(3, 0));
  TEST_ASSERT_EQUAL_UINT8('F', simulator_osd_test_char(1, 1));
  TEST_ASSERT_FALSE(osd_update()); // Flush after the final string.
  TEST_ASSERT_TRUE(osd_update());

  // Clearing a partial transfer must restart at the first row.
  osd_start(OSD_ATTR_TEXT, 0, 5);
  osd_write_str("G");
  TEST_ASSERT_FALSE(osd_update());
  osd_clear();
  osd_start(OSD_ATTR_TEXT, 0, 0);
  osd_write_str("H");
  TEST_ASSERT_FALSE(osd_update());
  TEST_ASSERT_EQUAL_UINT8('H', simulator_osd_test_char(0, 0));
  osd_clear();
}

void test_osd_render_completes_before_transfer() {
  const auto saved_osd = profile.osd;
  for (auto &p : profile.osd.profiles) {
    memset(p.elements, 0, sizeof(p.elements));
    p.elements[OSD_CALLSIGN] = ENCODE_OSD_ELEMENT(1, 0, 0, 0, 0, 0);
    p.elements[OSD_CELL_COUNT] = ENCODE_OSD_ELEMENT(1, 0, 0, 1, 0, 1);
    strcpy((char *)p.callsign, "TEST");
  }
  osd_device_init();
  osd_clear();
  osd_display_reset();
  simulator_osd_test_reset(false);
  // Allow system/profile detection and screen clearing to finish.
  for (uint32_t i = 0; i < 12 && osd_state.element == OSD_CALLSIGN; i++)
    osd_display();
  TEST_ASSERT_EQUAL_INT(OSD_CALLSIGN + 1, osd_state.element);
  TEST_ASSERT_EQUAL_UINT32(0, simulator_osd_test_push_count());
  osd_display();
  TEST_ASSERT_EQUAL_INT(OSD_CELL_COUNT + 1, osd_state.element);
  TEST_ASSERT_EQUAL_UINT32(0, simulator_osd_test_push_count());
  osd_display(); // Skip inactive elements and finish the pass.
  TEST_ASSERT_EQUAL_INT(OSD_CALLSIGN, osd_state.element);
  TEST_ASSERT_EQUAL_UINT32(0, simulator_osd_test_push_count());
  osd_display();
  TEST_ASSERT_EQUAL_UINT32(1, simulator_osd_test_push_count());
  TEST_ASSERT_EQUAL_UINT8('T', simulator_osd_test_char(0, 0));
  osd_clear();
  osd_display_reset();
  profile.osd = saved_osd;
}

static void assert_looptime_label(bool visible) {
  for (uint32_t i = 0; i < 64; i++) {
    if (osd_update())
      break;
  }
  char row[51] = {};
  for (uint8_t x = 0; x < 50; x++)
    row[x] = simulator_osd_test_char(x, 0);
  TEST_ASSERT_EQUAL(visible, strstr(row, "LOOPTIME") != nullptr);
}

void test_osd_looptime_warning_is_temporary_until_2khz() {
  const auto saved_state = state;
  const auto saved_flags = flags;
  const auto saved_guac = profile.osd.guac_mode;
  flags = {};
  flags.rx_ready = 1;
  state = {};
  state.looptime_autodetect = 125;
  profile.osd.guac_mode = false;
  osd_element_t el = {};
  osd_device_init();
  osd_clear();
  osd_status_reset();
  simulator_osd_test_reset(false);
  osd_status_update(&el);
  osd_status_update(&el);

  state.looptime_warning = 1;
  state.looptime_autodetect = 250;
  osd_status_update(&el);
  assert_looptime_label(true);
  time_test_advance_us(1001000);
  osd_status_update(&el);
  osd_status_update(&el);
  assert_looptime_label(false);
  for (unsigned i = 0; i < 10; i++)
    osd_status_update(&el);
  assert_looptime_label(false);

  // A new scheduler session can show the same temporary warning again.
  state.looptime_warning = 0;
  state.looptime_autodetect = 125;
  osd_status_update(&el);
  state.looptime_warning = 1;
  state.looptime_autodetect = 250;
  osd_status_update(&el);
  assert_looptime_label(true);

  // Promote an active temporary warning to a persistent one at 2 kHz.
  state.looptime_warning = 2;
  state.looptime_autodetect = 500;
  osd_status_update(&el);
  time_test_advance_us(2000000);
  osd_status_update(&el);
  osd_status_update(&el);
  assert_looptime_label(true);

  state = saved_state;
  flags = saved_flags;
  profile.osd.guac_mode = saved_guac;
  osd_clear();
  osd_status_reset();
}
