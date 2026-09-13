#include <unity.h>

#include "control/control.h"
#include "core/profile.h"
#include "rx/rx.h"

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
