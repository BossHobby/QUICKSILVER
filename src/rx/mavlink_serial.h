#pragma once

#include "rx/unified_serial.h"

float rx_serial_mavlink_expected_fps();
packet_status_t rx_serial_process_mavlink();
void rx_serial_send_mavlink_telemetry();
