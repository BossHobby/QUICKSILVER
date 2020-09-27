#include "rx_unified_serial.h"

#ifdef RX_UNIFIED_SERIAL

#include <stdbool.h>
#include <stdlib.h>

#include "control.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "profile.h"

extern int rx_bind_enable;
extern float rx_rssi;

extern uint8_t rx_buffer[RX_BUFF_SIZE];
extern uint8_t rx_data[RX_BUFF_SIZE];
extern uint8_t rx_frame_position;
extern uint8_t expected_frame_length;

extern frame_status_t frame_status;

extern uint16_t link_quality_raw;
extern uint8_t stat_frames_second;
extern uint32_t time_siglost;
extern uint32_t time_lastframe;

extern uint16_t bind_safety;
extern int32_t channels[16];

extern uint8_t failsafe_sbus_failsafe;
extern uint8_t failsafe_siglost;
extern uint8_t failsafe_noframes;

extern profile_t profile;
extern int current_pid_axis;
extern int current_pid_term;

extern uint8_t telemetry_offset;
extern uint8_t telemetry_packet[14];
extern uint8_t ready_for_next_telemetry;

#define USART usart_port_defs[serial_rx_port]

void rx_serial_process_crsf(void) {
  //We should probably put something here.
}

#endif