#include "rx/unified_serial.h"

#include <stdbool.h>
#include <stdlib.h>

#include "core/profile.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "flight/control.h"

static bool fport_debug_telemetry = false;
static uint8_t telemetry_counter = 0;

extern uint8_t rx_buffer[RX_BUFF_SIZE];
extern uint8_t rx_data[RX_BUFF_SIZE];

extern volatile uint8_t rx_frame_position;
extern volatile frame_status_t frame_status;

extern uint16_t bind_safety;
extern int32_t channels[16];

extern uint8_t failsafe_sbus_failsafe;

extern profile_t profile;
extern int current_pid_axis;
extern int current_pid_term;

extern uint8_t telemetry_packet[64];

#define USART usart_port_defs[serial_rx_port]

bool rx_serial_process_sbus() {
  for (uint8_t counter = 0; counter < 25; counter++) {    // First up, get therx_data out of the RX buffer and into somewhere safe
    rx_data[counter] = rx_buffer[counter % RX_BUFF_SIZE]; // This can probably go away, as long as the buffer is large enough
  }

  if (rx_data[23] & (1 << 2)) {
    // RX sets this bit when it knows it missed a frame. Presumably this is a timer in the RX.
    rx_lqi_lost_packet();
  } else {
    rx_lqi_got_packet();
  }
  if (rx_data[23] & (1 << 3)) {
    failsafe_sbus_failsafe = 1;              // Sbus packets have a failsafe bit. This is cool. If you forget to trust it you get programs though.
    flags.failsafe = failsafe_sbus_failsafe; // set failsafe rtf-now
  } else {
    failsafe_sbus_failsafe = 0;
  }

  // Sbus channels, this could be a struct and a memcpy but eh.
  // All 16 are decoded, even if Quicksilver doesn't want to use them.
  channels[0] = ((rx_data[1] | rx_data[2] << 8) & 0x07FF);
  channels[1] = ((rx_data[2] >> 3 | rx_data[3] << 5) & 0x07FF);
  channels[2] = ((rx_data[3] >> 6 | rx_data[4] << 2 | rx_data[5] << 10) & 0x07FF);
  channels[3] = ((rx_data[5] >> 1 | rx_data[6] << 7) & 0x07FF);
  channels[4] = ((rx_data[6] >> 4 | rx_data[7] << 4) & 0x07FF);
  channels[5] = ((rx_data[7] >> 7 | rx_data[8] << 1 | rx_data[9] << 9) & 0x07FF);
  channels[6] = ((rx_data[9] >> 2 | rx_data[10] << 6) & 0x07FF);
  channels[7] = ((rx_data[10] >> 5 | rx_data[11] << 3) & 0x07FF);
  channels[8] = ((rx_data[12] | rx_data[13] << 8) & 0x07FF);
  channels[9] = ((rx_data[13] >> 3 | rx_data[14] << 5) & 0x07FF); // This is the last channel Silverware previously supported.
  channels[10] = ((rx_data[14] >> 6 | rx_data[15] << 2 | rx_data[16] << 10) & 0x07FF);
  channels[11] = ((rx_data[16] >> 1 | rx_data[17] << 7) & 0x07FF);
  channels[12] = ((rx_data[17] >> 4 | rx_data[18] << 4) & 0x07FF);
  channels[13] = ((rx_data[18] >> 7 | rx_data[19] << 1 | rx_data[20] << 9) & 0x07FF);
  channels[14] = ((rx_data[20] >> 2 | rx_data[21] << 6) & 0x07FF);
  channels[15] = ((rx_data[21] >> 5 | rx_data[22] << 3) & 0x07FF);

  // normal rx mode
  bind_safety++;
  if (bind_safety < 130)
    flags.rx_mode = RXMODE_BIND; // this is rapid flash during bind safety

  // AETR channel order
  const float rc_channels[4] = {
      (channels[0] - 993.f) * 0.00122026f,
      (channels[1] - 993.f) * 0.00122026f,
      (channels[2] - 993.f) * 0.00122026f,
      (channels[3] - 993.f) * 0.00122026f,
  };

  rx_map_channels(rc_channels);

  // Here we have the AUX channels Silverware supports
  state.aux[AUX_CHANNEL_0] = (channels[4] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_1] = (channels[5] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_2] = (channels[6] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_3] = (channels[7] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_4] = (channels[8] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_5] = (channels[9] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_6] = (channels[10] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_7] = (channels[11] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_8] = (channels[12] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_9] = (channels[13] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_10] = (channels[14] > 1600) ? 1 : 0;
  state.aux[AUX_CHANNEL_11] = (channels[15] > 1600) ? 1 : 0;

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL && profile.receiver.aux[AUX_RSSI] <= AUX_CHANNEL_11) {
    rx_lqi_update_direct(0.0610128f * (channels[(profile.receiver.aux[AUX_RSSI] + 4)] - 173));
  }

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
    rx_lqi_update_direct(0); // no internal rssi data
  }

  frame_status = FRAME_DONE; // We're done with this frame now.

  if (bind_safety > 131) {        // requires 130 good frames to come in before rx_ready safety can be toggled to 1.  About a second of good data
    flags.rx_ready = 1;           // because aux channels initialize low and clear the binding while armed flag before aux updates high
    flags.rx_mode = !RXMODE_BIND; // restores normal led operation
    bind_safety = 131;            // reset counter so it doesnt wrap
  }

  return true;
}

bool rx_serial_process_fport() {
  bool channels_received = false;

  uint8_t frameLength = 0;
  static uint8_t escapedChars = 0;
  uint8_t tempEscapedChars = 0;
  for (uint8_t counter = 0; counter <= rx_frame_position; counter++) {         // First up, get the data out of the RX buffer and into somewhere safe
    rx_data[counter] = rx_buffer[(counter + tempEscapedChars) % RX_BUFF_SIZE]; // We're going to be messing with the data and it wouldn't do to destroy a packet
    frameLength++;

    if (rx_data[counter - 1] == 0x7D) { // 0x7D and 0x7E are reserved, 0x7D is the marker for a reserved / escaped character
      if (rx_data[counter] == 0x5E) {   // 0x5E in the byte following 0x7D means it was a 0x7E data byte.
        rx_data[counter - 1] = 0x7E;    // So, make it 0x7E.
        escapedChars++;                 // Now we have to get rid of the "current" byte, and adjust the incoming frame length.
        tempEscapedChars++;
        counter--;
        frameLength--;
      } else if (rx_data[counter] == 0x5D) {
        escapedChars++;
        tempEscapedChars++;
        counter--;
        frameLength--;
      }
    }

    if (rx_data[counter] == 0x7e && rx_data[counter - 1] == 0x7e && frameLength > 29) { // Looks like a complete frame, check CRC and process controls if it's good.
      frame_status = FRAME_RX_DONE;
      // counter = 200; //Breaks out of the for loop processing the data array. - NFE-IS THIS STILL NEEDED?
      // The telemetry request packet is not read, as it never seems to change. Less control lag if we ignore it.
      uint16_t crc_byte = 0;
      for (int x = 1; x < frameLength - 2; x++) {
        crc_byte = crc_byte + rx_data[x];
      }
      crc_byte = crc_byte + (crc_byte >> 8);
      crc_byte = crc_byte << 8;
      crc_byte = crc_byte >> 8;

      if (crc_byte == 0x00FF) {
        // CRC is good, check Failsafe bit(s) and shove it into controls
        // FPORT uses SBUS style data, but starts further in the packet

        // RX appears to set this bit when it knows it missed a frame.
        if (rx_data[25] & (1 << 2)) {
          rx_lqi_lost_packet();
        } else {
          rx_lqi_got_packet();
        }
        if (rx_data[25] & (1 << 3)) {
          failsafe_sbus_failsafe = 1;              // Sbus packets have a failsafe bit. This is cool.
          flags.failsafe = failsafe_sbus_failsafe; // set failsafe rtf-now
        } else {
          failsafe_sbus_failsafe = 0;
        }

        channels[0] = ((rx_data[3] | rx_data[4] << 8) & 0x07FF);
        channels[1] = ((rx_data[4] >> 3 | rx_data[5] << 5) & 0x07FF);
        channels[2] = ((rx_data[5] >> 6 | rx_data[6] << 2 | rx_data[7] << 10) & 0x07FF);
        channels[3] = ((rx_data[7] >> 1 | rx_data[8] << 7) & 0x07FF);
        channels[4] = ((rx_data[8] >> 4 | rx_data[9] << 4) & 0x07FF);
        channels[5] = ((rx_data[9] >> 7 | rx_data[10] << 1 | rx_data[11] << 9) & 0x07FF);
        channels[6] = ((rx_data[11] >> 2 | rx_data[12] << 6) & 0x07FF);
        channels[7] = ((rx_data[12] >> 5 | rx_data[13] << 3) & 0x07FF);
        channels[8] = ((rx_data[14] | rx_data[15] << 8) & 0x07FF);
        channels[9] = ((rx_data[15] >> 3 | rx_data[16] << 5) & 0x07FF); // This is the last channel Silverware previously supported.
        channels[10] = ((rx_data[16] >> 6 | rx_data[17] << 2 | rx_data[18] << 10) & 0x07FF);
        channels[11] = ((rx_data[18] >> 1 | rx_data[19] << 7) & 0x07FF);
        channels[12] = ((rx_data[19] >> 4 | rx_data[20] << 4) & 0x07FF);
        channels[13] = ((rx_data[20] >> 7 | rx_data[21] << 1 | rx_data[22] << 9) & 0x07FF);
        channels[14] = ((rx_data[22] >> 2 | rx_data[23] << 6) & 0x07FF);
        channels[15] = ((rx_data[23] >> 5 | rx_data[24] << 3) & 0x07FF);

        channels_received = true;

      } else {
        frame_status = FRAME_IDLE;
      }
    }

    if (frame_status == FRAME_RX_DONE) {
      // normal rx mode
      bind_safety++;
      if (bind_safety < 130)
        flags.rx_mode = RXMODE_BIND; // this is rapid flash during bind safety

      // AETR channel order
      const float rc_channels[4] = {
          (channels[0] - 993.f) * 0.00122026f,
          (channels[1] - 993.f) * 0.00122026f,
          (channels[2] - 993.f) * 0.00122026f,
          (channels[3] - 993.f) * 0.00122026f,
      };

      rx_map_channels(rc_channels);

      // Here we have the AUX channels Silverware supports
      state.aux[AUX_CHANNEL_0] = (channels[4] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_1] = (channels[5] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_2] = (channels[6] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_3] = (channels[7] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_4] = (channels[8] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_5] = (channels[9] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_6] = (channels[10] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_7] = (channels[11] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_8] = (channels[12] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_9] = (channels[13] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_10] = (channels[14] > 993) ? 1 : 0;
      state.aux[AUX_CHANNEL_11] = (channels[15] > 993) ? 1 : 0;

      channels_received = true;

      if (channels[12] > 993) { // Channel 13 is now FPORT Debug Telemetry switch. Integrate this better sometime
        fport_debug_telemetry = true;
      } else {
        fport_debug_telemetry = false;
      }

      if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL) {
        if (profile.receiver.aux[AUX_RSSI] <= AUX_CHANNEL_11) {
          rx_lqi_update_direct(0.0610128f * (channels[(profile.receiver.aux[AUX_RSSI] + 4)] - 173));
        }
      }

      if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
        rx_lqi_update_direct(0); // no internal rssi data
      }

      frame_status = FRAME_DONE; // We're done with this frame now.
      telemetry_counter++;       // Let the telemetry section know it's time to send.

      if (bind_safety > 131) {        // requires 130 good frames to come in before rx_ready safety can be toggled to 1.  About a second of good data
        flags.rx_ready = 1;           // because aux channels initialize low and clear the binding while armed flag before aux updates high
        flags.rx_mode = !RXMODE_BIND; // restores normal led operation
        bind_safety = 131;            // reset counter so it doesnt wrap
      }
    }
  } // end frame received

  return channels_received;
}

vec3_t *get_pid_value(uint8_t term) {
  switch (term) {
  case 0:
    return &profile.pid.pid_rates[profile.pid.pid_profile].kp;
  case 1:
    return &profile.pid.pid_rates[profile.pid.pid_profile].ki;
  case 2:
    return &profile.pid.pid_rates[profile.pid.pid_profile].kd;
  }
  return NULL;
}

void rx_serial_send_fport_telemetry() {
  if (telemetry_counter > 1 && rx_frame_position >= 41) { // Send telemetry back every other packet. This gives the RX time to send ITS telemetry back
    static uint8_t skip_a_loop;
    skip_a_loop++;
    if (skip_a_loop < 3) {
      return;
    }
    skip_a_loop = 0;
    telemetry_counter = 0;
    frame_status = FRAME_DONE;

    uint16_t telemetry_ids[] = {
        0x0210, // VFAS, use for vbat_compensated
        0x0211, // VFAS1, use for vbat_filtered
        // Everything past here is only active in FPORT-Debug-Telemetry mode
        0x0900, // A3_FIRST_ID, used for cell count
        0x0400, // T1, used for Axis Identifier
        0x0700, // ACC-X, misused for PID-P
        0x0710, // ACC-X, misused for PID-I
        0x0720, // ACC-X, misused for PID-D
    };
    // This iterates through the above, you can only send one sensor per frame.
    static uint8_t telemetry_position = 0;

    // Telemetry time! Let's have some variables
    telemetry_packet[0] = 0x08; // Bytes 0 through 2 are static in this implementation
    telemetry_packet[1] = 0x81;
    telemetry_packet[2] = 0x10;
    if (telemetry_position == 0) {                                  // vbat_compensated
      telemetry_packet[3] = telemetry_ids[telemetry_position];      // 0x10;
      telemetry_packet[4] = telemetry_ids[telemetry_position] >> 8; // 0x02;
      telemetry_packet[5] = (int)(state.vbat_compensated * 100);
      telemetry_packet[6] = (int)(state.vbat_compensated * 100) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    } else if (telemetry_position == 1) {                           // vbat_filtered
      telemetry_packet[3] = telemetry_ids[telemetry_position];      // x11;
      telemetry_packet[4] = telemetry_ids[telemetry_position] >> 8; // 0x02;
      telemetry_packet[5] = (int)(state.vbat_filtered * 100);
      telemetry_packet[6] = (int)(state.vbat_filtered * 100) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    } else if (telemetry_position == 2) { // Cell count
      telemetry_packet[3] = telemetry_ids[telemetry_position];
      telemetry_packet[4] = telemetry_ids[telemetry_position] >> 8;
      telemetry_packet[5] = (int)(state.lipo_cell_count * 100);
      telemetry_packet[6] = (int)(state.lipo_cell_count * 100) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    } else if (telemetry_position == 3) {                           // PID axis(hundreds column) and P/I/D (ones column) being adjusted currently
      uint16_t axisAndPidID = (current_pid_axis + 1) * 100;         // Adding one so there's always a value. 1 for Pitch (or Pitch/roll), 2 for Roll, 3 for Yaw
      axisAndPidID += current_pid_term + 1;                         // Adding one here too, humans don't deal well with counting starting at zero for this sort of thing
      telemetry_packet[4] = telemetry_ids[telemetry_position] >> 8; // Adding one to the above makes it match the LED flash codes too
      telemetry_packet[5] = (int)(axisAndPidID);
      telemetry_packet[6] = (int)(axisAndPidID) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    } else if (telemetry_position == 4) { // PID-P
      telemetry_packet[3] = telemetry_ids[telemetry_position];
      telemetry_packet[4] = telemetry_ids[telemetry_position] >> 8;
      telemetry_packet[5] = (int)(get_pid_value(0)->axis[current_pid_axis] * 100);
      telemetry_packet[6] = (int)(get_pid_value(0)->axis[current_pid_axis] * 100) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    } else if (telemetry_position == 5) { // PID-I
      telemetry_packet[3] = telemetry_ids[telemetry_position];
      telemetry_packet[4] = telemetry_ids[telemetry_position] >> 8;
      telemetry_packet[5] = (int)(get_pid_value(1)->axis[current_pid_axis] * 100);
      telemetry_packet[6] = (int)(get_pid_value(1)->axis[current_pid_axis] * 100) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    } else if (telemetry_position == 6) { // PID-D
      telemetry_packet[3] = telemetry_ids[telemetry_position];
      telemetry_packet[4] = telemetry_ids[telemetry_position] >> 8;
      telemetry_packet[5] = (int)(get_pid_value(2)->axis[current_pid_axis] * 100);
      telemetry_packet[6] = (int)(get_pid_value(2)->axis[current_pid_axis] * 100) >> 8;
      telemetry_packet[7] = 0x00;
      telemetry_packet[8] = 0x00;
    }

    // This *should* properly escape 0x7D and 0x7E characters. It doesn't.
    uint32_t telemetry_size = 0;
    for (uint8_t i = 8; i > 4; i--) {
      if (telemetry_packet[i] == 0x7D || telemetry_packet[i] == 0x7E) {
        for (uint8_t x = 8; x >= i; x--) {
          telemetry_packet[x + 1] = telemetry_packet[x];
        }
        telemetry_packet[i] = 0x7D;
        if (telemetry_packet[i + 1] == 0x7D) {
          telemetry_packet[i + 1] = 0x5D;
        } else {
          telemetry_packet[i + 1] = 0x5E;
        }
        telemetry_size++;
      }
    }
    telemetry_packet[0] += telemetry_size;

    uint16_t teleCRC = 0;
    // Calculate CRC for packet. This function does not support escaped characters.
    for (int x = 0; x < 9 + telemetry_size; x++) {
      teleCRC = teleCRC + telemetry_packet[x];
    }
    teleCRC = teleCRC + (teleCRC >> 8);
    teleCRC = 0xff - teleCRC;
    teleCRC = teleCRC << 8;
    teleCRC = teleCRC >> 8;
    telemetry_packet[9 + telemetry_size] = teleCRC; // 0x34;

    rx_serial_send_telemetry(telemetry_size + 10);

    telemetry_position++;
    if (fport_debug_telemetry) {
      if (telemetry_position >= sizeof(telemetry_ids) / 2) // 2 byte ints, so this should give the number of entries. It just incremented, which takes care of the count with 0 or 1
      {
        telemetry_position = 0;
      }
    } else {
      if (telemetry_position == 2) {
        telemetry_position = 0;
      }
    }
  }
}