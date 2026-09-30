#pragma once

#include <stdbool.h>
#include <stdint.h>

// QUIC over CRSF 0x7F frames, addressed to the flight controller:
// <dest> <origin> <control> <seq> <ack> <QUIC stream bytes...>
// Data frames carry consecutive sequence numbers; ack is the next sequence
// expected from the peer. A frame without stream bytes only acknowledges.
// RESET opens a session and is answered with RESET once both streams are clear.
// Its sequence field carries a session id chosen by the peer and echoed back;
// a repeated RESET with the current id does not reopen the session.
#define QUIC_CRSF_CONTROL_RESET 0x01
#define QUIC_CRSF_HEADER_SIZE 3

// Called by IO for each received 0x7F frame payload after the extended header.
void quic_crsf_receive(uint8_t origin, const uint8_t *payload, uint8_t size);
// Called by IO when it may queue a frame; returns the frame size or 0.
uint32_t quic_crsf_frame(uint8_t *buf);
// A session exchanged frames recently; normal arming stays disabled.
bool quic_crsf_active();
// Ground maintenance service, run by the USB thread.
void quic_crsf_update();
