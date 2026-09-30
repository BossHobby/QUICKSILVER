#include "io/quic_crsf.h"

#include <stdlib.h>
#include <string.h>

#include "control/control.h"
#include "core/tasks.h"
#include "driver/interrupt.h"
#include "driver/time.h"
#include "io/quic.h"
#include "rx/crsf.h"
#include "util/util.h"

// Frames in flight; the receiver drops CRSF frames once its 512 byte
// downlink queue is full, so the window must stay well below that.
#define QUIC_CRSF_WINDOW 4
#define QUIC_CRSF_DATA_MAX (CRSF_PAYLOAD_SIZE_MAX - CRSF_FRAME_ORIGIN_DEST_SIZE - QUIC_CRSF_HEADER_SIZE)
// Stream buffer sizes must be powers of two; indices are free-running.
#define QUIC_CRSF_RX_SIZE 512
#define QUIC_CRSF_TX_SIZE 4096
#define QUIC_CRSF_REQUEST_SIZE 4096
#define QUIC_CRSF_RETRANSMIT_MS 2000
#define QUIC_CRSF_ACTIVE_MS 3000

// Link state belongs to IO, which exchanges frames with the receiver.
static struct {
  uint8_t origin;
  uint8_t session; // chosen by the peer, carried in the RESET sequence field
  bool open;
  bool reset_reply;
  bool ack_pending;
  uint8_t rx_seq; // next sequence expected from the peer
  uint8_t tx_seq; // sequence of the oldest unacknowledged frame
  uint8_t inflight;
  uint32_t inflight_end[QUIC_CRSF_WINDOW];
  uint32_t tx_sent;
  uint32_t tx_time;
  uint32_t applied_reset;
} link;

// Streams between IO and the USB thread. Each index has a single writer:
// IO owns rx_head, tx_acked and reset_request; USB owns rx_tail, tx_head and
// reset_done. While a reset is pending IO leaves the streams alone, so USB may
// discard their contents.
static uint8_t rx_buffer[QUIC_CRSF_RX_SIZE];
static volatile uint32_t rx_head = 0;
static volatile uint32_t rx_tail = 0;
static uint8_t *tx_buffer = NULL;
static volatile uint32_t tx_head = 0;
static volatile uint32_t tx_acked = 0;
static volatile uint32_t reset_request = 0;
static volatile uint32_t reset_done = 0;
static volatile uint32_t active_time = 0;
static volatile bool active_seen = false;

// Request assembly belongs to the USB thread. Buffers are allocated with the
// first session and kept, so IO never observes them being released.
static uint8_t *request = NULL;
static uint32_t request_size = 0;

static void crsf_quic_send(uint8_t *data, uint32_t len, void *priv);

static quic_t quic = {
    .send = crsf_quic_send,
};

bool quic_crsf_active() {
  return active_seen && (time_millis() - active_time) < QUIC_CRSF_ACTIVE_MS;
}

void quic_crsf_receive(uint8_t origin, const uint8_t *payload, uint8_t size) {
  if (size < QUIC_CRSF_HEADER_SIZE || flags.arm_state)
    return;

  active_time = time_millis();
  active_seen = true;

  const uint8_t control = payload[0];
  const uint8_t seq = payload[1];
  const uint8_t ack = payload[2];

  if (control & QUIC_CRSF_CONTROL_RESET) {
    // A retried RESET for the open session only repeats the answer.
    if (link.open && link.origin == origin && link.session == seq) {
      link.reset_reply = true;
      return;
    }
    link.origin = origin;
    link.session = seq;
    link.open = false;
    MEMORY_BARRIER();
    if (reset_done == reset_request)
      reset_request = reset_request + 1;
    return;
  }

  if (!link.open)
    return;

  const uint8_t acked = ack - link.tx_seq;
  if (acked > 0 && acked <= link.inflight) {
    tx_acked = link.inflight_end[acked - 1];
    link.inflight -= acked;
    memmove(link.inflight_end, link.inflight_end + acked, link.inflight * sizeof(link.inflight_end[0]));
    link.tx_seq = ack;
    link.tx_time = time_millis();
  }

  const uint8_t data_size = size - QUIC_CRSF_HEADER_SIZE;
  if (data_size == 0)
    return;

  // Duplicates and frames that do not fit are acknowledged with the sequence
  // still expected; the peer resends from there.
  link.ack_pending = true;
  const uint32_t head = rx_head;
  if (seq != link.rx_seq || QUIC_CRSF_RX_SIZE - (head - rx_tail) < data_size)
    return;

  for (uint8_t i = 0; i < data_size; i++)
    rx_buffer[(head + i) % QUIC_CRSF_RX_SIZE] = payload[QUIC_CRSF_HEADER_SIZE + i];
  MEMORY_BARRIER();
  rx_head = head + data_size;
  link.rx_seq++;
}

uint32_t quic_crsf_frame(uint8_t *buf) {
  if (!link.open) {
    if (reset_done != reset_request || link.applied_reset == reset_done)
      return 0;

    link = {
        .origin = link.origin,
        .session = link.session,
        .open = true,
        .reset_reply = true,
        .tx_sent = tx_acked,
        .applied_reset = reset_done,
    };
  }

  if (!quic_crsf_active())
    return 0;

  const uint32_t now = time_millis();
  if (link.inflight && (now - link.tx_time) > QUIC_CRSF_RETRANSMIT_MS) {
    link.inflight = 0;
    link.tx_sent = tx_acked;
  }

  uint8_t payload[QUIC_CRSF_HEADER_SIZE + QUIC_CRSF_DATA_MAX];
  payload[0] = link.reset_reply ? QUIC_CRSF_CONTROL_RESET : 0;
  payload[1] = link.reset_reply ? link.session : link.tx_seq + link.inflight;
  payload[2] = link.rx_seq;
  uint8_t size = QUIC_CRSF_HEADER_SIZE;

  const uint32_t pending = tx_head - link.tx_sent;
  MEMORY_BARRIER();
  if (!link.reset_reply && pending && link.inflight < QUIC_CRSF_WINDOW) {
    const uint32_t chunk = MIN(pending, (uint32_t)QUIC_CRSF_DATA_MAX);
    for (uint32_t i = 0; i < chunk; i++)
      payload[size++] = tx_buffer[(link.tx_sent + i) % QUIC_CRSF_TX_SIZE];

    if (link.inflight == 0)
      link.tx_time = now;
    link.tx_sent += chunk;
    link.inflight_end[link.inflight++] = link.tx_sent;
  } else if (!link.reset_reply && !link.ack_pending) {
    return 0;
  }

  link.reset_reply = false;
  link.ack_pending = false;
  return crsf_frame_quic(buf, link.origin, payload, size);
}

// Responses are queued whole where possible. A full buffer waits for IO to
// drain acknowledged bytes, even while holding the profile mutex; ground
// Flight then pauses as it does for long USB commands.
static void crsf_quic_send(uint8_t *data, uint32_t len, void *priv) {
  const uint32_t session = reset_request;
  while (len) {
    const uint32_t head = tx_head;
    const uint32_t offset = head % QUIC_CRSF_TX_SIZE;
    const uint32_t space = QUIC_CRSF_TX_SIZE - (head - tx_acked);
    const uint32_t chunk = MIN(len, MIN(space, QUIC_CRSF_TX_SIZE - offset));
    if (chunk) {
      memcpy(tx_buffer + offset, data, chunk);
      MEMORY_BARRIER();
      tx_head = head + chunk;
      data += chunk;
      len -= chunk;
      continue;
    }
    if (reset_request != session || !quic_crsf_active())
      return;
    vTaskDelay(1);
  }
}

static bool quic_crsf_supported(uint8_t cmd) {
  switch (cmd) {
  case QUIC_CMD_GET:
  case QUIC_CMD_SET:
  case QUIC_CMD_CAL_IMU:
  case QUIC_CMD_CAL_STICKS:
    return true;
  default:
    // Motor tests, passthrough, blackbox and OSD font transfers stay on USB.
    return false;
  }
}

static uint32_t quic_crsf_request_total() {
  return QUIC_HEADER_LEN + ((uint32_t)request[2] << 8 | request[3]);
}

static void quic_crsf_reset() {
  if (tx_buffer == NULL) {
    tx_buffer = (uint8_t *)malloc(QUIC_CRSF_TX_SIZE);
    request = (uint8_t *)malloc(QUIC_CRSF_REQUEST_SIZE);
    if (tx_buffer == NULL || request == NULL) {
      free(tx_buffer);
      free(request);
      tx_buffer = NULL;
      request = NULL;
      return;
    }
  }

  const uint32_t session = reset_request;
  rx_tail = rx_head;
  tx_head = tx_acked;
  request_size = 0;
  MEMORY_BARRIER();
  reset_done = session;
}

void quic_crsf_update() {
  if (reset_done != reset_request) {
    quic_crsf_reset();
    return;
  }
  if (request == NULL)
    return;

  while (true) {
    if (request_size >= QUIC_HEADER_LEN) {
      const uint32_t total = quic_crsf_request_total();
      if (total > QUIC_CRSF_REQUEST_SIZE) {
        quic_send_str(&quic, QUIC_CMD_INVALID, QUIC_FLAG_ERROR, "EOF");
        request_size = 0;
        continue;
      }
      if (request_size == total) {
        if (quic_crsf_supported(request[1]))
          quic_process(&quic, request, request_size);
        else
          quic_send_str(&quic, (quic_command)request[1], QUIC_FLAG_ERROR, "UNSUPPORTED OVER CRSF");
        request_size = 0;
        // Configuration work is excluded from Flight loop-rate decisions.
        flight_reset_runtime();
        continue;
      }
    }

    const uint32_t tail = rx_tail;
    const uint32_t available = rx_head - tail;
    MEMORY_BARRIER();
    if (available == 0)
      return;

    // Resynchronize on the QUIC magic after a dropped or rejected request.
    if (request_size == 0 && rx_buffer[tail % QUIC_CRSF_RX_SIZE] != QUIC_MAGIC) {
      rx_tail = tail + 1;
      continue;
    }

    const uint32_t needed = request_size < QUIC_HEADER_LEN
                                ? QUIC_HEADER_LEN - request_size
                                : quic_crsf_request_total() - request_size;
    const uint32_t size = MIN(needed, available);
    for (uint32_t i = 0; i < size; i++)
      request[request_size++] = rx_buffer[(tail + i) % QUIC_CRSF_RX_SIZE];
    rx_tail = tail + size;
  }
}
