#include "io/quic_rt.h"

#include <atomic>
#include <string.h>

#include "control/control.h"
#include "control/mission.h"
#include "core/tasks.h"
#include "driver/time.h"

static constexpr uint32_t QUIC_RT_ACTIVE_MS = 3000;
static constexpr uint32_t QUIC_RT_STATUS_INTERVAL_MS = 200;
static constexpr uint8_t QUIC_RT_HEADER_SIZE = 2;  // version, type
static constexpr uint8_t QUIC_RT_COMMAND_SIZE = 2; // command type, id

// IO owns the ground station address and the status schedule.
static struct {
  bool heard;
  uint8_t origin;
  uint32_t heard_ms;
  uint32_t status_ms;
} station;

// Commands from IO to Flight, copied under a short task critical section.
// Flight records the last applied id; a new session forgets it.
static struct {
  nav_command_t command;
  bool pending;
  bool applied;
  uint8_t applied_id;
} mailbox;

// Written by Flight, and cleared by IO when a session starts. Read by IO for
// STATUS: id << 8 | result.
static std::atomic<uint16_t> completed{QUIC_RT_RESULT_NONE};
static_assert(std::atomic<uint16_t>::is_always_lock_free);

static bool quic_rt_decode_command(const uint8_t *body, uint8_t size, nav_command_t *cmd) {
  if (size < QUIC_RT_COMMAND_SIZE)
    return false;
  *cmd = {.type = body[0], .id = body[1]};
  const uint8_t *args = body + QUIC_RT_COMMAND_SIZE;
  const uint8_t args_size = size - QUIC_RT_COMMAND_SIZE;

  switch (cmd->type) {
  case NAV_COMMAND_GOTO: {
    quic_rt_goto_t msg;
    if (args_size < sizeof(msg))
      return false;
    memcpy(&msg, args, sizeof(msg));
    cmd->waypoint = {
        .coord = {.lon = msg.lon, .lat = msg.lat},
        .altitude = msg.altitude_cm * 0.01f,
        .speed = msg.speed_dms * 0.1f,
    };
    return true;
  }
  case NAV_COMMAND_SET_ALT: {
    quic_rt_set_alt_t msg;
    if (args_size < sizeof(msg))
      return false;
    memcpy(&msg, args, sizeof(msg));
    cmd->waypoint.altitude = msg.altitude_cm * 0.01f;
    return true;
  }
  case NAV_COMMAND_MISSION: {
    quic_rt_mission_t msg;
    if (args_size < sizeof(msg))
      return false;
    memcpy(&msg, args, sizeof(msg));
    cmd->index = msg.index;
    return true;
  }
  case NAV_COMMAND_HOLD:
  case NAV_COMMAND_RTH:
    return true;
  default:
    return false;
  }
}

void quic_rt_receive(uint8_t origin, const uint8_t *payload, uint8_t size) {
  if (size < QUIC_RT_HEADER_SIZE || payload[0] != QUIC_RT_VERSION)
    return;

  // After QUIC_RT_ACTIVE_MS of silence the ground station may have restarted
  // and reused its ids: neither acknowledge nor suppress them.
  const uint32_t now = time_millis();
  const bool session_start = !station.heard || now - station.heard_ms > QUIC_RT_ACTIVE_MS;
  station.heard = true;
  station.origin = origin;
  station.heard_ms = now;

  nav_command_t cmd;
  const bool command = payload[1] == QUIC_RT_COMMAND &&
                       quic_rt_decode_command(payload + QUIC_RT_HEADER_SIZE, size - QUIC_RT_HEADER_SIZE, &cmd);
  if (!session_start && !command)
    return;

  taskENTER_CRITICAL();
  if (session_start) {
    mailbox.pending = mailbox.applied = false;
    completed.store(QUIC_RT_RESULT_NONE, std::memory_order_relaxed);
  }
  if (command) {
    mailbox.command = cmd;
    mailbox.pending = true;
  }
  taskEXIT_CRITICAL();
}

static uint8_t quic_rt_flags() {
  uint8_t value = 0;
  if (flags.arm_state)
    value |= QUIC_RT_FLAG_ARMED;
  if (flags.in_air)
    value |= QUIC_RT_FLAG_IN_AIR;
  if (flags.failsafe)
    value |= QUIC_RT_FLAG_FAILSAFE;
  if (state.gps_lock)
    value |= QUIC_RT_FLAG_GPS_LOCK;
#if defined(VEHICLE_WING)
  if (state.wing_nav_state != WING_NAV_INACTIVE)
    value |= QUIC_RT_FLAG_NAVIGATING;
#else
  if (state.rth_active)
    value |= QUIC_RT_FLAG_NAVIGATING;
#endif
  if (state.mission_active)
    value |= QUIC_RT_FLAG_MISSION;
  if (state.guided_active)
    value |= QUIC_RT_FLAG_GUIDED;
  if (state.guided_ready)
    value |= QUIC_RT_FLAG_GUIDED_READY;
  return value;
}

// Display-only telemetry: each field is the latest value, possibly from
// different Flight passes, as on the OSD.
uint8_t quic_rt_status(uint8_t *payload, uint8_t *destination) {
  const uint32_t now = time_millis();
  if (!station.heard || now - station.heard_ms > QUIC_RT_ACTIVE_MS || now - station.status_ms < QUIC_RT_STATUS_INTERVAL_MS)
    return 0;
  station.status_ms = now;

  const uint16_t ack = completed.load(std::memory_order_relaxed);
  const uint8_t result = ack & 0xFF;
  const gps_coord_t target = nav_target_coord();
  const quic_rt_status_t status = {
      .command_id = (uint8_t)(ack >> 8),
      .flags = quic_rt_flags(),
      .result_index = (uint8_t)(result << QUIC_RT_RESULT_SHIFT | (state.mission_index & QUIC_RT_MISSION_INDEX_MASK)),
      .mission_count = mission.count,
      .target_lat = target.lat,
      .target_lon = target.lon,
  };

  payload[0] = QUIC_RT_VERSION;
  payload[1] = QUIC_RT_STATUS;
  memcpy(payload + QUIC_RT_HEADER_SIZE, &status, sizeof(status));
  *destination = station.origin;
  return QUIC_RT_STATUS_SIZE;
}

bool quic_rt_take(nav_command_t *cmd) {
  taskENTER_CRITICAL();
  // A resend of the applied command keeps its acknowledgement.
  const bool pending = mailbox.pending && !(mailbox.applied && mailbox.command.id == mailbox.applied_id);
  if (pending)
    *cmd = mailbox.command;
  mailbox.pending = false;
  taskEXIT_CRITICAL();
  return pending;
}

void quic_rt_complete(uint8_t id, nav_command_result_t result) {
  taskENTER_CRITICAL();
  mailbox.applied = true;
  mailbox.applied_id = id;
  completed.store((uint16_t)(id << 8 | result), std::memory_order_relaxed);
  taskEXIT_CRITICAL();
}
