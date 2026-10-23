#include "io/quic_rt.h"

#include <atomic>
#include <string.h>

#include "control/control.h"
#include "control/mission.h"
#include "core/tasks.h"
#include "driver/time.h"
#include "rx/crsf.h"

static constexpr uint32_t QUIC_RT_ACTIVE_MS = 3000;
static constexpr uint32_t QUIC_RT_STATUS_INTERVAL_MS = 200;
static constexpr uint8_t QUIC_RT_HEADER_SIZE = 2;  // version, type
static constexpr uint8_t QUIC_RT_COMMAND_SIZE = 2; // command type, id
static_assert(QUIC_RT_HEADER_SIZE + QUIC_RT_COMMAND_SIZE + 1 + NAV_COMMAND_ITEMS_MAX * sizeof(quic_rt_mission_item_t) <=
                  CRSF_PAYLOAD_SIZE_MAX - CRSF_FRAME_ORIGIN_DEST_SIZE,
              "MISSION_ITEMS must fit an extended CRSF frame");

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
  bool staged_busy; // Flight owns staging.mission for a queued commit
} mailbox;

// Mission edits, staged and checked by IO so Flight only swaps the result in.
// IO owns the staged mission except while mailbox.staged_busy.
static struct {
  bool open;
  mission_t mission;
} staging;

// IO's mission_crc() of the stored mission, recomputed once the revision moves.
// The revision is read before the mission, so a write during the computation
// moves it again and the next STATUS recomputes.
static struct {
  bool valid;
  uint32_t revision;
  uint16_t crc;
} stored_crc;

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
        .altitude = msg.altitude,
        .speed = msg.speed,
    };
    return true;
  }
  case NAV_COMMAND_SET_ALT: {
    quic_rt_set_alt_t msg;
    if (args_size < sizeof(msg))
      return false;
    memcpy(&msg, args, sizeof(msg));
    cmd->waypoint.altitude = msg.altitude;
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
  case NAV_COMMAND_MISSION_ITEMS: {
    if (args_size < 1)
      return false;
    const uint8_t items_size = args_size - 1;
    const uint8_t count = items_size / sizeof(quic_rt_mission_item_t);
    if (count == 0 || count > NAV_COMMAND_ITEMS_MAX || items_size % sizeof(quic_rt_mission_item_t))
      return false;
    cmd->index = args[0];
    cmd->item_count = count;
    for (uint8_t i = 0; i < count; i++) {
      quic_rt_mission_item_t msg;
      memcpy(&msg, args + 1 + i * sizeof(msg), sizeof(msg));
      cmd->items[i] = {
          .coord = {.lon = msg.lon, .lat = msg.lat},
          .altitude = msg.altitude,
          .speed = msg.speed,
          .type = msg.type,
      };
    }
    return true;
  }
  case NAV_COMMAND_MISSION_COMMIT: {
    quic_rt_mission_commit_t msg;
    if (args_size < sizeof(msg))
      return false;
    memcpy(&msg, args, sizeof(msg));
    cmd->index = msg.count;
    cmd->end_action = msg.end_action;
    cmd->crc = msg.crc;
    return true;
  }
  case NAV_COMMAND_HOLD:
  case NAV_COMMAND_RTH:
    return true;
  default:
    return false;
  }
}

static nav_command_result_t quic_rt_stage(const nav_command_t *cmd) {
  if (cmd->index >= MISSION_WAYPOINT_MAX || cmd->item_count > MISSION_WAYPOINT_MAX - cmd->index)
    return NAV_RESULT_INVALID;
  for (uint32_t i = 0; i < cmd->item_count; i++) {
    if (!mission_waypoint_valid(&cmd->items[i]))
      return NAV_RESULT_INVALID;
  }
  if (!staging.open) {
    staging.mission = mission;
    staging.open = true;
  }
  memcpy(&staging.mission.waypoints[cmd->index], cmd->items, cmd->item_count * sizeof(mission_waypoint_t));
  return NAV_RESULT_OK;
}

// A commit ends staging whatever the result. Only Flight replaces the mission
// in flight, and only with a commit queued here; a copy torn by a USB upload,
// which happens on the ground, fails the CRC.
static nav_command_result_t quic_rt_check_commit(const nav_command_t *cmd) {
  mission_t *updated = &staging.mission;
  if (!staging.open)
    *updated = mission;
  staging.open = false;

  updated->count = cmd->index;
  updated->end_action = cmd->end_action;
  if (!mission_valid(updated))
    return NAV_RESULT_INVALID;
  if (mission_crc(updated) != cmd->crc)
    return NAV_RESULT_MISMATCH;
  // Leave no waypoint of an earlier route behind the new end.
  memset(&updated->waypoints[updated->count], 0, (MISSION_WAYPOINT_MAX - updated->count) * sizeof(mission_waypoint_t));
  return NAV_RESULT_OK;
}

// Staging is acknowledged here; a valid commit goes to Flight to swap in.
static void quic_rt_edit(const nav_command_t *cmd) {
  if (cmd->type == NAV_COMMAND_MISSION_ITEMS) {
    quic_rt_complete(cmd->id, quic_rt_stage(cmd));
    return;
  }
  const nav_command_result_t result = quic_rt_check_commit(cmd);
  if (result != NAV_RESULT_OK) {
    quic_rt_complete(cmd->id, result);
    return;
  }
  taskENTER_CRITICAL();
  mailbox.command = *cmd;
  mailbox.pending = true;
  mailbox.staged_busy = true;
  taskEXIT_CRITICAL();
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

  const bool edit = command && (cmd.type == NAV_COMMAND_MISSION_ITEMS || cmd.type == NAV_COMMAND_MISSION_COMMIT);
  bool apply_edit = false;
  taskENTER_CRITICAL();
  // Dropping an unread command returns a queued commit's staging to IO.
  if (session_start) {
    mailbox.staged_busy = mailbox.staged_busy && !mailbox.pending;
    mailbox.pending = mailbox.applied = false;
    completed.store(QUIC_RT_RESULT_NONE, std::memory_order_relaxed);
  }
  if (edit) {
    // A resend of an applied edit keeps its acknowledgement; while Flight owns
    // the staged mission, a resend waits for its result.
    apply_edit = !mailbox.staged_busy && !(mailbox.applied && mailbox.applied_id == cmd.id);
  } else if (command) {
    mailbox.staged_busy = mailbox.staged_busy && !mailbox.pending;
    mailbox.command = cmd;
    mailbox.pending = true;
  }
  taskEXIT_CRITICAL();
  if (apply_edit)
    quic_rt_edit(&cmd);
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
  if (state.rth_state >= RTH_STATE_CLIMB)
    value |= QUIC_RT_FLAG_NAVIGATING;
#endif
  if (state.nav_target == NAV_TARGET_MISSION)
    value |= QUIC_RT_FLAG_MISSION;
  if (state.nav_target == NAV_TARGET_GUIDED)
    value |= QUIC_RT_FLAG_GUIDED;
  if (flags.guided_ready)
    value |= QUIC_RT_FLAG_GUIDED_READY;
  return value;
}

static uint16_t quic_rt_mission_crc() {
  const uint32_t revision = mission_revision();
  if (!stored_crc.valid || stored_crc.revision != revision)
    stored_crc = {.valid = true, .revision = revision, .crc = mission_crc(&mission)};
  return stored_crc.crc;
}

// Display-only telemetry: each field is the latest value, possibly from
// different Flight passes, as on the OSD. Only the ground station's own commit
// replaces the mission in flight; Flight acknowledges it after the swap, so the
// mission read after the acknowledgement is complete.
uint8_t quic_rt_status(uint8_t *payload, uint8_t *destination) {
  const uint32_t now = time_millis();
  if (!station.heard || now - station.heard_ms > QUIC_RT_ACTIVE_MS || now - station.status_ms < QUIC_RT_STATUS_INTERVAL_MS)
    return 0;
  station.status_ms = now;

  const uint16_t ack = completed.load(std::memory_order_acquire);
  const uint8_t result = ack & 0xFF;
  const gps_coord_t target = nav_target_coord();
  const quic_rt_status_t status = {
      .command_id = (uint8_t)(ack >> 8),
      .flags = quic_rt_flags(),
      .result_index = (uint8_t)(result << QUIC_RT_RESULT_SHIFT | (state.mission_index & QUIC_RT_MISSION_INDEX_MASK)),
      .mission_count = mission.count,
      .mission_crc = quic_rt_mission_crc(),
      .target_lat = target.lat,
      .target_lon = target.lon,
  };

  payload[0] = QUIC_RT_VERSION;
  payload[1] = QUIC_RT_STATUS;
  memcpy(payload + QUIC_RT_HEADER_SIZE, &status, sizeof(status));
  *destination = station.origin;
  return QUIC_RT_STATUS_SIZE;
}

const mission_t *quic_rt_staged() {
  return &staging.mission;
}

bool quic_rt_take(nav_command_t *cmd) {
  taskENTER_CRITICAL();
  // A resend of the applied command keeps its acknowledgement.
  const bool pending = mailbox.pending && !(mailbox.applied && mailbox.command.id == mailbox.applied_id);
  if (pending)
    *cmd = mailbox.command;
  // Flight owns the staged mission only while it applies a commit.
  mailbox.staged_busy = pending && cmd->type == NAV_COMMAND_MISSION_COMMIT;
  mailbox.pending = false;
  taskEXIT_CRITICAL();
  return pending;
}

void quic_rt_complete(uint8_t id, nav_command_result_t result) {
  taskENTER_CRITICAL();
  mailbox.applied = true;
  mailbox.applied_id = id;
  // Whatever completed, only a commit still queued holds the staged mission.
  mailbox.staged_busy = mailbox.pending && mailbox.command.type == NAV_COMMAND_MISSION_COMMIT;
  completed.store((uint16_t)(id << 8 | result), std::memory_order_release);
  taskEXIT_CRITICAL();
}
