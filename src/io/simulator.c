#include "io/simulator.h"

#ifdef SIMULATOR

#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "flight/control.h"

typedef struct {
  float gyro[3];
  float accel[3];

  uint8_t rc_updated;
  float rc_channels[4];
  uint8_t rc_aux[4];

  float motors[4];
} simulation_state_t;

typedef struct {
  pthread_mutex_t mutex;
  simulation_state_t state;
  uint8_t osd[50 * 18];
} shared_memory_t;

float simulator_motor_values[MOTOR_PIN_MAX];

static int shm_fd = 0;
static shared_memory_t *shared = NULL;
static uint8_t osd[50 * 18];

static bool rc_updated = false;

static void simulator_read_file(const char *filename, char *buf) {
  FILE *f = fopen(filename, "rb");
  if (!f) {
    return;
  }

  fseek(f, 0, SEEK_END);
  uint32_t length = ftell(f);
  fseek(f, 0, SEEK_SET);

  fread(buf, 1, length, f);
  buf[length] = '\0';

  fclose(f);
}

void simulator_init() {
  char name[128];
  simulator_read_file("/tmp/quac_sim", name);

  shm_fd = shm_open(name, O_RDWR, 0644);
  if (shm_fd < 0) {
    return;
  }

  shared = (shared_memory_t *)mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
}

void simulator_update() {
  pthread_mutex_lock(&shared->mutex);

  if (shared->state.rc_updated) {
    state.rx.roll = shared->state.rc_channels[0];
    state.rx.pitch = shared->state.rc_channels[1];
    state.rx.yaw = shared->state.rc_channels[2];
    state.rx.throttle = shared->state.rc_channels[3];

    state.aux[AUX_CHANNEL_0] = shared->state.rc_aux[0];
    state.aux[AUX_CHANNEL_1] = shared->state.rc_aux[1];
    state.aux[AUX_CHANNEL_2] = shared->state.rc_aux[2];
    state.aux[AUX_CHANNEL_3] = shared->state.rc_aux[3];
    state.aux[AUX_CHANNEL_4] = 0;
    state.aux[AUX_CHANNEL_5] = 0;
    state.aux[AUX_CHANNEL_6] = 0;
    state.aux[AUX_CHANNEL_7] = 0;
    state.aux[AUX_CHANNEL_8] = 0;
    state.aux[AUX_CHANNEL_9] = 0;
    state.aux[AUX_CHANNEL_10] = 0;
    state.aux[AUX_CHANNEL_11] = 0;

    rc_updated = true;
    shared->state.rc_updated = 0;
  }

  state.gyro.axis[0] = state.gyro_raw.axis[0] = -shared->state.gyro[2];
  state.gyro.axis[1] = state.gyro_raw.axis[1] = -shared->state.gyro[0];
  state.gyro.axis[2] = state.gyro_raw.axis[2] = shared->state.gyro[1];

  state.accel_raw.axis[0] = -(shared->state.accel[2] / 9.817);
  state.accel_raw.axis[1] = -(shared->state.accel[0] / 9.817);
  state.accel_raw.axis[2] = -(shared->state.accel[1] / 9.817);

  shared->state.motors[0] = simulator_motor_values[0],
  shared->state.motors[1] = simulator_motor_values[1],
  shared->state.motors[2] = simulator_motor_values[2],
  shared->state.motors[3] = simulator_motor_values[3],

  memcpy(shared->osd, osd, sizeof(osd));

  flags.rx_ready = 1;
  flags.failsafe = 0;

  pthread_mutex_unlock(&shared->mutex);
}

bool simulator_rx_check() {
  const bool value = rc_updated;
  rc_updated = false;
  return value;
}

bool simulator_osd_is_ready() {
  return true;
}

void simulator_osd_intro() {
  simulator_osd_clear_async();

  uint8_t buffer[24];
  for (uint8_t row = 0; row < 4; row++) {
    uint8_t start = 160 + row * 24;
    for (uint8_t i = 0; i < 24; i++) {
      buffer[i] = start + i;
    }
    simulator_osd_push_string(OSD_ATTR_TEXT, (50 / 2) - 12, (18 / 2) - 2 + row, buffer, 24);
  }

  simulator_update();
}

uint8_t simulator_osd_clear_async() {
  memset(osd, ' ', sizeof(osd));
  return 1;
}
osd_system_t simulator_osd_check_system() {
  return OSD_SYS_HD;
}

bool simulator_osd_can_fit(uint8_t size) {
  return true;
}
bool simulator_osd_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size) {
  for (uint32_t i = 0; i < size; i++) {
    osd[y * 50 + x + i] = data[i];
  }
  return true;
}
bool simulator_osd_flush() {
  return true;
}
#else

void simulator_init() {}
void simulator_update() {}

#endif