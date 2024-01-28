#include "driver/motor_dshot.h"

#include "core/profile.h"
#include "core/project.h"
#include "driver/time.h"
#include "flight/control.h"
#include "util/util.h"

#ifdef USE_MOTOR_DSHOT

#define GCR_INVALID 0xffffffff
#define GCR_MIN_VALID_COUNT ((21 - 2) * 3) // 57
#define GCR_MAX_VALID_COUNT ((21 + 2) * 3) // 69

#define DSHOT_BITBAND

#define BITBAND_SRAM_REF 0x20000000
#define BITBAND_SRAM_BASE 0x22000000
#define BITBAND_SRAM(a, b) ((BITBAND_SRAM_BASE + (((a)-BITBAND_SRAM_REF) << 5) + ((b) << 2)))

typedef struct {
  uint32_t value;
  uint32_t _other[15];
} dshot_bitband_t;

typedef enum {
  DIR_CHANGE_START,
  DIR_CHANGE_DELAY,
  DIR_CHANGE_CMD,
  DIR_CHANGE_STOP,
} dir_change_state_t;

bool dshot_inverted = true;
uint16_t dshot_packet[MOTOR_PIN_MAX]; // 16bits dshot data for 4 motors
motor_direction_t motor_dir = MOTOR_FORWARD;

static bool dir_change_done = true;
static const uint32_t gcr_dict[32] = {GCR_INVALID, GCR_INVALID, GCR_INVALID, GCR_INVALID, GCR_INVALID, GCR_INVALID, GCR_INVALID, GCR_INVALID, GCR_INVALID, 9, 10, 11, GCR_INVALID, 13, 14, 15, GCR_INVALID, GCR_INVALID, 2, 3, GCR_INVALID, 5, 6, 7, GCR_INVALID, 0, 8, 1, GCR_INVALID, 4, 12, GCR_INVALID};

extern void dshot_dma_start();

static uint32_t dshot_decode_eRPM_telemetry_value(uint16_t value) {
  // eRPM range
  if (value == 0x0fff) {
    return 0;
  }

  // Convert value to 16 bit from the GCR telemetry format (eeem mmmm mmmm)
  value = (value & 0x01ff) << ((value & 0xfe00) >> 9);
  if (!value) {
    return 0;
  }

  // Convert period to erpm * 100
  return (1000000 * 60 / 100 + value / 2) / value;
}

uint32_t dshot_decode_gcr(uint16_t *dma_buffer, uint32_t pin_mask) {
#ifdef DSHOT_BITBAND
  const dshot_bitband_t *buf = (dshot_bitband_t *)BITBAND_SRAM((uint32_t)dma_buffer, POSITION_VAL(pin_mask));
#define pin_type dshot_bitband_t
#define pin_value ((ptr++)->value)
#else
  const uint16_t *buf = dma_buffer;
#define pin_type uint16_t
#define pin_value (*ptr++ & pin_mask)
#endif

  const pin_type *ptr = buf + 10;
  const pin_type *ptr_end = buf + (GCR_DMA_BUFFER_SIZE - GCR_MIN_VALID_COUNT);

  // find falling edge
  while (ptr < ptr_end) {
    if (pin_value == 0 ||
        pin_value == 0 ||
        pin_value == 0 ||
        pin_value == 0) {
      break;
    }
  }
  if (ptr >= ptr_end) {
    return 0;
  }

  ptr_end = ptr + GCR_MAX_VALID_COUNT;

  uint32_t value = 0;
  uint32_t bit_len = 0;
  const pin_type *last_ptr = ptr;
  while (ptr < ptr_end) {
    // find rising edge
    while (ptr < ptr_end) {
      if (pin_value != 0 ||
          pin_value != 0 ||
          pin_value != 0 ||
          pin_value != 0) {
        break;
      }
    }
    if (ptr >= ptr_end) {
      break;
    }
    {
      const uint32_t len = max((ptr - last_ptr + 1) / 3, 1);
      value <<= len;
      last_ptr = ptr;
      bit_len += len;
    }

    // find falling edge
    while (ptr < ptr_end) {
      if (pin_value == 0 ||
          pin_value == 0 ||
          pin_value == 0 ||
          pin_value == 0) {
        break;
      }
    }
    if (ptr >= ptr_end) {
      break;
    }
    {
      const uint32_t len = max((ptr - last_ptr + 1) / 3, 1);
      value <<= len;
      value |= (0x1 << len) - 1;
      last_ptr = ptr;
      bit_len += len;
    }
  }

  if (bit_len < 18 || bit_len > 21) {
    return 0;
  }

  const uint32_t fill_len = (21 - bit_len);
  value <<= fill_len;
  value |= (0x1 << fill_len) - 1;
  value = (value ^ (value >> 1));

  const uint32_t decoded =
      gcr_dict[(value >> 0) & 0x1f] |
      gcr_dict[(value >> 5) & 0x1f] << 4 |
      gcr_dict[(value >> 10) & 0x1f] << 8 |
      gcr_dict[(value >> 15) & 0x1f] << 12;

  uint32_t csum = decoded;
  csum = csum ^ (csum >> 8);
  csum = csum ^ (csum >> 4);

  if ((csum & 0xf) != 0xf || decoded > 0xffff) {
    __NOP();
    return 0;
  }

  return decoded >> 4;
}

static void dshot_make_packet(uint8_t number, uint16_t value, bool telemetry) {
  const uint16_t packet = (value << 1) | (telemetry ? 1 : 0);

  uint16_t csum = 0;
  uint16_t csum_data = packet;
  for (uint8_t i = 0; i < 3; ++i) {
    csum ^= csum_data; // xor data by nibbles
    csum_data >>= 4;
  }

  if (dshot_inverted) {
    csum = ~csum;
  }
  csum &= 0xf;

  dshot_packet[number] = (packet << 4) | csum;
}

static void dshot_make_packet_all(uint16_t value, bool telemetry) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    dshot_make_packet(profile.motor.motor_pins[i], value, telemetry);
  }
}

void motor_dshot_write(float *values) {
  if (dir_change_done) {
    for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
      uint16_t value = 0;
      if (values[i] >= 0.0f) {
        const float pwm = constrain(values[i], 0.0f, 1.0f);
        value = mapf(pwm, 0.0f, 1.0f, 48, 2047);
      } else {
        value = 0;
      }
      dshot_make_packet(profile.motor.motor_pins[i], value, false);
    }

    dshot_dma_start();
  } else {
    static uint8_t counter = 0;
    static uint32_t dir_change_time = 0;
    static dir_change_state_t state = DIR_CHANGE_START;

    switch (state) {
    case DIR_CHANGE_START: {
      if (counter < 100) {
        dshot_make_packet_all(0, false);
        dshot_dma_start();
        counter++;
      } else {
        state = DIR_CHANGE_DELAY;
        counter = 0;
      }
      dir_change_time = time_micros();
      break;
    }

    case DIR_CHANGE_DELAY:
      if ((time_micros() - dir_change_time) < DSHOT_DIR_CHANGE_IDLE_TIME_US) {
        break;
      }
      state = DIR_CHANGE_CMD;
      dir_change_time = time_micros();
      break;

    case DIR_CHANGE_CMD: {
      if ((time_micros() - dir_change_time) < DSHOT_DIR_CHANGE_CMD_TIME_US) {
        break;
      }

      const uint16_t value = motor_dir == MOTOR_REVERSE ? DSHOT_CMD_ROTATE_REVERSE : DSHOT_CMD_ROTATE_NORMAL;
      if (counter < 10) {
        dshot_make_packet_all(value, true);
        dshot_dma_start();
        counter++;
      } else {
        state = DIR_CHANGE_STOP;
        counter = 0;
      }
      dir_change_time = time_micros();
      break;
    }

    case DIR_CHANGE_STOP:
      dir_change_done = true;
      state = DIR_CHANGE_START;
      break;
    }
  }
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    state.gcr_values[i] = dshot_decode_eRPM_telemetry_value(state.gcr_values[i]);
  }
}

void motor_dshot_set_direction(motor_direction_t dir) {
  if (dir_change_done) {
    motor_dir = dir;
    dir_change_done = false;
  }
}

bool motor_dshot_direction_change_done() {
  return dir_change_done;
}

void motor_dshot_beep() {
  static uint32_t last_time = 0;
  const uint32_t time = time_millis() - last_time;

  static uint8_t beep_command = DSHOT_CMD_BEEP1;
  dshot_make_packet_all(beep_command, true);
  dshot_dma_start();

  if (time >= 500) {
    beep_command++;
    if (beep_command > DSHOT_CMD_BEEP5) {
      beep_command = DSHOT_CMD_BEEP1;
    }

    last_time = time_millis();
  }
}
#endif