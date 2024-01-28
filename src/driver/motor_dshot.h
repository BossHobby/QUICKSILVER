#pragma once

#include "driver/motor.h"

#define DSHOT_CMD_BEEP1 1
#define DSHOT_CMD_BEEP2 2
#define DSHOT_CMD_BEEP3 3
#define DSHOT_CMD_BEEP4 4
#define DSHOT_CMD_BEEP5 5 // 5 currently uses the same tone as 4 in BLHeli_S.

#define DSHOT_CMD_ROTATE_NORMAL 20
#define DSHOT_CMD_ROTATE_REVERSE 21

#define DSHOT_DIR_CHANGE_IDLE_TIME_US 10000
#define DSHOT_DIR_CHANGE_CMD_TIME_US 1000

#define DSHOT_FREQ profile.motor.dshot_time
#define DSHOT_SYMBOL_FREQ (PWM_CLOCK_FREQ_HZ / (3 * DSHOT_FREQ * 1000 - 1))

#define DSHOT_DMA_BUFFER_SIZE (3 * (16 + 1))

#define GCR_FREQ ((DSHOT_FREQ * 5) / 4)
#define GCR_SYMBOL_FREQ (PWM_CLOCK_FREQ_HZ / (3 * GCR_FREQ * 1000 - 1))

// 30us delay, 5us buffer, 21 bits for worst case
#define GCR_SYMBOL_TIME_MAX (((DSHOT_TIME_MAX * 5) / 4) * 3)
#define GCR_DMA_BUFFER_SIZE (((30 + 5) * GCR_SYMBOL_TIME_MAX) / 1000 + 21 * 3)

#define DSHOT_MAX_PORT_COUNT 3

uint32_t dshot_decode_gcr(uint16_t *dma_buffer, uint32_t pin_mask);