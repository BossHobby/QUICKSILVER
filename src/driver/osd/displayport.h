#pragma once

#include <stdint.h>

#include "driver/osd/osd.h"
#include "io/msp.h"

typedef struct {
  // the goggle has polled us over MSP
  uint8_t connected;
  uint8_t cols;
  uint8_t rows;
} displayport_status_t;

extern msp_t displayport_msp;
extern displayport_status_t displayport_status;

void displayport_init();
bool displayport_is_ready(TickType_t *wait = nullptr);
void displayport_intro();

bool displayport_clear_async();
void displayport_set_canvas(uint8_t cols, uint8_t rows);

uint32_t displayport_can_fit();
bool displayport_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size);
bool displayport_flush();
