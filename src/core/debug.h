#pragma once

#include <cbor.h>

#include "core/project.h"

#include "io/usb_configurator.h"

void debug_pin_init();

void debug_pin_enable(uint8_t index);
void debug_pin_disable(uint8_t index);
void debug_pin_toggle(uint8_t index);

#if defined(DEBUG) && defined(DEBUG_LOGGING)
#define quic_debugf(args...) usb_quic_logf(args)
#else
#define quic_debugf(args...) __NOP()
#endif