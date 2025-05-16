#include "driver/serial_soft.h"

// Mock software serial implementation for simulator
uint8_t soft_serial_init(serial_port_config_t config) {
  // Simulate software serial initialization
  // In a real simulator, this might create virtual serial ports
  return 0;  // Return success
}

uint8_t soft_serial_read_byte(serial_ports_t port) {
  // In the simulator, software serial doesn't actually exist
  return 0;
}

void soft_serial_write_byte(serial_ports_t port, uint8_t byte) {
  // In the simulator, software serial doesn't actually exist
}

void soft_serial_enable_read(serial_ports_t port) {
  // In the simulator, software serial doesn't actually exist
}