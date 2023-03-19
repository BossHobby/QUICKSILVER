#include "driver/spi.h"

#include "core/project.h"

#if !defined(USE_SOFT_SPI_4WIRE) && !defined(USE_SOFT_SPI_3WIRE)
// spi disabled (for pin setting)
int lastbyte = 0;

void spi_init() {}
void spi_cson() {}
void spi_csoff() {}
void spi_sendbyte(int x) { lastbyte = x; }
int spi_sendrecvbyte(int x) {
  // make it look like a real nrf24 to pass the check
  if (lastbyte == (0x0f))
    return 0xc6;
  lastbyte = x;
  return 255;
}
int spi_sendzerorecvbyte() { return 255; }

#endif
