#include "stdint.h"

void spi_gyro_init(void);
uint8_t MPU6XXX_write(uint8_t reg, uint8_t data);
uint8_t MPU6XXX_read(uint8_t reg);

