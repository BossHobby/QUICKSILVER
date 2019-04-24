#include "stdint.h"

void spi_gyro_init(void);
void MPU6XXX_write(uint8_t reg, uint8_t data);
uint8_t spi_transfer_byte(uint8_t data);
uint8_t MPU6XXX_get_id(uint8_t reg);

