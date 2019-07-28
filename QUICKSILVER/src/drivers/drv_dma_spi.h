#include "stdint.h"

void spi_gyro_init(void);
uint8_t MPU6XXX_write(uint8_t reg, uint8_t data);
uint8_t MPU6XXX_read(uint8_t reg);
void spi_reset_prescaler(void);
void MPU6XXX_read_data(uint8_t reg, int *data, int size);
