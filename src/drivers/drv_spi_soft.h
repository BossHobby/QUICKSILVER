#pragma once

void spi_init();
void spi_cson();
void spi_csoff();
void spi_sendbyte(int);
int spi_sendrecvbyte(int);
int spi_sendzerorecvbyte();
