#pragma once

int readdata(unsigned int data);
void writeword(unsigned long address, unsigned long value);

unsigned long fmc_read(unsigned long address);
void fmc_read2(void);
float fmc_read_float(unsigned long address);

int fmc_write(int data1, int data2);
int fmc_write2(void);
void fmc_write_float(unsigned long address, float float_to_write);

void fmc_unlock(void);
void fmc_lock(void);

int fmc_erase(void);