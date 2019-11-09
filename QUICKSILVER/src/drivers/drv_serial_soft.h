/*
The MIT License

Copyright (c) 2017 Mike Morrison

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef DRV_SOFTSERIAL_H
#define DRV_SOFTSERIAL_H

#include "project.h"
#include "defines.h"
#include "drv_time.h"

typedef struct SoftSerialData_s
{
	GPIO_TypeDef* tx_port;
	uint16_t tx_pin;
	GPIO_TypeDef* rx_port;
	uint16_t rx_pin;
	uint32_t baud;
	uint32_t micros_per_bit;
	uint32_t micros_per_bit_half;
} SoftSerialData_t;

SoftSerialData_t softserial_init(GPIO_TypeDef* tx_port, uint16_t tx_pin, GPIO_TypeDef* rx_port, uint16_t rx_pin,  uint32_t baudrate);
int softserial_read_byte(uint8_t* byte);
void softserial_write_byte(uint8_t byte);
int softserial_read_byte_ex(const SoftSerialData_t* data, uint8_t* byte);
void softserial_write_byte_ex(const SoftSerialData_t* data, uint8_t byte);
void softserial_set_input(const SoftSerialData_t* data);
void softserial_set_output(const SoftSerialData_t* data);

inline void delay_until(uint32_t uS )
{
	while (gettime() < uS) ;
}


#endif
