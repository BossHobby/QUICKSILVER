/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


#include "project.h"
#include <stdint.h>
#include <stdio.h>

#include "drv_softi2c.h"
#include "config.h"

#ifndef USE_DUMMY_I2C
//#define i2cdebug

static GPIO_InitTypeDef  sdainit;

void delay(int);

// inter-version fix
#ifndef SOFTI2C_SPEED_SLOW1
#ifndef SOFTI2C_SPEED_SLOW2
#ifndef SOFTI2C_SPEED_FAST
	#define SOFTI2C_SPEED_FAST
#endif
#endif
#endif

#ifdef SOFTI2C_SPEED_FAST
#define _delay  //
#define _delay2 //
#endif

#ifdef SOFTI2C_SPEED_SLOW1
#ifdef __GNUC__
void delayraw()
{
	volatile uint8_t count = 1;
	while (count--);
}
#else
void delayraw()
{
	uint8_t count = 1;
	while (count--);
}
#endif
#define _delay  delayraw()
#define _delay2 //delay(1)
#endif

#ifdef SOFTI2C_SPEED_SLOW2
#define _delay  delay(1)
#define _delay2 //delay(1)
#endif

	#ifdef i2cdebug
	int debug = 1;   		// prints error info, set in setup()
	#endif
	
	int sda;
	int scl;	
	
	void _sendstart(void);
	void _sendstop(void);
	void sdalow(void);
	void sdahigh(void);
	void scllow(void);
	void sclhigh(void);
	void _restart(void);	
	int _readbyte( int); 
	int _sendbyte( int);
	int _readsda(void);
	
	int sdaout = 0;
void setoutput(void);

////////////////////////////////
/////////I2C Routines//////////


#ifdef F0
 void sdalow()
{
	if(!sdaout) setoutput();	
  I2C_SDAPORT->BRR = I2C_SDAPIN;
  sda=0;
	_delay;
}


  void sdahigh()
{
	if(!sdaout) setoutput();
	I2C_SDAPORT->BSRR = I2C_SDAPIN;
	_delay;
  sda = 1; 
}


  void scllow()
{
 I2C_SCLPORT->BRR = I2C_SCLPIN;
_delay;
 scl = 0;
}

  void sclhigh()
{
  I2C_SCLPORT->BSRR = I2C_SCLPIN;
	_delay;
 scl = 1; 
}

  void sclhighlow()
{
 I2C_SCLPORT->BSRR = I2C_SCLPIN;
	_delay;
 I2C_SCLPORT->BRR = I2C_SCLPIN;
_delay;
 scl = 0;
}

#endif

#ifdef F405
 void sdalow()
{
	if(!sdaout) setoutput();	
  I2C_SDAPORT->BSRRL = I2C_SDAPIN;
  sda=0;
	_delay;
}


  void sdahigh()
{
	if(!sdaout) setoutput();
	I2C_SDAPORT->BSRRH = I2C_SDAPIN;
	_delay;
  sda = 1; 
}


  void scllow()
{
 I2C_SCLPORT->BSRRL = I2C_SCLPIN;
_delay;
 scl = 0;
}

  void sclhigh()
{
  I2C_SCLPORT->BSRRH = I2C_SCLPIN;
	_delay;
 scl = 1; 
}

  void sclhighlow()
{
 I2C_SCLPORT->BSRRH = I2C_SCLPIN;
	_delay;
 I2C_SCLPORT->BSRRL = I2C_SCLPIN;
_delay;
 scl = 0;
}

#endif
void setinput()
{
	sdaout = 0;
  sdainit.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(I2C_SDAPORT, &sdainit);	
	
	_delay2;
 }

void setoutput()
{
	sdaout = 1;
  sdainit.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(I2C_SDAPORT, &sdainit);
	_delay2;
}

int _readsda()
{
#ifdef i2cdebug
  if (!sda)  printf("_readsda: sda low");
#endif
//if ( sdaout) setinput();	
//return ( GPIO_ReadInputDataBit(I2C_SDAPORT, I2C_SDAPIN) ); 
return	I2C_SDAPORT->IDR & I2C_SDAPIN;
}


void _sendstart()
{
 if (scl == 0) 
 {
 #ifdef i2cdebug
   printf("_sendstart: scl low"); 
 #endif
  sclhigh();
 } 
 if (sda == 1)
{
  if(!_readsda())
	{
	#ifdef i2cdebug
	printf("_sendstart: sda pulled low by slave"); 
	#endif
	//error1 = 1;
	}
  sdalow();
}
 else {
	  #ifdef i2cdebug
    printf("_sendstart: sda low"); 
	  #endif
       } 
}


void _restart()
{
 #ifdef i2cdebug 
 if (scl == 1) printf("_restart: scl high"); 
 #endif
 if (sda == 0) 
 {
   sdahigh();
 }
 sclhigh();
 sdalow();
}

void _sendstop()
{
  
  if (sda == 1) 
  {
    if (!scl) sdalow(); else 
		{
		#ifdef i2cdebug
		printf("stop: error");
		#endif
		}
  }
  if (scl == 0) sclhigh();
  else {
		#ifdef i2cdebug
		printf("stop: scl high");
		#endif
		}
  sdahigh();
 
}



int _sendbyte( int value )
{
int i;
 if (scl == 1) 
 {
  scllow();
 }
 
 for ( i = 7; i >= 0 ;i--)
 {
 if ((value >> i)&1) 
 {
  sdahigh();
 }
 else 
 {
  sdalow();
 }
 //sclhigh();
 //scllow();
 sclhighlow();
 }
 
 if (!sda) sdahigh(); // release the line
 //get ack

 sclhigh();
// skip ack since it is not used here
 uint8_t ack;// = _readsda();
	#ifdef i2cdebug
  if (ack)
	{
	if (debug) Serial.println("NOT RECEIVED"); 
	}
	#endif
 scllow();
return ack; 
}

int _readbyte(int ack)  //ACK 1 single byte ACK 0 multiple bytes
{
 uint8_t data=0;
#ifdef i2cdebug
 if (scl == 1)
	{	
	printf("read: scl high");
	}
#endif
 if ( sda == 0) 
 {
   sdahigh();
 }
 if(!sdaout) setoutput();
 int i;
 for( i = 7; i>=0;i--)
 {
  sclhigh(); 
 if (_readsda() ) data|=(1<<i);
  scllow();
 }

if (ack)  
{
  sdahigh();
} 
else 
{
  sdalow();
}
  sclhigh();
  scllow();
if (sda) sdalow(); 

return data;
}


int softi2c_write( int device_address , int address, int value)
{
 _sendstart();
 _sendbyte((device_address<<1));
 _sendbyte(address);
  uint8_t ack = _sendbyte(value);
  _sendstop();
  return ack;
}


int softi2c_read(int device_address , int register_address)  
{
 _sendstart();
 _sendbyte((device_address<<1));
 _sendbyte(register_address);
 _restart(); 
 _sendbyte((device_address<<1) + 1);
 uint8_t x = _readbyte(1); 
 _sendstop();
 return x; 
}


void softi2c_writedata(int device_address ,int register_address , int *data, int size ) 
{
	int index = 0;
 _sendstart();
 _sendbyte(device_address<<1);
 _sendbyte(register_address);
  
  _sendstop();

	while(index<size)
	{
	_sendbyte(data[index]);
	index++;	
	}
 _sendstop();

}


void softi2c_readdata(int device_address ,int register_address , int *data, int size ) 
{
	int index = 0;
 _sendstart();
 _sendbyte(device_address<<1);
 _sendbyte(register_address);
 _restart();
 _sendbyte( (device_address<<1) + 1);
	while(index<size-1)
	{
	data[index] = _readbyte(0);
	index++;	
	}
  data[index] = _readbyte(1);
 _sendstop();
 
}



void softi2c_init()
{

    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = I2C_SDAPIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_SCLPORT, &GPIO_InitStructure);

    // some boards have no SCL pullup, we drive SCL pullup for better speed on those
    // the factory firmware does this too, so it must be ok
    #ifdef SOFTI2C_PUSHPULL_CLK
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    #endif

    GPIO_InitStructure.GPIO_Pin = I2C_SCLPIN;
    GPIO_Init(I2C_SDAPORT, &GPIO_InitStructure);


    sdainit.GPIO_Pin = I2C_SDAPIN;
    sdainit.GPIO_Mode = GPIO_Mode_IN;
    sdainit.GPIO_OType = GPIO_OType_OD;
    sdainit.GPIO_PuPd = GPIO_PuPd_UP;
    sdainit.GPIO_Speed = GPIO_Speed_50MHz;

    sdaout = 1;
    sda = 0;
    scl = 0;


    sdahigh();
    sclhigh();

}

#endif



///////////////////////////////END I2C///////
