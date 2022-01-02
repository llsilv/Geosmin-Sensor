/*
 * SPI.c
 *
 * Created: 06-10-2021 13:28:13
 *  Author: mikkel
 */ 
#include "SPI.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>




void SPI_Init(void)
{
	DDRB |= (1<<PINB3) | (1<< PINB5) | (1<<PINB1) | (1<<PINB0);
	SPCR = (1<<SPIE) | (1<<SPE) | (1<<MSTR) | (1<<CPHA);
	SPSR = (1<<SPI2X);
	DONE = 1;
}

void SPI_Send(char byte1, char byte2, char byte3, char ln)
{
	SPDR = byte1;
	BYTE1 = byte2;
	BYTE2 = byte3;
	if(ln ==2) BYTE2 = byte2;
	inc = ln;
	DONE = 0;
	
}


ISR(SPI_STC_vect)
{
	if(--inc == 2) SPDR = BYTE1;
	else if(inc == 1) SPDR = BYTE2;
	else if(inc == 0)
	{
	 DONE = 1; 
	 PORTB |= 0b00000011;
	 }
}

char SPI_free(void) {return DONE;}