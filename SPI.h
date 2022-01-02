/*
 * SPI.h
 *
 * Created: 06-10-2021 13:28:24
 *  Author: mikkel
 */ 
#ifndef SPI_H
#define SPI_H


char DONE;
char BYTE1, BYTE2, inc;


void SPI_Send(char byte1, char byte2, char byte3, char ln);
void SPI_Init(void);
char SPI_free(void);
#endif /* SPI_H_ */