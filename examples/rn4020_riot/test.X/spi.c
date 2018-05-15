/* SPI Master Driver */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "spi.h"
#include "config.h"

void SPI_Init(void)
{
	SPI_CS0 = 1;
	SPI_CS1 = 1;
	spi_init(SPI_DEV(2));
	spi_acquire(SPI_DEV(2), 0, SPI_MODE_0, SPI_CLK_1MHZ);
}

void SPI_Speed(const uint8_t speed)
{
	switch (speed) {
	case 1:
		// SPI MASTER SCK speed 16MHz
		break;
	default:
		// SPI MASTER SCK speed 1MHz
		break;
	}
}

void SPI_ClearBufs(void)
{
}

void SPI_TxStart(void)
{
	return;
}

bool SPI_IsNewRxData(void)
{
	return(true); //There are bytes in the buffer
}

uint8_t SPI_ReadRxBuffer(void)
{
	uint8_t Temp;

	Temp = 0;
	return(Temp);
}

bool SPI_IsTxData(void)
{
	return(true); //There are bytes in the buffer
}

void SPI_WriteTxBuffer(const uint8_t TxByte)
{
	(void) TxByte;
}

//**********************************************************************************************************************
// Return the number of bytes free in the TX buffer

uint16_t SPI_GetTXBufferFreeSpace(void)
{
	uint16_t space;

	space = 0;
	return space;
}

//Peek at buffer tail

uint8_t SPI_PeekRxBuffer(void)
{
	return 0;
}
