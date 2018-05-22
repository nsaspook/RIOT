/* SPI Master Driver */

#include "spi.h"
#include "config.h"

void SPI_Init(void)
{
	SPI_CS0 = 1;
	SPI_CS1 = 1;
	SPI_CS2 = 1;
	spi_init(SPI_DEV(2));
	spi_acquire(SPI_DEV(2), 0, SPI_MODE_0, SPI_CLK_1MHZ);
}

void SPI_Speed(spi_t bus, spi_mode_t mode, spi_clk_t clk)

{
	spi_speed_config(bus, mode, clk);
}
