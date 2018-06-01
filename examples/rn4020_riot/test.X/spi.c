/* SPI Master Driver */

#include "spi.h"
#include "config.h"
#include <stdbool.h>

void SPI_Init(void)
{
    static bool spi_is_init = false;

    if (!spi_is_init) {
        SPI_CS0_1_J10;
        SPI_CS1_1;
        SPI_CS2_1;
        spi_init(SPI_DEV(2));
        spi_acquire(SPI_DEV(2), 0, SPI_MODE_0, SPI_CLK_1MHZ);
        spi_is_init = true;
    }
}

void SPI_Speed(spi_t bus, spi_mode_t mode, spi_clk_t clk)

{
    spi_speed_config(bus, mode, clk);
}
