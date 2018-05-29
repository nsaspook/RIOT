#include <stdint.h>
#include <stdbool.h>
#include "assert.h"
#include "board.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "periph/dac.h"

static uint8_t *td;
static uint8_t *rd;

int8_t dac_init(dac_t line)
{
	static bool dac_is_init = false;

	(void) line;
	if (!dac_is_init) {
		gpio_set(Ja10_13); // deselect the ADC
		td = __pic32_alloc_coherent(16); /* uncached memory for spi transfers */
		rd = __pic32_alloc_coherent(16);
		dac_is_init = true;
	}
	return true;
}

void dac_set(dac_t line, uint16_t value)
{
	uint32_t chan = line, range = 1;
	assert(line <= 1);

	td[1] = value & 0xff;
	td[0] = (0x10 | ((chan & 0x01) << 7) | ((~range & 0x01) << 5) | ((value >> 8)& 0x0f));

	gpio_clear(Ja10_13); // select the DAC
	spi_speed_config(SPI_DEV(2), 0, SPI_CLK_10MHZ); /* mode 0, speed */
	spi_transfer_bytes(SPI_DEV(2), 0, true, td, rd, 2);
	gpio_set(Ja10_13); // deselect the DAC
}
