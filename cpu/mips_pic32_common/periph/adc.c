#include <stdint.h>
#include <stdbool.h>
#include "assert.h"
#include "board.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "periph/adc.h"

static uint8_t *td;
static uint8_t *rd;

static MCP_ADC_DATA riot_adc;

static void ADC_Init(void)
{
	static bool adc_is_init = false;

	if (!adc_is_init) {
		gpio_set(Ja10_2); // deselect the ADC
		td = __pic32_alloc_coherent(32); /* uncached memory for spi transfers */
		rd = __pic32_alloc_coherent(32);
		adc_is_init = true;
	}
}

/* for RIOT-OS 8 lines, 12-bit resolution only */
int adc_sample(adc_t line, adc_res_t res)
{
	assert(line <= 7);

	(void) res;
	riot_adc.mcp3208_cmd.ld = 0; // clear the command word
	riot_adc.mcp3208_cmd.map.start_bit = 1;
	riot_adc.mcp3208_cmd.map.single_diff = 1;
	riot_adc.mcp3208_cmd.map.index = line;
	td[0] = riot_adc.mcp3208_cmd.bd[2];
	td[1] = riot_adc.mcp3208_cmd.bd[1];
	td[2] = riot_adc.mcp3208_cmd.bd[0];
	gpio_clear(Ja10_2); // select the ADC
	spi_speed_config(SPI_DEV(2), 0, SPI_CLK_1MHZ); /* mode 0, speed */
	spi_transfer_bytes(SPI_DEV(2), 0, true, td, rd, 3);
	gpio_set(Ja10_2); // deselect the ADC

	/* lsb array index 2 */
	riot_adc.potValue = (rd[1]&0x0f) << 8;
	riot_adc.potValue += rd[2];
	return riot_adc.potValue;
}

int adc_init(adc_t line)
{
	(void) line;

	ADC_Init();
	return true;
}

