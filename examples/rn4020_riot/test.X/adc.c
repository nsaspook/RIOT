#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "spi.h"
#include "app.h"
#include "config.h"
#include "timers.h"
#include "ads1220.h"

extern APP_DATA appData;
extern ADC_DATA adcData;

static uint8_t *td;
static uint8_t *rd;

void ADC_Init(void)
{
	static bool adc_is_init = false;

	if (!adc_is_init) {
		adcData.mcp3208_cmd.ld = 0; // clear the command word
		adcData.chan = 0;
		adcData.mcp3208_cmd.map.start_bit = 1;
		adcData.mcp3208_cmd.map.single_diff = 1;
		adcData.mcp3208_cmd.map.index = 0; // channel
		appData.ADCcalFlag = true;
		SPI_CS0_1_J10;
		td = __pic32_alloc_coherent(32); /* uncached memory for spi transfers */
		rd = __pic32_alloc_coherent(32);
		adc_is_init = true;
	}
}

static void mcp_spi_transfer_bytes_async(spi_t bus, spi_cs_t cs, bool cont,
	const void *out, void *in, size_t len)
{
	spi_speed_config(bus, 0, SPI_CLK_1MHZ); /* mode 0, speed */
	spi_transfer_bytes_async(bus, cs, cont, out, in, len);
}

/*
 * State machine for restarting ADC and taking new readings from pot
 * Returns true when SPI data has been returned from the mpc3208; false otherwise
 */

bool ADC_Tasks(void)
{

	/* send the command sequence to the adc */
	if (!adcData.mcp3208_cmd.map.in_progress) {
		adcData.mcp3208_cmd.map.in_progress = true;
		adcData.mcp3208_cmd.map.finish = false;
		adcData.mcp3208_cmd.map.single_diff = 1;
		adcData.mcp3208_cmd.map.index = adcData.chan;
		td[0] = adcData.mcp3208_cmd.bd[2];
		td[1] = adcData.mcp3208_cmd.bd[1];
		td[2] = adcData.mcp3208_cmd.bd[0];
		SPI_CS0_0_J10; // select the ADC
		mcp_spi_transfer_bytes_async(SPI_DEV(2), 0, true, td, rd, 3);
	}

	/* read the returned spi data from the buffer and format it */
	if (adcData.mcp3208_cmd.map.in_progress) {
		if (spi_complete(SPI_DEV(2))) {
			SPI_CS0_1_J10; // deselect the ADC

			/* lsb array index 2 */
			adcData.potValue = (rd[1]&0x0f) << 8;
			adcData.potValue += rd[2];
			adcData.mcp3208_cmd.map.finish = true;
		} else {
			return false;
		}
	}

	/* cleanup for next time */
	if (adcData.mcp3208_cmd.map.finish) {
		adcData.mcp3208_cmd.map.in_progress = false;
		appData.accumReady = true;
		return true;
	}

	return false;
}

/*
 * Process the accumulator value once it is ready
 * And update stored potentiometer values
 */

void ADC_ProcAccum(void)
{
	appData.potValueOld = appData.potValue; //Save previous value
	appData.potValue = adcData.potValue;
}

void GetNewADC_Chan(void)
{
	adcData.chan = appData.receive_packet[9] == '1' ? 1 : 0; // update adc channel
	adcData.chan += appData.receive_packet[11] == '1' ? 2 : 0;
	adcData.chan += appData.receive_packet[13] == '1' ? 4 : 0;
}
