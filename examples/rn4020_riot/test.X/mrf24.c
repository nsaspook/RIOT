#include <stdint.h>
#include <stdbool.h>
#include "mrf24.h"
#include "spi.h"
#include "app.h"
#include "config.h"
#include "timers.h"
#include "ads1220.h"

extern APP_DATA appData;
extern ADC_DATA adcData;

uint8_t *mr24f_rxb;
uint8_t *mr24f_txb;

void Mrf24_Init(void)
{
	mr24f_rxb = __pic32_alloc_coherent(32); /* uncached memory for spi transfers */
	mr24f_txb = __pic32_alloc_coherent(32);

	spi_init(SPI_DEV(3));
	spi_acquire(SPI_DEV(3), 0, SPI_MODE_0, SPI_CLK_1MHZ);
}

void mrf24f_testing(void)
{
	static int i = 0;

	if (i++ % 320 == 0) {
		mr24f_txb[0] = 'f';
		mr24f_txb[1] = 'g';
		mr24f_txb[2] = 'b';

		gpio_clear(C_RF24F_CS);
		spi_transfer_bytes(SPI_DEV(3), 0, true, mr24f_txb, mr24f_rxb, 3);
		gpio_set(C_RF24F_CS);
	}
	ads1220_testing();
}