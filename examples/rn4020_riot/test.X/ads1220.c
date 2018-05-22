#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "spi.h"
#include "app.h"
#include "config.h"
#include "timers.h"
#include "ads1220.h"

uint8_t *tx_buff;
uint8_t *rx_buff;

void ads_spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont,
	const void *out, void *in, size_t len)
{
	spi_speed_config(bus, 1, 0); /* mode 1, no speed change */
	SPI_CS1 = 0;
	spi_transfer_bytes(bus, cs, cont, out, in, len);
	SPI_CS1 = 1;
}

int ads1220_init(void)
{
	tx_buff = __pic32_alloc_coherent(32); /* uncached memory for spi transfers */
	rx_buff = __pic32_alloc_coherent(32);
	/* 
	 * setup ads1220 registers
	 */
	tx_buff[0] = ADS1220_CMD_WREG + 3;
	tx_buff[1] = ads1220_r0;
	tx_buff[2] = ads1220_r1;
	tx_buff[3] = ads1220_r2;
	tx_buff[4] = ads1220_r3;
	spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 5);
	//	usleep_range(40, 50);
	tx_buff[0] = ADS1220_CMD_RREG + 3;
	tx_buff[1] = 0;
	tx_buff[2] = 0;
	tx_buff[3] = 0;
	tx_buff[4] = 0;
	ads_spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 5);
	//	usleep_range(40, 50);
	/*
	 * Check to be sure we have a device
	 */
	if ((rx_buff[1] != ads1220_r0) ||
		(rx_buff[2] != ads1220_r1)) {
		printf(
			"\r\nADS1220 configuration error: %x %x %x %x\r\n",
			rx_buff[1], rx_buff[2],
			rx_buff[3], rx_buff[4]);
		return(-1);
	}
	tx_buff[0] = ADS1220_CMD_SYNC;
	ads_spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 1);
	return 0;
}

static void ADS1220WriteRegister(int32_t StartAddress, int32_t NumRegs, uint32_t * pData)
{
	int32_t i;

	tx_buff[0] = ADS1220_CMD_WREG | (((StartAddress << 2) & 0x0c) | ((NumRegs - 1)&0x03));

	for (i = 0; i < NumRegs; i++) {
		tx_buff[i + 1] = *pData++;
	}

	ads_spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, NumRegs + 2);
	return;
}

static void ai_set_chan_range_ads1220(void)
{
	uint32_t range = 1;
	uint32_t chan = 0;
	uint32_t cMux;

	/*
	 * convert chan/range to input MUX switches/gains if needed
	 * we could just feed the raw bits to the Mux if needed
	 */

	switch (chan) {
	case 0:
		cMux = ADS1220_MUX_0_1;
		break;
	case 1:
		cMux = ADS1220_MUX_2_3;
		break;
	case 2:
		cMux = ADS1220_MUX_2_G;
		break;
	case 3:
		cMux = ADS1220_MUX_3_G;
		break;
	case 4:
		cMux = ADS1220_MUX_DIV2;
		break;
	default:
		cMux = ADS1220_MUX_0_1;
	}
	cMux |= ((range & 0x03) << 1); /* setup the gain bits for range with NO pga */
	cMux |= ads1220_r0_for_mux_gain;
	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &cMux);
}

int ads1220_testing(void)
{
	static int i = 0;
	static int32_t val = 0;

	if (i++ % 320 == 0) {

		ai_set_chan_range_ads1220();

		/* read the ads1220 3 byte data result */
		tx_buff[0] = ADS1220_CMD_RDATA;
		tx_buff[1] = 0;
		tx_buff[2] = 0;
		tx_buff[3] = 0;
		ads_spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 4);
		val = rx_buff[1];
		val = (val << 8) | rx_buff[2];
		val = (val << 8) | rx_buff[3];

		/* mangle the data as necessary */
		/* Bipolar Offset Binary */
		val &= 0x0ffffff;
		val ^= 0x0800000;

		tx_buff[0] = ADS1220_CMD_SYNC;
		ads_spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 1);
	}
	return val;
}
