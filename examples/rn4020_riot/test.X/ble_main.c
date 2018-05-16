/*
 * PIC32MZ EF Curiosity Development Board, RIOT-OS RN4020 BLE testing
 */

#include <stdio.h>
#include <string.h>
#include "xtimer.h"
#include "timex.h"
#include "periph/uart.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "config.h"
#include "app.h"

/* set interval to 1 second */
#define INTERVAL (1U * US_PER_SEC)

extern void set_cache_policy(uint32_t);

int main(void)
{

	const uint8_t tdata[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
	/* allocate buffer memory in kseg1 uncached */
	//	uint8_t *td = __pic32_alloc_coherent(256);
	uint8_t *bd = __pic32_alloc_coherent(256);
	int dd;
	xtimer_ticks32_t last_wakeup = xtimer_now();

	void initBoard(void);

	/* testing cache modes */
	set_cache_policy(WB_WA);
	LED3_ON;
	/*
	 * setup serial ports, uart 1,2,4 @115200 bps and spi 1,2
	 * uart callback uses a 4 byte variable for data so SPI can
	 * transfer 4 bytes in the callback
	 */

	uart_init(4, DEBUG_UART_BAUD, NULL, 0);

	(void) bd;

	while (1) {
		/* stop unused variable warning from compiler */
		(void) last_wakeup;

		APP_Tasks();
		/*
		 * repeat the data stream to all serial ports by sending data to uart #4
		 */

		spi_transfer_bytes(SPI_DEV(2), 0, true, tdata, bd, 18);

		/* cpu busy loop delay */
		for (dd = 0; dd < 100000; dd++) {
			last_wakeup = xtimer_now();
		}

		LED3_OFF;
		LED2_ON;
	}

	return 0;
}

void initBoard(void)
{
	// LEDs are outputs and off
	RELAY1 = 1;
	RELAY2 = 1;
	RELAY3 = 1;
	RELAY4 = 1;

	//RN4020 module - UART1
	BT_WAKE_HW = 1; //Dormant line is set high
	BT_WAKE_HW_TRIS = 0; //Dormant line is output

	BT_WAKE_SW = 0; //keep low until after UART is initialized
	BT_WAKE_SW_TRIS = 0;

	BT_CMD = 0; //Command mode on
	BT_CMD_TRIS = 0;

	BT_WS_TRIS = 1;
	BT_MLDP_EV_TRIS = 1;
	BT_CONNECTED_TRIS = 1;

	U1CTS_TRIS = 1;
	U1RTS_LAT = 1;
	U1RTS_TRIS = 0;

	// SPI Master Devices
	SPI_CS0_TRIS = 0;
	SPI_CS1_TRIS = 0;
}
