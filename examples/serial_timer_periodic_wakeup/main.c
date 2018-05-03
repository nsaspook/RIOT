/*
 * PIC32MZ EF Curiosity Development Board, RIOT-OS port testing example
 */

#include <stdio.h>
#include <string.h>
#include "xtimer.h"
#include "timex.h"
#include "periph/uart.h"
#include "periph/gpio.h"
#include "periph/spi.h"

/* set interval to 1 second */
#define INTERVAL (1U * US_PER_SEC)

/* serial #1 interrupt received data callback processing */
static void _rx_cb1(void* data, uint8_t c)
{
	uint8_t *recd = data, rdata[4];

	*recd = c;
	/* write received data to TX and send SPI byte */
	uart_write(1, &c, 1);
	/* SPI in interrupt context for testing, bus has mutex_lock 
	 * we receive one byte from the uart and transfer 4 bytes using SPI
	 */
	spi_transfer_bytes(SPI_DEV(1), 0, true, recd, rdata, 4);
}

/* serial #2 interrupt received data callback processing */
static void _rx_cb2(void* data, uint8_t c)
{
	uint8_t *recd = data, rdata[4];

	*recd = c;
	/* write received data to TX and send SPI byte */
	uart_write(2, &c, 1);
	/* SPI in interrupt context for testing, bus has mutex_lock 
	 * we receive one byte from the uart and transfer 4 bytes using SPI
	 */
	spi_transfer_bytes(SPI_DEV(2), 0, true, recd, rdata, 4);
}

int main(void)
{
	/* variable data[1..2] byte 4 has SPI id data for testing */
	uint32_t data1 = 0x0f000000, data2 = 0xf0000000;
	char buffer[128];
	int dd, times_count = 0;
	xtimer_ticks32_t last_wakeup = xtimer_now();
	/*
	 * setup serial ports, uart 1,2,4 @115200 bps and spi 1,2
	 * uart callback uses a 4 byte variable for data so SPI can 
	 * transfer 4 bytes in the callback
	 */
	uart_init(1, DEBUG_UART_BAUD, _rx_cb1, &data1);
	uart_init(2, DEBUG_UART_BAUD, _rx_cb2, &data2);
	uart_init(4, DEBUG_UART_BAUD, NULL, 0);
	spi_init(SPI_DEV(1));
	spi_init(SPI_DEV(2));
	spi_acquire(SPI_DEV(1), 0, SPI_MODE_0, SPI_CLK_1MHZ);
	spi_acquire(SPI_DEV(2), 0, SPI_MODE_0, SPI_CLK_1MHZ);

	while (1) {
		/* stop unused variable warning from compiler */
		(void) last_wakeup;
		/*
		 * repeat the data stream to all serial ports by sending data to uart #4
		 */
		sprintf(buffer, "Times %d, Testing longer string %" PRIu32 "\n", times_count++, xtimer_usec_from_ticks(xtimer_now()));
		/* send string to serial device #4, TX pin out looped to device #1 and 2 RX pin inputs */
		uart_write(4, (uint8_t *) buffer, strlen(buffer));
		/* cpu busy loop delay */
		for (dd = 0; dd < 100000; dd++) {
			last_wakeup = xtimer_now();
		}
	}

	return 0;
}
