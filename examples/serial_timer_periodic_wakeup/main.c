/*
 * PIC32MZ EF Curiosity Development Board, port testing example
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
	uint8_t *recd = data;

	*recd = c;
	/* write received data to TX and send SPI byte */
	uart_write(1, &c, 1);
	spi_transfer_bytes(1, 0, true, recd, NULL, 1);
}

/* serial #2 interrupt received data callback processing */
static void _rx_cb2(void* data, uint8_t c)
{
	uint8_t *recd = data;

	*recd = c;
	/* write received data to TX and send SPI byte */
	uart_write(2, &c, 1);
	spi_transfer_bytes(2, 0, true, recd, NULL, 1);
}

int main(void)
{
	uint8_t data1, data2;
	char buffer[128];
	int dd, times_count = 0;
	xtimer_ticks32_t last_wakeup = xtimer_now();
	/*
	 * setup serial ports, uart 1,2,4 and spi 1,2
	 */
	uart_init(1, DEBUG_UART_BAUD, _rx_cb1, &data1);
	uart_init(2, DEBUG_UART_BAUD, _rx_cb2, &data2);
	uart_init(4, DEBUG_UART_BAUD, NULL, 0);
	spi_init(1);
	spi_init(2);
	spi_acquire(1, 0, SPI_MODE_0, SPI_CLK_1MHZ);
	spi_acquire(2, 0, SPI_MODE_0, SPI_CLK_1MHZ);

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
