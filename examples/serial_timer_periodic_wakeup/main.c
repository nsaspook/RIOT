/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     xtimer_examples
 * @{
 *
 * @file
 * @brief       example application for setting a periodic wakeup
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */

#include <stdio.h>
#include "xtimer.h"
#include "timex.h"
#include "periph/uart.h"
#include "periph/gpio.h"

/* set interval to 1 second */
#define INTERVAL (1U * US_PER_SEC)

static void _rx_cb1(void* data, uint8_t c)
{
	uint8_t *recd = data;

	*recd = c;
	uart_write(1, &c, 1);
}

static void _rx_cb2(void* data, uint8_t c)
{
	uint8_t *recd = data;

	*recd = c;
	uart_write(2, &c, 1);
}

int main(void)
{
	uint8_t data1,data2;
	xtimer_ticks32_t last_wakeup = xtimer_now();
	uart_init(1, DEBUG_UART_BAUD, _rx_cb1, &data1);
	uart_init(2, DEBUG_UART_BAUD, _rx_cb2, &data2);


	while (1) {
		xtimer_periodic_wakeup(&last_wakeup, INTERVAL);
		printf("slept until %" PRIu32 "\n", xtimer_usec_from_ticks(xtimer_now()));
	}

	return 0;
}
