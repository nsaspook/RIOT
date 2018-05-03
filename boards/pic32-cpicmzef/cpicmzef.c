/*
 * Copyright(C) 2016,2017, Imagination Technologies Limited and/or its
 *                 affiliated group companies.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

#include <stdio.h>
#include <stdint.h>
#include "periph/gpio.h"
#include "periph/hwrng.h"
#include "periph/spi.h"
#include "periph/uart.h"
#include "bitarithm.h"
#include "board.h"
#include "cpu.h"

extern void dummy(void);

void board_init(void)
{
	/*
	 * Setup pin mux for UARTS 
	 */
	U4RXR = INPUT_PIN_RPB2; /* connect pin RPB2 to UART 4 RX */
	RPF8R = OUTPUT_FUNC_U4TX; /* connect pin RPF8 to UART 4 TX */
	U2RXR = INPUT_PIN_RPC3; /* RPC3 */
	RPC2R = OUTPUT_FUNC_U2TX; /* UART 2 TX */
	U1RXR = INPUT_PIN_RPD10; /* RPD10 */
	RPD15R = OUTPUT_FUNC_U1TX; /* UART 1 TX */

	/* init UART used for debug (printf) */
#ifdef DEBUG_VIA_UART
	uart_init(DEBUG_VIA_UART, DEBUG_UART_BAUD, NULL, 0);
#endif

	/* init uart ports */
	gpio_init(GPIO_PIN(PORT_F, 8), GPIO_OUT);
	gpio_init(GPIO_PIN(PORT_C, 2), GPIO_OUT);
	gpio_init(GPIO_PIN(PORT_D, 15), GPIO_OUT);
	gpio_init(GPIO_PIN(PORT_B, 2), GPIO_IN);
	gpio_init(GPIO_PIN(PORT_C, 3), GPIO_IN);
	gpio_init(GPIO_PIN(PORT_D, 10), GPIO_IN);
	/* Turn off all LED's */
	gpio_init(LED1_PIN, GPIO_OUT);
	gpio_init(LED2_PIN, GPIO_OUT);
	gpio_init(LED3_PIN, GPIO_OUT);
	gpio_init(LED4_PINR, GPIO_OUT);
	gpio_init(LED4_PING, GPIO_OUT);
	gpio_init(LED4_PINB, GPIO_OUT);
	gpio_init(PDEBUG1_PIN, GPIO_OUT);
	gpio_init(PDEBUG2_PIN, GPIO_OUT);
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	LED4R_OFF;
	LED4G_OFF;
	LED4B_OFF;
	PDEBUG1_OFF;
	PDEBUG2_OFF;
	
	/* board device defaults */
	gpio_init(C_WIFI_SLEEP, GPIO_OUT);
	LATACLR = (1 << 0);
	gpio_init(C_STBY_RST, GPIO_OUT);
	LATACLR = (1 << 5);
	gpio_init(C_BLE_IO_WAKE, GPIO_OUT);
	LATACLR = (1 << 9);
	gpio_init(C_BLE_IO_CONN, GPIO_IN);
	gpio_init(C_WIFI_INT, GPIO_IN);
	gpio_init(C_SWITCH_1, GPIO_IN);
	CNPUGSET= (1 << 12);
	gpio_init(C_USB_VBUS_SWITCH, GPIO_OUT);
	LATGCLR = (1 << 13);

	hwrng_init();

	/* Initialize all SPI modules */
	for (unsigned i = 1; i <= SPI_NUMOF; i++)
		spi_init(SPI_DEV(i));


	/* Stop the linker from throwing away the PIC32 config register settings */
	dummy();
}

void pm_reboot(void)
{
	/* TODO, note this is needed to get 'default' example to build */
}
