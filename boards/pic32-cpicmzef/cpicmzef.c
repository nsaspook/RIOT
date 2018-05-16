/*
 * Copyright(C) 2016,2017, Imagination Technologies Limited and/or its
 *                 affiliated group companies.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 * Many thanks to for his hard work on the pic32 port
 * https://github.com/francois-berder/RIOT
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
#include <mips/cpu.h>
#include <mips/m32c0.h>

extern void dummy(void);

/* L1 cache control 
 * CP0 Register 16, Select 0
 * bit 2-0 K0<2:0>: Kseg0 bits
 * Kseg0 coherency algorithm.
 * http://ww1.microchip.com/downloads/en/AppNotes/00001600C.pdf
 */
void set_cache_policy(uint32_t cc)
{
	uint32_t cp0;

	mips_flush_cache();
	cp0 = _mips_mfc0(16);
	cp0 &= ~0x03;
	cp0 |= cc;
	_mips_mtc0(16, cp0);
	asm("nop"); /* re-sequence the pipeline after cp0 write */
	asm("nop");
	asm("sync" ::);
}

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
	/* all analog off */
	ANSELA = 0;
	ANSELB = 0;
	ANSELC = 0;
	ANSELD = 0;
	ANSELE = 0;
	ANSELF = 0;
	ANSELG = 0;

	/* UART1 control signals to BLE */

	gpio_init(Ja5_1, GPIO_IN); // connect
	gpio_init(Ja5_2, GPIO_OUT); // wake_sw
	gpio_init(Ja5_4, GPIO_OUT); // wake_hw
	gpio_init(Ja5_5, GPIO_IN); // wake from rn4020
	gpio_init(Ja5_6, GPIO_IN); // mldp event from rn4020
	gpio_init(Ja5_11, GPIO_OUT); // RTS to RN4020
	gpio_init(Ja5_12, GPIO_IN); // CTS from RN4020
	gpio_init(Ja5_16, GPIO_OUT); // CMD to rn4020

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
	gpio_init(PDEBUG3_PIN, GPIO_OUT);
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	LED4R_OFF;
	LED4G_OFF;
	LED4B_OFF;
	PDEBUG1_OFF;
	PDEBUG2_OFF;
	PDEBUG3_OFF;

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
	CNPUGSET = (1 << 12);
	gpio_init(C_USB_VBUS_SWITCH, GPIO_OUT);
	LATGCLR = (1 << 13);

	hwrng_init();

	/* Initialize all SPI modules */
	//	for (unsigned i = 1; i <= SPI_NUMOF; i++)
	//		spi_init(SPI_DEV(i));


	/* Stop the linker from throwing away the PIC32 config register settings */
	dummy();
}

void pm_reboot(void)
{
	/* TODO, note this is needed to get 'default' example to build */
}
