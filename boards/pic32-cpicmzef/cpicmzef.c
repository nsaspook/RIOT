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
#include "periph/adc.h"
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
	SYSKEY = 0xAA996655; //unlock sequence
	SYSKEY = 0x556699AA; //unlock sequence
	CFGCONbits.IOLOCK = 0; //unlock PPS registers
	/*
	 * Setup default pin mux for UARTS
	 * done early and static for debug output during boot
	 */
	U4RXR = INPUT_PIN_RPB2; /* connect pin RPB2 to UART 4 RX */
	RPF8R = OUTPUT_FUNC_U4TX; /* connect pin RPF8 to UART 4 TX */
	U3RXR = INPUT_PIN_RPF5; /* RPF5 RX */
	RPG1R = OUTPUT_FUNC_U3TX; /* UART 3 TX */
	U2RXR = INPUT_PIN_RPC3; /* RPC3 RX */
	RPC2R = OUTPUT_FUNC_U2TX; /* UART 2 TX */
	U1RXR = INPUT_PIN_RPD10; /* RPD10 RX */
	RPD15R = OUTPUT_FUNC_U1TX; /* UART 1 TX */

	//External interrupt
	INT2R = INPUT_PIN_RPC3; /* mikro port #2 interrupt */

	//	CFGCONbits.IOLOCK = 1; //lock PPS registers
	//	SYSKEY = 0x33333333; //lock sequnace


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

	/* mikro BUS #1 control signals to BLE2 click */
	gpio_init(Ja5_1, GPIO_IN); // connect
	gpio_init(Ja5_2, GPIO_OUT); // wake_sw
	gpio_init(Ja5_4, GPIO_OUT); // wake_hw
	gpio_init(Ja5_5, GPIO_IN); // wake from rn4020
	gpio_init(Ja5_6, GPIO_IN); // mldp event from rn4020
	gpio_init(Ja5_11, GPIO_OUT); // RTS to RN4020
	gpio_init(Ja5_12, GPIO_IN); // CTS from RN4020
	gpio_init(Ja5_16, GPIO_OUT); // CMD to rn4020

	/* mikro BUS #2 spi related control signals*/
	gpio_init(Ja10_13, GPIO_OUT); // CS2
	gpio_init(Ja10_2, GPIO_OUT); // CS1
	gpio_init(Ja10_3, GPIO_OUT); // CS0
	gpio_init(Ja10_14, GPIO_IN_PU); // INT2 for ADC

	/* init uart ports */
	gpio_init(Ja5_13, GPIO_OUT);
	gpio_init(Ja5_14, GPIO_IN);
	gpio_init(Ja17_9, GPIO_OUT);
	gpio_init(Ja17_15, GPIO_IN);
	gpio_init(Ja17_14, GPIO_OUT);
	gpio_init(Ja17_5, GPIO_IN);

	/*	gpio_init(Ja10_13, GPIO_OUT);
	 *  used by spi
	 */

	/*	gpio_init(Ja10_14, GPIO_IN);
	 *  used by spi
	 */

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
	/* init mf24f pins, the WiFi chip is not useful with riot-os
	 * because of no register data
	 */
	gpio_init(C_WIFI_SLEEP, GPIO_OUT);
	gpio_set(C_WIFI_SLEEP);
	gpio_init(C_RF24F_CS, GPIO_OUT);
	gpio_set(C_RF24F_CS);
	gpio_init(C_WIFI_INT, GPIO_IN_PU);

	gpio_init(C_STBY_RST, GPIO_OUT);
	gpio_clear(C_STBY_RST);

	gpio_init(C_BLE_IO_WAKE_SW, GPIO_OUT);
	gpio_clear(C_BLE_IO_WAKE_SW); //keep low until after UART is initialized
	gpio_init(C_BLE_IO_WAKE_HW, GPIO_OUT);
	gpio_set(C_BLE_IO_WAKE_HW); //Dormant line is set high
	gpio_init(C_BT_CMD, GPIO_OUT);
	gpio_clear(C_BT_CMD); //Command mode on
	gpio_init(C_BLE_IO_CONN, GPIO_IN_PU);
	gpio_init(C_BT_MLDP_EV, GPIO_IN_PU);
	gpio_init(C_BT_WS, GPIO_IN_PU);
	gpio_clear(Ja5_11); // send RTS low to RN4020

	gpio_init(C_SWITCH_1, GPIO_IN_PU);
	gpio_init(C_USB_VBUS_SWITCH, GPIO_OUT);
	gpio_clear(C_USB_VBUS_SWITCH);

	hwrng_init();
	adc_init(0);

	/* Stop the linker from throwing away the PIC32 config register settings */
	dummy();
}

void pm_reboot(void)
{
	/* TODO, note this is needed to get 'default' example to build */
}
