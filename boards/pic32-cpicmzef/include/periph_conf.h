/*
 * Copyright(C) 2016,2017, Imagination Technologies Limited and/or its
 *                 affiliated group companies.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @defgroup    boards_pic32-wifire Digilent PIC32 WiFire
 * @ingroup     boards
 * @brief       peripheral configuration for the Digilent PIC32 WiFire
 * @{
 *
 * @file
 * @brief       peripheral configuration for the Digilent PIC32 WiFire
 *
 * @author       Neil Jones <Neil.Jones@imgtec.com>
 */
#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#include "periph_cpu.h"
#include "vendor/p32mz2048efm100.h"
#include "vendor/ports_p32mz2048efm100.h"

#ifdef __cplusplus
extern "C" {
#endif

	/**
	 * @brief   The peripheral clock is required for the UART Baud rate calculation
	 *          It is configured by the 'config' registers (see pic32_config_settings.c)
	 */
#define PERIPHERAL_CLOCK (100000000)  /* Hz */

	/**
	 * @name    Timer definitions
	 * @{
	 */
#define TIMER_NUMOF         (1)
#define TIMER_0_CHANNELS    (3)
	/** @} */

	/**
	 * @name    UART Definitions
	 *          There are 6 UARTS available on this CPU.
	 *          We route debug via UART4 on this board,
	 *          this is the UART connected to the pic32 I/O header.
	 *
	 *          Note Microchip number the UARTS 1->4.
	 * @{
	 */
#define UART_NUMOF          (6)
#define DEBUG_VIA_UART      (4)
#define DEBUG_UART_BAUD     (115200)
	/** @} */

	/**
	 * @name    SPI device configuration
	 *
	 * @{
	 */

	static const spi_conf_t spi_config[] = {
		{
			.mosi_pin = 0,
			.mosi_reg = 0,
			.mosi_af = 0,
			.miso_pin = 0,
			.miso_reg = 0,
			.miso_af = 0
		}, /* No SPI0 on PIC32, dummy to compile */

		{ /*
         * SPI 1 (MikBUS 1)
         *      MOSI -> RD3
         *      MISO -> RD14
         *      SCK  -> RD1
         */
			.mosi_pin = GPIO_PIN(PORT_D, 3),
			.mosi_reg = (volatile uint32_t*) & RPD3R,
			.mosi_af = OUTPUT_FUNC_SDO1,
			.miso_pin = GPIO_PIN(PORT_D, 14),
			.miso_reg = (volatile uint32_t*) & SDI1R,
			.miso_af = INPUT_PIN_RPD14,
		},

		{ /*
         * SPI 2 (MikBUS 2)
         *      MOSI -> RG7
         *      MISO -> RG0
         *      SCK  -> RG6
         */
			.mosi_pin = GPIO_PIN(PORT_G, 7),
			.mosi_reg = (volatile uint32_t*) & RPG7R,
			.mosi_af = OUTPUT_FUNC_SDO2,
			.miso_pin = GPIO_PIN(PORT_G, 0),
			.miso_reg = (volatile uint32_t*) & SDI2R,
			.miso_af = INPUT_PIN_RPG0,
		},

		{ /*
         * SPI 3 (MRF24WN0MA-1/RM100 - wifi module)
         *   MOSI -> RB9
         *   MISO -> RB10
         *   SCK  -> RB14
         */
			.mosi_pin = GPIO_PIN(PORT_B, 9),
			.mosi_reg = (volatile uint32_t*) & RPB9R,
			.mosi_af = OUTPUT_FUNC_SDO3,
			.miso_pin = GPIO_PIN(PORT_B, 10),
			.miso_reg = (volatile uint32_t*) & SDI3R,
			.miso_af = INPUT_PIN_RPB10,
		},
	};

#define SPI_NUMOF	(3)
#define DMA_NUMOF	(8)
	/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
