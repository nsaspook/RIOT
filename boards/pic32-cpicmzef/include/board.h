/*
 * Copyright(C) 2017, Imagination Technologies Limited and/or its
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
 * @brief       Support for the Digilent PIC32 WiFire
 * @details
 * See:
 * http://store.digilentinc.com/chipkit-wi-fire-wifi-enabled-mz-microcontroller-board/
 * for more information on the board.
 *
 * @{
 *
 * @file
 * @brief       board configuration for the Digilent PIC32 WiFire
 *
 * @author      Neil Jones <Neil.Jones@imgtec.com>
 */

#ifndef BOARD_H
#define BOARD_H

#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "vendor/p32mz2048efm100.h"
#include "vendor/ports_p32mz2048efm100.h"

	/**
	 * @brief   Set how many increments of the count register per uS
	 *          needed by the timer code.
	 */
#define TICKS_PER_US (100)

	/**
	 * @brief   We are using an External Interrupt Controller (all pic32 devices use this mode)
	 */
#define EIC_IRQ      (1)

	/**
	 * @name    LED pin configuration
	 * @{
	 */
#define LED1_PIN            GPIO_PIN(PORT_E, 3)
#define LED2_PIN            GPIO_PIN(PORT_E, 4)
#define LED3_PIN            GPIO_PIN(PORT_E, 6)
#define LED4_PINR           GPIO_PIN(PORT_B, 5)
#define LED4_PING           GPIO_PIN(PORT_B, 1)	
#define LED4_PINB           GPIO_PIN(PORT_B, 0)

#define LED1_MASK           (1 << 3)
#define LED2_MASK           (1 << 4)
#define LED3_MASK           (1 << 6)
#define LED4_MASKR          (1 << 5)
#define LED4_MASKG          (1 << 1)
#define LED4_MASKB          (1 << 0)

#define LED1_ON             (LATESET = LED1_MASK)
#define LED1_OFF            (LATECLR = LED1_MASK)
#define LED1_TOGGLE         (LATEINV = LED1_MASK)

#define LED2_ON             (LATESET = LED2_MASK)
#define LED2_OFF            (LATECLR = LED2_MASK)
#define LED2_TOGGLE         (LATEINV = LED2_MASK)

#define LED3_ON             (LATESET = LED3_MASK)
#define LED3_OFF            (LATECLR = LED3_MASK)
#define LED3_TOGGLE         (LATEINV = LED3_MASK)

#define LED4R_OFF           (LATBSET = LED4_MASKR)
#define LED4R_ON            (LATBCLR = LED4_MASKR)
#define LED4R_TOGGLE        (LATBINV = LED4_MASKR)

#define LED4G_OFF           (LATBSET = LED4_MASKG)
#define LED4G_ON            (LATBCLR = LED4_MASKG)
#define LED4G_TOGGLE        (LATBINV = LED4_MASKG)

#define LED4B_OFF           (LATBSET = LED4_MASKB)
#define LED4B_ON            (LATBCLR = LED4_MASKB)
#define LED4B_TOGGLE        (LATBINV = LED4_MASKB)
	/** @} */

	/**
	 * @brief   Board level initialization
	 */
	void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
