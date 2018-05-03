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
 * @defgroup    boards_pic32-cpicmzef PIC32MZ EF Curiosity Development Board
 * @ingroup     boards
 * @brief       Support for the PIC32MZ EF Curiosity Development Board
 * @details
 * See:http://ww1.microchip.com/downloads/en/DeviceDoc/70005282B.pdf
 * 
 * for more information on the board.
 *
 * @{
 *
 * @file
 * @brief       board configuration for the PIC32MZ EF Curiosity Development Board
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
#define PDEBUG1_PIN         GPIO_PIN(PORT_E, 7)	
#define PDEBUG2_PIN         GPIO_PIN(PORT_E, 1)

#define C_WIFI_SLEEP	    GPIO_PIN(PORT_A, 0)
#define C_STBY_RST	    GPIO_PIN(PORT_A, 5)
#define C_BLE_IO_WAKE	    GPIO_PIN(PORT_A, 9)
#define C_BLE_IO_CONN	    GPIO_PIN(PORT_B, 4)
#define C_WIFI_INT	    GPIO_PIN(PORT_F, 4)
#define C_SWITCH_1	    GPIO_PIN(PORT_G, 12)
#define C_USB_VBUS_SWITCH   GPIO_PIN(PORT_G, 13)

#define C_SPI1_CS	    GPIO_PIN(PORT_D, 4)
#define C_SPI2_CS	    GPIO_PIN(PORT_D, 5)
#define C_SPI3_CS	    GPIO_PIN(PORT_B, 8)

#define LED1_MASK           (1 << 3)
#define LED2_MASK           (1 << 4)
#define LED3_MASK           (1 << 6)
#define LED4_MASKR          (1 << 5)
#define LED4_MASKG          (1 << 1)
#define LED4_MASKB          (1 << 0)
#define PDEBUG1_MASK        (1 << 7)
#define PDEBUG2_MASK        (1 << 1)

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

#define PDEBUG1_ON          (LATESET = PDEBUG1_MASK)
#define PDEBUG1_OFF         (LATECLR = PDEBUG1_MASK)
#define PDEBUG1_TOGGLE      (LATEINV = PDEBUG1_MASK)

#define PDEBUG2_ON          (LATESET = PDEBUG2_MASK)
#define PDEBUG2_OFF         (LATECLR = PDEBUG2_MASK)
#define PDEBUG2_TOGGLE      (LATEINV = PDEBUG2_MASK)
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
