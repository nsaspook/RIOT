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
#include "periph/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "vendor/p32mz2048efm100.h"
#include "vendor/ports_p32mz2048efm100.h"
#include <malloc.h>
#include <mips/notlb.h>

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
#define PDEBUG3_PIN         GPIO_PIN(PORT_E, 2)

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
#define PDEBUG3_MASK        (1 << 2)

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

#define PDEBUG3_ON          (LATESET = PDEBUG3_MASK)
#define PDEBUG3_OFF         (LATECLR = PDEBUG3_MASK)
#define PDEBUG3_TOGGLE      (LATEINV = PDEBUG3_MASK)
	/** @} */

	/* L1 cache modes, boot code defaults to WB_WA, best performance
	 * Uncached
	 * Cacheable, non-coherent, write-back, write allocate
	 * Cacheable, non-coherent, write-through, write allocate
	 * Cacheable, non-coherent, write-through, no write allocate
	 */
#define UNCACHED	0x02
#define WB_WA		0x03
#define WT_WA		0x01
#define WT_NWA		0x00

	/* from the TABLE 7-2: INTERRUPT IRQ, VECTOR, AND BIT LOCATION 
	 * PIC32MZ Embedded Connectivity with Floating Point Unit (EF) Family Data Sheet
	 */
#define EIC_IRQ_UART_1_RX	113 /* Persistent interrupts */
#define EIC_IRQ_UART_2_RX	146
#define EIC_IRQ_UART_4_RX	171

#define EIC_IRQ_SPI_1_TX	111
#define EIC_IRQ_SPI_2_TX	144
#define EIC_IRQ_SPI_3_TX	156

#define EIC_IRQ_SPI_1_RX	110
#define EIC_IRQ_SPI_2_RX	143
#define EIC_IRQ_SPI_3_RX	155

	/*
	 * Address typedefs
	 */
	typedef unsigned long _paddr_t; /* a physical address */
	typedef unsigned long _vaddr_t; /* a virtual address */

	/* 
	 * Translate a kernel virtual address in KSEG0 or KSEG1 to a real
	 * physical address and back.
	 */
	//#define KVA_TO_PA(v) 	((_paddr_t)(v) & 0x1fffffff)
#define PA_TO_KVA0(pa)	((void *) ((pa) | 0x80000000))
#define PA_TO_KVA1(pa)	((void *) ((pa) | 0xa0000000))

	/* translate between KSEG0 and KSEG1 virtual addresses */
#define KVA0_TO_KVA1(v)	((void *) ((unsigned)(v) | 0x20000000))
#define KVA1_TO_KVA0(v)	((void *) ((unsigned)(v) & ~0x20000000))

	/* Test for KSEGS */
#define IS_KVA(v)	((int)(v) < 0)
#define IS_KVA0(v)	(((unsigned)(v) >> 29) == 0x4)
#define IS_KVA1(v)	(((unsigned)(v) >> 29) == 0x5)
#define IS_KVA01(v) (((unsigned)(v) >> 30) == 0x2)

	/*  Access a KSEG0 Virtual Address variable as uncached (KSEG1) */
#define __PIC32_UNCACHED_VAR(v) __PIC32_KVA0_TO_KVA1_VAR(v)
	/*  Access a KSEG0 Virtual Address pointer as uncached (KSEG1) */
#define __PIC32_UNCACHED_PTR(v) __PIC32_KVA0_TO_KVA1_PTR(v)
	/*  Access a KSEG1 Virtual Address variable as cached (KSEG0) */
#define __PIC32_CACHED_VAR(v) __PIC32_KVA1_TO_KVA0_VAR(v)
	/*  Access a KSEG1 Virtual Address pointer as cached (KSEG0) */
#define __PIC32_CACHED_PTR(v) __PIC32_KVA1_TO_KVA0_PTR(v)

	/* Helper macros used by those above. */

	/*  Convert a KSEG0 Virtual Address variable or pointer to a KSEG1 virtual 
	 *  address access.
	 */
#define __PIC32_KVA0_TO_KVA1_VAR(v) (*(__typeof__(v)*)((unsigned long)&(v) | 0x20000000u))
#define __PIC32_KVA0_TO_KVA1_PTR(v) ((__typeof__(v)*)((unsigned long)(v) | 0x20000000u))
#define __PIC32_KVA1_TO_KVA0_VAR(v) (*(__typeof__(v)*)((unsigned long)&(v) & ~0x20000000u))
#define __PIC32_KVA1_TO_KVA0_PTR(v) ((__typeof__(v)*)((unsigned long)(v) & ~0x20000000u))

	static __inline__ void * __pic32_alloc_coherent(size_t size)
	{
		void *retptr;
		retptr = malloc(size);
		if (retptr == NULL) {
			return NULL;
		}
		/* malloc returns a cached pointer, but convert it to an uncached pointer */
		return __PIC32_UNCACHED_PTR(retptr);
	}

	static __inline__ void __pic32_free_coherent(void* ptr)
	{
		/* Convert back to a cached pointer before calling free. */
		free(__PIC32_CACHED_PTR(ptr));
	}



	void spi_transfer_bytes_async(spi_t bus, spi_cs_t cs, bool cont,
		const void *out, void *in, size_t len);
	int32_t spi_complete(spi_t bus);
	/**
	 * @brief   Board level initialization
	 */
	void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
