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
 * @name    jack pin configuration
 * @{
 */

#define Ja17_1       GPIO_PIN(PORT_E, 1)
#define Ja17_2       GPIO_PIN(PORT_E, 2)
#define Ja17_3       GPIO_PIN(PORT_E, 9)
#define Ja17_4       GPIO_PIN(PORT_B, 3)
#define Ja17_5       GPIO_PIN(PORT_B, 2)
#define Ja17_6       GPIO_PIN(PORT_A, 10)
#define Ja17_7       GPIO_PIN(PORT_B, 15)
#define Ja17_8       GPIO_PIN(PORT_E, 0)
#define Ja17_9       GPIO_PIN(PORT_G, 1)
#define Ja17_10      GPIO_PIN(PORT_D, 13)
#define Ja17_11      GPIO_PIN(PORT_D, 12)
#define Ja17_12      GPIO_PIN(PORT_C, 13)
#define Ja17_13      GPIO_PIN(PORT_D, 9)
#define Ja17_14      GPIO_PIN(PORT_F, 8)
#define Ja17_15      GPIO_PIN(PORT_F, 5)
#define Ja17_16      GPIO_PIN(PORT_A, 6)
#define Ja17_17      GPIO_PIN(PORT_A, 4)
#define Ja17_18      GPIO_PIN(PORT_E, 7)

#define Ja5_1            GPIO_PIN(PORT_B, 4)
#define Ja5_2            GPIO_PIN(PORT_A, 9)
#define Ja5_3            GPIO_PIN(PORT_D, 4)
#define Ja5_4            GPIO_PIN(PORT_D, 1)
#define Ja5_5            GPIO_PIN(PORT_D, 14)
#define Ja5_6            GPIO_PIN(PORT_D, 3)
#define Ja5_7            "+3.3v"
#define Ja5_8            "gnd"
#define Ja5_9            "gnd"
#define Ja5_10           "+5v"
#define Ja5_11           GPIO_PIN(PORT_A, 15)
#define Ja5_12           GPIO_PIN(PORT_A, 14)
#define Ja5_13           GPIO_PIN(PORT_D, 15)
#define Ja5_14           GPIO_PIN(PORT_D, 10)
#define Ja5_15           GPIO_PIN(PORT_F, 13)
#define Ja5_16           GPIO_PIN(PORT_E, 8)

#define Ja10_1           GPIO_PIN(PORT_A, 1)
#define Ja10_2           GPIO_PIN(PORT_A, 5)
#define Ja10_3           GPIO_PIN(PORT_D, 5)
#define Ja10_4           GPIO_PIN(PORT_G, 6)
#define Ja10_5           GPIO_PIN(PORT_G, 0)
#define Ja10_6           GPIO_PIN(PORT_G, 7)
#define Ja10_7           "+3.3v"
#define Ja10_8           "gnd"
#define Ja10_9           "gnd"
#define Ja10_10          "+5v"
#define Ja10_11          GPIO_PIN(PORT_A, 15)
#define Ja10_12          GPIO_PIN(PORT_A, 14)
#define Ja10_13          GPIO_PIN(PORT_C, 2)
#define Ja10_14          GPIO_PIN(PORT_C, 3)
#define Ja10_15          GPIO_PIN(PORT_F, 12) /* INT2 not working on this pin */
#define Ja10_16          GPIO_PIN(PORT_F, 2)

/** @} */

/**
 * @name    pin configuration
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

#define C_WIFI_SLEEP        GPIO_PIN(PORT_A, 0)
#define C_RF24F_CS      GPIO_PIN(PORT_B, 8)
#define C_WIFI_INT      GPIO_PIN(PORT_F, 4)

#define C_BLE_IO_WAKE_SW    Ja5_2
#define C_BLE_IO_WAKE_HW    Ja5_4
#define C_BLE_IO_CONN       Ja5_1
#define C_BT_CMD        Ja5_16
#define C_BT_WS         Ja5_5
#define C_BT_MLDP_EV        Ja5_6

#define C_STBY_RST      GPIO_PIN(PORT_A, 5)
#define C_SWITCH_1      GPIO_PIN(PORT_G, 12)
#define C_USB_VBUS_SWITCH   GPIO_PIN(PORT_G, 13)

#define C_SPI0_CS_J5        Ja5_3
#define C_SPI0_CS_J10       Ja10_3
#define C_SPI1_CS       Ja10_2
#define C_SPI2_CS       Ja10_13

#define C_INT2          Ja10_14

#define LED1_ON             gpio_set(LED1_PIN)
#define LED1_OFF            gpio_clear(LED1_PIN)
#define LED1_TOGGLE         gpio_toggle(LED1_PIN)

#define LED2_ON             gpio_set(LED2_PIN)
#define LED2_OFF            gpio_clear(LED2_PIN)
#define LED2_TOGGLE         gpio_toggle(LED2_PIN)

#define LED3_ON             gpio_set(LED3_PIN)
#define LED3_OFF            gpio_clear(LED3_PIN)
#define LED3_TOGGLE         gpio_toggle(LED3_PIN)

#define LED4R_OFF           gpio_set(LED4_PINR)
#define LED4R_ON            gpio_clear(LED4_PINR)
#define LED4R_TOGGLE        gpio_toggle(LED4_PINR)

#define LED4G_OFF           gpio_set(LED4_PING)
#define LED4G_ON            gpio_clear(LED4_PING)
#define LED4G_TOGGLE        gpio_toggle(LED4_PING)

#define LED4B_OFF           gpio_set(LED4_PINB)
#define LED4B_ON            gpio_clear(LED4_PINB)
#define LED4B_TOGGLE        gpio_toggle(LED4_PINB)

#define PDEBUG1_ON          gpio_set(PDEBUG1_PIN)
#define PDEBUG1_OFF         gpio_clear(PDEBUG1_PIN)
#define PDEBUG1_TOGGLE      gpio_toggle(PDEBUG1_PIN)

#define PDEBUG2_ON          gpio_set(PDEBUG2_PIN)
#define PDEBUG2_OFF         gpio_clear(PDEBUG2_PIN)
#define PDEBUG2_TOGGLE      gpio_toggle(PDEBUG2_PIN)

#define PDEBUG3_ON          gpio_set(PDEBUG3_PIN)
#define PDEBUG3_OFF         gpio_clear(PDEBUG3_PIN)
#define PDEBUG3_TOGGLE      gpio_toggle(PDEBUG3_PIN)
/** @} */

/* L1 cache modes, boot code defaults to WB_WA, best performance
 * Uncached
 * Cacheable, non-coherent, write-back, write allocate
 * Cacheable, non-coherent, write-through, write allocate
 * Cacheable, non-coherent, write-through, no write allocate
 */
#define UNCACHED    0x02
#define WB_WA       0x03
#define WT_WA       0x01
#define WT_NWA      0x00

/* from the TABLE 7-2: INTERRUPT IRQ, VECTOR, AND BIT LOCATION
 * PIC32MZ Embedded Connectivity with Floating Point Unit (EF) Family Data Sheet
 */
#define EIC_IRQ_UART_1_RX   113 /* Persistent interrupts */
#define EIC_IRQ_UART_2_RX   146
#define EIC_IRQ_UART_4_RX   171

#define EIC_IRQ_SPI_1_TX    111
#define EIC_IRQ_SPI_2_TX    144
#define EIC_IRQ_SPI_3_TX    156

#define EIC_IRQ_SPI_1_RX    110
#define EIC_IRQ_SPI_2_RX    143
#define EIC_IRQ_SPI_3_RX    155

/*
 * Address typedefs
 */
typedef unsigned long _paddr_t;     /* a physical address */
typedef unsigned long _vaddr_t;     /* a virtual address */

/*
 * Translate a kernel virtual address in KSEG0 or KSEG1 to a real
 * physical address and back.
 * using compiler KVA_TO_PA macro
 */
/* #define KVA_TO_PA(v)     ((_paddr_t)(v) & 0x1fffffff) */
#define PA_TO_KVA0(pa)  ((void *) ((pa) | 0x80000000))
#define PA_TO_KVA1(pa)  ((void *) ((pa) | 0xa0000000))

/* translate between KSEG0 and KSEG1 virtual addresses */
#define KVA0_TO_KVA1(v) ((void *) ((unsigned)(v) | 0x20000000))
#define KVA1_TO_KVA0(v) ((void *) ((unsigned)(v) & ~0x20000000))

/* Test for KSEGS */
#define IS_KVA(v)   ((int)(v) < 0)
#define IS_KVA0(v)  (((unsigned)(v) >> 29) == 0x4)
#define IS_KVA1(v)  (((unsigned)(v) >> 29) == 0x5)
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
#define __PIC32_KVA0_TO_KVA1_VAR(v) (*(__typeof__(v) *)((unsigned long)& (v) | 0x20000000u))
#define __PIC32_KVA0_TO_KVA1_PTR(v) ((__typeof__(v) *)((unsigned long)(v) | 0x20000000u))
#define __PIC32_KVA1_TO_KVA0_VAR(v) (*(__typeof__(v) *)((unsigned long)& (v) & ~0x20000000u))
#define __PIC32_KVA1_TO_KVA0_PTR(v) ((__typeof__(v) *)((unsigned long)(v) & ~0x20000000u))

static __inline__ void *__pic32_alloc_coherent(size_t size)
{
    void *retptr;

    retptr = malloc(size);
    if (retptr == NULL) {
        return NULL;
    }
    /* malloc returns a cached pointer, but convert it to an uncached pointer */
    return __PIC32_UNCACHED_PTR(retptr);
}

static __inline__ void __pic32_free_coherent(void *ptr)
{
    /* Convert back to a cached pointer before calling free. */
    free(__PIC32_CACHED_PTR(ptr));
}

/*******************************************************************/
/** FlushCache - flush cache to RAM                               **/
/** size - size of memory block to flush *MUST BE MULTIPLE OF 16* **/
/** vaddr - start address of block to flush                        **/
/*******************************************************************/
/* void FlushCache(unsigned size, void *vaddr) */

/* spi driver extras */
void spi_transfer_bytes_async(spi_t bus, spi_cs_t cs, bool cont,
                              const void *out, void *in, size_t len);
int32_t spi_complete(spi_t bus);
void spi_speed_config(spi_t, spi_mode_t, spi_clk_t);

/* timer extras */
void timer_shortdelay(uint32_t);
/**
 * @brief   Board level initialization
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
