/*
 * Copyright 2016, Imagination Technologies Limited and/or its
 *                 affiliated group companies.
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_mips32r2_common
 * @ingroup     drivers_periph_timer
 * @{
 *
 * @file
 * @brief       Implementation of the low-level timer driver
 *
 * @}
 */

#include <mips/cpu.h>
#include <mips/m32c0.h>
#include <mips/regdef.h>
#include <mips/asm.h>
#include <string.h>
#include <sys/time.h>
#include <stdio.h>
#include <periph/timer.h>
#include "periph/uart.h"
#include "cpu_conf.h"
#include "sched.h"
#include "thread.h"
#include "board.h"
#include "irq.h"
#include "timex.h"
#include "div.h"


#ifdef EIC_IRQ
#include "eic_irq.h"
#endif

/* core timer blocking delay
 * example:     timer_shortdelay(50 * US_TO_CT_TICKS);
 * 50us delay time
 */
void timer_shortdelay(uint32_t DelayCount)
{
    uint32_t StartTime;

    StartTime = _mips_mfc0(9); // Get CoreTimer value for StartTime
    while ((uint32_t) (_mips_mfc0(9) - StartTime) < DelayCount) {}
}

/*
 * setting TIMER_ACCURACY_SHIFT lower will improve accuracy
 * at the cost of more regular interrupts (hence less power efficient).
 * */
#define TIMER_ACCURACY_SHIFT (10)
#define TIMER_ACCURACY       (1 << TIMER_ACCURACY_SHIFT)
#define CHANNELS             (3)

extern void uart_1_isr_rx(void);
extern void uart_2_isr_rx(void);
extern void uart_3_isr_rx(void);
extern void uart_4_isr_rx(void);

extern void spi_1_isr_rx(void);
extern void spi_2_isr_rx(void);
extern void spi_3_isr_rx(void);

extern void dma_spi_1_isr_rx(void);
extern void dma_spi_2_isr_rx(void);
extern void dma_spi_3_isr_rx(void); /* not used */

extern void rn4020_t1interrupt(void);
extern void rn4020_int_2_isr(void);

/*
 * The base MIPS count / compare timer is fixed frequency at core clock / 2
 * and is pretty basic This timer is currently only supported in Vectored
 * Interrupt Mode (VI), EIC mode is partially supported in non-vectored mode
 * only.
 *
 * RIOT's xtimer expects the timer to operate at 1MHZ or any 2^n multiple or
 * factor of this, thus we maintain a software timer which counts at 1MHz.
 * This is not particularly power efficient and may add latency too.
 *
 * If other SoC specific timers are available which are more flexible then
 * it is advised to use them, this timer is a fallback version
 * that should work on all MIPS implementations.
 *
 */

static timer_isr_ctx_t timer_isr_ctx;
volatile unsigned int counter;
volatile unsigned int compares[CHANNELS];

/*
 * The mips toolchain C library does not implement gettimeofday()
 *
 * implement it here using the timer.
 *
 */
int gettimeofday(struct timeval *__restrict __p, void *__restrict __tz)
{
    (void) __tz;

    uint64_t now = counter * US_PER_MS;
    __p->tv_sec = div_u64_by_1000000(now);
    __p->tv_usec = now - (__p->tv_sec * US_PER_SEC);

    return 0;
}

int timer_init(tim_t dev, unsigned long freq, timer_cb_t cb, void *arg)
{
    assert(dev == 0);

    (void) dev;
    (void) freq; /* Cannot adjust Frequency */

    timer_isr_ctx.cb = cb;
    timer_isr_ctx.arg = arg;

    /* Clear down soft counters */
    memset((void *) compares, 0, sizeof(compares));

    counter = (1 << TIMER_ACCURACY_SHIFT);

    /* Set compare up */
    mips_setcompare(mips_getcount() + TICKS_PER_US * TIMER_ACCURACY);

    /* Start the timer if stopped */
    mips32_bc_c0(C0_CAUSE, CR_DC);

    /* Enable Timer Interrupts */
#ifdef EIC_IRQ
    eic_irq_configure(EIC_IRQ_TIMER);
#else
    mips32_bs_c0(C0_STATUS, SR_HINT5);
#endif


    return 0;
}

int timer_set(tim_t dev, int channel, unsigned int timeout)
{
    assert(dev == 0);
    assert(channel < CHANNELS);

    (void) dev;

    timeout >>= TIMER_ACCURACY_SHIFT;
    timeout <<= TIMER_ACCURACY_SHIFT;

    uint32_t status = irq_disable();
    compares[channel] = counter + timeout;
    irq_restore(status);

    return 1;
}

int timer_set_absolute(tim_t dev, int channel, unsigned int value)
{
    assert(dev == 0);
    assert(channel < CHANNELS);

    (void) dev;

    value >>= TIMER_ACCURACY_SHIFT;
    value <<= TIMER_ACCURACY_SHIFT;

    uint32_t status = irq_disable();
    compares[channel] = value;
    irq_restore(status);

    return 1;
}

int timer_clear(tim_t dev, int channel)
{
    assert(dev == 0);
    assert(channel < CHANNELS);

    (void) dev;

    uint32_t status = irq_disable();
    compares[channel] = 0;
    irq_restore(status);

    return 1;
}

unsigned int timer_read(tim_t dev)
{
    assert(dev == 0);

    (void) dev;

    return counter;
}

void timer_start(tim_t dev)
{
    (void) dev;
    mips32_bc_c0(C0_CAUSE, CR_DC);
}

void timer_stop(tim_t dev)
{
    (void) dev;
    mips32_bs_c0(C0_CAUSE, CR_DC);
}

void timer_irq_enable(tim_t dev)
{
    (void) dev;
#ifdef EIC_IRQ
    eic_irq_enable(EIC_IRQ_TIMER);
#else
    mips32_bs_c0(C0_STATUS, SR_HINT5);
#endif

}

void timer_irq_disable(tim_t dev)
{
    (void) dev;
#ifdef EIC_IRQ
    eic_irq_disable(EIC_IRQ_TIMER);
#else
    mips32_bc_c0(C0_STATUS, SR_HINT5);
#endif
}

/* note Compiler inserts GP context save + restore code (to current stack). */
#ifdef EIC_IRQ
/*
 * This is a hack - currently the toolchain does not support correct placement
 * of EIC mode vectors (it is coming though) But we can support non-vectored EIC
 * mode and note the default PIC32 interrupt controller (which uses EIC +
 * MCU-ASE) defaults to non vectored mode anyway with all interrupts coming via
 * vector 0 which is equivalent to 'sw0' in 'VI' mode.
 *
 * Thus all EIC interrupts should be decoded here
 *
 * When toolchain support is available we could move to full vector mode but
 * this does take up significant space (MCU-ASE provides 256 vectors at 32B
 * spacing (the default) thats 8KB of vector space!), So a single entry point
 * may be better anyway.
 *
 */
void __attribute__((interrupt("vector=sw0"), keep_interrupts_masked)) _mips_isr_sw0(void)
#else

void __attribute__((interrupt("vector=hw5"))) _mips_isr_hw5(void)
#endif
{
    register int cr = mips_getcr();

    if (cr & CR_TI) {
#ifdef EIC_IRQ
        eic_irq_ack(EIC_IRQ_TIMER);
#endif
        uint32_t status = irq_disable();
        counter += TIMER_ACCURACY;
        irq_restore(status);

        if (counter == compares[0]) {
            /*
             * The Xtimer code expects the ISR to take some time
             * but our counter is a fake software one, so bump it a
             * bit to give the impression some time elapsed in the ISR.
             * Without this the callback ( _shoot(timer) on xtimer_core.c )
             * never fires.
             */
            counter += TIMER_ACCURACY;
            timer_isr_ctx.cb(timer_isr_ctx.arg, 0);

            if (sched_context_switch_request) {
                thread_yield();
            }
        }
        if (counter == compares[1]) {
            timer_isr_ctx.cb(timer_isr_ctx.arg, 1);

            if (sched_context_switch_request) {
                thread_yield();
            }
        }
        if (counter == compares[2]) {
            timer_isr_ctx.cb(timer_isr_ctx.arg, 2);

            if (sched_context_switch_request) {
                thread_yield();
            }
        }

        mips_setcompare(mips_getcount() + TICKS_PER_US * TIMER_ACCURACY);

    }

#ifdef _PORTS_P32MZ2048EFM100_H
    /* process all device interrupts here */

    /* Bus TX - SPI1 DMA 1 Master */
    if (IEC4bits.DMA1IE && IFS4bits.DMA1IF) {
        DCH1INTCLR = 0xFF;
        IFS4CLR = _IFS4_DMA1IF_MASK;
    }

    /* Bus RX SPI1 DMA 0 */
    if (IEC4bits.DMA0IE && IFS4bits.DMA0IF) {
        dma_spi_1_isr_rx();
        DCH0INTCLR = 0xFF;
        IFS4CLR = _IFS4_DMA0IF_MASK;
    }

    /* Bus TX - SPI2 DMA 3 Master */
    if (IEC4bits.DMA3IE && IFS4bits.DMA3IF) {
        DCH3INTCLR = 0xFF;
        IFS4CLR = _IFS4_DMA3IF_MASK;
    }

    /* Bus RX SPI2 DMA 2 */
    if (IEC4bits.DMA2IE && IFS4bits.DMA2IF) {
        dma_spi_2_isr_rx();
        DCH2INTCLR = 0xFF;
        IFS4CLR = _IFS4_DMA2IF_MASK;
    }

    if (IEC3bits.SPI1RXIE && IFS3bits.SPI1RXIF) {
        spi_1_isr_rx();
        IFS3CLR = _IFS3_SPI1RXIF_MASK;
    }

    if (IEC4bits.SPI2RXIE && IFS4bits.SPI2RXIF) {
        spi_2_isr_rx();
        IFS4CLR = _IFS4_SPI2RXIF_MASK;
    }

    if (IEC4bits.SPI3RXIE && IFS4bits.SPI3RXIF) {
        spi_3_isr_rx();
        IFS4CLR = _IFS4_SPI3RXIF_MASK;
    }

    /* process uart receive interrupts here */
    if (IEC3bits.U1RXIE && IFS3bits.U1RXIF) {
        IFS3CLR = _IFS3_U1RXIF_MASK;
        uart_1_isr_rx();
    }

    if (IEC4bits.U2RXIE && IFS4bits.U2RXIF) {
        IFS4CLR = _IFS4_U2RXIF_MASK;
        uart_2_isr_rx();
    }

    if (IEC4bits.U3RXIE && IFS4bits.U3RXIF) {
        IFS4CLR = _IFS4_U3RXIF_MASK;
        uart_3_isr_rx();
    }

    if (IEC5bits.U4RXIE && IFS5bits.U4RXIF) {
        IFS5CLR = _IFS5_U4RXIF_MASK;
        uart_4_isr_rx();
    }

    /* Timer0 real interrupt for BLE state machine timers */
    if (IEC0bits.T1IE && IFS0bits.T1IF) {
        IFS0CLR = _IFS0_T1IF_MASK;
        rn4020_t1interrupt();
    }

    /* ads1220 conversion done interrupt */
    if (IEC0bits.INT2IE && IFS0bits.INT2IF) {
        IFS0CLR = _IFS0_INT2IF_MASK;
        rn4020_int_2_isr();
    }
#endif
}
