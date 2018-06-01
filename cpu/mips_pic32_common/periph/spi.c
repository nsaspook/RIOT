/*
 * Copyright(C) 2017 Francois Berder <fberder@outlook.fr>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 * May 2018
 * modified for receive interrupt, async and fifo operation @nsaspook
 * The 'normal' sync transfer function is a wrapper on the async engine
 * _spi_transfer_bytes_async
 * ALL SPI transfer buffers in and out MUST be pic32mz uncached memory
 *
 * DMA links
 * https://tutorial.cytron.io/2017/09/01/i2s-pic32mxmz-direct-memory-access-dma/
 * http://www.microchip.com/forums/m1022425.aspx
 *
 * L1-cache memory allocation
 * https://github.com/jasonkajita/pic32-part-support/blob/master/include/sys/l1cache.h
 */

#include <stdio.h>
#include <string.h>
#include "assert.h"
#include "board.h"
#include "mutex.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "periph/uart.h"

#define SPIxCON(U)          ((U).regs[0x00 / 4])
#define SPIxCONCLR(U)       ((U).regs[0x04 / 4])
#define SPIxCONSET(U)       ((U).regs[0x08 / 4])
#define SPIxSTAT(U)         ((U).regs[0x10 / 4])
#define SPIxSTATCLR(U)      ((U).regs[0x14 / 4])
#define SPIxBUF(U)          ((U).regs[0x20 / 4])
#define SPIxBRG(U)          ((U).regs[0x30 / 4])
#define SPIxCON2(U)         ((U).regs[0x40 / 4])
#define SPI_REGS_SPACING    (_SPI2_BASE_ADDRESS - _SPI1_BASE_ADDRESS)

#define DCHxCON(U)          ((U).regs[0x00 / 4])
#define DCHxCONCLR(U)       ((U).regs[0x04 / 4])
#define DCHxCONSET(U)       ((U).regs[0x08 / 4])
#define DCHxECON(U)         ((U).regs[0x10 / 4])
#define DCHxECONCLR(U)      ((U).regs[0x14 / 4])
#define DCHxECONSET(U)      ((U).regs[0x18 / 4])
#define DCHxINT(U)          ((U).regs[0x20 / 4])
#define DCHxINTCLR(U)       ((U).regs[0x24 / 4])
#define DCHxINTSET(U)       ((U).regs[0x28 / 4])
#define DCHxSSA(U)          ((U).regs[0x30 / 4])
#define DCHxDSA(U)          ((U).regs[0x40 / 4])
#define DCHxSSIZ(U)         ((U).regs[0x50 / 4])
#define DCHxDSIZ(U)         ((U).regs[0x60 / 4])
#define DCHxSPTR(U)         ((U).regs[0x70 / 4])
#define DCHxDPTR(U)         ((U).regs[0x80 / 4])
#define DCHxCSIZ(U)         ((U).regs[0x90 / 4])
#define DCHxCPTR(U)         ((U).regs[0xA0 / 4])
#define DCHxDAT(U)          ((U).regs[0xB0 / 4])
#define DMA_REGS_SPACING    (&DCH1CON - &DCH0CON)

#define SPIxPRI_SW0 2
#define SPIxSUBPRI_SW0  1

/* PERIPHERAL_CLOCK must be defined in board file */

typedef struct pic32_spi_tag {
    volatile uint32_t *regs;
    uint8_t *in;
    size_t len;
    volatile int32_t complete;
} pic32_spi_t;

typedef struct pic_32_dma_tag {
    volatile uint32_t *regs;
    spi_t bus;
} pic32_dma_t;

static pic32_spi_t pic_spi[SPI_NUMOF + 1];
static mutex_t locks[SPI_NUMOF + 1];
static pic32_dma_t pic_dma[DMA_NUMOF];

int32_t spi_complete(spi_t);
static void init_dma_chan(uint8_t, uint32_t, volatile unsigned int *, volatile unsigned int *, spi_t);

static void release_dma_chan(uint8_t chan)
{
    assert(chan < DMA_NUMOF);

    IEC4CLR = _IEC4_DMA0IE_MASK << chan;    /* Disable the DMA interrupt. */
    IFS4CLR = _IFS4_DMA0IF_MASK << chan;    /* Clear the DMA interrupt flag. */
    DCHxCON(pic_dma[chan]) = 0;
    DCHxECON(pic_dma[chan]) = 0;
    DCHxINT(pic_dma[chan]) = 0;
}

static void init_dma_chan(uint8_t chan, uint32_t irq_num, volatile unsigned int *SourceDma, volatile unsigned int *DestDma, spi_t bus)
{
    assert(chan < DMA_NUMOF);
    assert(bus != 0 && bus <= SPI_NUMOF_USED);

    pic_dma[chan].regs = (volatile uint32_t *)(&DCH0CON + (chan * DMA_REGS_SPACING));
    pic_dma[chan].bus = bus;

    IEC4CLR = _IEC4_DMA0IE_MASK << chan;    /* Disable the DMA chan interrupt. */
    IFS4CLR = _IFS4_DMA0IF_MASK << chan;    /* Clear the DMA chan interrupt flag. */
    DMACONSET = _DMACON_ON_MASK;            /* Enable the DMA module. */
    DCHxCON(pic_dma[chan]) = 0;
    DCHxECON(pic_dma[chan]) = 0;
    DCHxINT(pic_dma[chan]) = 0;
    DCHxSSA(pic_dma[chan]) = KVA_TO_PA(SourceDma);                  /* Source start address. */
    DCHxDSA(pic_dma[chan]) = KVA_TO_PA(DestDma);                    /* Destination start address. */
    DCHxSSIZ(pic_dma[chan]) = 1;                                    /* default Source bytes. */
    DCHxDSIZ(pic_dma[chan]) = 1;                                    /* default Destination bytes. */
    DCHxCSIZ(pic_dma[chan]) = 1;                                    /* Bytes to transfer per event. */
    DCHxECON(pic_dma[chan]) = irq_num << _DCH0ECON_CHSIRQ_POSITION; /* cell trigger interrupt */
    DCHxECONSET(pic_dma[chan]) = _DCH0ECON_SIRQEN_MASK;             /* Start cell transfer if an interrupt matching CHSIRQ occurs */
    DCHxINTSET(pic_dma[chan]) = _DCH0INT_CHBCIE_MASK;               /* enable Channel block transfer complete interrupt. */
    /*
     * set vector priority and receiver DMA trigger enables for the board hardware configuration
     */
    *(dma_config[chan].ipc_regset) = dma_config[chan].ipc_mask_p & (SPIxPRI_SW0 << dma_config[chan].ipc_mask_pos_p);
    *(dma_config[chan].ipc_regset) = dma_config[chan].ipc_mask_s & (SPIxSUBPRI_SW0 << dma_config[chan].ipc_mask_pos_s);
    IEC4SET = dma_config[chan].iec_mask << chan; /* DMA interrupt enable if needed */
}

/* disable receive interrupts and set the UART buffer mode for DMA */
static void spi_reset_dma_irq(spi_t bus)
{
    assert(bus != 0 && bus <= SPI_NUMOF_USED);

    switch (bus) {
        case 1:
            IEC3CLR = _IEC3_SPI1RXIE_MASK;  /* disable SPIxRX interrupt */
            SPI1CONbits.SRXISEL = 1;        /* not empty */
            IFS3CLR = _IFS3_SPI1RXIF_MASK;  /* clear SPIxRX flag */
            break;
        case 2:
            IEC4CLR = _IEC4_SPI2RXIE_MASK;
            SPI2CONbits.SRXISEL = 1;
            IFS4CLR = _IFS4_SPI2RXIF_MASK;

            break;
        default:
            break;
    }
}

static void trigger_bus_dma_tx(uint8_t chan, size_t len, uint32_t physSourceDma)
{
    assert(chan < DMA_NUMOF);

    DCHxSSA(pic_dma[chan]) = physSourceDma;
    DCHxSSIZ(pic_dma[chan]) = (len & _DCH0SSIZ_CHSSIZ_MASK);
    DCHxCONSET(pic_dma[chan]) = _DCH0CON_CHEN_MASK; /* Channel enable. */
}

static void trigger_bus_dma_rx(uint8_t chan, size_t len, uint32_t physDestDma)
{
    assert(chan < DMA_NUMOF);

    spi_reset_dma_irq(pic_dma[chan].bus);
    DCHxDSA(pic_dma[chan]) = physDestDma;
    DCHxDSIZ(pic_dma[chan]) = (len & _DCH0DSIZ_CHDSIZ_MASK);
    DCHxCONSET(pic_dma[chan]) = _DCH0CON_CHEN_MASK; /* Channel enable. */
}

/* adjust speed on the fly, these extra functions are prototyped in board.h */
void spi_speed_config(spi_t bus, spi_mode_t mode, spi_clk_t clk)
{
    assert(bus != 0 && bus <= SPI_NUMOF_USED);

    pic_spi[bus].regs = (volatile uint32_t *)(_SPI1_BASE_ADDRESS + (bus - 1) * SPI_REGS_SPACING);

    SPIxCONCLR(pic_spi[bus]) = (_SPI1CON_ON_MASK);
    if (clk) {
        SPIxBRG(pic_spi[bus]) = (PERIPHERAL_CLOCK / (2 * clk)) - 1;
    }

    switch (mode) {
        case SPI_MODE_0:
            SPIxCONCLR(pic_spi[bus]) = _SPI1CON_CKP_MASK;
            SPIxCONSET(pic_spi[bus]) = _SPI1CON_CKE_MASK;
            break;
        case SPI_MODE_1:
            SPIxCONCLR(pic_spi[bus]) = (_SPI1CON_CKP_MASK | _SPI1CON_CKE_MASK);
            break;
        case SPI_MODE_2:
            SPIxCONCLR(pic_spi[bus]) = _SPI1CON_CKE_MASK;
            SPIxCONSET(pic_spi[bus]) = _SPI1CON_CKP_MASK;
            break;
        case SPI_MODE_3:
            SPIxCONSET(pic_spi[bus]) = (_SPI1CON_CKP_MASK | _SPI1CON_CKE_MASK);
            break;
        default:
            break;
    }
    SPIxCONSET(pic_spi[bus]) = (_SPI1CON_ON_MASK);
}

/* 1,2,3 are the active spi devices on the cpicmzef board configuration
 * DMA channels are allocated for 1&2 tx/rx
 */
static void spi_irq_enable(spi_t bus)
{
    uint32_t mask;

    assert(bus != 0 && bus <= SPI_NUMOF_USED);

    /* set enable and flag mask */
    mask = spi_config[bus].int_mask;
    *(spi_config[bus].iec_regclr) = mask; /* disable SPIxRX interrupt */

    switch (bus) {
        case 1:
            init_dma_chan(SPI1_DMA_TX, _SPI1_TX_VECTOR, &SPI1BUF, &SPI1BUF, bus);
            init_dma_chan(SPI1_DMA_RX, _SPI1_RX_VECTOR, &SPI1BUF, &SPI1BUF, bus);
            break;
        case 2:
            init_dma_chan(SPI2_DMA_TX, _SPI2_TX_VECTOR, &SPI2BUF, &SPI2BUF, bus);
            init_dma_chan(SPI2_DMA_RX, _SPI2_RX_VECTOR, &SPI2BUF, &SPI2BUF, bus);
            break;
        default:
            break;
    }

    SPIxCONCLR(pic_spi[bus]) = _SPI1CON_SRXISEL_MASK & (3 << _SPI1CON_SRXISEL_POSITION);    /* clear all */
    /* interrupt when not full */
    SPIxCONSET(pic_spi[bus]) = _SPI1CON_SRXISEL_MASK & (1 << _SPI1CON_SRXISEL_POSITION);    /* set mode */
    /*  last transfer is shifted out */
    SPIxCONCLR(pic_spi[bus]) = _SPI1CON_STXISEL_MASK & (3 << _SPI1CON_STXISEL_POSITION);    /* clear all */
    /*
     * set vector priority and receiver interrupt enables for the board hardware configuration
     */
    *(spi_config[bus].ifs_regclr) = mask;                                                   /* clear SPIxRX flag */
    *(spi_config[bus].ipc_regset) = spi_config[bus].ipc_mask_p & (SPIxPRI_SW0 << spi_config[bus].ipc_mask_pos_p);
    *(spi_config[bus].ipc_regset) = spi_config[bus].ipc_mask_s & (SPIxSUBPRI_SW0 << spi_config[bus].ipc_mask_pos_s);
    *(spi_config[bus].iec_regset) = mask; /* enable SPIxRX interrupt */
}

static void spi_irq_disable(spi_t bus)
{

    switch (bus) {
        case 1:
            release_dma_chan(SPI1_DMA_RX);
            release_dma_chan(SPI1_DMA_TX);
            break;
        case 2:
            release_dma_chan(SPI2_DMA_RX);
            release_dma_chan(SPI2_DMA_TX);
            break;
        default:
            break;
    }

    *(spi_config[bus].iec_regclr) = spi_config[bus].int_mask; /* disable SPIxRX interrupt */
}

void spi_init(spi_t bus)
{
    assert(bus != 0 && bus <= SPI_NUMOF_USED);

    mutex_init(&locks[bus]);

    PMD5SET = _PMD5_SPI1MD_MASK << (bus - 1);
    spi_init_pins(bus);
}

void spi_init_pins(spi_t bus)
{
    assert(bus != 0 && bus <= SPI_NUMOF_USED);

    gpio_init(spi_config[bus].mosi_pin, GPIO_OUT);
    gpio_init(spi_config[bus].miso_pin, GPIO_IN);
    *(spi_config[bus].mosi_reg) = spi_config[bus].mosi_af;
    *(spi_config[bus].miso_reg) = spi_config[bus].miso_af;
}

int spi_acquire(spi_t bus, spi_cs_t cs, spi_mode_t mode, spi_clk_t clk)
{
    volatile int rdata __attribute__((unused));

    (void) cs;
    assert(bus != 0 && bus <= SPI_NUMOF_USED);

    pic_spi[bus].regs = (volatile uint32_t *)(_SPI1_BASE_ADDRESS + (bus - 1) * SPI_REGS_SPACING);

    mutex_lock(&locks[bus]);

    PMD5CLR = _PMD5_SPI1MD_MASK << (bus - 1);

    SPIxCON(pic_spi[bus]) = 0;
    SPIxCON2(pic_spi[bus]) = 0;

    /* Clear receive FIFO */
    rdata = SPIxBUF(pic_spi[bus]);

    switch (mode) {
        case SPI_MODE_0:
            SPIxCONCLR(pic_spi[bus]) = (_SPI1CON_CKP_MASK | _SPI1CON_CKE_MASK);
            break;
        case SPI_MODE_1:
            SPIxCONCLR(pic_spi[bus]) = _SPI1CON_CKP_MASK;
            SPIxCONSET(pic_spi[bus]) = _SPI1CON_CKE_MASK;
            break;
        case SPI_MODE_2:
            SPIxCONCLR(pic_spi[bus]) = _SPI1CON_CKE_MASK;
            SPIxCONSET(pic_spi[bus]) = _SPI1CON_CKP_MASK;
            break;
        case SPI_MODE_3:
            SPIxCONSET(pic_spi[bus]) = (_SPI1CON_CKP_MASK | _SPI1CON_CKE_MASK);
            break;
        default:
            return SPI_NOMODE;
    }

    /* try to make the FIFO work in some modes */
    SPIxCONSET(pic_spi[bus]) = _SPI1CON_ENHBUF_MASK; /* enable FIFO */
    SPIxBRG(pic_spi[bus]) = (PERIPHERAL_CLOCK / (2 * clk)) - 1;
    SPIxSTATCLR(pic_spi[bus]) = _SPI1STAT_SPIROV_MASK;
    spi_irq_enable(bus);
    SPIxCONSET(pic_spi[bus]) = (_SPI1CON_ON_MASK | _SPI1CON_MSTEN_MASK);

    return SPI_OK;
}

void spi_release(spi_t bus)
{
    assert(bus != 0 && bus <= SPI_NUMOF_USED);

    spi_irq_disable(bus);
    /* SPI module must be turned off before powering it down */
    SPIxCON(pic_spi[bus]) = 0;
    PMD5SET = _PMD5_SPI1MD_MASK << (bus - 1);
    mutex_unlock(&locks[bus]);
}

static inline void _spi_transfer_bytes_async(spi_t bus, spi_cs_t cs, bool cont,
                                             const void *out, void *in, size_t len)
{
    const uint8_t *out_buffer = (const uint8_t *) out;
    uint8_t dma_able = 8; /* default to NO DMA to trigger default method */

    (void) cs;
    (void) cont;

    /* set input buffer params for the non-dma isr mode */
    pic_spi[bus].in = in;
    pic_spi[bus].len = len;
    pic_spi[bus].complete = false;

    /* check of we have both buffers */
    if (out && in) {
        dma_able = 0;
    }

    /* Translate a kernel (KSEG) virtual address to a physical address. */
    switch (bus + dma_able) {
        case 1:
            trigger_bus_dma_rx(SPI1_DMA_RX, len, KVA_TO_PA(in));
            trigger_bus_dma_tx(SPI1_DMA_TX, len, KVA_TO_PA(out_buffer));
            break;
        case 2:
            trigger_bus_dma_rx(SPI2_DMA_RX, len, KVA_TO_PA(in));
            trigger_bus_dma_tx(SPI2_DMA_TX, len, KVA_TO_PA(out_buffer));
            break;
        default: /* non-dma mode */
            while (len--) {
                if (out_buffer) {
                    SPIxBUF(pic_spi[bus]) = *out_buffer++;
                    /* Wait until TX FIFO is empty */
                    while ((SPIxSTAT(pic_spi[bus]) & _SPI1STAT_SPITBF_MASK)) {}
                }
                else {
                    SPIxBUF(pic_spi[bus]) = 0;
                    /* Wait until TX FIFO is empty */
                    while ((SPIxSTAT(pic_spi[bus]) & _SPI1STAT_SPITBF_MASK)) {}
                }
            }
    }
}

void spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont,
                        const void *out, void *in, size_t len)
{
    assert(bus != 0 && bus <= SPI_NUMOF_USED);
    /* make sure at least one input or one output buffer is given */
    assert(out || in);

    if (cs != SPI_CS_UNDEF) {
        gpio_clear((gpio_t) cs);
    }

    _spi_transfer_bytes_async(bus, cs, cont, out, in, len);

    while (!spi_complete(bus)) {}

    if (!cont && cs != SPI_CS_UNDEF) {

        gpio_set((gpio_t) cs);
    }
}

void spi_transfer_bytes_async(spi_t bus, spi_cs_t cs, bool cont,
                              const void *out, void *in, size_t len)
{
    assert(bus != 0 && bus <= SPI_NUMOF_USED);
    /* make sure at least one input or one output buffer is given */
    assert(out || in);

    if (cs != SPI_CS_UNDEF) {

        gpio_clear((gpio_t) cs);
    }

    _spi_transfer_bytes_async(bus, cs, cont, out, in, len);
    /* don't mess with cs on exit */
}

/* spi interrupt in single vector sw0 */
static void spi_rx_irq(spi_t bus)
{
    uint8_t rdata __attribute__((unused));

    while (!((SPIxSTAT(pic_spi[bus]) & _SPI1STAT_SPIRBE_MASK))) {
        if (pic_spi[bus].in) {
            *pic_spi[bus].in++ = SPIxBUF(pic_spi[bus]);
        }
        else {
            /* dump the received data with no callback */
            rdata = SPIxBUF(pic_spi[bus]);
        }
        if (!--pic_spi[bus].len) {

            pic_spi[bus].complete = true;
        }
    }
}

void spi_1_isr_rx(void)
{
    spi_rx_irq(1);
}

void spi_2_isr_rx(void)
{
    spi_rx_irq(2);
}

void spi_3_isr_rx(void)
{
    spi_rx_irq(3);
}

/* set transfer complete flag */
void dma_spi_1_isr_rx(void)
{
    pic_spi[1].complete = true;
}

void dma_spi_2_isr_rx(void)
{
    pic_spi[2].complete = true;
}

/* not currently used */
void dma_spi_3_isr_rx(void)
{
    pic_spi[3].complete = true;
}

int32_t spi_complete(spi_t bus)
{
    assert(bus != 0 && bus <= SPI_NUMOF);
    return pic_spi[bus].complete;
}
