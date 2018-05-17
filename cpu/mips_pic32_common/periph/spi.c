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
 * using non-atomic set and clears for setup functions, might need fixing
 *
 * DMA links
 * https://tutorial.cytron.io/2017/09/01/i2s-pic32mxmz-direct-memory-access-dma/
 * http://www.microchip.com/forums/m1022425.aspx
 *
 * L1-cache memory allocation
 * https://github.com/jasonkajita/pic32-part-support/blob/master/include/sys/l1cache.h
 */

#include "assert.h"
#include "board.h"
#include "mutex.h"
#include "periph/gpio.h"
#include "periph/spi.h"

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

#define SPIxPRI_SW0 1
#define SPIXSUBPRI_SW0  0

/* PERIPHERAL_CLOCK must be defined in board file */

typedef struct PIC32_SPI_tag {
	volatile uint32_t *regs;
	uint8_t *in;
	size_t len;
	volatile int32_t complete;
} PIC32_SPI_T;

typedef struct PIC32_DMA_tag {
	volatile uint32_t *regs;
} PIC32_DMA_T;

static PIC32_SPI_T pic_spi[SPI_NUMOF + 1];
static mutex_t locks[SPI_NUMOF + 1];
static PIC32_DMA_T pic_dma[DMA_NUMOF];

int32_t spi_complete(spi_t);
static void Init_Dma_Chan(uint8_t, uint32_t, volatile unsigned int *, volatile unsigned int *);

static void Init_Dma_Chan(uint8_t chan, uint32_t irq_num, volatile unsigned int * SourceDma, volatile unsigned int * DestDma)
{
	assert(chan < DMA_NUMOF);

	pic_dma[chan].regs = (volatile uint32_t *)(&DCH0CON + (chan * DMA_REGS_SPACING));

	IEC4CLR = _IEC4_DMA0IE_MASK << chan; /* Disable the DMA interrupt. */
	IFS4CLR = _IFS4_DMA0IF_MASK << chan; /* Clear the DMA interrupt flag. */
	DCRCCON = 0;
	DMACONSET = _DMACON_ON_MASK; /* Enable the DMA module. */
	DCHxCON(pic_dma[chan]) = 0;
	DCHxECON(pic_dma[chan]) = 0;
	DCHxINT(pic_dma[chan]) = 0;
	DCHxSSA(pic_dma[chan]) = KVA_TO_PA(SourceDma); /* Source start address. */
	DCHxDSA(pic_dma[chan]) = KVA_TO_PA(DestDma); /* Destination start address. */
	DCHxSSIZ(pic_dma[chan]) = 1; /* Source bytes. */
	DCHxDSIZ(pic_dma[chan]) = 1; /* Destination bytes. */
	DCHxCSIZ(pic_dma[chan]) = 1; /* Bytes to transfer per event. */
	DCHxECON(pic_dma[chan]) = irq_num << _DCH0ECON_CHSIRQ_POSITION; /* cell trigger interrupt */
	DCHxECONSET(pic_dma[chan]) = _DCH0ECON_SIRQEN_MASK; /* Start cell transfer if an interrupt matching CHSIRQ occurs */
	DCHxINTSET(pic_dma[chan]) = _DCH0INT_CHBCIE_MASK; /* enable Channel block transfer complete interrupt. */

	/*
	 * set vector priority and receiver DMA trigger enables for the board hardware configuration 
	 */
	switch (chan) {
	case 0:
		IPC33bits.DMA0IP = SPIxPRI_SW0; /* DMA interrupt priority. */
		IPC33bits.DMA0IS = SPIXSUBPRI_SW0; /* DMA sub-priority. */
		IEC4SET = _IEC4_DMA0IE_MASK; /* DMA interrupt enable for rx dma  */
		break;
	case 1:
		IPC33bits.DMA1IP = SPIxPRI_SW0;
		IPC33bits.DMA1IS = SPIXSUBPRI_SW0;
		break;
	case 2:
		IPC34bits.DMA2IP = SPIxPRI_SW0;
		IPC34bits.DMA2IS = SPIXSUBPRI_SW0;
		IEC4SET = _IEC4_DMA2IE_MASK;
		break;
	case 3:
		IPC34bits.DMA3IP = SPIxPRI_SW0;
		IPC34bits.DMA3IS = SPIXSUBPRI_SW0;
		break;
	default:
		break;
	}
}

static void Trigger_Bus_DMA_Tx1(size_t len, uint32_t physSourceDma)
{
	DCH1SSA = physSourceDma;
	DCH1SSIZbits.CHSSIZ = len;
	DCH1CONbits.CHEN = 1; /* Channel enable. */
}

static void Trigger_Bus_DMA_Rx1(size_t len, uint32_t physDestDma)
{
	IEC3CLR = _IEC3_SPI1RXIE_MASK; /* disable SPI1RX interrupt */
	SPI1CONbits.SRXISEL = 1; /* not empty */
	IFS3CLR = _IFS3_SPI1RXIF_MASK; /* clear SPI1RX flag */
	DCH0DSA = physDestDma;
	DCH0DSIZbits.CHDSIZ = len;
	DCH0CONbits.CHEN = 1; /* Channel enable. */
}

static void Trigger_Bus_DMA_Tx2(size_t len, uint32_t physSourceDma)
{
	DCH3SSA = physSourceDma;
	DCH3SSIZbits.CHSSIZ = len;
	DCH3CONbits.CHEN = 1;
}

static void Trigger_Bus_DMA_Rx2(size_t len, uint32_t physDestDma)
{
	IEC4CLR = _IEC4_SPI2RXIE_MASK; /* disable SPI2RX interrupt */
	SPI2CONbits.SRXISEL = 1; /* not empty */
	IFS4CLR = _IFS4_SPI2RXIF_MASK; /* clear SPI2RX flag */
	DCH2DSA = physDestDma;
	DCH2DSIZbits.CHDSIZ = len;
	DCH2CONbits.CHEN = 1; /* Channel enable. */
}

/* adjust speed on the fly, these extra functions are prototyped in board.h */
void spi_speed_config(spi_t bus, spi_clk_t clk)
{
	assert(bus != 0 && bus <= SPI_NUMOF);

	pic_spi[bus].regs = (volatile uint32_t *)(_SPI1_BASE_ADDRESS + (bus - 1) * SPI_REGS_SPACING);
	SPIxBRG(pic_spi[bus]) = (PERIPHERAL_CLOCK / (2 * clk)) - 1;
}

/* 1,2,3 are the active spi devices on the cpicmzef board configuration */
static void spi_irq_enable(spi_t bus)
{
	assert(bus != 0 && bus <= SPI_NUMOF);

	if (bus == 1) {
		IEC3CLR = _IEC3_SPI1RXIE_MASK; /* disable SPI1RX interrupt */
		SPI1CONbits.SRXISEL = 1; /* interrupt when not full */
		SPI1CONbits.STXISEL = 0; /*  last transfer is shifted out */
		IFS3CLR = _IFS3_SPI1RXIF_MASK; /* clear SPI1RX flag */
		IPC27bits.SPI1RXIP = SPIxPRI_SW0; /* Set IRQ 0 to priority 1.x */
		IPC27bits.SPI1RXIS = SPIXSUBPRI_SW0;
		IEC3SET = _IEC3_SPI1RXIE_MASK; /* enable SPI1RX interrupt */
		Init_Dma_Chan(1, _SPI1_TX_VECTOR, &SPI1BUF, &SPI1BUF);
		Init_Dma_Chan(0, _SPI1_RX_VECTOR, &SPI1BUF, &SPI1BUF);
	}
	if (bus == 2) {
		IEC4CLR = _IEC4_SPI2RXIE_MASK;
		SPI2CONbits.SRXISEL = 1;
		SPI2CONbits.STXISEL = 0;
		IFS4CLR = _IFS4_SPI2RXIF_MASK;
		IPC35bits.SPI2RXIP = SPIxPRI_SW0;
		IPC35bits.SPI2RXIS = SPIXSUBPRI_SW0;
		IEC4SET = _IEC4_SPI2RXIE_MASK;
		Init_Dma_Chan(3, _SPI2_TX_VECTOR, &SPI2BUF, &SPI2BUF);
		Init_Dma_Chan(2, _SPI2_RX_VECTOR, &SPI2BUF, &SPI2BUF);
	}
	if (bus == 3) {
		IEC4CLR = _IEC4_SPI3RXIE_MASK;
		SPI3CONbits.SRXISEL = 1;
		IFS4CLR = _IFS4_SPI3RXIF_MASK;
		IPC38bits.SPI3RXIP = SPIxPRI_SW0;
		IPC38bits.SPI3RXIS = SPIXSUBPRI_SW0;
		IEC4SET = _IEC4_SPI3RXIE_MASK;
	}
}

static void spi_irq_disable(spi_t bus)
{
	assert(bus != 0 && bus <= SPI_NUMOF);

	if (bus == 1) {
		IEC3CLR = _IEC3_SPI1RXIE_MASK;
	}
	if (bus == 2) {
		IEC4CLR = _IEC4_SPI2RXIE_MASK;
	}
	if (bus == 3) {
		IEC4CLR = _IEC4_SPI3RXIE_MASK;
	}
}

void spi_init(spi_t bus)
{
	assert(bus != 0 && bus <= SPI_NUMOF);

	mutex_init(&locks[bus]);

	PMD5SET = _PMD5_SPI1MD_MASK << (bus - 1);
	spi_init_pins(bus);
}

void spi_init_pins(spi_t bus)
{
	assert(bus != 0 && bus <= SPI_NUMOF);

	gpio_init(spi_config[bus].mosi_pin, GPIO_OUT);
	gpio_init(spi_config[bus].miso_pin, GPIO_IN);
	*(spi_config[bus].mosi_reg) = spi_config[bus].mosi_af;
	*(spi_config[bus].miso_reg) = spi_config[bus].miso_af;
}

int spi_acquire(spi_t bus, spi_cs_t cs, spi_mode_t mode, spi_clk_t clk)
{
	volatile int rdata __attribute__((unused));

	(void) cs;
	assert(bus != 0 && bus <= SPI_NUMOF);

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
	assert(bus != 0 && bus <= SPI_NUMOF);

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
	if (out && in)
		dma_able = 0;

	/* Translate a kernel (KSEG) virtual address to a physical address. */
	switch (bus + dma_able) {
	case 1:
		Trigger_Bus_DMA_Rx1(len, KVA_TO_PA(in));
		Trigger_Bus_DMA_Tx1(len, KVA_TO_PA(out_buffer));
		break;
	case 2:
		Trigger_Bus_DMA_Rx2(len, KVA_TO_PA(in));
		Trigger_Bus_DMA_Tx2(len, KVA_TO_PA(out_buffer));
		break;
	default: /* non-dma mode */
		while (len--) {
			if (out_buffer) {
				SPIxBUF(pic_spi[bus]) = *out_buffer++;
				/* Wait until TX FIFO is empty */
				while ((SPIxSTAT(pic_spi[bus]) & _SPI1STAT_SPITBF_MASK)) {
				}
			} else {
				SPIxBUF(pic_spi[bus]) = 0;
				/* Wait until TX FIFO is empty */
				while ((SPIxSTAT(pic_spi[bus]) & _SPI1STAT_SPITBF_MASK)) {
				}
			}
		}
	}
}

void spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont,
	const void *out, void *in, size_t len)
{
	assert(bus != 0 && bus <= SPI_NUMOF);
	/* make sure at least one input or one output buffer is given */
	assert(out || in);

	if (cs != SPI_CS_UNDEF) {
		gpio_clear((gpio_t) cs);
	}

	_spi_transfer_bytes_async(bus, cs, cont, out, in, len);

	while (!spi_complete(bus)) {
	}

	if (!cont && cs != SPI_CS_UNDEF) {
		gpio_set((gpio_t) cs);
	}
}

void spi_transfer_bytes_async(spi_t bus, spi_cs_t cs, bool cont,
	const void *out, void *in, size_t len)
{
	assert(bus != 0 && bus <= SPI_NUMOF);
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
		} else {
			/* dump the received data with no callback */
			rdata = SPIxBUF(pic_spi[bus]);
		}
		if (!--pic_spi[bus].len) {
			pic_spi[bus].complete = true;
		}
	}
}

void SPI_1_ISR_RX(void)
{
	spi_rx_irq(1);
}

void SPI_2_ISR_RX(void)
{
	spi_rx_irq(2);
}

void SPI_3_ISR_RX(void)
{
	spi_rx_irq(3);
}

/* set transfer complete flag */
void DMA_SPI_1_ISR_RX(void)
{
	pic_spi[1].complete = true;
}

void DMA_SPI_2_ISR_RX(void)
{
	pic_spi[2].complete = true;
}

void DMA_SPI_3_ISR_RX(void)
{
	pic_spi[3].complete = true;
}

int32_t spi_complete(spi_t bus)
{
	assert(bus != 0 && bus <= SPI_NUMOF);
	return pic_spi[bus].complete;
}