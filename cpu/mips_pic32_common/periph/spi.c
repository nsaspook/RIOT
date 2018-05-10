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

#define SPIxPRI_SW0	1
#define SPIXSUBPRI_SW0	0

/* PERIPHERAL_CLOCK must be defined in board file */

typedef struct PIC32_SPI_tag {
	volatile uint32_t *regs;
	uint8_t *in;
	size_t len;
	volatile int32_t complete;
} PIC32_SPI_T;

static PIC32_SPI_T pic_spi[SPI_NUMOF + 1];
static mutex_t locks[SPI_NUMOF + 1];

int32_t spi_complete(spi_t);

void Init_Bus_Dma_Tx1(void)
{
	uint32_t physDestDma;
	/* DMA channel 1 - SPI1 TX. */

	physDestDma = KVA_TO_PA(&SPI1BUF);

	IEC4bits.DMA1IE = 0; /* Disable the DMA interrupt. */
	IFS4bits.DMA1IF = 0; /* Clear the DMA interrupt flag. */
	DMACONbits.ON = 1; /* Enable the DMA module. */
	DCH1SSAbits.CHSSA = physDestDma; /* Source start address. */
	DCH1DSAbits.CHDSA = physDestDma; /* Destination start address. */
	DCH1SSIZbits.CHSSIZ = 1; /* Source bytes. */
	DCH1DSIZbits.CHDSIZ = 1; /* Destination bytes. */
	DCH1CSIZbits.CHCSIZ = 1; /* Bytes to transfer per event. */
	DCH1ECONbits.CHSIRQ = EIC_IRQ_SPI_1_TX; /* from board.h defines */
	DCH1ECONbits.SIRQEN = 1; /* Start cell transfer if an interrupt matching CHSIRQ occurs */
	DCH1INTbits.CHBCIE = 0; /* enable Channel block transfer complete interrupt. */
	IPC33bits.DMA1IP = 1; /* DMA interrupt priority. */
	IPC33bits.DMA1IS = 0; /* DMA subpriority. */
	IEC4bits.DMA1IE = 0; /* DMA interrupt enable.  */
}

void Init_Bus_Dma_Tx2(void)
{
	uint32_t physDestDma;
	/* DMA channel 3 - SPI2 TX. */

	physDestDma = KVA_TO_PA(&SPI2BUF);

	IEC4bits.DMA3IE = 0;
	IFS4bits.DMA3IF = 0;
	DMACONbits.ON = 1;
	DCH3SSAbits.CHSSA = physDestDma;
	DCH3DSAbits.CHDSA = physDestDma;
	DCH3SSIZbits.CHSSIZ = 1;
	DCH3DSIZbits.CHDSIZ = 1;
	DCH3CSIZbits.CHCSIZ = 1;
	DCH3ECONbits.CHSIRQ = EIC_IRQ_SPI_2_TX;
	DCH3ECONbits.SIRQEN = 1;
	DCH3INTbits.CHBCIE = 0;
	IPC34bits.DMA3IP = 1;
	IPC34bits.DMA3IS = 0;
	IEC4bits.DMA3IE = 0;
}

void Trigger_Bus_DMA_Tx1(size_t len, uint32_t physSourceDma)
{
	DCH1SSAbits.CHSSA = physSourceDma;
	DCH1SSIZbits.CHSSIZ = len;
	DCH1CSIZbits.CHCSIZ = 1;
	DCH1CONbits.CHEN = 1; /* Channel enable. */
}

void Trigger_Bus_DMA_Tx2(size_t len, uint32_t physSourceDma)
{
	DCH3SSAbits.CHSSA = physSourceDma;
	DCH3SSIZbits.CHSSIZ = len;
	DCH3CSIZbits.CHCSIZ = 1;
	DCH3CONbits.CHEN = 1;
}

/* 1,2,3 are the active spi devices on the cpicmzef board configuration */
void spi_irq_enable(spi_t bus)
{
	if (bus == 1) {
		IEC3CLR = _IEC3_SPI1RXIE_MASK; /* disable SPI1RX interrupt */
		SPI1CONbits.SRXISEL = 1; /* interrupt when not full */
		SPI1CONbits.STXISEL = 0; /*  last transfer is shifted out */
		IFS3CLR = _IFS3_SPI1RXIF_MASK; /* clear SPI1RX flag */
		IPC27bits.SPI1RXIP = SPIxPRI_SW0; /* Set IRQ 0 to priority 1.x */
		IPC27bits.SPI1RXIS = SPIXSUBPRI_SW0;
		IEC3SET = _IEC3_SPI1RXIE_MASK; /* enable SPI1RX interrupt */
		Init_Bus_Dma_Tx1();
	}
	if (bus == 2) {
		IEC4CLR = _IEC4_SPI2RXIE_MASK;
		SPI2CONbits.SRXISEL = 1;
		SPI2CONbits.STXISEL = 0;
		IFS4CLR = _IFS4_SPI2RXIF_MASK;
		IPC35bits.SPI2RXIP = SPIxPRI_SW0;
		IPC35bits.SPI2RXIS = SPIXSUBPRI_SW0;
		IEC4SET = _IEC4_SPI2RXIE_MASK;
		Init_Bus_Dma_Tx2();
	}
	if (bus == 4) {
		IEC5CLR = _IEC5_SPI4RXIE_MASK;
		SPI4CONbits.SRXISEL = 1;
		IFS5CLR = _IFS5_SPI4RXIF_MASK;
		IPC41bits.SPI4RXIP = SPIxPRI_SW0;
		IPC41bits.SPI4RXIS = SPIXSUBPRI_SW0;
		IEC5SET = _IEC5_SPI4RXIE_MASK;
	}
}

void spi_irq_disable(spi_t bus)
{
	if (bus == 1) {
		IEC3CLR = _IEC3_SPI1RXIE_MASK;
	}
	if (bus == 2) {
		IEC4CLR = _IEC4_SPI2RXIE_MASK;
	}
	if (bus == 4) {
		IEC5CLR = _IEC5_SPI4RXIE_MASK;
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
	const uint8_t *out_buffer = (const uint8_t*) out;
	uint8_t *in_buffer = (uint8_t*) in;
	uint32_t physSourceDma;

	assert(bus != 0 && bus <= SPI_NUMOF);

#ifdef _PORTS_P32MZ2048EFM100_H
	PDEBUG3_ON;
#endif
	(void) cs;
	(void) cont;
	/* Translate a kernel (KSEG) virtual address to a physical address. */
	physSourceDma = KVA_TO_PA(out_buffer);

	/* set input buffer params */
	pic_spi[bus].in = in_buffer;
	pic_spi[bus].len = len;
	pic_spi[bus].complete = false;

	switch (bus) {
	case 1:
		Trigger_Bus_DMA_Tx1(len, physSourceDma);
		break;
	case 2:
		Trigger_Bus_DMA_Tx2(len, physSourceDma);
		break;
	default: /* non-dma mode for testing */
		while (len--) {
			if (out_buffer) {
				SPIxBUF(pic_spi[bus]) = *out_buffer++;
				/* Wait until TX FIFO is empty */
				while ((SPIxSTAT(pic_spi[bus]) & _SPI1STAT_SPITBF_MASK)) {
				}
			}
		}
	}

#ifdef _PORTS_P32MZ2048EFM100_H
	PDEBUG3_OFF;
#endif
}

void spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont,
	const void *out, void *in, size_t len)
{
	assert(bus != 0 && bus <= SPI_NUMOF);

	if (cs != SPI_CS_UNDEF)
		gpio_clear((gpio_t) cs);

	_spi_transfer_bytes_async(bus, cs, cont, out, in, len);

	while (!spi_complete(bus)) {
	};

	if (!cont && cs != SPI_CS_UNDEF)
		gpio_set((gpio_t) cs);
}

void spi_transfer_bytes_async(spi_t bus, spi_cs_t cs, bool cont,
	const void *out, void *in, size_t len)
{
	assert(bus != 0 && bus <= SPI_NUMOF);

	if (cs != SPI_CS_UNDEF)
		gpio_clear((gpio_t) cs);

	_spi_transfer_bytes_async(bus, cs, cont, out, in, len);
	/* don't mess with cs on exit */
}

/* spi interrupt in single vector sw0 */
static void spi_rx_irq(spi_t bus)
{
	uint8_t rdata __attribute__((unused));

#ifdef _PORTS_P32MZ2048EFM100_H
	PDEBUG1_ON; // FIFO has data
#endif
	while (!((SPIxSTAT(pic_spi[bus]) & _SPI1STAT_SPIRBE_MASK))) {
#ifdef _PORTS_P32MZ2048EFM100_H
		PDEBUG1_TOGGLE; // FIFO has data
#endif
		if (pic_spi[bus].in) {
			*pic_spi[bus].in++ = SPIxBUF(pic_spi[bus]);
		} else {
			/* dump the received data with no callback */
			rdata = SPIxBUF(pic_spi[bus]);
		}
		if (!--pic_spi[bus].len)
			pic_spi[bus].complete = true;
#ifdef _PORTS_P32MZ2048EFM100_H
		PDEBUG1_TOGGLE; // FIFO has data
#endif
	}
	/* time ref toggle */
#ifdef _PORTS_P32MZ2048EFM100_H
	PDEBUG1_OFF; // FIFO has data
	PDEBUG1_ON; // FIFO has data
	PDEBUG1_OFF; // FIFO has data
#endif	
}

void SPI_1_ISR_RX(void)
{
	spi_rx_irq(1);
}

void SPI_2_ISR_RX(void)
{
	spi_rx_irq(2);
}

void SPI_4_ISR_RX(void)
{
	spi_rx_irq(4);
}

int32_t spi_complete(spi_t bus)
{
	return pic_spi[bus].complete;
}