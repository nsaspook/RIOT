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
#define DMA_REGS_SPACING    (0xBF811120 - 0xBF811060)

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
void Init_Dma_Chan(uint8_t, uint32_t);

void Init_Dma_Chan(uint8_t chan, uint32_t irq_num)
{
	uint32_t physDestDma = 0, physSourceDma = 0;
	assert(chan < DMA_NUMOF);

	pic_dma[chan].regs = (volatile uint32_t *)(0xBF811060 + (chan * DMA_REGS_SPACING));

	/* DMA channel configuration to device */
	switch (chan) {
	case 0:
		IEC4bits.DMA0IE = 0; /* Disable the DMA interrupt. */
		IFS4bits.DMA0IF = 0; /* Clear the DMA interrupt flag. */
		physSourceDma = KVA_TO_PA(&SPI1BUF);
		break;
	case 1:
		IEC4bits.DMA1IE = 0;
		IFS4bits.DMA1IF = 0;
		physDestDma = KVA_TO_PA(&SPI1BUF);
		break;
	case 2:
		IEC4bits.DMA2IE = 0;
		IFS4bits.DMA2IF = 0;
		physSourceDma = KVA_TO_PA(&SPI2BUF);
		break;
	case 3:
		IEC4bits.DMA3IE = 0;
		IFS4bits.DMA3IF = 0;
		physDestDma = KVA_TO_PA(&SPI2BUF);
		break;
	default:
		break;
	}

	DMACONSET = _DMACON_ON_MASK; /* Enable the DMA module. */
	DCHxSSA(pic_dma[chan]) = physSourceDma; /* Source start address. */
	DCHxDSA(pic_dma[chan]) = physDestDma; /* Destination start address. */
	DCHxSSIZ(pic_dma[chan]) = 1; /* Source bytes. */
	DCHxDSIZ(pic_dma[chan]) = 1; /* Destination bytes. */
	DCHxCSIZ(pic_dma[chan]) = 1; /* Bytes to transfer per event. */
	DCHxECONSET(pic_dma[chan]) = _DCH0ECON_CHSIRQ_MASK & (irq_num << _DCH0ECON_CHSIRQ_POSITION); /* cell trigger interrupt */
	DCHxECONSET(pic_dma[chan]) = _DCH0ECON_SIRQEN_MASK; /* Start cell transfer if an interrupt matching CHSIRQ occurs */
	DCHxINTSET(pic_dma[chan]) = _DCH0INT_CHBCIE_MASK; /* enable Channel block transfer complete interrupt. */

	switch (chan) {
	case 0:
		IPC33bits.DMA0IP = 1; /* DMA interrupt priority. */
		IPC33bits.DMA0IS = 0; /* DMA subpriority. */
		IEC4bits.DMA0IE = 1; /* DMA interrupt enable.  */
		break;
	case 1:
		IPC33bits.DMA1IP = 1;
		IPC33bits.DMA1IS = 0;
		IEC4bits.DMA1IE = 0;
		break;
	case 2:
		IPC34bits.DMA2IP = 1;
		IPC34bits.DMA2IS = 0;
		IEC4bits.DMA2IE = 1;
		break;
	case 3:
		IPC34bits.DMA3IP = 1;
		IPC34bits.DMA3IS = 0;
		IEC4bits.DMA3IE = 0;
		break;
	default:
		break;
	}
}

void Init_Bus_Dma_Tx1(void)
{
	uint32_t physDestDma;

	/* DMA channel 1 - SPI1 TX. */

	physDestDma = KVA_TO_PA(&SPI1BUF);

	IEC4bits.DMA1IE = 0; /* Disable the DMA interrupt. */
	IFS4bits.DMA1IF = 0; /* Clear the DMA interrupt flag. */
	DMACONSET = _DMACON_ON_MASK; /* Enable the DMA module. */
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

void Init_Bus_Dma_Rx1(void)
{
	uint32_t physSourceDma;

	/* DMA channel 0 - SPI1 RX. */

	physSourceDma = KVA_TO_PA(&SPI1BUF);

	IEC4bits.DMA0IE = 0; /* Disable the DMA interrupt. */
	IFS4bits.DMA0IF = 0; /* Clear the DMA interrupt flag. */
	DMACONbits.ON = 1; /* Enable the DMA module. */
	DCH0SSAbits.CHSSA = physSourceDma; /* Source start address. */
	DCH0DSAbits.CHDSA = physSourceDma; /* Destination start address. */
	DCH0SSIZbits.CHSSIZ = 1; /* Source bytes. */
	DCH0DSIZbits.CHDSIZ = 1; /* Destination bytes. */
	DCH0CSIZbits.CHCSIZ = 1; /* Bytes to transfer per event. */
	DCH0ECONbits.CHSIRQ = EIC_IRQ_SPI_1_RX; /* from board.h defines */
	DCH0ECONbits.SIRQEN = 1; /* Start cell transfer if an interrupt matching CHSIRQ occurs */
	DCH0INTbits.CHBCIE = 1; /* enable Channel block transfer complete interrupt. */
	IPC33bits.DMA0IP = 1; /* DMA interrupt priority. */
	IPC33bits.DMA0IS = 0; /* DMA subpriority. */
	IEC4bits.DMA0IE = 1; /* DMA interrupt enable.  */
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

void Init_Bus_Dma_Rx2(void)
{
	uint32_t physSourceDma;

	/* DMA channel 2 - SPI2 RX. */

	physSourceDma = KVA_TO_PA(&SPI2BUF);

	IEC4bits.DMA2IE = 0; /* Disable the DMA interrupt. */
	IFS4bits.DMA2IF = 0; /* Clear the DMA interrupt flag. */
	DMACONbits.ON = 1; /* Enable the DMA module. */
	DCH2SSAbits.CHSSA = physSourceDma; /* Source start address. */
	DCH2DSAbits.CHDSA = physSourceDma; /* Destination start address. */
	DCH2SSIZbits.CHSSIZ = 1; /* Source bytes. */
	DCH2DSIZbits.CHDSIZ = 1; /* Destination bytes. */
	DCH2CSIZbits.CHCSIZ = 1; /* Bytes to transfer per event. */
	DCH2ECONbits.CHSIRQ = EIC_IRQ_SPI_2_RX; /* from board.h defines */
	DCH2ECONbits.SIRQEN = 1; /* Start cell transfer if an interrupt matching CHSIRQ occurs */
	DCH2INTbits.CHBCIE = 1; /* enable Channel block transfer complete interrupt. */
	IPC34bits.DMA2IP = 1; /* DMA interrupt priority. */
	IPC34bits.DMA2IS = 0; /* DMA subpriority. */
	IEC4bits.DMA2IE = 1; /* DMA interrupt enable.  */
}

static void Trigger_Bus_DMA_Tx1(size_t len, uint32_t physSourceDma)
{
	DCH1SSAbits.CHSSA = physSourceDma;
	DCH1SSIZbits.CHSSIZ = len;
	DCH1CONbits.CHEN = 1; /* Channel enable. */
}

static void Trigger_Bus_DMA_Rx1(size_t len, uint32_t physDestDma)
{
	IEC3CLR = _IEC3_SPI1RXIE_MASK; /* disable SPI1RX interrupt */
	SPI1CONbits.SRXISEL = 1; /* not empty */
	IFS3CLR = _IFS3_SPI1RXIF_MASK; /* clear SPI1RX flag */
	DCH0DSAbits.CHDSA = physDestDma;
	DCH0DSIZbits.CHDSIZ = len;
	DCH0CONbits.CHEN = 1; /* Channel enable. */
}

static void Trigger_Bus_DMA_Tx2(size_t len, uint32_t physSourceDma)
{
	DCH3SSAbits.CHSSA = physSourceDma;
	DCH3SSIZbits.CHSSIZ = len;
	DCH3CONbits.CHEN = 1;
}

static void Trigger_Bus_DMA_Rx2(size_t len, uint32_t physDestDma)
{
	IEC4CLR = _IEC4_SPI2RXIE_MASK; /* disable SPI2RX interrupt */
	SPI2CONbits.SRXISEL = 1; /* not empty */
	IFS4CLR = _IFS4_SPI2RXIF_MASK; /* clear SPI2RX flag */
	DCH2DSAbits.CHDSA = physDestDma;
	DCH2DSIZbits.CHDSIZ = len;
	DCH2CONbits.CHEN = 1; /* Channel enable. */
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
		//Init_Dma_Chan(1, EIC_IRQ_SPI_1_TX);
		Init_Bus_Dma_Tx1();
		//Init_Dma_Chan(0, EIC_IRQ_SPI_1_RX);
		Init_Bus_Dma_Rx1();
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
		Init_Bus_Dma_Rx2();
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

void spi_irq_disable(spi_t bus)
{
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
	uint8_t *in_buffer = (uint8_t *) in;
	uint32_t physSourceDma, physDestDma;

	assert(bus != 0 && bus <= SPI_NUMOF);

#ifdef _PORTS_P32MZ2048EFM100_H
	PDEBUG3_ON;
#endif
	(void) cs;
	(void) cont;
	/* Translate a kernel (KSEG) virtual address to a physical address. */
	physSourceDma = KVA_TO_PA(out_buffer);
	physDestDma = KVA_TO_PA(in_buffer);

	/* set input buffer params */
	pic_spi[bus].in = in_buffer;
	pic_spi[bus].len = len;
	pic_spi[bus].complete = false;

	switch (bus) {
	case 1:
		Trigger_Bus_DMA_Rx1(len, physDestDma);
		Trigger_Bus_DMA_Tx1(len, physSourceDma);
		break;
	case 2:
		Trigger_Bus_DMA_Rx2(len, physDestDma);
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
		if (!--pic_spi[bus].len) {
			pic_spi[bus].complete = true;
		}
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

void SPI_3_ISR_RX(void)
{
	spi_rx_irq(3);
}

/* set transfer complete flag */
void DMA_SPI_1_ISR_RX(void)
{
#ifdef _PORTS_P32MZ2048EFM100_H
	PDEBUG1_ON;
#endif
	pic_spi[1].complete = true;
	/* time ref toggle */
#ifdef _PORTS_P32MZ2048EFM100_H
	PDEBUG1_OFF; // FIFO has data
#endif
}

void DMA_SPI_2_ISR_RX(void)
{
#ifdef _PORTS_P32MZ2048EFM100_H
	PDEBUG1_ON;
#endif
	pic_spi[2].complete = true;
	/* time ref toggle */
#ifdef _PORTS_P32MZ2048EFM100_H
	PDEBUG1_OFF; // FIFO has data
#endif
}

void DMA_SPI_3_ISR_RX(void)
{
#ifdef _PORTS_P32MZ2048EFM100_H
	PDEBUG1_ON;
#endif
	pic_spi[3].complete = true;
	/* time ref toggle */
#ifdef _PORTS_P32MZ2048EFM100_H
	PDEBUG1_OFF; // FIFO has data
#endif
}

int32_t spi_complete(spi_t bus)
{
	return pic_spi[bus].complete;
}