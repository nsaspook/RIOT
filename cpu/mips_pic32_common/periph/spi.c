/*
 * Copyright(C) 2017 Francois Berder <fberder@outlook.fr>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
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

#define SPI_USE_TX_FIFO

/* PERIPHERAL_CLOCK must be defined in board file */


typedef struct PIC32_SPI_tag {
	volatile uint32_t *regs;
	uint8_t *in;
	size_t len;
	volatile int32_t complete;
} PIC32_SPI_T;

static PIC32_SPI_T pic_spi[SPI_NUMOF + 1];
static mutex_t locks[SPI_NUMOF + 1];

static spi_isr_ctx_t isr_ctx[SPI_NUMOF + 1];

/* 1,2,3 are the active spi devices on the cpicmzef board configuration */
void spi_irq_enable(spi_t bus)
{
	if (bus == 1) {
		IEC3CLR = _IEC3_SPI1RXIE_MASK; /* disable SPI1RX interrupt */
		SPI1CONbits.SRXISEL = 1; /* interrupt when not full */
		IFS3CLR = _IFS3_SPI1RXIF_MASK; /* clear SPI1RX flag */
		IEC3SET = _IEC3_SPI1RXIE_MASK;
		IPC27bits.SPI1RXIP = SPIxPRI_SW0; /* Set IRQ 0 to priority 1.x */
		IPC27bits.SPI1RXIS = SPIXSUBPRI_SW0;
	}
	if (bus == 2) {
		IEC4CLR = _IEC4_SPI2RXIE_MASK;
		SPI2CONbits.SRXISEL = 1;
		IFS4CLR = _IFS4_SPI2RXIF_MASK;
		IEC4SET = _IEC4_SPI2RXIE_MASK;
		IPC35bits.SPI2RXIP = SPIxPRI_SW0;
		IPC35bits.SPI2RXIS = SPIXSUBPRI_SW0;
	}
	if (bus == 4) {
		IEC5CLR = _IEC5_SPI4RXIE_MASK;
		SPI4CONbits.SRXISEL = 1;
		IFS5CLR = _IFS5_SPI4RXIF_MASK;
		IEC5SET = _IEC5_SPI4RXIE_MASK;
		IPC41bits.SPI4RXIP = SPIxPRI_SW0;
		IPC41bits.SPI4RXIS = SPIXSUBPRI_SW0;
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

static void _rx_cb_sync(void*, uint8_t);

void spi_init(spi_t bus)
{
	assert(bus != 0 && bus <= SPI_NUMOF);

	mutex_init(&locks[bus]);
	isr_ctx[bus].rx_cb = _rx_cb_sync; /* internal callback */
	isr_ctx[bus].arg = &pic_spi[bus]; /* pointer to the structure */

	PMD5SET = _PMD5_SPI1MD_MASK << (bus - 1);
	spi_init_pins(bus);
}

void spi_init_async(spi_t bus, spi_rx_cb_t rx_cb)
{
	assert(bus != 0 && bus <= SPI_NUMOF);

	mutex_init(&locks[bus]);
	isr_ctx[bus].rx_cb = rx_cb;
	isr_ctx[bus].arg = &pic_spi[bus];

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

	/* try to make the FIFO work in some modes of transmit only */
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

void spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont,
	const void *out, void *in, size_t len)
{
	const uint8_t *out_buffer = (const uint8_t*) out;
	uint8_t *in_buffer = (uint8_t*) in;

	assert(bus != 0 && bus <= SPI_NUMOF);

	if (cs != SPI_CS_UNDEF)
		gpio_clear((gpio_t) cs);

	/* set input buffer address */
	pic_spi[bus].in = in_buffer;
	pic_spi[bus].len = len;
	pic_spi[bus].complete = false;
	LED2_OFF;

	while (len--) {
		if (out_buffer) {
#ifdef _PORTS_P32MZ2048EFM100_H
			PDEBUG3_TOGGLE; // buffer has data
#endif
			SPIxBUF(pic_spi[bus]) = *out_buffer++;
#ifdef	SPI_USE_TX_FIFO
			/* Wait until TX FIFO is empty */
			while ((SPIxSTAT(pic_spi[bus]) & _SPI1STAT_SPITBF_MASK)) {
#else
			/* Wait until TX BUFFER is empty */
			while (!((SPIxSTAT(pic_spi[bus]) & _SPI1STAT_SPITBE_MASK))) {
			}
#endif
			}
		}
	}

	if (!cont && cs != SPI_CS_UNDEF)
		gpio_set((gpio_t) cs);
}

static inline void pic32mzef_rd_fifo(spi_t bus, void *in, int len)
{
	uint8_t byte;
	uint8_t *in_buffer = (uint8_t*) in;

	while (len--) {
		byte = SPIxBUF(pic_spi[bus]);
		if (in_buffer)
			*in_buffer++ = byte;
	}
}

static inline void pic32mzef_wr_fifo(spi_t bus, const void *out, int len)
{
	uint8_t byte;
	const uint8_t *out_buffer = (const uint8_t*) out;

	if (len > 16)
		len = 16;

	while (len--) {
		byte = out_buffer ? *out_buffer++ : 0;
		SPIxBUF(pic_spi[bus]) = byte;
	}
}

/* spi interrupt in single vector sw0 */
static void spi_rx_irq(spi_t bus)
{
	uint8_t rdata __attribute__((unused));

	while (!((SPIxSTAT(pic_spi[bus]) & _SPI1STAT_SPIRBE_MASK))) {
#ifdef _PORTS_P32MZ2048EFM100_H
		PDEBUG1_TOGGLE; // FIFO has data
#endif
		if (isr_ctx[bus].rx_cb) {
			/* run the callback with the pointer to the correct pic_spi[bus] */
			isr_ctx[bus].rx_cb(isr_ctx[bus].arg, SPIxBUF(pic_spi[bus]));
		} else {
			/* dump the received data with no callback */
			rdata = SPIxBUF(pic_spi[bus]);
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

void SPI_4_ISR_RX(void)
{
	spi_rx_irq(4);
}

/* spi interrupt received data callback processing */
static void _rx_cb_sync(void* data, uint8_t c)
{
	/* load the internal pointers with the pic_spi[bus] data */
	PIC32_SPI_T *pic_spi_ptr = data;
	uint8_t *recd = pic_spi_ptr->in;

	if (!pic_spi_ptr->len--) {
		*recd++ = c;
	} else {
		/* flag received data complete */
		pic_spi_ptr->complete = true;
		LED2_ON;
	}
}
