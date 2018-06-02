#include <stdint.h>
#include <stdbool.h>
#include "assert.h"
#include "board.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "periph/adc.h"

static uint8_t *td;
static uint8_t *rd;

static mcp3208_data_t riot_adc;

int adc_init_internal(adc_t);

/* for RIOT-OS 8 lines, 12-bit resolution only */
int adc_sample(adc_t line, adc_res_t res)
{
	assert(line <= 10);

	(void) res;
	switch (line) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
		riot_adc.mcp3208_cmd.ld = 0; // clear the command word
		riot_adc.mcp3208_cmd.map.start_bit = 1;
		riot_adc.mcp3208_cmd.map.single_diff = 1;
		riot_adc.mcp3208_cmd.map.index = line;
		td[0] = riot_adc.mcp3208_cmd.bd[2];
		td[1] = riot_adc.mcp3208_cmd.bd[1];
		td[2] = riot_adc.mcp3208_cmd.bd[0];
		gpio_clear(Ja10_3); // select the ADC
		spi_speed_config(SPI_DEV(2), 0, SPI_CLK_1MHZ); /* mode 0, speed */
		spi_transfer_bytes(SPI_DEV(2), 0, true, td, rd, 3);
		gpio_set(Ja10_3); // deselect the ADC

		/* lsb array index 2 */
		riot_adc.potValue = (rd[1] & 0x0f) << 8;
		riot_adc.potValue += rd[2];
		break;
	case 8:
		PDEBUG1_ON;
		ADCCON3bits.GSWTRG = 1;
		riot_adc.potValue = ADCDATA15;
		PDEBUG1_OFF;
		break;
	case 9:
		ADCCON3bits.GSWTRG = 1;
		riot_adc.potValue = ADCDATA26;
		break;
	case 10:
		ADCCON3bits.GSWTRG = 1;
		riot_adc.potValue = ADCDATA28;
		break;
	default:
		riot_adc.potValue = 0;
	}
	return riot_adc.potValue;
}

int adc_init(adc_t line)
{
	(void) line;
	static bool adc_is_init = false;

	if (!adc_is_init) {
		gpio_set(Ja10_3); // deselect the ADC
		td = __pic32_alloc_coherent(32); /* uncached memory for spi transfers */
		rd = __pic32_alloc_coherent(32);
		if (td && rd) {
			if (adc_init_internal(0)) {
				adc_is_init = true;
			}
		}
	}
	return adc_is_init;
}

int adc_init_internal(adc_t line)
{
	(void) line;

	/* set port pins to analog input configuration */
	ANSELEbits.ANSE7 = 1;
	ANSELBbits.ANSB3 = 1;
	ANSELBbits.ANSB15 = 1;
	ANSELEbits.ANSE9 = 1;
	/* copy calibration data in to config registers */
	ADC0CFG = DEVADC0;
	ADC1CFG = DEVADC1;
	ADC2CFG = DEVADC2;
	ADC3CFG = DEVADC3;
	ADC4CFG = DEVADC4;
	ADC7CFG = DEVADC7;
	/* Configure ADCCON1 */
	ADCCON1 = 0;
	/* CVD mode, Fractional mode and scan trigger source */
	ADCCON1bits.SELRES = 3; /* ADC7 resolution is 12 bits */
	ADCCON1bits.STRGSRC = 1; /* Select scan trigger, GSWTRG */
	/* Configure ADCCON2 */
	ADCCON2bits.SAMC = 5; // ADC7 sampling time = 5 * TAD7
	ADCCON2bits.ADCDIV = 1; // ADC7 clock freq is half of control clock = TAD7
	/* Initialize warm up time register */
	ADCANCON = 0;
	ADCANCONbits.WKUPCLKCNT = 5; /* Wakeup exponent = 32 * TADx */
	/* Clock setting */
	ADCCON3 = 0;
	ADCCON3bits.ADCSEL = 0; /* Select input clock source */
	ADCCON3bits.CONCLKDIV = 9;
	ADCCON3bits.VREFSEL = 0; /* Select AVDD and AVSS as reference source */
	ADC0TIMEbits.ADCDIV = 1;
	ADC0TIMEbits.SAMC = 5;
	ADC0TIMEbits.SELRES = 3;
	ADCTRGMODEbits.SH0ALT = 0;
	/* Select ADC input mode */
	ADCIMCON1bits.SIGN15 = 0;
	ADCIMCON1bits.DIFF15 = 0;
	ADCIMCON2bits.SIGN26 = 0;
	ADCIMCON2bits.DIFF26 = 0;
	ADCIMCON2bits.SIGN28 = 0;
	ADCIMCON2bits.DIFF28 = 0;
	/* Configure ADCGIRQENx */
	ADCGIRQEN1 = 0; /* No interrupts are used */
	ADCGIRQEN2 = 0;
	/* Configure ADCCSSx */
	ADCCSS1 = 0;
	ADCCSS2 = 0;
	ADCCSS1bits.CSS15 = 1; /*  (Class set for scan */
	ADCCSS1bits.CSS26 = 1;
	ADCCSS1bits.CSS28 = 1;
	/* Configure ADCCMPCONx */
	ADCCMPCON1 = 0; /* No digital comparators are used */
	ADCCMPCON2 = 0;
	ADCCMPCON3 = 0;
	ADCCMPCON4 = 0;
	ADCCMPCON5 = 0;
	ADCCMPCON6 = 0;
	/* Configure ADCFLTRx */
	ADCFLTR1 = 0; /* No oversampling filters are used */
	ADCFLTR2 = 0;
	ADCFLTR3 = 0;
	ADCFLTR4 = 0;
	ADCFLTR5 = 0;
	ADCFLTR6 = 0;

	/* Always uses scan trigger source */
	/* Early interrupt */
	ADCEIEN1 = 0; // No early interrupt
	ADCEIEN2 = 0;
	/* Turn the ADC on */
	ADCCON1bits.ON = 1;
	/* Wait for voltage reference to be stable */
	while (!ADCCON2bits.BGVRRDY) { /* Wait, reference voltage is ready */
	}
	while (ADCCON2bits.REFFLT) { /* Wait, fault with the reference voltage */
	}
	/* Enable clock to the module */
	ADCCON3bits.DIGEN7 = 1; /* Enable ADC7 */
	ADCANCONbits.ANEN7 = 1; /* Enable clock, ADC7 */
	while (!ADCANCONbits.WKRDY7) { /* Wait until ADC7 is ready */
	}
	ADCCON3bits.GSWTRG = 1;
	/* Enable the ADC module */
	return true;
}
