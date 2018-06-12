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
    assert(line <= 11);

    (void) res;
    switch (line) {
        case 0:
            PDEBUG1_ON;
            ADCCON3bits.GSWTRG = 1;
            while (!ADCDSTAT1bits.ARDY3) {}
            riot_adc.potValue = ADCDATA3;
            PDEBUG1_OFF;
            break;
        case 1:
            ADCCON3bits.GSWTRG = 1;
            while (!ADCDSTAT1bits.ARDY15) {}
            riot_adc.potValue = ADCDATA15;
            break;
        case 2:
            ADCCON3bits.GSWTRG = 1;
            while (!ADCDSTAT1bits.ARDY26) {}
            riot_adc.potValue = ADCDATA26;
            break;
        case 3:
            ADCCON3bits.GSWTRG = 1;
            while (!ADCDSTAT1bits.ARDY28) {}
            riot_adc.potValue = ADCDATA28;
            break;
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
            riot_adc.mcp3208_cmd.ld = 0; // clear the command word
            riot_adc.mcp3208_cmd.map.start_bit = 1;
            riot_adc.mcp3208_cmd.map.single_diff = 1;
            riot_adc.mcp3208_cmd.map.index = line - ADC_CPU_CHANS;
            td[0] = riot_adc.mcp3208_cmd.bd[2];
            td[1] = riot_adc.mcp3208_cmd.bd[1];
            td[2] = riot_adc.mcp3208_cmd.bd[0];
            gpio_clear(Ja10_3);                             // select the ADC
            spi_speed_config(SPI_DEV(2), 0, SPI_CLK_1MHZ);  /* mode 0, speed */
            spi_transfer_bytes(SPI_DEV(2), 0, true, td, rd, 3);
            gpio_set(Ja10_3);                               // deselect the ADC

            /* lsb array index 2 */
            riot_adc.potValue = (rd[1] & 0x0f) << 8;
            riot_adc.potValue += rd[2];
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
        gpio_set(Ja10_3);                   // deselect the ADC
        td = __pic32_alloc_coherent(32);    /* uncached memory for spi transfers */
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
    ANSELBbits.ANSB3 = 1;   /* an3 */
    ANSELEbits.ANSE7 = 1;   /* an15 */
    ANSELEbits.ANSE9 = 1;   /* an26 */
    ANSELAbits.ANSA10 = 1;  /* an28 */
    /* copy calibration data into config registers */
    ADC0CFG = DEVADC0;
    ADC1CFG = DEVADC1;
    ADC2CFG = DEVADC2;
    ADC3CFG = DEVADC3;
    ADC4CFG = DEVADC4;
    ADC7CFG = DEVADC7;
    /* Configure ADCCON1 */
    ADCCON1 = 0;
    /* CVD mode, Fractional mode and scan trigger source */
    ADCCON1bits.SELRES = 3;     /* ADC7 resolution is 12 bits */
    ADCCON1bits.STRGSRC = 1;    /* Select scan trigger, GSWTRG */
    /* Configure ADCCON2 */
    ADCCON2bits.SAMC = 5;       // ADC7 sampling time = 5 * TAD7
    ADCCON2bits.ADCDIV = 8;     // ADC7 clock freq
    /* Initialize warm up time register */
    ADCANCON = 0;
    ADCANCONbits.WKUPCLKCNT = 5; /* Wakeup exponent = 32 * TADx */
    /* Clock setting */
    ADCCON3 = 0;
    ADCCON3bits.ADCSEL = 0;     /* Select input clock source */
    ADCCON3bits.CONCLKDIV = 2;
    ADCCON3bits.VREFSEL = 0;    /* Select AVDD and AVSS as reference source */
    /* adc 3 setup */
    ADC3TIMEbits.ADCDIV = 1;    /* ADC3 clock frequency is half of control clock = TAD3 */
    ADC3TIMEbits.SAMC = 5;      /* ADC3 sampling time = 5 * TAD3 */
    ADC3TIMEbits.SELRES = 3;
    ADCTRGMODEbits.SH3ALT = 0;  /* ADC3 = AN3 */
    ADCTRG1bits.TRGSRC3 = 3;    /* Set AN3 to trigger STRIG */
    /* Select ADC input mode */
    ADCIMCON1bits.DIFF3 = 0;
    ADCIMCON1bits.SIGN3 = 0;
    ADCIMCON1bits.DIFF15 = 0;
    ADCIMCON1bits.SIGN15 = 0;
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
    ADCCSS1bits.CSS3 = 1;
    ADCCSS1bits.CSS15 = 1; /* Class set for scan */
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
    while (!ADCCON2bits.BGVRRDY) {  /* Wait, reference voltage is ready */
    }
    while (ADCCON2bits.REFFLT) {    /* Wait, fault with the reference voltage */
    }
    /* Enable clock to the module */
    ADCCON3bits.DIGEN3 = 1; /* Enable ADC3 */
    ADCANCONbits.ANEN3 = 1;
    while (!ADCANCONbits.WKRDY3) {}
    ADCCON3bits.DIGEN7 = 1;         /* Enable ADC7 */
    ADCANCONbits.ANEN7 = 1;         /* Enable clock, ADC7 */
    while (!ADCANCONbits.WKRDY7) {  /* Wait until ADC7 is ready */
    }
    ADCCON3bits.GSWTRG = 1;
    /* Enable the ADC module */
    return true;
}
