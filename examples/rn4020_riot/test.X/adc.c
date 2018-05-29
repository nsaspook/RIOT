#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "spi.h"
#include "app.h"
#include "config.h"
#include "timers.h"
#include "ads1220.h"
#include "periph/adc.h"

extern APP_DATA appData;
extern ADC_DATA adcData;

void ADC_Init(void)
{
	adc_init(0);
}

bool ADC_Tasks(void)
{
	/* read value from the adc */
	adcData.potValue = adc_sample(adcData.chan, ADC_RES_12BIT);
	adcData.mcp3208_cmd.map.finish = true;
	adcData.mcp3208_cmd.map.in_progress = false;
	appData.accumReady = true;
	return true;
}

/*
 * Process the accumulator value once it is ready
 * And update stored potentiometer values
 */

void ADC_ProcAccum(void)
{
	appData.potValueOld = appData.potValue; //Save previous value
	appData.potValue = adcData.potValue;
}

void GetNewADC_Chan(void)
{
	adcData.chan = appData.receive_packet[9] == '1' ? 1 : 0; // update adc channel
	adcData.chan += appData.receive_packet[11] == '1' ? 2 : 0;
	adcData.chan += appData.receive_packet[13] == '1' ? 4 : 0;
}
