#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "spi.h"
#include "app.h"
#include "config.h"
#include "timers.h"
#include "ads1220.h"
#include "periph/adc.h"

extern rn4020_appdata_t rn4020_appdata;
extern rn4020_adcdata_t rn4020_adcdata;

void rm4020_adc_Init(void)
{
    adc_init(0);
}

bool rn4040_adc_tasks(void)
{
    /* read value from the adc */
    rn4020_adcdata.potvalue = adc_sample(rn4020_adcdata.chan, ADC_RES_12BIT);
    rn4020_adcdata.mcp3208_cmd.map.finish = true;
    rn4020_adcdata.mcp3208_cmd.map.in_progress = false;
    rn4020_appdata.accumready = true;
    return true;
}

/*
 * Process the accumulator value once it is ready
 * And update stored potentiometer values
 */

void rm4020_adc_procaccum(void)
{
    rn4020_appdata.potvalueold = rn4020_appdata.potvalue;
    rn4020_appdata.potvalue = rn4020_adcdata.potvalue;
}

/* update adc channel from received data */
void rn4040_getnewadc_chan(void)
{
    rn4020_adcdata.chan = rn4020_appdata.receive_packet[9] == '1' ? 1 : 0;
    rn4020_adcdata.chan += rn4020_appdata.receive_packet[11] == '1' ? 2 : 0;
    rn4020_adcdata.chan += rn4020_appdata.receive_packet[13] == '1' ? 4 : 0;
}
