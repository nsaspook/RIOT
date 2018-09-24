#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "spi.h"
#include "app.h"
#include "config.h"
#include "timers.h"
#include "ads1220.h"
#include "periph/dac.h"
#include "periph/adc.h"

static uint8_t *tx_buff;
static uint8_t *rx_buff;
static bool upd = false;

extern rn4020_appdata_t rn4020_appdata;

static void tpic6b595_spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont,
                                   const void *out, void *in, size_t len)
{
    spi_speed_config(bus, 1, SPI_CLK_2MHZ); /* mode , no speed change */
    SPI_CS3_0;
    timer_shortdelay(75);
    spi_transfer_bytes(bus, cs, cont, out, in, len);
    timer_shortdelay(25);
    SPI_CS3_1;
}

int tpic6b595_init(void)
{
    tx_buff = __pic32_alloc_coherent(8); /* uncached memory for spi transfers */
    rx_buff = __pic32_alloc_coherent(8);

    if (!(tx_buff && rx_buff)) {
        return false;
    }
    return true;
}

int tpic6b595_testing(void)
{
    static int i = 0;

    if (upd || (i++ > 90000)) {
        static int a1 = 2048, b1 = 16;
        PDEBUG3_OFF;

        /* read the ads1220 3 byte data result */
        tx_buff[0] = ADS1220_CMD_RDATA;
        tpic6b595_spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 1);
        upd = false;
        i = 0;
        dac_set(0, a1 += 100);
        dac_set(1, b1 += 100);
    }
    return 0;
}

