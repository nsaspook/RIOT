#include <stdint.h>
#include <stdbool.h>
#include "spi.h"
#include "app.h"
#include "config.h"
#include "timers.h"
#include "tpic6b595.h"

static uint8_t *tx_buff;
static uint8_t *rx_buff;
static bool upd = false;

extern rn4020_appdata_t rn4020_appdata;

static void tpic6b595_spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont,
                                   const void *out, void *in, size_t len)
{
    spi_speed_config(bus, 0, SPI_CLK_2MHZ); /* mode , no speed change */
    SPI_CS3_0;
//    timer_shortdelay(1);
    spi_transfer_bytes(bus, cs, cont, out, in, len);
    timer_shortdelay(1);
    SPI_CS3_1;
}

int tpic6b595_init(void)
{
    tx_buff = __pic32_alloc_coherent(32); /* uncached memory for spi transfers */
    rx_buff = __pic32_alloc_coherent(32);

    if (!(tx_buff && rx_buff)) {
        return false;
    }
    return true;
}

int tpic6b595_testing(void)
{
    static int i = 0;

    if (upd || (i++ > 90000)) {
        PDEBUG3_OFF;

        /* send test pattern */
        tx_buff[0] = TPIC6B595_TEST_DATA;
        tpic6b595_spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 1);
        upd = false;
        i = 0;
    }
    return 0;
}

