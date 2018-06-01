#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "spi.h"
#include "app.h"
#include "config.h"
#include "timers.h"
#include "ads1220.h"
#include "periph/dac.h"

static uint8_t *tx_buff;
static uint8_t *rx_buff;
static bool upd = false;

extern rn4020_appdata_t rn4020_appdata;

static void ads_spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont,
                                   const void *out, void *in, size_t len)
{
    spi_speed_config(bus, 1, SPI_CLK_2MHZ); /* mode , no speed change */
    SPI_CS1_0;
    timer_shortdelay(75);
    spi_transfer_bytes(bus, cs, cont, out, in, len);
    tx_buff[0] = ADS1220_CMD_SYNC;
    spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 1);
    timer_shortdelay(25);
    SPI_CS1_1;
}

static void ads_int_setup(void)
{
    IEC0CLR = _IEC0_INT2IE_MASK;    /* disable interrupt */
    INTCONbits.INT2EP = 0;
    IFS0CLR = _IFS0_INT2IF_MASK;    /* clear flag */
    IPC3bits.INT2IP = 1;
    IPC3bits.INT2IS = 1;
    IEC0SET = _IEC0_INT2IE_MASK; /* enable interrupt */
}

int ads1220_init(void)
{
    tx_buff = __pic32_alloc_coherent(32); /* uncached memory for spi transfers */
    rx_buff = __pic32_alloc_coherent(32);
    /*
     * setup ads1220 registers
     */
    spi_speed_config(SPI_DEV(2), 1, SPI_CLK_2MHZ); /* mode , no speed change */
    SPI_CS1_0;
    timer_shortdelay(50);
    tx_buff[0] = ADS1220_CMD_RESET;
    spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 1);
    timer_shortdelay(250 * US_TO_CT_TICKS);
    tx_buff[0] = ADS1220_CMD_WREG + 3;
    tx_buff[1] = ads1220_r0;
    tx_buff[2] = ads1220_r1 | ADS1220_TEMP_SENSOR;
    tx_buff[3] = ads1220_r2;
    tx_buff[4] = ads1220_r3;
    spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 5);
    tx_buff[0] = ADS1220_CMD_RREG + 3;
    tx_buff[1] = 0;
    tx_buff[2] = 0;
    tx_buff[3] = 0;
    tx_buff[4] = 0;
    spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 5);
    timer_shortdelay(10 * US_TO_CT_TICKS);
    /*
     * Check to be sure we have a device
     */
    ads_int_setup();
    if ((rx_buff[1] != ads1220_r0) ||
        (rx_buff[2] != (ads1220_r1 | ADS1220_TEMP_SENSOR))) { /* checking for sensor flag too */
        printf(
            "\r\nADS1220 configuration error: %x,%x %x,%x %x %x\r\n",
            ads1220_r0, rx_buff[1], ads1220_r1, rx_buff[2],
            rx_buff[3], rx_buff[4]);
        SPI_CS1_1;
        return(-1);
    }
    tx_buff[0] = ADS1220_CMD_SYNC;
    spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 1);
    SPI_CS1_1;
    return 0;
}

static void ads1220_writeregister(int32_t StartAddress, int32_t NumRegs, uint32_t *pData)
{
    int32_t i;

    tx_buff[0] = ADS1220_CMD_WREG | (((StartAddress << 2) & 0x0c) | ((NumRegs - 1) & 0x03));

    for (i = 0; i < NumRegs; i++) {
        tx_buff[i + 1] = *pData++;
    }

    ads_spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, NumRegs + 2);
    return;
}

static void ai_set_chan_range_ads1220(uint32_t chan, uint32_t range)
{
    static uint32_t range_last = 99;
    static uint32_t chan_last = 99;
    uint32_t cMux;

    if ((chan != chan_last) || (range != range_last)) {
        chan_last = chan;
        range_last = range;
        /*
         * convert chan/range to input MUX switches/gains if needed
         * we could just feed the raw bits to the Mux if needed
         */

        switch (chan) {
            case 0:
                cMux = ADS1220_MUX_0_1;
                break;
            case 1:
                cMux = ADS1220_MUX_0_2;
                break;
            case 2:
                cMux = ADS1220_MUX_0_3;
                break;
            case 3:
                cMux = ADS1220_MUX_1_2;
                break;
            case 8:
                cMux = ADS1220_MUX_0_G;
                break;
            case 9:
                cMux = ADS1220_MUX_1_G;
                break;
            case 10:
                cMux = ADS1220_MUX_2_G;
                break;
            case 11:
                cMux = ADS1220_MUX_3_G;
                break;
            case 15:
                cMux = ADS1220_MUX_DIV2;
                break;
            default:
                cMux = ADS1220_MUX_0_1;
        }
        cMux |= ((range & 0x03) << 1); /* setup the gain bits for range with NO pga */
        cMux |= ads1220_r0_for_mux_gain;
        ads1220_writeregister(ADS1220_0_REGISTER, 0x01, &cMux);
    }
}

int ads1220_testing(void)
{
    static int i = 0, a1 = 2048, b1 = 16;

    if (upd || (i++ > 90000)) {
        PDEBUG3_OFF;
        ai_set_chan_range_ads1220(8, 0);

        /* read the ads1220 3 byte data result */
        tx_buff[0] = ADS1220_CMD_RDATA;
        tx_buff[1] = 0;
        tx_buff[2] = 0;
        tx_buff[3] = 0;
        ads_spi_transfer_bytes(SPI_DEV(2), 0, true, tx_buff, rx_buff, 4);
        rn4020_appdata.ads1220Value = rx_buff[1];
        rn4020_appdata.ads1220Value = (rn4020_appdata.ads1220Value << 8) | rx_buff[2];
        rn4020_appdata.ads1220Value = (rn4020_appdata.ads1220Value << 8) | rx_buff[3];

        /* mangle the data as necessary */
        /* Bipolar Offset Binary */
        //		rn4020_appdata.ads1220Value &= 0x0ffffff;
        //		rn4020_appdata.ads1220Value ^= 0x0800000;

        if (SWITCH1 == 0) {
            printf(" ADS1220 value: %x\r\n", (int) (rn4020_appdata.ads1220Value >> 10));
        }
        upd = false;
        i = 0;
        dac_set(0, a1 += 100);
        dac_set(1, b1 += 100);
    }
    return rn4020_appdata.ads1220Value;
}

void rn4020_int_2_isr(void)
{
    PDEBUG3_ON;
    rn4020_appdata.heatValue = (int16_t) ((rn4020_appdata.ads1220Value >> 10) - 0x0370);
    upd = true;
}
