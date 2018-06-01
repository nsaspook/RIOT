/*
 * PIC32MZ EF Curiosity Development Board, RIOT-OS RN4020 BLE testing
 */

#include <stdio.h>
#include <string.h>
#include "periph/uart.h"
#include "periph/gpio.h"
#include "config.h"
#include "app.h"
#include "mrf24.h"

extern void set_cache_policy(uint32_t);

int main(void)
{
    void rn4020_initboard(void);

    /* testing cache modes */
    set_cache_policy(WB_WA);
    LED3_ON;
    /*
     * setup debug serial ports #4 @115200 bps
     */
    uart_init(4, DEBUG_UART_BAUD, NULL, 0);
    printf("\r\n rn4020 app %s\r\n", APP_VERSION_STR);

    while (1) {
        rn4020_app_tasks();
        LED3_OFF;
        LED2_ON;
        mrf24f_testing();
    }

    return 0;
}

void rn4020_initboard(void)
{
    RELAY1;
}
