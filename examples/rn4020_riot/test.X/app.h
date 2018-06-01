

#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "link.h"

#define ERROR_NONE              1
#define ERROR_INITIALIZATION    -2
#define ERROR_RN_FW             -3

typedef enum {
    APP_INITIALIZE = 0,
    APP_INITIALIZATION_ERROR,
    APP_BLUETOOTH_ADVERTISE,
    APP_BLUETOOTH_PAIRED,
    APP_SLEEP
} rn4020_appstate_t;

typedef struct {
    rn4020_appstate_t state;
    char receive_packet[BT_RX_PKT_SZ];
    char transmit_packet[BT_TX_PKT_SZ];
    bool got_packet,
         update_packet,
         sendswitches,
         adc_cal_flag,
         led1, led2, led3, led4, led5, led6,
         oled1, oled2, oled3, oled4;
    int8_t error_code;
    volatile bool sw1, sw2, sw3, sw4,
                  sw1changed, sw2changed, sw3changed, sw4changed,
                  rtccalarm,
                  accumready,
                  adcinuse,
                  timer1flag,
                  cn_int,
                  sleepflag;
    uint16_t potvalue, potvalueold, potvaluelast_tx, version_code, hrm_energy, heat_value;
    int32_t ads1220value;
    struct LINK_DATA *packet_data;
} rn4020_appdata_t;

/* for 24-bit transmit and extra status data */
typedef struct rn4020_adata_t {
    uint32_t dummy12 : 12;
    uint32_t nullbits : 2;
    uint32_t index : 3;
    uint32_t single_diff : 1;
    uint32_t start_bit : 1;
    uint32_t dummy8 : 8;
    uint32_t finish : 1;
    uint32_t in_progress : 1;
} rn4020_adata_t;

/* upper-> lower bytes to 32 bit word for ADC/DAC, etc ... */
union bytes4 {
    uint32_t ld;
    uint8_t bd[4];
};

/* upper/lower bytes to 16 bit word for ADC/DAC, etc ... */
union bytes2 {
    uint16_t ld;
    uint8_t bd[2];
};

/* used to hold 24-bit adc buffer, index and control bits */
union rn4020_adcbuf_t {
    uint32_t ld;
    uint8_t bd[4];
    struct rn4020_adata_t map;
};

typedef struct {
    union rn4020_adcbuf_t mcp3208_cmd;
    uint16_t potvalue;
    uint8_t chan;
} rn4020_adcdata_t;

void rn4020_app_tasks(void);
bool rn4020_app_initialize(void);

#endif //APP_H
