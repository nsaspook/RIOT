

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "xtimer.h"
#include "timex.h"
#include "periph/uart.h"
#include "periph/gpio.h"
#include "periph/spi.h"

#define APP_VERSION_STR "4.3 RIOT-OS"       /* This firmware version */

/*
    2.8	increase ADC sampling and message transmission rates
    2.9	minor spelling fixes
    3.0	Add some public service support
    3.1	heart rate service added (demo data)
 *	makes software version 1.33.4 firmware dependant
    3.2	add automation io service
    4.0	riot-os port
    4.1	basic BLE funtions
    4.2	generize gpio and hardware functions
 *  4.3 restyle and cleanup for riot-os
 */

/*******************************************************************************
 * Application settings - these will change application behavior
 ******************************************************************************/

#define POT_KEEP_AWAKE_DELTA    5

#define MCP1642B_EN    1

#define CVR_BITS            18

#define ADC_NUM_AVGS        5

/* If defined, the RN4020's firmware version will be checked
 * as part of initialization.
 */

#define VERIFY_RN_FW_VER
#define RN_FW_VER_MAJOR     1
#define RN_FW_VER_MINOR     23
#define RN_FW_VER_PATCH     5

#define RN_FW_VER_MAJOR133     1
#define RN_FW_VER_MINOR133     33
#define RN_FW_VER_PATCH133     4

/* Application timers */
#define SLEEP_TIME          TIMER_5MIN_PERIOD_PS256
#define DEBOUNCE_MS         75          /* debounce time for switches 1 - 4 */
#define ADC_REFRESH_MS      50          /* delay between ADC reads, 10 nom value */
#define POT_TX_MS           100         /* delay between transmitting new pot values, */
/* 10 min value */
#define BATT_TX_MS          500
#define HR_TX_MS            100
#define AIO_TX_MS           50
#define LED_BLINK_MS        900         /* LED blink rate for advertise mode */
#define BT_TX_MS            10          /* minimum time between consecutive */
//					/* BTLE message transmissions */
#define BAT_CHK_DELAY_MS    30000
#define BAT_CHK_WAIT_MS     10

/* Buffer sizes */
#define SIZE_RxBuffer   1024
#define SIZE_TxBuffer   256
#define SIZE_SPI_Buffer 64

#define BT_RX_PKT_SZ    100
#define BT_TX_PKT_SZ    100

/* BTLE services */
#define PRIVATE_SERVICE         "28238791ec55413086e0002cd96aec9d"
#define PRIVATE_SERVICE_SPI     "8ee15902ee6f49dc9cfb5c4c2eff6057"
#define PRIVATE_CHAR_SWITCHES       "8f7087bdfdf34b87b10fabbf636b1cd5"
#define PRIVATE_CHAR_POTENTIOMETER  "362232e5c5a94af6b30ce208f1a9ae3e"
#define PRIVATE_CHAR_LEDS       "cd8306093afa4a9da58b8224cd2ded70"
#define PRIVATE_CHAR_RELAYS     "cd83060a3afa4a9da58b8224cd2ded70"
#define PRIVATE_CHAR_ADC_CHAN       "cd83060b3afa4a9da58b8224cd2ded70"
#define PRIVATE_CHAR_PIC_SLAVE      "cd83060c3afa4a9da58b8224cd2ded70"

/* Battery */
#define PUBLIC_BATT_UUID       "180F"
#define PUBLIC_BATT_CHAR_BL    "2A19"

/* Heartbeat */
#define PUBLIC_HR_UUID         "180D"
#define PUBLIC_HR_CHAR_HRM     "2A37"
#define PUBLIC_HR_CHAR_BSL     "2A38"
#define PUBLIC_HR_CHAR_RCP     "2A39"
#define PUBLIC_AIO_UUID        "1815"
#define PUBLIC_AIO_CHAR_DIG    "2A56"
#define PUBLIC_AIO_CHAR_ANA    "2A58"
#define PUBLIC_AIO_CHAR_AGG    "2A5A"

/* handles that change with added services and characteristics
    manually parse the LS command for UUID handles */

#define PUBLIC_HR_CHAR_HRM_H        "001B"
#define PUBLIC_HR_CHAR_HRM_C        "001C"
#define PUBLIC_HR_CHAR_BSL_H        "001E"
#define PUBLIC_HR_CHAR_RCP_H        "0020"
#define PUBLIC_AIO_CHAR_DIG_H       "0023"
#define PUBLIC_AIO_CHAR_ANA_H       "0026"
#define PUBLIC_AIO_CHAR_AGG_H       "0029"
#define PUBLIC_AIO_CHAR_AGG_C       "002A"
#define PUBLIC_BATT_CHAR_H      "003C"
#define PUBLIC_BATT_CHAR_C          "003D"
#define PRIVATE_CHAR_SWITCHES_H     "002D"
#define PRIVATE_CHAR_SWITCHES_C     "002E"
#define PRIVATE_CHAR_POTENTIOMETER_H    "0030"
#define PRIVATE_CHAR_POTENTIOMETER_C    "0031"
#define PRIVATE_CHAR_LEDS_H         "0033"
#define PRIVATE_CHAR_RELAYS_H       "0035"
#define PRIVATE_CHAR_ADC_CHAN_H     "0037"
#define PRIVATE_CHAR_PIC_SLAVE_H    "0039"

/*******************************************************************************
 * End application configuration settings
 * Hardware settings below
 ******************************************************************************/

/* PHY Clock frequency */
#define FCY (100000000)     /* timer and module clock base */

/* RN4020 BTLE */

#define BT_WAKE_HW_SET  gpio_set(C_BLE_IO_WAKE_HW)  /* Hardware wake from dormant state; BT_WAKE_HW, OK */
#define BT_WAKE_HW_CLR  gpio_clear(C_BLE_IO_WAKE_HW)
#define BT_WAKE_SW_SET  gpio_set(C_BLE_IO_WAKE_SW)  /* Deep sleep wake; BT_WAKE_SW, OK */
#define BT_WAKE_SW_CLR  gpio_clear(C_BLE_IO_WAKE_SW)
#define BT_CMD_SET  gpio_set(C_BT_CMD)              /* Place RN4020 module in command mode, low for MLDP mode, OK */
#define BT_CMD_CLR  gpio_clear(C_BT_CMD)
#define BT_CONNECTED    gpio_read(C_BLE_IO_CONN)    /* RN4020 module is connected to central device, OK */
#define BT_WS       gpio_read(C_BT_WS)              /* RN4020 module is awake and active, OK */
#define BT_MLDP_EV      gpio_read(C_BT_MLDP_EV)     /* RN4020 module in MLDP mode has a pending event, NC, OK */

/* RELAY outputs */
#define RELAY1  LED3_TOGGLE /* output 0 (low) turns on relay */
#define RELAY2  LED3_TOGGLE
#define RELAY3  LED3_TOGGLE
#define RELAY4  LED3_TOGGLE

#define SWITCH1     gpio_read(C_SWITCH_1)

/* LED outputs */

#define SLED LED4R_TOGGLE
#define SLED_ON     LED4R_ON
#define SLED_OFF    LED4R_OFF

#define G_LED_ON    LED4G_ON
#define G_LED_OFF   LED4G_OFF

#define B_LED_ON    LED4B_ON
#define B_LED_OFF   LED4B_OFF
#define B_LED_TOGGLE    LED4B_TOGGLE

#define SPI_CS0_1_J10 gpio_set(C_SPI0_CS_J10)
#define SPI_CS1_1 gpio_set(C_SPI1_CS)
#define SPI_CS2_1 gpio_set(C_SPI2_CS)

#define SPI_CS0_0_J10 gpio_clear(C_SPI0_CS_J10)
#define SPI_CS1_0 gpio_clear(C_SPI1_CS)
#define SPI_CS2_0 gpio_clear(C_SPI2_CS)

/* Timer initialization */
#define TIMER_OFF 0
#define TIMER_ON_PRESCALE1      0x8000
#define TIMER_ON_PRESCALE8      0x8010
#define TIMER_ON_PRESCALE64     0x8020
#define TIMER_ON_PRESCALE256    0x8030

/* Timer periods
   32-bit mode with 1:256 postscale below */
#define TIMER_5MIN_PERIOD_PS256 ((uint32_t)((FCY / 256) * 300 - 1))
#define TIMER_1MIN_PERIOD_PS256 ((uint32_t)((FCY / 256) * 60 - 1))
#define TIMER_10S_PERIOD_PS256  ((uint32_t)((FCY / 256) * 10 - 1))
#define TIMER_1S_PERIOD_PS256   ((uint16_t)(FCY / 256 - 1))

/* 16-bit mode with 1:1 postscale below */
#define TIMER_1MS_PERIOD        ((uint16_t)(FCY / 1000 - 1))
#define TIMER_100US_PERIOD      ((uint16_t)(FCY / 10000 - 1))
#define TIMER_500US_PERIOD      ((uint16_t)(FCY / 2000 - 1))

#endif
