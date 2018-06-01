/*
 * Copyright (C) 2014 Microchip Technology Inc. and its subsidiaries.  You may use this software and any derivatives
 * exclusively with Microchip products.
 *
 * MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:  You may use this software, and any derivatives created by any person or
 * entity by or on your behalf, exclusively with Microchip?s products.  Microchip and its licensors retain all ownership
 * and intellectual property rights in the accompanying software and in all derivatives hereto.
 *
 * This software and any accompanying information is for suggestion only.  It does not modify Microchip?s standard
 * warranty for its products.  You agree that you are solely responsible for testing the software and determining its
 * suitability.  Microchip has no obligation to modify, test, certify, or support the software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,
 * BUT NOT LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
 * APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP?S PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN
 * ANY APPLICATION.
 *
 * IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
 * STATUTORY DUTY), STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
 * TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS
 * SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE TERMS.
 *
 *
 * File:        app.c
 * Date:        July 24, 2014
 * Compiler:    XC16 v1.23
 *
 */

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "app.h"
#include "config.h"
#include "timers.h"
#include "uart.h"
#include "bluetooth.h"
#include "adc.h"
#include "leds.h"
#include "switches.h"
#include "rtcc.h"
#include "sleep.h"
#include "spi.h"
#include "automio.h"
#include "mrf24.h"
#include "ads1220.h"

rn4020_appdata_t rn4020_appdata;
rn4020_adcdata_t rn4020_adcdata;
aio_data_t rn4020_aiodata;

/* Primary application state machine */

void rn4020_app_tasks(void)
{
    /* Update LED outputs */
    LED_Tasks();

    switch (rn4020_appdata.state) {
        /* Initial state */
        case APP_INITIALIZE:
            if (rn4020_app_initialize()) {
                rn4020_appdata.state = APP_BLUETOOTH_ADVERTISE;
            }
            else {
                rn4020_appdata.state = APP_INITIALIZATION_ERROR;
            }
            break;

        /* Initialization failed */
        case APP_INITIALIZATION_ERROR:
            rn4040_led_set_lightshow(LED_ERROR);
            break;

        /* We're not connected to a device - advertise mode */
        case APP_BLUETOOTH_ADVERTISE:
            rn4040_led_set_lightshow(LED_BTLE_ADVERTISING);
            if (BT_CONNECTED) {
                rn4020_appdata.state = APP_BLUETOOTH_PAIRED;
            }
            break;

        /* We are connected to a BTLE device */
        case APP_BLUETOOTH_PAIRED:
            /* Update LEDs */
            rn4040_led_set_lightshow(LED_BTLE_PAIRED);
            /* Check to see if we are still connected; return to advertise state if not */
            if (!BT_CONNECTED) {
                rn4020_appdata.update_packet = false;
                rn4040_led_set_lightshow(LED_BTLE_ADVERTISING);
                rn4020_appdata.state = APP_BLUETOOTH_ADVERTISE;
                G_LED_OFF;
                break;
            }

            /* Check if switches have changed and debounce timers are expired */
            Switch_Tasks();
            if (rn4020_appdata.sendSwitches) { //New switch status to send?
                sprintf(rn4020_appdata.transmit_packet, "shw," PRIVATE_CHAR_SWITCHES_H ",%d%d%d%d\r", rn4020_appdata.sw1, rn4020_appdata.sw2, rn4020_appdata.sw3, rn4020_appdata.sw4);
                if (rn4020_bt_sendcommand(rn4020_appdata.transmit_packet, true)) {
                    rn4020_appdata.sendSwitches = false;
                }
            }

            /* Process ADC accumulator value if oversampling is complete */
            if (rn4020_appdata.accumReady) {
                ADC_ProcAccum();
                rn4020_appdata.accumReady = false; //Clear app flags
                rn4020_appdata.ADCinUse = false;
            }

            /*
             *  packet transmission queue, rn4020_timerdone puts packet in the transmission stream
             */

            /* Start new ADC read if timer expired, not currently sampling, and not waiting to process accumulator */
            if (rn4020_timerdone(TMR_ADC) && rn4020_appdata.ADCinUse == false) {
                if (rn4040_adc_tasks()) {
                    rn4020_starttimer(TMR_ADC, ADC_REFRESH_MS);
                }
            }

            /* Transmit new potentiometer reading? */
            if (rn4020_timerdone(TMR_POT)) {
                /* Send message only if pot value has changed */
                if (rn4020_appdata.potValue != rn4020_appdata.potValueLastTX) {
                    sprintf(rn4020_appdata.transmit_packet, "shw," PRIVATE_CHAR_POTENTIOMETER_H ",%04d\r\n", rn4020_appdata.potValue);
                    if (rn4020_bt_sendcommand(rn4020_appdata.transmit_packet, true)) {
                        rn4020_appdata.potValueLastTX = rn4020_appdata.potValue;
                        rn4020_starttimer(TMR_POT, POT_TX_MS);
                    }
                }
                else {
                    rn4020_starttimer(TMR_POT, POT_TX_MS);
                } /* value not changed - skip this transmission */
            }

            if (rn4020_timerdone(TMR_BATT)) {
                sprintf(rn4020_appdata.transmit_packet, "suw,"PUBLIC_BATT_CHAR_BL ",%02d\r", (rn4020_appdata.potValue >> 6) & 0b00111111);
                if (rn4020_bt_sendcommand(rn4020_appdata.transmit_packet, true)) {
                    rn4020_starttimer(TMR_BATT, BATT_TX_MS);
                }
            }

            if (rn4020_timerdone(TMR_HR)) {
                sprintf(rn4020_appdata.transmit_packet, "suw,"PUBLIC_HR_CHAR_HRM ","
                        "%02x%02x%02x%02x\r", 0x08,
                        (rn4020_appdata.heatValue) & 0xff,
                        rn4020_appdata.hrmEnergy & 0x00ff,
                        rn4020_appdata.hrmEnergy >> 8); // format mask and ADC data
                if (rn4020_bt_sendcommand(rn4020_appdata.transmit_packet, true)) {
                    rn4020_starttimer(TMR_HR, HR_TX_MS);
                    sprintf(rn4020_appdata.transmit_packet, "suw,"PUBLIC_HR_CHAR_BSL ",%02x\r", 3);
                    rn4020_bt_sendcommand(rn4020_appdata.transmit_packet, false);
                    rn4020_appdata.hrmEnergy++;
                }
            }

            if (rn4020_timerdone(TMR_AIO_DIG)) {
                //Form message
                sprintf(rn4020_appdata.transmit_packet, "shw,"PUBLIC_AIO_CHAR_DIG_H ",0101010101010101\r"); /* digital data */
                //Try to transmit the message; reset timer if successful
                if (rn4020_bt_sendcommand(rn4020_appdata.transmit_packet, true)) {
                    rn4020_starttimer(TMR_AIO_DIG, AIO_TX_MS);
                }
            }

            /* Process any new messages received from RN module */
            rn4020_appdata.got_packet = BT_ReceivePacket(rn4020_appdata.receive_packet);        /* Get new message if one has been received from the RN4020 */
            if (rn4020_appdata.got_packet == true) {                                            /* true if new packet received */
                if (strstr(rn4020_appdata.receive_packet, "WV," PRIVATE_CHAR_LEDS_H ",")) {     /* Check for LED update message 1.33 */
                    rn4040_getnewleds();
                    rn4020_appdata.update_packet = true;
                }
                /*
                   //Other message handling can be added here
                 */
                /* receive new SPI ADC channel */
                if (strstr(rn4020_appdata.receive_packet, "WV," PRIVATE_CHAR_ADC_CHAN_H ",")) {
                    rn4040_getnewadc_chan(); /* new ADC config data */
                }
                /* receive new SPI SLAVE request */
                if (strstr(rn4020_appdata.receive_packet, "WV," PRIVATE_CHAR_PIC_SLAVE_H ",")) {}
                /* HRM energy expended reset */
                if (strstr(rn4020_appdata.receive_packet, "WV,"PUBLIC_HR_CHAR_RCP_H ",01.")) {
                    rn4020_appdata.hrmEnergy = 0;
                }
                /* receive new BATTERY request */
                if (strstr(rn4020_appdata.receive_packet, "RV,"PUBLIC_BATT_CHAR_H ".")) {
                    sprintf(rn4020_appdata.transmit_packet, "suw,"PUBLIC_BATT_CHAR_BL ",%d\r", 63);
                    rn4020_bt_sendcommand(rn4020_appdata.transmit_packet, false);
                }
            }
            break;

        default:
            break;
    }   /* end switch(rn4020_appdata.state) */
}       /* end rn4020_app_tasks() */

//Sets up the RN module

bool rn4020_app_initialize(void)
{
    /****************************************************************************
     * Initialize rn4020_appdata structure
     ***************************************************************************/
    rn4020_appdata.error_code = ERROR_NONE;
    rn4020_appdata.got_packet = false;
    rn4020_appdata.potValue = 0;
    rn4020_appdata.potValueOld = 0xFFFF;
    rn4020_appdata.potValueLastTX = 0xFFFF;
    rn4020_appdata.state = APP_INITIALIZE;
    rn4020_appdata.sw1 = false;
    rn4020_appdata.sw2 = false;
    rn4020_appdata.sw3 = false;
    rn4020_appdata.sw4 = false;
    rn4020_appdata.led1 = 0;
    rn4020_appdata.led2 = 0;
    rn4020_appdata.led3 = 0;
    rn4020_appdata.led4 = 0;
    rn4020_appdata.led5 = 0;
    rn4020_appdata.led6 = 0;
    rn4020_appdata.update_packet = true;
    rn4020_appdata.sw1Changed = false;
    rn4020_appdata.sw2Changed = false;
    rn4020_appdata.sw3Changed = false;
    rn4020_appdata.sw4Changed = false;
    rn4020_appdata.sendSwitches = false;
    rn4020_appdata.ADCcalFlag = false;
    rn4020_appdata.sleepFlag = false;
    rn4020_appdata.RTCCalarm = false;
    rn4020_appdata.accumReady = false;
    rn4020_appdata.ADCinUse = false;
    rn4020_appdata.timer1Flag = false;

    /****************************************************************************
     * Peripherals Init
     ***************************************************************************/
    SPI_Init();
    ADC_Init();
    UART_Init();
    rn4020_timers_init();
    Mrf24_Init();
    ads1220_init();

    B_LED_ON;
    BT_WAKE_SW_SET; //wake module
    /* Wait for WS status high */
    rn4020_starttimer(TMR_RN_COMMS, 4000);
    while (BT_WS == 0) {
        if (rn4020_timerdone(TMR_RN_COMMS)) {
            rn4020_appdata.error_code = ERROR_INITIALIZATION;
            return false;
        }
    }

    B_LED_OFF;
    G_LED_ON;

    /* Wait for end of "CMD\r\n" - we don't check for full "CMD\r\n" string because we may
       //miss some bits or bytes at the beginning while the UART starts up */
    rn4020_starttimer(TMR_RN_COMMS, 4000);
    while (UART_ReadRxBuffer() != '\n') {
        if (rn4020_timerdone(TMR_RN_COMMS)) {
            rn4020_appdata.error_code = ERROR_INITIALIZATION;
            return false;
        }
    }

    /* Module is now in command mode and ready for input */
    if (!BT_SetupModule()) { /* Setup RN4020 module */
        rn4020_appdata.error_code = ERROR_INITIALIZATION;
        return false;
    }


#ifdef VERIFY_RN_FW_VER
    if (!(rn4020_appdata.version_code = BT_CheckFwVer())) {
        rn4020_appdata.error_code = ERROR_RN_FW;
        return false;
    }
#endif

    /* flush UART RX buffer as a precaution before starting app state machine */
    while (UART_IsNewRxData()) {
        UART_ReadRxBuffer();
        if (!UART_IsNewRxData()) {
            rn4020_wait_ms(100);
        }
    }

    SLED; /* init completed */
    G_LED_OFF;
    return true;
}
