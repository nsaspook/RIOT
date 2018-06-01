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
 * File:        leds.c
 * Date:        January 20, 2015
 * Compiler:    XC16 v1.23
 * add relay outputs
 *
 * LED functions
 *
 */

#include "timers.h"
#include "config.h"
#include "leds.h"
#include "app.h"

extern rn4020_appdata_t rn4020_appdata;
extern rn4020_adcdata_t rn4020_adcdata;
static LED_LIGHTSHOW_T lightShow = LED_IDLE;

void LED_Tasks(void)
{
    switch (lightShow) {
        case LED_IDLE:
            SLED_ON;
            break;

        case LED_BTLE_ADVERTISING:
            if (rn4020_timerdone(TMR_LEDS)) {
                SLED;
                rn4020_starttimer(TMR_LEDS, LED_BLINK_MS);
            }
            break;

        case LED_BTLE_PAIRED:
            if (!rn4020_appdata.led1) {
                RELAY1; /* toggle led 3 for testing */
            }
            /*
             * logic low turns on relay
               RELAY1 = !rn4020_appdata.led1;
               RELAY2 = !rn4020_appdata.led2;
               RELAY3 = !rn4020_appdata.led3;
               RELAY4 = !rn4020_appdata.led4;
             */
            SLED_OFF;
            G_LED_ON;
            break;

        case LED_ERROR:
            switch (rn4020_appdata.error_code) {
                case ERROR_INITIALIZATION:
                    SLED_OFF;
                    break;
                case ERROR_RN_FW:
                    SLED_OFF;
                    break;
                default:
                    SLED_OFF;
                    break;
            }
            break;

        case LED_SLEEP:
            SLED_OFF;
            break;

        default:
            break;
    }
}

void rn4040_led_set_lightshow(LED_LIGHTSHOW_T setting)
{
    lightShow = setting;
}

//Update LEDs with status from LED update message

void rn4040_getnewleds(void)
{
    if (!rn4020_appdata.update_packet) {
        rn4020_appdata.led1 = rn4020_appdata.oled1;
        rn4020_appdata.led2 = rn4020_appdata.oled2;
        rn4020_appdata.led3 = rn4020_appdata.oled3;
        rn4020_appdata.led4 = rn4020_appdata.oled4;
    }
    else {
        rn4020_appdata.led1 = rn4020_appdata.receive_packet[9] == '1' ? 1 : 0;
        rn4020_appdata.led2 = rn4020_appdata.receive_packet[11] == '1' ? 1 : 0;
        rn4020_appdata.led3 = rn4020_appdata.receive_packet[13] == '1' ? 1 : 0;
        rn4020_appdata.led4 = rn4020_appdata.receive_packet[15] == '1' ? 1 : 0;
        rn4020_appdata.oled1 = rn4020_appdata.led1;
        rn4020_appdata.oled2 = rn4020_appdata.led2;
        rn4020_appdata.oled3 = rn4020_appdata.led3;
        rn4020_appdata.oled4 = rn4020_appdata.led4;
    }
}
