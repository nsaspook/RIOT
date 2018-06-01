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
         sendSwitches,
         ADCcalFlag,
         led1, led2, led3, led4, led5, led6,
         oled1, oled2, oled3, oled4;
    int8_t error_code;
    volatile bool sw1, sw2, sw3, sw4,
                  sw1Changed, sw2Changed, sw3Changed, sw4Changed,
                  RTCCalarm,
                  accumReady,
                  ADCinUse,
                  timer1Flag,
                  CNint,
                  sleepFlag;
    uint16_t potValue, potValueOld, potValueLastTX, version_code, hrmEnergy, heatValue;
    int32_t ads1220Value;
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
    uint16_t potValue;
    uint8_t chan;
} rn4020_adcdata_t;

void rn4020_app_tasks(void);
bool rn4020_app_initialize(void);

#endif //APP_H
