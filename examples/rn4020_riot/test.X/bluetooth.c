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
 * File:        bluetooth.c
 * Date:        January 20, 2015
 * Compiler:    XC16 v1.23
 *
 * Functions to communicate with a RN4020 Bluetooth LE module over a UART
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "bluetooth.h"
#include "config.h"
#include "app.h"
#include "uart.h"
#include "timers.h"

uint16_t bt_checkfwver(void);

bool bt_receivepacket(char *Message)
{
    static enum BluetoothDecodeState btDecodeState = WaitForCR;
    static uint16_t i = 0;

    if (UART_IsNewRxData()) {
        PDEBUG2_ON;
        Message[i++] = UART_ReadRxBuffer();
        if (i == BT_RX_PKT_SZ) {
            i = 0;
        }

        switch (btDecodeState) {
            case WaitForCR:
                if (Message[i - 1] == '\r') {
                    btDecodeState = WaitForLF;
                }
                break;

            case WaitForLF:
                btDecodeState = WaitForCR;
                if (Message[i - 1] == '\n') {
                    Message[i] = 0;
                    i = 0;
                    PDEBUG2_OFF;
                    return true;
                }
                break;

            default:
                btDecodeState = WaitForCR;
        }
    }
    return false;
}

bool rn4020_bt_sendcommand(const char *data, bool wait)
{
    /*
     * Only transmit a message if TX timer expired, or wait flag is set to false
     * We limit transmission frequency to avoid overwhelming the BTLE link
     */
    if (rn4020_timerdone(TMR_BT_TX) || wait == false) {
        PDEBUG1_ON;
        uart_write(1, (uint8_t *) data, strlen(data));
        rn4020_starttimer(TMR_BT_TX, BT_TX_MS);
        PDEBUG1_OFF;
        return true;
    }
    return false;
}

void bt_sendbyte(char data)
{
    UART_WriteTxBuffer(data);
}


bool bt_getresponse(char *data)
{
    uint16_t byteCount = 0;
    char newByte;

    rn4020_starttimer(TMR_RN_COMMS, 600);

    while (byteCount < BT_RX_PKT_SZ) {
        if (UART_IsNewRxData()) {
            newByte = UART_ReadRxBuffer();
            *data++ = newByte;
            byteCount++;
            if (newByte == '\n') {
                return true;
            }
        }
        if (rn4020_timerdone(TMR_RN_COMMS)) {
            return false;
        }
    }
    return false;
}

bool bt_compareresponse(const char *data1, const char *data2)
{
    uint16_t i;

    for (i = 0; i < 50; i++) {
        if (*data1 == '\0') {
            return true;
        }
        else if (*data1++ != *data2++) {
            return false;
        }
    }
    return false;
}

bool bt_checkresponse(const char *data)
{
    uint16_t i, ByteCount = 0;
    char NewByte, Buffer[50], *BufPtr;

    rn4020_starttimer(TMR_RN_COMMS, 600);

    BufPtr = Buffer;
    while (ByteCount < 50) {
        if (UART_IsNewRxData()) {
            NewByte = (char) UART_ReadRxBuffer();
            *BufPtr++ = NewByte;
            ByteCount++;
            if (NewByte == '\n') {
                break;
            }
        }
        if (rn4020_timerdone(TMR_RN_COMMS)) {
            return false;
        }
    }

    BufPtr = Buffer;
    for (i = 0; i < ByteCount; i++) {
        if (*data == '\0') {
            return true;
        }
        else if (*data++ != *BufPtr++) {
            return false;
        }
    }
    return true;
}


/*
 *  Get a response from the RN4020 module and compare with an expected response
 * All incoming bytes in the position of the wildcard character are ignored
 * Use this to ignore text that changes, like MAC addresses.
 */

bool bt_checkresponsewithwildcard(const char *data, char Wildcard)
{
    uint16_t i, ByteCount = 0;
    char NewByte, Buffer[50], *BufPtr;

    rn4020_starttimer(TMR_RN_COMMS, 600);

    BufPtr = Buffer;
    while (ByteCount < 50) {
        if (UART_IsNewRxData()) {
            NewByte = UART_ReadRxBuffer();
            *BufPtr++ = NewByte;
            ByteCount++;
            if (NewByte == '\n') {
                break;
            }
        }
        if (rn4020_timerdone(TMR_RN_COMMS)) {
            return false;
        }
    }

    BufPtr = Buffer;
    for (i = 0; i < ByteCount; i++) {
        if (*data == '\0') {
            return true;
        }
        else if (*data == Wildcard) {
            data++;
            BufPtr++;
        }
        else if (*data++ != *BufPtr++) {
            return false;
        }
    }
    return true;
}

/*
 *  Set up the RN4020 module
 */

bool bt_setupmodule(void)
{
    uint16_t version_code;

    version_code = bt_checkfwver();

    LED4G_ON;
    rn4020_bt_sendcommand("sf,2\r\n", false); /* Get RN4020 module feature settings */
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    //Send "GR" to get feature settings
    rn4020_bt_sendcommand("gr\r\n", false);
    if (!bt_checkresponse("26060000\r\n")) {
        rn4020_bt_sendcommand("sr,26060000\r\n", false);
        if (!bt_checkresponse(AOK)) {
            return false;
        }
    }

    char macAddr[16];
    rn4020_bt_sendcommand("gds\r\n", false); /* Get mac address */
    while (!bt_receivepacket(macAddr)) {}

    char message[12];
    macAddr[12] = '\0';
    sprintf(message, "sn,%s_BT\r\n", &macAddr[8]);

    rn4020_bt_sendcommand(message, false); /* Set advertise name */
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    rn4020_bt_sendcommand("gs\r\n", false);
    if (!bt_checkresponse("F0000001\r\n")) {
        /*
         * Send "SS" to set user defined private profiles
         * and ID/Battery in 1.33 firmware
         */
        rn4020_bt_sendcommand("ss,F0000001\r\n", false);
        if (!bt_checkresponse(AOK)) {
            return false;
        }
    }

    rn4020_bt_sendcommand("s-,FRC-\r\n", false);
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    //  initial connection parameters
    rn4020_bt_sendcommand("st,003c,0000,0064\r\n", false);
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    /* Clear all settings of defined services and characteristics */
    rn4020_bt_sendcommand("pz\r\n", false);
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    if (version_code >= 33) { /* V1.33.4 public services */
        // Public BTLE services and characteristics

        /* heart rate service with standard 16-bit UUID */
        rn4020_bt_sendcommand("ps,"PUBLIC_HR_UUID ",\r", false);
        if (!bt_checkresponse(AOK)) {
            return false;
        }

        rn4020_bt_sendcommand("pc,"PUBLIC_HR_CHAR_HRM ",12,04\r", false);
        if (!bt_checkresponse(AOK)) {
            return false;
        }

        rn4020_bt_sendcommand("pc,"PUBLIC_HR_CHAR_BSL ",06,01\r", false);
        if (!bt_checkresponse(AOK)) {
            return false;
        }

        rn4020_bt_sendcommand("pc,"PUBLIC_HR_CHAR_RCP ",06,01\r", false);
        if (!bt_checkresponse(AOK)) {
            return false;
        }

        rn4020_bt_sendcommand("ps,"PUBLIC_AIO_UUID ",\r", false);
        if (!bt_checkresponse(AOK)) {
            return false;
        }

        rn4020_bt_sendcommand("pc,"PUBLIC_AIO_CHAR_DIG ",16,08,33\r", false);
        if (!bt_checkresponse(AOK)) {
            return false;
        }

        rn4020_bt_sendcommand("pc,"PUBLIC_AIO_CHAR_ANA ",16,02,33\r", false);
        if (!bt_checkresponse(AOK)) {
            return false;
        }

        rn4020_bt_sendcommand("pc,"PUBLIC_AIO_CHAR_AGG ",12,0F,33\r", false);
        if (!bt_checkresponse(AOK)) {
            return false;
        }
    }

    /* set power to max */
    rn4020_bt_sendcommand("sp,7\r\n", false);
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    rn4020_bt_sendcommand("sdr,"APP_VERSION_STR "\r\n", false);
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    /* Private BTLE services and characteristics */

    /* Send "ps" to set user defined service UUID */
    rn4020_bt_sendcommand("ps," PRIVATE_SERVICE ",\r", false);
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    rn4020_bt_sendcommand("pc," PRIVATE_CHAR_SWITCHES ",22,02\r", false);
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    rn4020_bt_sendcommand("pc," PRIVATE_CHAR_POTENTIOMETER ",22,02\r", false);
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    rn4020_bt_sendcommand("pc," PRIVATE_CHAR_LEDS ",0A,04\r", false);
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    rn4020_bt_sendcommand("pc," PRIVATE_CHAR_RELAYS ",0A,04\r", false);
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    rn4020_bt_sendcommand("pc," PRIVATE_CHAR_ADC_CHAN ",0A,04\r", false);
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    rn4020_bt_sendcommand("pc," PRIVATE_CHAR_PIC_SLAVE ",0A,0F\r", false);
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    rn4020_bt_sendcommand("wc\r\n", false); /* Command to clear script */
    if (!bt_checkresponse(AOK)) {
        return false;
    }

    return bt_rebootenflow();
}

/*
 * Reboot the module and enable flow control on PIC UART
 */

bool bt_rebootenflow(void)
{
    bool do_ls = true, good_boot;

    /* Send "R,1" to save changes and reboot */
    rn4020_bt_sendcommand("r,1\r\n", false);
    if (!bt_checkresponse("Reboot\r\n")) {
        return false;
    }

    /* Clear out NULL char(s) and other garbage if present after reboot,
     * wait for first char of CMD\r\n response
     */
    rn4020_starttimer(TMR_RN_COMMS, 4000);
    while (UART_ReadRxBuffer() != 'C') {
        while (!UART_IsNewRxData()) {
            if (rn4020_timerdone(TMR_RN_COMMS)) {
                return false;
            }
        }
    }

    good_boot = bt_checkresponse("MD\r\n");

    if (do_ls) {
        rn4020_bt_sendcommand("LS\r\n", false);
        rn4020_wait_ms(1000);
    }


    rn4020_wait_ms(2);
    if (SWITCH1 == 0) {
        LED4G_OFF;
        BT_WAKE_SW_SET;
        BT_WAKE_HW_SET;
        BT_CMD_CLR;

        rn4020_bt_sendcommand("SF,2\r\n", false); /* perform complete factory reset */
        rn4020_wait_ms(100);
        bt_checkresponse(AOK);

        rn4020_bt_sendcommand("SF,2\r\n", false);
        rn4020_wait_ms(100);
        if (!bt_checkresponse(AOK)) {
            return false;
        }

        rn4020_bt_sendcommand("SDH,4.1\r\n", false);
        rn4020_wait_ms(100);
        if (!bt_checkresponse(AOK)) {
            return false;
        }
        rn4020_bt_sendcommand("SDM,RN4020\r\n", false);
        rn4020_wait_ms(100);
        if (!bt_checkresponse(AOK)) {
            return false;
        }

        rn4020_bt_sendcommand("SDN,Microchip\r\n", false);
        rn4020_wait_ms(100);
        if (!bt_checkresponse(AOK)) {
            return false;
        }

        rn4020_bt_sendcommand("SP,7\r\n", false);
        rn4020_wait_ms(100);
        if (!bt_checkresponse(AOK)) {
            return false;
        }

        rn4020_bt_sendcommand("SS,C0000000\r\n", false);
        rn4020_wait_ms(100);
        if (!bt_checkresponse(AOK)) {
            return false;
        }

        /* support MLDP, enable OTA (peripheral mode is enabled by default) */
        rn4020_bt_sendcommand("SR,32008000\r\n", false);
        rn4020_wait_ms(100);
        if (!bt_checkresponse(AOK)) {
            return false;
        }
        rn4020_bt_sendcommand("R,1\r\n", false); //Force reboot


        rn4020_starttimer(TMR_RN_COMMS, 4000);      //Start 4s timeout
        while (BT_WS == 0) {
            if (rn4020_timerdone(TMR_RN_COMMS)) {   //Check if timed out
                return false;
            }
        }

        /* Wait for end of "CMD\r\n" - we don't check for full "CMD\r\n"
         * string because we may
         * miss some bits or bytes at the beginning while the UART starts up
         */
        rn4020_starttimer(TMR_RN_COMMS, 4000);
        while (UART_ReadRxBuffer() != '\n') {
            if (rn4020_timerdone(TMR_RN_COMMS)) {
                return false;
            }
        }

        rn4020_bt_sendcommand("I\r\n", false);  // MLDP mode
        rn4020_bt_sendcommand("A\r\n", false);  // start advertising

        /* wait loop controller for power cycle/reset */
        while (true) {
            while (true) {
                while (UART_IsNewRxData()) {
                    UART_ReadRxBuffer();
                    if (!UART_IsNewRxData()) {
                        rn4020_wait_ms(200);
                    }
                }
                rn4020_wait_ms(200);
                B_LED_TOGGLE;
            }

        }

    }

    if (do_ls) {

        while (UART_IsNewRxData()) {
            UART_ReadRxBuffer();
            if (!UART_IsNewRxData()) {
                rn4020_wait_ms(100);
            }
        }
        //Clear any UART error bits
        U1STAbits.FERR = 0;
        U1STAbits.PERR = 0;
    }

    return good_boot;
}

#ifdef VERIFY_RN_FW_VER

uint16_t bt_checkfwver(void)
{
    char fpVer[20];
    char *pfpVer = fpVer;
    char strVer[100];
    char *pstrVer = strVer;
    unsigned int verMajor,
                 verMinor,
                 verPatch;

    while (UART_IsNewRxData()) {
        UART_ReadRxBuffer();
        rn4020_wait_ms(100);
    }

    rn4020_starttimer(TMR_RN_COMMS, 2000);
    rn4020_bt_sendcommand("v\r", false);
    while (!bt_receivepacket(strVer)) {
        if (rn4020_timerdone(TMR_RN_COMMS)) {
            return false;
        }
    }

    //Skip to first digit
    while ((*pstrVer < '0' || *pstrVer > '9') && *pstrVer != 0) {
        pstrVer++;
    }
    //Extract version number
    while ((*pstrVer >= '0' && *pstrVer <= '9') || *pstrVer == '.') {
        *pfpVer = *pstrVer;
        pfpVer++;
        pstrVer++;
    }
    *pfpVer = '\0';

    //Tokenize and convert to unsigned
    sscanf(fpVer, "%u.%u.%u", &verMajor, &verMinor, &verPatch);

    //Verify version number
    if ((verMajor != RN_FW_VER_MAJOR133) && (verMajor != RN_FW_VER_MAJOR)) {
        return false;
    }
    if ((verMinor != RN_FW_VER_MINOR133) && (verMinor != RN_FW_VER_MINOR)) {
        return false;
    }
    if ((verPatch != RN_FW_VER_PATCH133) && (verPatch != RN_FW_VER_PATCH)) {
        return false;
    }

    return verMinor;
}
#endif
