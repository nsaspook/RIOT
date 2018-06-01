/*
 * Copyright (C) 2015 Microchip Technology Inc. and its subsidiaries.  You may use this software and any derivatives
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
 * File:        switches.c
 * Date:        September 17, 2015
 * Compiler:    XC16 v1.23
 *
 *
 */

#include "app.h"
#include "config.h"
#include "timers.h"

extern rn4020_appdata_t rn4020_appdata;

//Switches state machine

void Switch_Tasks(void)
{
    //Check if switches have changed and debounce timers are expired
    if (rn4020_appdata.sw1Changed && rn4020_timerdone(TMR_SW1_DEBOUNCE)) {

        rn4020_appdata.sw1Changed = false;      //clear individual flag
        rn4020_appdata.sendSwitches = true;     //set group flag to request TX
    }
    if (rn4020_appdata.sw2Changed && rn4020_timerdone(TMR_SW2_DEBOUNCE)) {

        rn4020_appdata.sw2Changed = false;
        rn4020_appdata.sendSwitches = true;
    }
    if (rn4020_appdata.sw3Changed && rn4020_timerdone(TMR_SW3_DEBOUNCE)) {

        rn4020_appdata.sw3Changed = false;
        rn4020_appdata.sendSwitches = true;
    }
    if (rn4020_appdata.sw4Changed && rn4020_timerdone(TMR_SW4_DEBOUNCE)) {

        rn4020_appdata.sw4Changed = false;
        rn4020_appdata.sendSwitches = true;
    }
}

//Change notification interrupt
//Process and start debounce timers for switch changes
//The switches are well debounced in hardware
//Adding the software debounce limits unneeded switch update messages
//and groups together multiple switch presses that occur within the debounce period

void _CNInterrupt(void)
{}
