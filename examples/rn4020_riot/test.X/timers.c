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
 * File:        timers.c
 * Date:        January 20, 2015
 * Compiler:    XC16 v1.23
 *
 * Timer functions
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "thread.h"
#include "config.h"
#include "timers.h"
#include "app.h"

extern rn4020_appdata_t rn4020_appdata;

static volatile uint16_t tickCount[TMR_COUNT] = { 0 };

void rn4020_timers_init(void)
{
    /* Timer 1 is used for interrupt based software timers
     * counting 1ms intervals to a resolution of 500us
     */
    T1CON = TIMER_OFF;
    TMR1 = 0;
    PR1 = TIMER_500US_PERIOD;
    T1CON |= TIMER_ON_PRESCALE1;
    IFS0bits.T1IF = 0;
    IPC1bits.T1IP = 3;
    IPC1bits.T1IS = 1;
    IEC0bits.T1IE = 1;
}

void rn4020_starttimer(uint8_t timer, uint16_t count)
{
    tickCount[timer] = count << 1;
    /* Interrupt is every 500us but rn4020_starttimer()
     * takes multiple of 1ms so multiply by 2
     */
}

bool rn4020_timerdone(uint8_t timer)
{
    if (tickCount[timer] == 0) {
        return true;
    }
    return false;
}

void rn4020_wait_ms(uint16_t numMilliseconds)
{
    rn4020_starttimer(TMR_INTERNAL, numMilliseconds);
    while (!rn4020_timerdone(TMR_INTERNAL)) {}
}

/* Timer 1 interrupt routine for application software timers */
void rn4020_t1interrupt(void)
{
    uint8_t i;

    //Decrement each software timer
    for (i = 0; i < TMR_COUNT; i++) {
        if (tickCount[i] != 0) {
            tickCount[i]--;
        }
    }
}
