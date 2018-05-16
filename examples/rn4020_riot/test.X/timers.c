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

extern APP_DATA appData;

static volatile uint16_t tickCount[TMR_COUNT] = {0};

void Timers_Init(void)
{
	//Timer 1 is used for interrupt based software timers counting 1ms intervals to a resolution of 500us
	T1CON = TIMER_OFF; //Timer 1 off
	TMR1 = 0; //Clear timer 1
	PR1 = TIMER_500US_PERIOD; //Set the period value for 500us
	T1CON |= TIMER_ON_PRESCALE1; //using 1:1 prescaler and turn on timer 1
	IFS0bits.T1IF = 0; //Clear the interrupt flag
	IPC1bits.T1IP = 1; /* DMA interrupt priority. */
	IPC1bits.T1IS = 0; /* DMA sub-priority. */
	IEC0bits.T1IE = 1; //Enable the timer 1 interrupt
}

void StartTimer(uint8_t timer, uint16_t count)
{
	tickCount[timer] = count << 1; //Interrupt is every 500us but StartTimer() takes multiple of 1ms so multiply by 2
}

bool TimerDone(uint8_t timer)
{
	if (tickCount[timer] == 0) { //Check if counted down to zero
		return true; //then return true
	}
	return false; //else return false
}

void WaitMs(uint16_t numMilliseconds)
{
	StartTimer(TMR_INTERNAL, numMilliseconds); //Start software timer and wait for it to count down
	while (!TimerDone(TMR_INTERNAL)) {
	}
}

/* Timer 1 interrupt routine for application software timers */

void _T1Interrupt(void)
{
	uint8_t i;

	//Decrement each software timer
	for (i = 0; i < TMR_COUNT; i++) {
		if (tickCount[i] != 0) {
			tickCount[i]--;
		}
	}
}

