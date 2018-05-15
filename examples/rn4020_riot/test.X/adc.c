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
 * File:        adc.c
 * Date:        January 20, 2015
 * Compiler:    XC16 v1.23
 * modified for MCP3208 device Nov 2016
 *
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "spi.h"
#include "app.h"
#include "config.h"
#include "timers.h"

extern APP_DATA appData;
extern ADC_DATA adcData;

/******************************************************************************
 * Function:        void ADC_Init()
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine initializes the ADC 
 *                 
 * Note:
 *
 *****************************************************************************/

void ADC_Init(void)
{
	adcData.mcp3208_cmd.ld = 0; // clear the command word
	adcData.chan = 0;
	adcData.mcp3208_cmd.map.start_bit = 1;
	adcData.mcp3208_cmd.map.single_diff = 1;
	adcData.mcp3208_cmd.map.index = 0; // channel
	appData.ADCcalFlag = true;
	SPI_CS0 = 1;
}

//State machine for restarting ADC and taking new readings from pot
//Returns true when SPI data has been returned from the mpc3208; false otherwise

bool ADC_Tasks(void)
{
	static uint8_t count = 0;

	/* send the command sequence to the adc */
	if (!adcData.mcp3208_cmd.map.in_progress) {
		adcData.mcp3208_cmd.map.in_progress = true;
		adcData.mcp3208_cmd.map.finish = false;
		count = 0;
		if (!SPI_IsTxData()) {
			adcData.mcp3208_cmd.map.single_diff = 1;
			adcData.mcp3208_cmd.map.index = adcData.chan;
			SPI_ClearBufs(); // dump the spi buffers
			SPI_WriteTxBuffer(adcData.mcp3208_cmd.bd[2]);
			SPI_WriteTxBuffer(adcData.mcp3208_cmd.bd[1]);
			SPI_WriteTxBuffer(adcData.mcp3208_cmd.bd[0]);
			SPI_CS0 = 0; // select the ADC
			SPI_Speed(0);
			SPI_TxStart();
		}
		return false;
	}

	/* read the returned spi data from the buffer and format it */
	if (adcData.mcp3208_cmd.map.in_progress) {
		while (SPI_IsNewRxData()) {
			switch (count) {
			case 1:
				adcData.potValue = (SPI_ReadRxBuffer()&0x0f) << 8;
				break;
			case 2:
				adcData.potValue += SPI_ReadRxBuffer();
				adcData.mcp3208_cmd.map.finish = true;
				break;
			default:
				SPI_ReadRxBuffer(); // eat extra bytes
				break;
			}
			count++;
		}
	}

	/* cleanup for next time */
	if (adcData.mcp3208_cmd.map.finish) {
		adcData.mcp3208_cmd.map.in_progress = false;
		appData.accumReady = true;
		return true;
	}

	return false;
}

//Process the accumulator value once it is ready
//And update stored potentiometer values

void ADC_ProcAccum(void)
{
	appData.potValueOld = appData.potValue; //Save previous value
	appData.potValue = adcData.potValue;
}

//ADC ISR

void _ADC1Interrupt(void)
{
	//Accumulation complete

}

void GetNewADC_Chan(void)
{
	adcData.chan = appData.receive_packet[9] == '1' ? 1 : 0; // update adc channel 
	adcData.chan += appData.receive_packet[11] == '1' ? 2 : 0;
	adcData.chan += appData.receive_packet[13] == '1' ? 4 : 0;
}
