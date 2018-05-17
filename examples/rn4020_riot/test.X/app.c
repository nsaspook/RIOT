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

APP_DATA appData;
ADC_DATA adcData;
AIO_DATA aioData;

/* Primary application state machine */

void APP_Tasks(void)
{
	/* Update LED outputs */
	LED_Tasks();

	switch (appData.state) {
		/* Initial state */
	case APP_INITIALIZE:
		if (APP_Initialize()) {
			appData.state = APP_BLUETOOTH_ADVERTISE;
		} else {
			appData.state = APP_INITIALIZATION_ERROR;
		}
		break;

		/* Initialization failed */
	case APP_INITIALIZATION_ERROR:
		LED_SET_LightShow(LED_ERROR);
		break;

		/* We're not connected to a device - advertise mode */
	case APP_BLUETOOTH_ADVERTISE:
		LED_SET_LightShow(LED_BTLE_ADVERTISING);
		if (BT_CONNECTED) {
			appData.state = APP_BLUETOOTH_PAIRED;
		}
		break;

		/* We are connected to a BTLE device */
	case APP_BLUETOOTH_PAIRED:
		/* Update LEDs */
		LED_SET_LightShow(LED_BTLE_PAIRED);
		/* Check to see if we are still connected; return to advertise state if not */
		if (!BT_CONNECTED) {
			appData.update_packet = false;
			LED_SET_LightShow(LED_BTLE_ADVERTISING);
			appData.state = APP_BLUETOOTH_ADVERTISE;
			G_LED_OFF;
			break;
		}

		/* Check if switches have changed and debounce timers are expired */
		Switch_Tasks();
		if (appData.sendSwitches) { //New switch status to send?
			sprintf(appData.transmit_packet, "shw,"PRIVATE_CHAR_SWITCHES_H",%d%d%d%d\r", appData.sw1, appData.sw2, appData.sw3, appData.sw4);
			if (BT_SendCommand(appData.transmit_packet, true)) {
				appData.sendSwitches = false;
			}
		}

		/* Process ADC accumulator value if oversampling is complete */
		if (appData.accumReady) {
			ADC_ProcAccum();
			appData.accumReady = false; //Clear app flags
			appData.ADCinUse = false;
		}

		/*
		 *  packet transmission queue, TimerDone puts packet in the transmission stream
		 */

		/* Start new ADC read if timer expired, not currently sampling, and not waiting to process accumulator */
		if (TimerDone(TMR_ADC) && appData.ADCinUse == false) {
			if (ADC_Tasks()) {
				StartTimer(TMR_ADC, ADC_REFRESH_MS);
			}
		}

		/* Transmit new potentiometer reading? */
		if (TimerDone(TMR_POT)) {
			/* Send message only if pot value has changed */
			if (appData.potValue != appData.potValueLastTX) {
				sprintf(appData.transmit_packet, "shw,"PRIVATE_CHAR_POTENTIOMETER_H",%04d\r\n", appData.potValue);
				if (BT_SendCommand(appData.transmit_packet, true)) {
					appData.potValueLastTX = appData.potValue;
					StartTimer(TMR_POT, POT_TX_MS);
				}
			} else {
				StartTimer(TMR_POT, POT_TX_MS);
			} /* value not changed - skip this transmission */
		}

		if (TimerDone(TMR_BATT)) {
			sprintf(appData.transmit_packet, "suw,"PUBLIC_BATT_CHAR_BL",%02d\r", (appData.potValue >> 6)&0b00111111);
			if (BT_SendCommand(appData.transmit_packet, true)) {
				StartTimer(TMR_BATT, BATT_TX_MS);
			}
		}

		if (TimerDone(TMR_HR)) {
			sprintf(appData.transmit_packet, "suw,"PUBLIC_HR_CHAR_HRM",%02x%02x%02x%02x\r", 0x08, (appData.potValue >> 4)&0xff, appData.hrmEnergy & 0x00ff, appData.hrmEnergy >> 8); // format mask and ADC data
			if (BT_SendCommand(appData.transmit_packet, true)) {
				StartTimer(TMR_HR, HR_TX_MS);
				sprintf(appData.transmit_packet, "suw,"PUBLIC_HR_CHAR_BSL",%02x\r", 3);
				BT_SendCommand(appData.transmit_packet, false);
				appData.hrmEnergy++;
			}
		}

		if (TimerDone(TMR_AIO_DIG)) {
			//Form message
			sprintf(appData.transmit_packet, "shw,"PUBLIC_AIO_CHAR_DIG_H",0101010101010101\r"); /* digital data */
			//Try to transmit the message; reset timer if successful
			if (BT_SendCommand(appData.transmit_packet, true)) {
				StartTimer(TMR_AIO_DIG, AIO_TX_MS);
			}
		}

		/* Process any new messages received from RN module */
		appData.got_packet = BT_ReceivePacket(appData.receive_packet); /* Get new message if one has been received from the RN4020 */
		if (appData.got_packet == true) { /* true if new packet received */
			if (strstr(appData.receive_packet, "WV,"PRIVATE_CHAR_LEDS_H",")) { /* Check for LED update message 1.33 */
				GetNewLEDs(); //Latch new LED values
				appData.update_packet = true;
			}
			/*
			//Other message handling can be added here
			*/
			/* receive new SPI ADC channel */
			if (strstr(appData.receive_packet, "WV,"PRIVATE_CHAR_ADC_CHAN_H",")) {
				GetNewADC_Chan(); /* new ADC config data */
			}
			/* receive new SPI SLAVE request */
			if (strstr(appData.receive_packet, "WV,"PRIVATE_CHAR_PIC_SLAVE_H",")) {

			}
			/* HRM energy expended reset */
			if (strstr(appData.receive_packet, "WV,"PUBLIC_HR_CHAR_RCP_H",01.")) {
				appData.hrmEnergy = 0;
			}
			/* receive new BATTERY request */
			if (strstr(appData.receive_packet, "RV,"PUBLIC_BATT_CHAR_H".")) {
				sprintf(appData.transmit_packet, "suw,"PUBLIC_BATT_CHAR_BL",%d\r", 63);
				BT_SendCommand(appData.transmit_packet, false);
			}
		}
		break;

	default:
		break;
	} /* end switch(appData.state) */
} /* end APP_Tasks() */

//Sets up the RN module

bool APP_Initialize(void)
{
	/****************************************************************************
	 * Initialize appData structure
	 ***************************************************************************/
	appData.error_code = ERROR_NONE;
	appData.got_packet = false;
	appData.potValue = 0;
	appData.potValueOld = 0xFFFF;
	appData.potValueLastTX = 0xFFFF;
	appData.state = APP_INITIALIZE;
	appData.sw1 = false;
	appData.sw2 = false;
	appData.sw3 = false;
	appData.sw4 = false;
	appData.led1 = 0;
	appData.led2 = 0;
	appData.led3 = 0;
	appData.led4 = 0;
	appData.led5 = 0;
	appData.led6 = 0;
	appData.update_packet = true;
	appData.sw1Changed = false;
	appData.sw2Changed = false;
	appData.sw3Changed = false;
	appData.sw4Changed = false;
	appData.sendSwitches = false;
	appData.ADCcalFlag = false;
	appData.sleepFlag = false;
	appData.RTCCalarm = false;
	appData.accumReady = false;
	appData.ADCinUse = false;
	appData.timer1Flag = false;

	/****************************************************************************
	 * Peripherals Init
	 ***************************************************************************/
	ADC_Init();
	UART_Init();
	Timers_Init();
	SPI_Init();

	B_LED_ON;
	BT_WAKE_SW = 1; //wake module
	/* Wait for WS status high */
	StartTimer(TMR_RN_COMMS, 4000);
	while (BT_WS == 0) {
		if (TimerDone(TMR_RN_COMMS))
		{
			appData.error_code = ERROR_INITIALIZATION;
			return false;
		}
	}

	B_LED_OFF;
	G_LED_ON;

	/* Wait for end of "CMD\r\n" - we don't check for full "CMD\r\n" string because we may 
	//miss some bits or bytes at the beginning while the UART starts up */
	StartTimer(TMR_RN_COMMS, 4000);
	while (UART_ReadRxBuffer() != '\n') {
		if (TimerDone(TMR_RN_COMMS))
		{
			appData.error_code = ERROR_INITIALIZATION;
			return false;
		}
	}

	/* Module is now in command mode and ready for input */
	if (!BT_SetupModule()) { /* Setup RN4020 module */
		appData.error_code = ERROR_INITIALIZATION;
		return false;
	}


#ifdef VERIFY_RN_FW_VER
	if (!(appData.version_code = BT_CheckFwVer())) {
		appData.error_code = ERROR_RN_FW;
		return false;
	}
#endif

	/* flush UART RX buffer as a precaution before starting app state machine */
	while (UART_IsNewRxData()) {
		UART_ReadRxBuffer();
		if (!UART_IsNewRxData()) {
			WaitMs(100);
		}
	}

	SLED; /* init completed */
	G_LED_OFF;
	return true;
}
