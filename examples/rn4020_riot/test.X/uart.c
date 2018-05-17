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
 * File:        uart.c
 * Date:        January 20, 2015
 * Compiler:    XC16 v1.23
 *
 * Uart functions
 * 
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "periph/uart.h"
#include "periph/gpio.h"
#include "uart.h"
#include "config.h"
#include "tsrb.h"

/* UART receive buffer type */

typedef struct {
	char buffer[SIZE_RxBuffer];
} UART_RX_BUFFER_T;

/* UART transmit buffer type */

typedef struct {
	char buffer[SIZE_TxBuffer];
} UART_TX_BUFFER_T;

/* Buffer instances */
static UART_RX_BUFFER_T rxBuf;
static UART_TX_BUFFER_T txBuf;

tsrb_t rn4020_rx, rn4020_tx;

/* serial #1 interrupt received data callback processing */
void _rx_cb1(void *data, uint8_t c)
{
	(void) data;
	/* Put received byte in the buffer */
	tsrb_add_one(&rn4020_rx, (char) c);
}

/* 
// Initialize the UART to communicate with the Bluetooth module */

void UART_Init(void)
{
	tsrb_init(&rn4020_rx, rxBuf.buffer, SIZE_RxBuffer);
	tsrb_init(&rn4020_tx, txBuf.buffer, SIZE_TxBuffer);
	uart_init(1, 115200, _rx_cb1, &rxBuf.buffer[0]);
}

/* 
// See if there are one or more bytes in the receive buffer */

bool UART_IsNewRxData(void)
{
	if (tsrb_empty(&rn4020_rx)) {
		return(false);
	} else {
		return(true); /* There are bytes in the buffer */
	}
}

/* 
// Read a byte from the receive buffer */

uint8_t UART_ReadRxBuffer(void)
{
	return(uint8_t) tsrb_get_one(&rn4020_rx);
}

/* 
// Write a byte to the transmit buffer */

void UART_WriteTxBuffer(const uint8_t TxByte)
{
	tsrb_add_one(&rn4020_tx, (char) TxByte);
}

/* 
// Return the number of bytes free in the TX buffer */

uint16_t UART_GetTXBufferFreeSpace(void)
{
	return(uint16_t) tsrb_free(&rn4020_tx);
}


uint8_t UART_PeekRxBuffer(void)
{
	return(0); //No bytes in the buffer so return 0

}

