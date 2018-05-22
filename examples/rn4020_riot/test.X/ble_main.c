/*
 * PIC32MZ EF Curiosity Development Board, RIOT-OS RN4020 BLE testing
 */

#include <stdio.h>
#include <string.h>
#include "periph/uart.h"
#include "periph/gpio.h"
#include "config.h"
#include "app.h"
#include "mrf24.h"

extern void set_cache_policy(uint32_t);

int main(void)
{
	void initBoard(void);

	/* testing cache modes */
	set_cache_policy(WB_WA);
	LED3_ON;
	/*
	 * setup debug serial ports #4 @115200 bps
	 */
	uart_init(4, DEBUG_UART_BAUD, NULL, 0);
	printf("\r\n rn4020 app\r\n");

	while (1) {
		APP_Tasks();
		LED3_OFF;
		LED2_ON;
		mrf24f_testing();
	}

	return 0;
}

void initBoard(void)
{
	// LEDs are outputs and off
	RELAY1 = 1;
	RELAY2 = 1;
	RELAY3 = 1;
	RELAY4 = 1;

	//RN4020 module - UART1
	BT_WAKE_HW = 1; //Dormant line is set high
	BT_WAKE_HW_TRIS = 0; //Dormant line is output

	BT_WAKE_SW = 0; //keep low until after UART is initialized
	BT_WAKE_SW_TRIS = 0;

	BT_CMD = 0; //Command mode on
	BT_CMD_TRIS = 0;

	BT_WS_TRIS = 1;
	BT_MLDP_EV_TRIS = 1;
	BT_CONNECTED_TRIS = 1;

	SWITCH1_TRIS = 1;

	U1RX_TRIS = 1;
	U1TX_TRIS = 0;
	U1CTS_TRIS = 1;
	U1RTS_LAT = 0;
	U1RTS_TRIS = 0;

	// SPI Master Devices
	SPI_CS0_TRIS = 0;
	SPI_CS1_TRIS = 0;
	SPI_CS2_TRIS = 0;

	RF24F_INT_TRIS = 1; /* slave interrupt request */
	RF24F_CS = 1; /* device select */
	RF24F_CS_TRIS = 0;

	RF24F_SLEEP = 1; /* doze off */
	RF24F_SLEEP_TRIS = 0;
}
