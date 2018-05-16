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
 * File:        config.h
 * Date:        January 20, 2015
 * Compiler:    XC16 v1.23
 *
 * General definitions for the project
 * 
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "xtimer.h"
#include "timex.h"
#include "periph/uart.h"
#include "periph/gpio.h"
#include "periph/spi.h"

#define APP_VERSION_STR "4.0"       //This firmware version
//	2.8	increase ADC sampling and message transmission rates
//	2.9	minor spelling fixes
//	3.0	Add some public service support
//	3.1	heart rate service added (demo data) makes software version 1.33.4 firmware dependant
//	3.2	add automation io service
//	4.0	riot-os port

/*******************************************************************************
 * Application settings - these will change application behavior
 ******************************************************************************/

//Selectively set PMD bits if this is defined
//Disables clocking to peripherals that are not used in this demo code
//Reduces operating current by about 2 mA
//Risk is trying to add code that uses a disabled peripheral and
//forgetting that it is disabled by PMD
//The bits are selectively set in main.c --> initBoard()
//#define SET_PMD_BITS

//Enables sleep mode based on an inactivity timer.
//Switch presses and potentiometer changes will reset inactivity timer window.
//D6 will go solid on when device first goes to sleep and then will
//pulse periodically during sleep.
//Any switch press will wake device from sleep.
//Reduces power consumption but might not be wanted, depending on use case
//Sleep mode will be enabled when this is defined
//Inactivity time-out is set below in Application timers section
//#define USE_SLEEP

//Determines mode of sleep operation, to illustrate different possible methods.
//When defined, sleep mode will clock switch to the LPRC for the system clock
//and will use the RTCC (clocked by LPRC) for periodic wake-up to flash the status LED.
//If not defined, sleep mode will use Timer 1 (clocked by LPRC) for periodic wakeup
//to flash the status LED. Each mode has its strengths.
//If timing accuracy is required using RTCC mode, the RTCC should be clocked
//from the external crystal on SOSC instead.
//Setting has no effect if USE_SLEEP is not defined.
//#define SLEEP_MODE_RTCC

//A change in potentiometer reading greater than this will reset the sleep inactivity timer
//A threshold greater than 1 prevents potentiometer noise / ADC drift from keeping board awake
//Has no effect if sleep is disabled
#define POT_KEEP_AWAKE_DELTA    5               //ADC count delta after oversampling, averaging, and 10-bit conversion

//Enable / Disable the MCP1642B 5V boost power supply for 5V power pin on MikroBUS header
//Set to 0 to disable; 1 to enable (Enable this for 5V Click Boards)

//*!*!*!*!*!*!*!*!*!*!*!*!*   NOTE   *!*!*!*!*!*!*!*!*!*!*!*!*
//If using a 5V Click Board with analog output, also set S7 on the PCB to the "5V"
//setting to enable the analog voltage divider which will scale the output to 0 - 3.3V
//#define MCP1642B_EN    0
#define MCP1642B_EN    1

// UART baud rate - RN module defaults to 115200
#define BRG_115200 (FCY/(4*115200) - 1)        //BRG value for 115,200 baud with BRGH = 1 (with rounding)

//Comparator voltage reference CVR setting
#define CVR_BITS            18                 //Determines base voltage threshold for low battery indication

//Number of samples to average for potentiometer ADC reading, in addition to hardware oversampling
#define ADC_NUM_AVGS        5

//If defined, the RN4020's firmware version will be checked as part of initialization.
//If the version is not equal to the version specified below, the board will
//error out with the initialization error code display and not operate
#define VERIFY_RN_FW_VER
#define RN_FW_VER_MAJOR     1           //Require 1.23.5
#define RN_FW_VER_MINOR     23          //These values can be changed as needed
#define RN_FW_VER_PATCH     5

#define RN_FW_VER_MAJOR133     1 
#define RN_FW_VER_MINOR133     33 
#define RN_FW_VER_PATCH133     4

//Application timers
#define SLEEP_TIME          TIMER_5MIN_PERIOD_PS256     //inactivity timer for sleep - applies only when USE_SLEEP is defined
#define DEBOUNCE_MS         75          //debounce time for switches 1 - 4
#define ADC_REFRESH_MS      10           //delay between ADC reads, 10 nom value
#define POT_TX_MS           50         //delay between transmitting new pot values, 10 min value
#define BATT_TX_MS	    100 
#define HR_TX_MS            80
#define AIO_TX_MS           40 
#define LED_BLINK_MS        900         //LED blink rate for advertise mode
#define BT_TX_MS            10         //minimum time between consecutive BTLE message transmissions
#define BAT_CHK_DELAY_MS    30000       //delay between input voltage checks
#define BAT_CHK_WAIT_MS     10          //CVref & CMP stabilization time
//Periods for timer 1 sleep mode (for periodic sleep wakeup); 31KHz LPRC; 1:256 prescale
#define T1_WAKE_PERIOD      2       //2 ~= 16 ms
#define T1_SLEEP_PERIOD     605     //605 ~= 5 seconds

//Buffer sizes
#define SIZE_RxBuffer   256               //UART RX software buffer size in bytes
#define SIZE_TxBuffer   256               //UART TX software buffer size in bytes
#define SIZE_SPI_Buffer 64

#define BT_RX_PKT_SZ    100               //Max receive packet length
#define BT_TX_PKT_SZ    100               //Max transmit packet length

#define    ESP_GATT_PERM_READ                  (1 << 0)   /* bit 0 -  0x0001 */    /* relate to BTA_GATT_PERM_READ in bta_gatt_api.h */
#define    ESP_GATT_PERM_READ_ENCRYPTED        (1 << 1)   /* bit 1 -  0x0002 */    /* relate to BTA_GATT_PERM_READ_ENCRYPTED in bta_gatt_api.h */
#define    ESP_GATT_PERM_READ_ENC_MITM         (1 << 2)   /* bit 2 -  0x0004 */    /* relate to BTA_GATT_PERM_READ_ENC_MITM in bta_gatt_api.h */
#define    ESP_GATT_PERM_WRITE                 (1 << 4)   /* bit 4 -  0x0010 */    /* relate to BTA_GATT_PERM_WRITE in bta_gatt_api.h */
#define    ESP_GATT_PERM_WRITE_ENCRYPTED       (1 << 5)   /* bit 5 -  0x0020 */    /* relate to BTA_GATT_PERM_WRITE_ENCRYPTED in bta_gatt_api.h */
#define    ESP_GATT_PERM_WRITE_ENC_MITM        (1 << 6)   /* bit 6 -  0x0040 */    /* relate to BTA_GATT_PERM_WRITE_ENC_MITM in bta_gatt_api.h */
#define    ESP_GATT_PERM_WRITE_SIGNED          (1 << 7)   /* bit 7 -  0x0080 */    /* relate to BTA_GATT_PERM_WRITE_SIGNED in bta_gatt_api.h */
#define    ESP_GATT_PERM_WRITE_SIGNED_MITM     (1 << 8)   /* bit 8 -  0x0100 */    /* relate to BTA_GATT_PERM_WRITE_SIGNED_MITM in bta_gatt_api.h */

/* relate to BTA_GATT_CHAR_PROP_BIT_xxx in bta_gatt_api.h */
/* definition of characteristic properties */
#define    ESP_GATT_CHAR_PROP_BIT_BROADCAST    (1 << 0)       /* 0x01 */    /* relate to BTA_GATT_CHAR_PROP_BIT_BROADCAST in bta_gatt_api.h */
#define    ESP_GATT_CHAR_PROP_BIT_READ         (1 << 1)       /* 0x02 */    /* relate to BTA_GATT_CHAR_PROP_BIT_READ in bta_gatt_api.h */
#define    ESP_GATT_CHAR_PROP_BIT_WRITE_NR     (1 << 2)       /* 0x04 */    /* relate to BTA_GATT_CHAR_PROP_BIT_WRITE_NR in bta_gatt_api.h */
#define    ESP_GATT_CHAR_PROP_BIT_WRITE        (1 << 3)       /* 0x08 */    /* relate to BTA_GATT_CHAR_PROP_BIT_WRITE in bta_gatt_api.h */
#define    ESP_GATT_CHAR_PROP_BIT_NOTIFY       (1 << 4)       /* 0x10 */    /* relate to BTA_GATT_CHAR_PROP_BIT_NOTIFY in bta_gatt_api.h */
#define    ESP_GATT_CHAR_PROP_BIT_INDICATE     (1 << 5)       /* 0x20 */    /* relate to BTA_GATT_CHAR_PROP_BIT_INDICATE in bta_gatt_api.h */
#define    ESP_GATT_CHAR_PROP_BIT_AUTH         (1 << 6)       /* 0x40 */    /* relate to BTA_GATT_CHAR_PROP_BIT_AUTH in bta_gatt_api.h */
#define    ESP_GATT_CHAR_PROP_BIT_EXT_PROP     (1 << 7)       /* 0x80 */    /* relate to BTA_GATT_CHAR_PROP_BIT_EXT_PROP in bta_gatt_api.h */


//BTLE services
#define PRIVATE_SERVICE			"28238791ec55413086e0002cd96aec9d"
#define PRIVATE_SERVICE_SPI		"8ee15902ee6f49dc9cfb5c4c2eff6057"
#define PRIVATE_CHAR_SWITCHES		"8f7087bdfdf34b87b10fabbf636b1cd5"
#define PRIVATE_CHAR_POTENTIOMETER	"362232e5c5a94af6b30ce208f1a9ae3e"
#define PRIVATE_CHAR_LEDS		"cd8306093afa4a9da58b8224cd2ded70"
#define PRIVATE_CHAR_RELAYS		"cd83060a3afa4a9da58b8224cd2ded70"
#define PRIVATE_CHAR_ADC_CHAN		"cd83060b3afa4a9da58b8224cd2ded70"
#define PRIVATE_CHAR_PIC_SLAVE		"cd83060c3afa4a9da58b8224cd2ded70"

// Battery
#define PUBLIC_BATT_UUID       "180F" // Battery level service
#define PUBLIC_BATT_CHAR_BL    "2A19"

// Heartbeat
#define PUBLIC_HR_UUID         "180D" // Heart Rate service
#define PUBLIC_HR_CHAR_HRM     "2A37" // Heart Rate Measurement
#define PUBLIC_HR_CHAR_BSL     "2A38" // Heart body sensor location
#define PUBLIC_HR_CHAR_RCP     "2A39" // Heart rate control point
#define PUBLIC_AIO_UUID		"1815" // Automation IO service 
#define PUBLIC_AIO_CHAR_DIG     "2A56" // Automation IO digital
#define PUBLIC_AIO_CHAR_ANA     "2A58" // Automation IO analog
#define PUBLIC_AIO_CHAR_AGG     "2A5A" // Automation IO Aggregate

// handles that change with added services and characteristics
// manually parse the LS command for UUID handles

#define PUBLIC_HR_CHAR_HRM_H		"001B"
#define PUBLIC_HR_CHAR_HRM_C		"001C"
#define PUBLIC_HR_CHAR_BSL_H		"001E"
#define PUBLIC_HR_CHAR_RCP_H		"0020"
#define PUBLIC_AIO_CHAR_DIG_H		"0023" 
#define PUBLIC_AIO_CHAR_ANA_H		"0026"
#define PUBLIC_AIO_CHAR_AGG_H		"0029"
#define PUBLIC_AIO_CHAR_AGG_C		"002A"
#define PUBLIC_BATT_CHAR_H		"003C"
#define PUBLIC_BATT_CHAR_C		"003D"
#define PRIVATE_CHAR_SWITCHES_H		"002D"
#define PRIVATE_CHAR_SWITCHES_C		"002E"
#define PRIVATE_CHAR_POTENTIOMETER_H	"0030"
#define PRIVATE_CHAR_POTENTIOMETER_C	"0031"
#define PRIVATE_CHAR_LEDS_H		"0033"
#define PRIVATE_CHAR_RELAYS_H		"0035"
#define PRIVATE_CHAR_ADC_CHAN_H		"0037"
#define PRIVATE_CHAR_PIC_SLAVE_H	"0039"

//attribute for ISRs that do not alter PSV registers
#define _ISR_NO_AUTO_PSV __attribute__((interrupt,no_auto_psv))

// code copied from the ESP device lib

// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

typedef struct {
#define ESP_UUID_LEN_16     2
#define ESP_UUID_LEN_32     4
#define ESP_UUID_LEN_128    16
	uint16_t len; /*!< UUID length, 16bit, 32bit or 128bit */

	union {
		uint16_t uuid16;
		uint32_t uuid32;
		uint8_t uuid128[ESP_UUID_LEN_128];
	} uuid; /*!< UUID */
} __attribute__((packed)) esp_bt_uuid_t;

typedef struct {
	esp_bt_uuid_t uuid; /*!< UUID */
	uint8_t inst_id; /*!< Instance id */
} __attribute__((packed)) esp_gatt_id_t;

/**
 * @brief Gatt service id, include id
 *        (uuid and instance id) and primary flag
 */
typedef struct {
	esp_gatt_id_t id; /*!< Gatt id, include uuid and instance */
	bool is_primary; /*!< This service is primary or not */
} __attribute__((packed)) esp_gatt_srvc_id_t;

typedef uint16_t esp_gatt_perm_t;
typedef uint8_t esp_gatt_char_prop_t;

/**
 * @brief set the attribute value type
 */
typedef struct {
	uint16_t attr_max_len; /*!<  attribute max value length */
	uint16_t attr_len; /*!<  attribute current value length */
	uint8_t *attr_value; /*!<  the pointer to attribute value */
} esp_attr_value_t;

struct gatts_service_inst {
	uint16_t gatts_if;
	uint16_t app_id;
	uint16_t conn_id;
	uint16_t service_handle;
	esp_gatt_srvc_id_t service_id;
	uint16_t num_handles;
};

struct gatts_char_inst {
	uint32_t service_pos;
	esp_gatt_perm_t char_perm;
	char char_property[8];
	esp_attr_value_t *char_val;
	char char_handle[16];
	char char_nvs[16];
};

/*******************************************************************************
 * End application configuration settings
 * Hardware settings below
 ******************************************************************************/

// Clock frequency
#define FCY (50000000)                              //8MHz FRC, 4XPLL, /2 for Fcy

//RN4020 BTLE
#define BT_OTA_UPD	PORTEbits.RE9			// update jumper on board, if pin is low start update routine., OK
#define BT_OTA_UPD_TRIS	TRISEbits.TRISE9

#define BT_WAKE_HW      LATDbits.LATD1                       //Hardware wake from dormant state; BT_WAKE_HW, OK
#define BT_WAKE_HW_TRIS TRISDbits.TRISD1

#define BT_WAKE_SW      LATAbits.LATA9                       //Deep sleep wake; BT_WAKE_SW, OK
#define BT_WAKE_SW_TRIS TRISAbits.TRISA9

#define BT_CMD      LATEbits.LATE8                 //Place RN4020 module in command mode, low for MLDP mode, OK
#define BT_CMD_TRIS TRISEbits.TRISE8

#define BT_CONNECTED        PORTBbits.RB4                     //RN4020 module is connected to central device, OK
#define BT_CONNECTED_TRIS   TRISBbits.TRISB4

#define BT_WS       PORTDbits.RD14                         //RN4020 module is awake and active, OK
#define BT_WS_TRIS  TRISDbits.TRISD14

#define BT_MLDP_EV      PORTDbits.RD3                         //RN4020 module in MLDP mode has a pending event, NC, OK
#define BT_MLDP_EV_TRIS TRISDbits.TRISD3

//UART
#define U1CTS_TRIS      TRISAbits.TRISA14
#define U1CTS_PORT	PORTAbits.RA14

#define U1RTS_TRIS      TRISAbits.TRISA15
#define U1RTS_LAT       LATAbits.LATA15

// RELAY outputs
#define RELAY1	LATBbits.LATB3 // output 0 (low) turns on relay
#define RELAY2	LATBbits.LATB3
#define RELAY3	LATBbits.LATB3
#define RELAY4	LATBbits.LATB3

// LED outputs

#define SLED LED4R_TOGGLE
#define SLED_ON		LED4R_ON	
#define SLED_OFF	LED4R_OFF

#define SPI_CS0 LATDbits.LATD5
#define SPI_CS1 LATAbits.LATA1

#define SPI_CS0_TRIS	TRISDbits.TRISD5
#define SPI_CS1_TRIS	TRISAbits.TRISA1

//Timer initialization
#define TIMER_OFF 0
#define TIMER_ON_PRESCALE1      0x8000
#define TIMER_ON_PRESCALE8      0x8010
#define TIMER_ON_PRESCALE64     0x8020
#define TIMER_ON_PRESCALE256    0x8030

//Timer periods
//32-bit mode with 1:256 postscale below
#define TIMER_5MIN_PERIOD_PS256 ((uint32_t)((FCY / 256) * 300 - 1))
#define TIMER_1MIN_PERIOD_PS256 ((uint32_t)((FCY / 256) * 60 - 1))
#define TIMER_10S_PERIOD_PS256  ((uint32_t)((FCY / 256) * 10 - 1))
#define TIMER_1S_PERIOD_PS256   ((uint16_t)(FCY / 256 - 1))

//16-bit mode with 1:1 postscale below
#define TIMER_1MS_PERIOD        ((uint16_t)(FCY / 1000 - 1))
#define TIMER_100US_PERIOD      ((uint16_t)(FCY / 10000 - 1))
#define TIMER_500US_PERIOD      ((uint16_t)(FCY / 2000 - 1))

#endif //CONFIG_H
