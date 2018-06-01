/* Microchip Technology Inc. and its subsidiaries.  You may use this software
 * and any derivatives exclusively with Microchip products.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
 * TERMS.
 */

/*
 * File:
 * Author:
 * Comments:
 * Revision history:
 */

// This is a guard condition so that contents of this file are not included
// more than once.
#ifndef XC_HEADER_TEMPLATE_H
#define XC_HEADER_TEMPLATE_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "link.h"

typedef enum {
    AUTOMIO_INITIALIZE = 0,         // Initialize application
    AUTOMIO_INITIALIZATION_ERROR,   // Initialization Error
} AUTOMIO_STATE_T;

typedef struct {
    AUTOMIO_STATE_T state;      //AUTOMIO_Tasks state
    uint8_t digital_packet[8];  // digital value storage
    uint16_t analog_packet[8];  // analog value storage
} AIO_DATA;

enum {
    BLE_DIGITAL_INACTIVE = 0,
    BLE_DIGITAL_ACTIVE,
    BLE_DIGITAL_TRI_STATE,
    BLE_DIGITAL_OUTPUT_STATE
};

struct ble_automio_gpio {
    uint32_t pin_number;
    bool output;
    bool default_value;
};

struct ble_automation_io {

    uint8_t value_digital;

    struct ble_automio_gpio gpio;
};

#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */

// TODO If C++ is being used, regular C code needs function names to have C
// linkage so the functions can be used by the c code.

#ifdef  __cplusplus
}
#endif  /* __cplusplus */

#endif  /* XC_HEADER_TEMPLATE_H */
