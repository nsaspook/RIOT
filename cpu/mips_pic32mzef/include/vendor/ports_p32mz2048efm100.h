/* Created by plibgen $Revision: 1.31 $ */

#ifndef _PORTS_P32MZ2048EFM100_H
#define _PORTS_P32MZ2048EFM100_H

/* Section 1 - Enumerate instances, define constants, VREGs */

#include <stdbool.h>

typedef enum {

    PORTS_ID_0              = 0,
    PORTS_NUMBER_OF_MODULES = 1

} PORTS_MODULE_ID;

typedef enum {

    PORTS_PIN_MODE_ANALOG   = 0,
    PORTS_PIN_MODE_DIGITAL  = 1

} PORTS_PIN_MODE;

typedef enum {

    PORTS_CHANGE_NOTICE_PIN_NONE

} PORTS_CHANGE_NOTICE_PIN;

typedef enum {

    PORTS_CN_PIN_NONE

} PORTS_CN_PIN;

typedef enum {

    PORTS_ANALOG_PIN_0  = 16,
    PORTS_ANALOG_PIN_1  = 17,
    PORTS_ANALOG_PIN_2  = 18,
    PORTS_ANALOG_PIN_3  = 19,
    PORTS_ANALOG_PIN_4  = 20,
    PORTS_ANALOG_PIN_5  = 26,
    PORTS_ANALOG_PIN_6  = 27,
    PORTS_ANALOG_PIN_7  = 28,
    PORTS_ANALOG_PIN_8  = 29,
    PORTS_ANALOG_PIN_9  = 30,
    PORTS_ANALOG_PIN_10 = 31,
    PORTS_ANALOG_PIN_11 = 105,
    PORTS_ANALOG_PIN_12 = 104,
    PORTS_ANALOG_PIN_13 = 103,
    PORTS_ANALOG_PIN_14 = 102,
    PORTS_ANALOG_PIN_15 = 71,
    PORTS_ANALOG_PIN_16 = 70,
    PORTS_ANALOG_PIN_17 = 69,
    PORTS_ANALOG_PIN_18 = 68,
    PORTS_ANALOG_PIN_19 = 36,
    PORTS_ANALOG_PIN_20 = 35,
    PORTS_ANALOG_PIN_21 = 34,
    PORTS_ANALOG_PIN_22 = 33,
    PORTS_ANALOG_PIN_23 = 111,
    PORTS_ANALOG_PIN_24 = 0,
    PORTS_ANALOG_PIN_25 = 72,
    PORTS_ANALOG_PIN_26 = 73,
    PORTS_ANALOG_PIN_27 = 9,
    PORTS_ANALOG_PIN_28 = 10,
    PORTS_ANALOG_PIN_29 = 1,
    PORTS_ANALOG_PIN_30 = 93,
    PORTS_ANALOG_PIN_31 = 92,
    PORTS_ANALOG_PIN_32 = 62,
    PORTS_ANALOG_PIN_33 = 63,
    PORTS_ANALOG_PIN_34 = 5,
    PORTS_ANALOG_PIN_45 = 21,
    PORTS_ANALOG_PIN_46 = 22,
    PORTS_ANALOG_PIN_47 = 23,
    PORTS_ANALOG_PIN_48 = 24,
    PORTS_ANALOG_PIN_49 = 25

} PORTS_ANALOG_PIN;

typedef enum {

    PORTS_AN_PIN_NONE

} PORTS_AN_PIN;

typedef enum {

    PORTS_BIT_POS_0     = 0,
    PORTS_BIT_POS_1     = 1,
    PORTS_BIT_POS_2     = 2,
    PORTS_BIT_POS_3     = 3,
    PORTS_BIT_POS_4     = 4,
    PORTS_BIT_POS_5     = 5,
    PORTS_BIT_POS_6     = 6,
    PORTS_BIT_POS_7     = 7,
    PORTS_BIT_POS_8     = 8,
    PORTS_BIT_POS_9     = 9,
    PORTS_BIT_POS_10    = 10,
    PORTS_BIT_POS_11    = 11,
    PORTS_BIT_POS_12    = 12,
    PORTS_BIT_POS_13    = 13,
    PORTS_BIT_POS_14    = 14,
    PORTS_BIT_POS_15    = 15

} PORTS_BIT_POS;

typedef enum {

    INPUT_FUNC_INT1     = 0,
    INPUT_FUNC_INT2     = 1,
    INPUT_FUNC_INT3     = 2,
    INPUT_FUNC_INT4     = 3,
    INPUT_FUNC_T2CK     = 5,
    INPUT_FUNC_T3CK     = 6,
    INPUT_FUNC_T4CK     = 7,
    INPUT_FUNC_T5CK     = 8,
    INPUT_FUNC_T6CK     = 9,
    INPUT_FUNC_T7CK     = 10,
    INPUT_FUNC_T8CK     = 11,
    INPUT_FUNC_T9CK     = 12,
    INPUT_FUNC_IC1      = 13,
    INPUT_FUNC_IC2      = 14,
    INPUT_FUNC_IC3      = 15,
    INPUT_FUNC_IC4      = 16,
    INPUT_FUNC_IC5      = 17,
    INPUT_FUNC_IC6      = 18,
    INPUT_FUNC_IC7      = 19,
    INPUT_FUNC_IC8      = 20,
    INPUT_FUNC_IC9      = 21,
    INPUT_FUNC_OCFA     = 23,
    INPUT_FUNC_U1RX     = 25,
    INPUT_FUNC_U1CTS    = 26,
    INPUT_FUNC_U2RX     = 27,
    INPUT_FUNC_U2CTS    = 28,
    INPUT_FUNC_U3RX     = 29,
    INPUT_FUNC_U3CTS    = 30,
    INPUT_FUNC_U4RX     = 31,
    INPUT_FUNC_U4CTS    = 32,
    INPUT_FUNC_U5RX     = 33,
    INPUT_FUNC_U5CTS    = 34,
    INPUT_FUNC_U6RX     = 35,
    INPUT_FUNC_U6CTS    = 36,
    INPUT_FUNC_SDI1     = 38,
    INPUT_FUNC_SS1      = 39,
    INPUT_FUNC_SDI2     = 41,
    INPUT_FUNC_SS2      = 42,
    INPUT_FUNC_SDI3     = 44,
    INPUT_FUNC_SS3      = 45,
    INPUT_FUNC_SDI4     = 47,
    INPUT_FUNC_SS4      = 48,
    INPUT_FUNC_SDI5     = 50,
    INPUT_FUNC_SS5      = 51,
    INPUT_FUNC_SDI6     = 53,
    INPUT_FUNC_SS6      = 54,
    INPUT_FUNC_C1RX     = 55,
    INPUT_FUNC_C2RX     = 56,
    INPUT_FUNC_REFCLKI1 = 57,
    INPUT_FUNC_REFCLKI3 = 59,
    INPUT_FUNC_REFCLKI4 = 60

} PORTS_REMAP_INPUT_FUNCTION;

typedef enum {

    INPUT_PIN_RPD2  = 0x00,
    INPUT_PIN_RPG8  = 0x01,
    INPUT_PIN_RPF4  = 0x02,
    INPUT_PIN_RPD10 = 0x03,
    INPUT_PIN_RPF1  = 0x04,
    INPUT_PIN_RPB9  = 0x05,
    INPUT_PIN_RPB10 = 0x06,
    INPUT_PIN_RPC14 = 0x07,
    INPUT_PIN_RPB5  = 0x08,
    INPUT_PIN_RPC1  = 0x0A,
    INPUT_PIN_RPD14 = 0x0B,
    INPUT_PIN_RPG1  = 0x0C,
    INPUT_PIN_RPA14 = 0x0D,
    INPUT_PIN_RPD3  = 0x00,
    INPUT_PIN_RPG7  = 0x01,
    INPUT_PIN_RPF5  = 0x02,
    INPUT_PIN_RPD11 = 0x03,
    INPUT_PIN_RPF0  = 0x04,
    INPUT_PIN_RPB1  = 0x05,
    INPUT_PIN_RPE5  = 0x06,
    INPUT_PIN_RPC13 = 0x07,
    INPUT_PIN_RPB3  = 0x08,
    INPUT_PIN_RPC4  = 0x0A,
    INPUT_PIN_RPD15 = 0x0B,
    INPUT_PIN_RPG0  = 0x0C,
    INPUT_PIN_RPA15 = 0x0D,
    INPUT_PIN_RPD9  = 0x00,
    INPUT_PIN_RPG6  = 0x01,
    INPUT_PIN_RPB8  = 0x02,
    INPUT_PIN_RPB15 = 0x03,
    INPUT_PIN_RPD4  = 0x04,
    INPUT_PIN_RPB0  = 0x05,
    INPUT_PIN_RPE3  = 0x06,
    INPUT_PIN_RPB7  = 0x07,
    INPUT_PIN_RPF12 = 0x09,
    INPUT_PIN_RPD12 = 0x0A,
    INPUT_PIN_RPF8  = 0x0B,
    INPUT_PIN_RPC3  = 0x0C,
    INPUT_PIN_RPE9  = 0x0D,
    INPUT_PIN_RPD1  = 0x00,
    INPUT_PIN_RPG9  = 0x01,
    INPUT_PIN_RPB14 = 0x02,
    INPUT_PIN_RPD0  = 0x03,
    INPUT_PIN_RPB6  = 0x05,
    INPUT_PIN_RPD5  = 0x06,
    INPUT_PIN_RPB2  = 0x07,
    INPUT_PIN_RPF3  = 0x08,
    INPUT_PIN_RPF13 = 0x09,
    INPUT_PIN_NC    = 0x0A,
    INPUT_PIN_RPF2  = 0x0B,
    INPUT_PIN_RPC2  = 0x0C,
    INPUT_PIN_RPE8  = 0x0D

} PORTS_REMAP_INPUT_PIN;

typedef enum {

    OTPUT_FUNC_U3TX         = 0x01,
    OTPUT_FUNC_U4RTS        = 0x02,
    OTPUT_FUNC_SDO1         = 0x05,
    OTPUT_FUNC_SDO2         = 0x06,
    OTPUT_FUNC_SDO3         = 0x07,
    OTPUT_FUNC_SDO5         = 0x09,
    OTPUT_FUNC_SS6          = 0x0A,
    OTPUT_FUNC_OC3          = 0x0B,
    OTPUT_FUNC_OC6          = 0x0C,
    OTPUT_FUNC_REFCLKO4     = 0x0D,
    OTPUT_FUNC_C2OUT        = 0x0E,
    OTPUT_FUNC_C1TX         = 0x0F,
    OTPUT_FUNC_U1TX         = 0x01,
    OTPUT_FUNC_U2RTS        = 0x02,
    OTPUT_FUNC_U5TX         = 0x03,
    OTPUT_FUNC_U6RTS        = 0x04,
    OTPUT_FUNC_SDO4         = 0x08,
    OTPUT_FUNC_OC4          = 0x0B,
    OTPUT_FUNC_OC7          = 0x0C,
    OTPUT_FUNC_REFCLKO1     = 0x0F,
    OTPUT_FUNC_U3RTS        = 0x01,
    OTPUT_FUNC_U4TX         = 0x02,
    OTPUT_FUNC_U6TX         = 0x04,
    OTPUT_FUNC_SS1          = 0x05,
    OTPUT_FUNC_SS3          = 0x07,
    OTPUT_FUNC_SS4          = 0x08,
    OTPUT_FUNC_SS5          = 0x09,
    OTPUT_FUNC_SDO6         = 0x0A,
    OTPUT_FUNC_OC5          = 0x0B,
    OTPUT_FUNC_OC8          = 0x0C,
    OTPUT_FUNC_C1OUT        = 0x0E,
    OTPUT_FUNC_REFCLKO3     = 0x0F,
    OTPUT_FUNC_U1RTS        = 0x01,
    OTPUT_FUNC_U2TX         = 0x02,
    OTPUT_FUNC_U5RTS        = 0x03,
    OTPUT_FUNC_SS2          = 0x06,
    OTPUT_FUNC_OC2          = 0x0B,
    OTPUT_FUNC_OC1          = 0x0C,
    OTPUT_FUNC_OC9          = 0x0D,
    OTPUT_FUNC_C2TX         = 0x0F,
    OTPUT_FUNC_NO_CONNECT   = 0x00,
    OUTPUT_FUNC_U3TX        = 0x01,
    OUTPUT_FUNC_U4RTS       = 0x02,
    OUTPUT_FUNC_SDO1        = 0x05,
    OUTPUT_FUNC_SDO2        = 0x06,
    OUTPUT_FUNC_SDO3        = 0x07,
    OUTPUT_FUNC_SDO5        = 0x09,
    OUTPUT_FUNC_SS6         = 0x0A,
    OUTPUT_FUNC_OC3         = 0x0B,
    OUTPUT_FUNC_OC6         = 0x0C,
    OUTPUT_FUNC_REFCLKO4    = 0x0D,
    OUTPUT_FUNC_C2OUT       = 0x0E,
    OUTPUT_FUNC_C1TX        = 0x0F,
    OUTPUT_FUNC_U1TX        = 0x01,
    OUTPUT_FUNC_U2RTS       = 0x02,
    OUTPUT_FUNC_U5TX        = 0x03,
    OUTPUT_FUNC_U6RTS       = 0x04,
    OUTPUT_FUNC_SDO4        = 0x08,
    OUTPUT_FUNC_OC4         = 0x0B,
    OUTPUT_FUNC_OC7         = 0x0C,
    OUTPUT_FUNC_REFCLKO1    = 0x0F,
    OUTPUT_FUNC_U3RTS       = 0x01,
    OUTPUT_FUNC_U4TX        = 0x02,
    OUTPUT_FUNC_U6TX        = 0x04,
    OUTPUT_FUNC_SS1         = 0x05,
    OUTPUT_FUNC_SS3         = 0x07,
    OUTPUT_FUNC_SS4         = 0x08,
    OUTPUT_FUNC_SS5         = 0x09,
    OUTPUT_FUNC_SDO6        = 0x0A,
    OUTPUT_FUNC_OC5         = 0x0B,
    OUTPUT_FUNC_OC8         = 0x0C,
    OUTPUT_FUNC_C1OUT       = 0x0E,
    OUTPUT_FUNC_REFCLKO3    = 0x0F,
    OUTPUT_FUNC_U1RTS       = 0x01,
    OUTPUT_FUNC_U2TX        = 0x02,
    OUTPUT_FUNC_U5RTS       = 0x03,
    OUTPUT_FUNC_SS2         = 0x06,
    OUTPUT_FUNC_OC2         = 0x0B,
    OUTPUT_FUNC_OC1         = 0x0C,
    OUTPUT_FUNC_OC9         = 0x0D,
    OUTPUT_FUNC_C2TX        = 0x0F,
    OUTPUT_FUNC_NO_CONNECT  = 0x00

} PORTS_REMAP_OUTPUT_FUNCTION;

typedef enum {

    OUTPUT_PIN_RPA14    = 0,
    OUTPUT_PIN_RPA15    = 1,
    OUTPUT_PIN_RPB0     = 2,
    OUTPUT_PIN_RPB1     = 3,
    OUTPUT_PIN_RPB2     = 4,
    OUTPUT_PIN_RPB3     = 5,
    OUTPUT_PIN_RPB5     = 7,
    OUTPUT_PIN_RPB6     = 8,
    OUTPUT_PIN_RPB7     = 9,
    OUTPUT_PIN_RPB8     = 10,
    OUTPUT_PIN_RPB9     = 11,
    OUTPUT_PIN_RPB10    = 12,
    OUTPUT_PIN_RPB14    = 16,
    OUTPUT_PIN_RPB15    = 17,
    OUTPUT_PIN_RPC1     = 19,
    OUTPUT_PIN_RPC2     = 20,
    OUTPUT_PIN_RPC3     = 21,
    OUTPUT_PIN_RPC4     = 22,
    OUTPUT_PIN_RPC13    = 31,
    OUTPUT_PIN_RPC14    = 32,
    OUTPUT_PIN_RPD0     = 34,
    OUTPUT_PIN_RPD1     = 35,
    OUTPUT_PIN_RPD2     = 36,
    OUTPUT_PIN_RPD3     = 37,
    OUTPUT_PIN_RPD4     = 38,
    OUTPUT_PIN_RPD5     = 39,
    OUTPUT_PIN_RPD9     = 43,
    OUTPUT_PIN_RPD10    = 44,
    OUTPUT_PIN_RPD11    = 45,
    OUTPUT_PIN_RPD12    = 46,
    OUTPUT_PIN_RPD14    = 48,
    OUTPUT_PIN_RPD15    = 49,
    OUTPUT_PIN_RPE3     = 53,
    OUTPUT_PIN_RPE5     = 55,
    OUTPUT_PIN_RPE8     = 58,
    OUTPUT_PIN_RPE9     = 59,
    OUTPUT_PIN_RPF0     = 66,
    OUTPUT_PIN_RPF1     = 67,
    OUTPUT_PIN_RPF2     = 68,
    OUTPUT_PIN_RPF3     = 69,
    OUTPUT_PIN_RPF4     = 70,
    OUTPUT_PIN_RPF5     = 71,
    OUTPUT_PIN_RPF8     = 74,
    OUTPUT_PIN_RPF12    = 78,
    OUTPUT_PIN_RPF13    = 79,
    OUTPUT_PIN_RPG0     = 82,
    OUTPUT_PIN_RPG1     = 83,
    OUTPUT_PIN_RPG6     = 88,
    OUTPUT_PIN_RPG7     = 89,
    OUTPUT_PIN_RPG8     = 90,
    OUTPUT_PIN_RPG9     = 91

} PORTS_REMAP_OUTPUT_PIN;

typedef enum {

    PORT_CHANNEL_A  = 0x00,
    PORT_CHANNEL_B  = 0x01,
    PORT_CHANNEL_C  = 0x02,
    PORT_CHANNEL_D  = 0x03,
    PORT_CHANNEL_E  = 0x04,
    PORT_CHANNEL_F  = 0x05,
    PORT_CHANNEL_G  = 0x06

} PORTS_CHANNEL;

typedef enum {

    PORTS_CHANGE_NOTICE_EDGE_RISING     = 0,
    PORTS_CHANGE_NOTICE_EDGE_FALLING    = 1

} PORTS_CHANGE_NOTICE_EDGE;

typedef enum {

    PORTS_PIN_SLEW_RATE_FASTEST = 0x05,
    PORTS_PIN_SLEW_RATE_FAST    = 0x06,
    PORTS_PIN_SLEW_RATE_SLOW    = 0x09,
    PORTS_PIN_SLEW_RATE_SLOWEST = 0x0A

} PORTS_PIN_SLEW_RATE;

typedef enum {

    PORTS_CHANGE_NOTICE_METHOD_SAMPLED      = 0,
    PORTS_CHANGE_NOTICE_METHOD_EDGE_DETECT  = 1

} PORTS_CHANGE_NOTICE_METHOD;

#endif
