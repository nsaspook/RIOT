/*
 * Copyright(C) 2016,2017, Imagination Technologies Limited and/or its
 *                 affiliated group companies.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @defgroup    boards_pic32-cpicmzef PIC32MZ EF Curiosity Development Board
 * @ingroup     boards
 * @brief       Support for the PIC32MZ EF Curiosity Development Board
 * @{
 *
 * @file
 * @brief       Support for the PIC32MZ EF Curiosity Development Board
 *
 * @author       Neil Jones <Neil.Jones@imgtec.com>
 */
#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#include "periph_cpu.h"
#include "vendor/p32mz2048efm100.h"
#include "vendor/ports_p32mz2048efm100.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   The peripheral clock is required for the UART Baud rate calculation
 *          It is configured by the 'config' registers (see pic32_config_settings.c)
 */
#define PERIPHERAL_CLOCK (100000000)  /* Hz */

/**
 * @name    Timer definitions
 * @{
 */
#define TIMER_NUMOF         (1)
#define TIMER_0_CHANNELS    (3)
/** @} */

#define CPU_CLOCK_HZ             (200000000UL)      // CPU Clock Speed in Hz
#define CPU_CT_HZ            (CPU_CLOCK_HZ / 2)     // CPU CoreTimer   in Hz
#define PERIPHERAL_CLOCK_HZ      (100000000UL)      // Peripheral Bus  in Hz

#define US_TO_CT_TICKS  (CPU_CT_HZ / 1000000UL)     // uS to CoreTimer Ticks

/**
 * @name    UART Definitions
 *          There are 6 UARTS available on this CPU.
 *          We route debug via UART4 on this board,
 *          this is the UART connected to the pic32 I/O header.
 *
 *          Note Microchip number the UARTS 1->6.
 * @{
 */
#define UART_NUMOF          (6)
#define UART_NUMOF_USED     (4)
#define DEBUG_VIA_UART      (4)
#define DEBUG_UART_BAUD     (115200)
/** @} */

/**
 * @name    SPI device configuration
 *
 * @{
 */
#define SPI_NUMOF   (6)
#define SPI_NUMOF_USED  (3)

static const spi_conf_t spi_config[] = {
    {
        .mosi_pin = 0,
    },      /* No SPI0 on PIC32, dummy to compile */

    {       /*
             * SPI 1 (MikBUS 1)
             *      MOSI -> RD3
             *      MISO -> RD14
             *      SCK  -> RD1
             */
        .mosi_pin = GPIO_PIN(PORT_D, 3),
        .mosi_reg = (volatile uint32_t *) &RPD3R,
        .mosi_af = OUTPUT_FUNC_SDO1,
        .miso_pin = GPIO_PIN(PORT_D, 14),
        .miso_reg = (volatile uint32_t *) &SDI1R,
        .miso_af = INPUT_PIN_RPD14,
        .int_mask = _IEC3_SPI1RXIE_MASK,     /* enable & flag mask */
        .iec_regclr = (volatile uint32_t *) &IEC3CLR,
        .iec_regset = (volatile uint32_t *) &IEC3SET,
        .ifs_regclr = (volatile uint32_t *) &IFS3CLR,
        .ipc_regset = (volatile uint32_t *) &IPC27SET,  /* IPC SFR */
        .ipc_mask_p = _IPC27_SPI1RXIP_MASK,             /* priority data mask */
        .ipc_mask_pos_p = _IPC27_SPI1RXIP_POSITION,     /* priority in SFR */
        .ipc_mask_s = _IPC27_SPI1RXIS_MASK,             /* sub-priority */
        .ipc_mask_pos_s = _IPC27_SPI1RXIS_POSITION,     /* sub-priority */
    },

    {     /*
           * SPI 2 (MikBUS 2)
           *      MOSI -> RG7
           *      MISO -> RG0
           *      SCK  -> RG6
           */
        .mosi_pin = GPIO_PIN(PORT_G, 7),
        .mosi_reg = (volatile uint32_t *) &RPG7R,
        .mosi_af = OUTPUT_FUNC_SDO2,
        .miso_pin = GPIO_PIN(PORT_G, 0),
        .miso_reg = (volatile uint32_t *) &SDI2R,
        .miso_af = INPUT_PIN_RPG0,
        .int_mask = _IEC4_SPI2RXIE_MASK,     /* enable & flag mask */
        .iec_regclr = (volatile uint32_t *) &IEC4CLR,
        .iec_regset = (volatile uint32_t *) &IEC4SET,
        .ifs_regclr = (volatile uint32_t *) &IFS4CLR,
        .ipc_regset = (volatile uint32_t *) &IPC35SET,  /* IPC SFR */
        .ipc_mask_p = _IPC35_SPI2RXIP_MASK,             /* priority data mask */
        .ipc_mask_pos_p = _IPC35_SPI2RXIP_POSITION,     /* priority in SFR */
        .ipc_mask_s = _IPC35_SPI2RXIS_MASK,             /* sub-priority */
        .ipc_mask_pos_s = _IPC35_SPI2RXIS_POSITION,     /* sub-priority */
    },

    {     /*
           * SPI 3 (MRF24WN0MA-1/RM100 - wifi module)
           *   MOSI -> RB9
           *   MISO -> RB10
           *   SCK  -> RB14
           */
        .mosi_pin = GPIO_PIN(PORT_B, 9),
        .mosi_reg = (volatile uint32_t *) &RPB9R,
        .mosi_af = OUTPUT_FUNC_SDO3,
        .miso_pin = GPIO_PIN(PORT_B, 10),
        .miso_reg = (volatile uint32_t *) &SDI3R,
        .miso_af = INPUT_PIN_RPB10,
        .int_mask = _IEC4_SPI3RXIE_MASK,     /* enable & flag mask */
        .iec_regclr = (volatile uint32_t *) &IEC4CLR,
        .iec_regset = (volatile uint32_t *) &IEC4SET,
        .ifs_regclr = (volatile uint32_t *) &IFS4CLR,
        .ipc_regset = (volatile uint32_t *) &IPC38SET,  /* IPC SFR */
        .ipc_mask_p = _IPC38_SPI3RXIP_MASK,             /* priority data mask */
        .ipc_mask_pos_p = _IPC38_SPI3RXIP_POSITION,     /* priority in SFR */
        .ipc_mask_s = _IPC38_SPI3RXIS_MASK,             /* sub-priority */
        .ipc_mask_pos_s = _IPC38_SPI3RXIS_POSITION,     /* sub-priority */
    },
    {
        .mosi_pin = 0,
    },
    {
        .mosi_pin = 0,
    },
    {
        .mosi_pin = 0,
    },
};
/** @} */

/**
 * @name    DMA device configuration
 *
 * @{
 */

#define DMA_NUMOF   (8)
/* DMA [0..3] used for SPI ports 1 and 2 */
#define SPI1_DMA_RX 0
#define SPI1_DMA_TX 1
#define SPI2_DMA_RX 2
#define SPI2_DMA_TX 3

static const dma_conf_t dma_config[] = {
    {
        .iec_mask = _IEC4_DMA0IE_MASK,                  /* enable */
        .ipc_regset = (volatile uint32_t *) &IPC33SET,  /* IPC SFR */
        .ipc_mask_p = _IPC33_DMA0IP_MASK,               /* priority data mask */
        .ipc_mask_pos_p = _IPC33_DMA0IP_POSITION,       /* priority in SFR */
        .ipc_mask_s = _IPC33_DMA0IS_MASK,               /* sub-priority */
        .ipc_mask_pos_s = _IPC33_DMA0IS_POSITION,       /* sub-priority */
    },
    {
        .iec_mask = 0,     /* DON'T enable */
        .ipc_regset = (volatile uint32_t *) &IPC33SET,
        .ipc_mask_p = _IPC33_DMA1IP_MASK,
        .ipc_mask_pos_p = _IPC33_DMA1IP_POSITION,
        .ipc_mask_s = _IPC33_DMA1IS_MASK,
        .ipc_mask_pos_s = _IPC33_DMA1IS_POSITION,
    },
    {
        .iec_mask = _IEC4_DMA0IE_MASK,
        .ipc_regset = (volatile uint32_t *) &IPC34SET,
        .ipc_mask_p = _IPC34_DMA2IP_MASK,
        .ipc_mask_pos_p = _IPC34_DMA2IP_POSITION,
        .ipc_mask_s = _IPC34_DMA2IS_MASK,
        .ipc_mask_pos_s = _IPC34_DMA2IS_POSITION,
    },
    {
        .iec_mask = 0,
        .ipc_regset = (volatile uint32_t *) &IPC34SET,
        .ipc_mask_p = _IPC34_DMA3IP_MASK,
        .ipc_mask_pos_p = _IPC34_DMA3IP_POSITION,
        .ipc_mask_s = _IPC34_DMA3IS_MASK,
        .ipc_mask_pos_s = _IPC34_DMA3IS_POSITION,
    },
    {
        .iec_mask = 0,
    },
    {
        .iec_mask = 0,
    },
    {
        .iec_mask = 0,
    },
    {
        .iec_mask = 0,
    },
};
/** @} */

/**
 * @name    UART device configuration
 *
 * @{
 */

static const uart_conf_t uart_config[] = {
    {
        .int_mask = 0,
    },
    {
        .int_mask = _IEC3_U1RXIE_MASK,     /* enable & flag mask */
        .iec_regclr = (volatile uint32_t *) &IEC3CLR,
        .iec_regset = (volatile uint32_t *) &IEC3SET,
        .ifs_regclr = (volatile uint32_t *) &IFS3CLR,
        .ipc_regset = (volatile uint32_t *) &IPC28SET,  /* IPC SFR */
        .ipc_mask_p = _IPC28_U1RXIP_MASK,               /* priority data mask */
        .ipc_mask_pos_p = _IPC28_U1RXIP_POSITION,       /* priority in SFR */
        .ipc_mask_s = _IPC28_U1RXIS_MASK,               /* sub-priority */
        .ipc_mask_pos_s = _IPC28_U1RXIS_POSITION,       /* sub-priority */
    },
    {
        .int_mask = _IEC4_U2RXIE_MASK,
        .iec_regclr = (volatile uint32_t *) &IEC4CLR,
        .iec_regset = (volatile uint32_t *) &IEC4SET,
        .ifs_regclr = (volatile uint32_t *) &IFS4CLR,
        .ipc_regset = (volatile uint32_t *) &IPC36SET,
        .ipc_mask_p = _IPC36_U2RXIP_MASK,
        .ipc_mask_pos_p = _IPC36_U2RXIP_POSITION,
        .ipc_mask_s = _IPC36_U2RXIS_MASK,
        .ipc_mask_pos_s = _IPC36_U2RXIS_POSITION,
    },
    {
        .int_mask = _IEC4_U3RXIE_MASK,
        .iec_regclr = (volatile uint32_t *) &IEC4CLR,
        .iec_regset = (volatile uint32_t *) &IEC4SET,
        .ifs_regclr = (volatile uint32_t *) &IFS4CLR,
        .ipc_regset = (volatile uint32_t *) &IPC39SET,
        .ipc_mask_p = _IPC39_U3RXIP_MASK,
        .ipc_mask_pos_p = _IPC39_U3RXIP_POSITION,
        .ipc_mask_s = _IPC39_U3RXIS_MASK,
        .ipc_mask_pos_s = _IPC39_U3RXIS_POSITION,
    },
    {
        .int_mask = _IEC5_U4RXIE_MASK,
        .iec_regclr = (volatile uint32_t *) &IEC5CLR,
        .iec_regset = (volatile uint32_t *) &IEC5SET,
        .ifs_regclr = (volatile uint32_t *) &IFS5CLR,
        .ipc_regset = (volatile uint32_t *) &IPC42SET,
        .ipc_mask_p = _IPC42_U4RXIP_MASK,
        .ipc_mask_pos_p = _IPC42_U4RXIP_POSITION,
        .ipc_mask_s = _IPC42_U4RXIS_MASK,
        .ipc_mask_pos_s = _IPC42_U4RXIS_POSITION,
    },
    {
        .int_mask = 0,
    },
    {
        .int_mask = 0,
    },
};
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
