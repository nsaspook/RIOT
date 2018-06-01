/*
 * Copyright(C) 2017, Imagination Technologies Limited and/or its
 *                 affiliated group companies.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup         cpu_mips_pic32_common
 * @brief           Shared CPU specific definitions for the MIPS family.
 * @{
 *
 * @file
 * @brief           Shared CPU specific definitions for the MIPS family.
 *
 * @author          Francois Berder <francois.berder@imgtec.com>
 */

#ifndef PERIPH_CPU_COMMON_H
#define PERIPH_CPU_COMMON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAVE_GPIO_T
/**
 * @brief   GPIO type identifier
 */
typedef unsigned int gpio_t;
#endif

/**
 * @name    Power management configuration
 * @{
 */
#define PROVIDES_PM_SET_LOWEST
/** @} */

/**
 * @brief   Length of the CPU_ID in bytes
 */
#define CPUID_LEN           (4U)

/**
 * @brief   Override GPIO pin selection macro
 */
#define GPIO_PIN(x, y)       ((x << 4) | (y & 0xf))

/**
 * @brief   Available ports on the PIC32 family
 */
enum {
    PORT_A  = 0,    /**< port A */
    PORT_B  = 1,    /**< port B */
    PORT_C  = 2,    /**< port C */
    PORT_D  = 3,    /**< port D */
    PORT_E  = 4,    /**< port E */
    PORT_F  = 5,    /**< port F */
    PORT_G  = 6,    /**< port G */
};

/**
 * @brief   Prevent shared timer functions from being used
 */
#define PERIPH_TIMER_PROVIDES_SET

/**
 * @brief   Use some common SPI functions
 * @{
 */
#define PERIPH_SPI_NEEDS_INIT_CS
#define PERIPH_SPI_NEEDS_TRANSFER_BYTE
#define PERIPH_SPI_NEEDS_TRANSFER_REG
#define PERIPH_SPI_NEEDS_TRANSFER_REGS
/** @} */

/**
 * @brief   Override SPI clock speed values
 * @{
 */
#define HAVE_SPI_CLK_T

typedef enum {
    SPI_CLK_100KHZ  = 100000U,      /**< drive the SPI bus with 100KHz */
    SPI_CLK_400KHZ  = 400000U,      /**< drive the SPI bus with 400KHz */
    SPI_CLK_1MHZ    = 1000000U,     /**< drive the SPI bus with 1MHz */
    SPI_CLK_2MHZ    = 2000000U,     /**< drive the SPI bus with 2MHz */
    SPI_CLK_5MHZ    = 5000000U,     /**< drive the SPI bus with 5MHz */
    SPI_CLK_10MHZ   = 10000000U,    /**< drive the SPI bus with 10MHz */
    SPI_CLK_25MHZ   = 25000000U,    /**< drive the SPI bus with 25MHz */
    SPI_CLK_50MHZ   = 50000000U     /**< drive the SPI bus with 50MHz */
} spi_clk_t;
/** @} */

/**
 * @brief   device configuration structures
 */
typedef struct {
    volatile uint32_t *mosi_reg;    /**< Output pin mux register address */
    volatile uint32_t *miso_reg;    /**< MISO pin mux register address */
    uint8_t mosi_af;                /**< Specify function of output pin */
    uint8_t miso_af;                /**< Specify input pin for MISO */
    gpio_t mosi_pin;                /**< GPIO pin for MOSI */
    gpio_t miso_pin;                /**< GPIO pin for MISO */
    volatile uint32_t *ipc_regset;  /* interrupt controller register */
    volatile uint32_t *iec_regset;
    volatile uint32_t *iec_regclr;
    volatile uint32_t *ifs_regset;
    volatile uint32_t *ifs_regclr;
    uint32_t int_mask;      /* enables and flags */
    uint32_t ipc_mask_p;    /* vector pri/sub-pri SFR masks and offsets */
    uint32_t ipc_mask_s;
    uint32_t ipc_mask_pos_p;
    uint32_t ipc_mask_pos_s;
} spi_conf_t;

typedef struct {
    volatile uint32_t *ipc_regset;  /* interrupt controller register */
    uint32_t iec_mask;              /* enables, needed to disable if set to zero */
    uint32_t ipc_mask_p;            /* vector pri/sub-pri SFR masks and offsets */
    uint32_t ipc_mask_s;
    uint32_t ipc_mask_pos_p;
    uint32_t ipc_mask_pos_s;
} dma_conf_t;

typedef struct {
    volatile uint32_t *ipc_regset;     /* interrupt controller register */
    volatile uint32_t *iec_regset;
    volatile uint32_t *iec_regclr;
    volatile uint32_t *ifs_regset;
    volatile uint32_t *ifs_regclr;
    uint32_t int_mask;      /* enables and flags */
    uint32_t ipc_mask_p;    /* vector pri/sub-pri SFR masks and offsets */
    uint32_t ipc_mask_s;
    uint32_t ipc_mask_pos_p;
    uint32_t ipc_mask_pos_s;
} uart_conf_t;

/* for 24-bit transmit and extra status data */
typedef struct mcp_adc_t {
    uint32_t dummy12 : 12;  // dummy space for adc data
    uint32_t nullbits : 2;
    uint32_t index : 3;     //adc channel select
    uint32_t single_diff : 1;
    uint32_t start_bit : 1;
    uint32_t dummy8 : 8;
    uint32_t finish : 1;
    uint32_t in_progress : 1;
} mcp_adc_t;

/* upper-> lower bytes to 32 bit word for ADC/DAC, etc ... */
union adc_bytes4 {
    uint32_t ld;
    uint8_t bd[4];
};

/* upper/lower bytes to 16 bit word for ADC/DAC, etc ... */
union adc_bytes2 {
    uint16_t ld;
    uint8_t bd[2];
};

/* used to hold 24-bit adc buffer, index and control bits */
union mcp_adc_buf_t {
    uint32_t ld;
    uint8_t bd[4];
    struct mcp_adc_t map;
};

typedef struct {
    union mcp_adc_buf_t mcp3208_cmd;
    uint16_t potValue;
    uint8_t chan;
} mcp3208_data_t;

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CPU_COMMON_H */
/** @} */
