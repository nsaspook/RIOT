/* 
 * File:   spi.h
 * Author: root
 *
 * Created on October 25, 2016, 1:53 PM
 */

#ifndef SPI_H
#define	SPI_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

	void SPI_Init(void);
	void SPI_Speed(spi_t, spi_clk_t);

#ifdef	__cplusplus
}
#endif

#endif	/* SPI_H */

